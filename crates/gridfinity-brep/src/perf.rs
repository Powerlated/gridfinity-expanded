//! Call counts, wall time and allocation attribution for the scopes a build
//! passes through.
//!
//! `Metric` names the kernel's own scopes and nothing else. A crate built on
//! the kernel gets rows in the same table through `user`: `USER_SLOTS` further
//! slots it claims by index and names once with `name_user`. They have to share
//! this table rather than keep their own, because `CountingAlloc` attributes an
//! allocation to whatever scope is innermost on one thread-local stack, and two
//! tables would mean two stacks and each crate's allocations landing in the
//! other's rows.

use std::alloc::{GlobalAlloc, Layout};
use std::cell::Cell;
use std::sync::OnceLock;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed};
use std::time::Instant;

#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Metric {
    SplitRegions,
    SegSegPoints,
    MinLoopDistance,
    PointInSegs,
    BuilderVertex,
    BuilderArc,
    BuilderFace,
    FilletEdges,
    Tessellate,
    BuildSlabs,
    EmitSlabs,
    IntersectSurfaces,
    ProgramRun,
}

impl Metric {
    pub const ALL: [Metric; 13] = [
        Metric::SplitRegions,
        Metric::SegSegPoints,
        Metric::MinLoopDistance,
        Metric::PointInSegs,
        Metric::BuilderVertex,
        Metric::BuilderArc,
        Metric::BuilderFace,
        Metric::FilletEdges,
        Metric::Tessellate,
        Metric::BuildSlabs,
        Metric::EmitSlabs,
        Metric::IntersectSurfaces,
        Metric::ProgramRun,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Metric::SplitRegions => "region2d::split_regions",
            Metric::SegSegPoints => "region2d::seg_seg_points",
            Metric::MinLoopDistance => "region2d::min_loop_distance",
            Metric::PointInSegs => "sketch::point_in_segs",
            Metric::BuilderVertex => "topo::Builder::vertex",
            Metric::BuilderArc => "topo::Builder::arc",
            Metric::BuilderFace => "topo::Builder::face",
            Metric::FilletEdges => "fillet::fillet_edges",
            Metric::Tessellate => "tess::tessellate",
            Metric::BuildSlabs => "slab::build_slabs",
            Metric::EmitSlabs => "slab::emit_slabs",
            Metric::IntersectSurfaces => "isect::intersect_surfaces",
            Metric::ProgramRun => "program::run",
        }
    }
}

/// How many rows of the table are left for crates built on the kernel to claim.
/// A slot is one scope: `user(i)` addresses it and `name_user(i, ..)` says what
/// it is called, and until something names one it holds no row at all.
pub const USER_SLOTS: usize = 8;

const N: usize = Metric::ALL.len() + USER_SLOTS;

/// A row of the table, either one of the kernel's own or a claimed user slot.
/// Its `usize` is the index every counter array is addressed by.
#[derive(Clone, Copy, PartialEq, Eq)]
pub struct Scoped(u8);

impl From<Metric> for Scoped {
    fn from(m: Metric) -> Scoped {
        Scoped(m as u8)
    }
}

/// User slot `slot`, which is `slot`-th of `USER_SLOTS` after the kernel's own
/// rows. Panics unless `slot < USER_SLOTS`, a slot past the end being a row
/// that does not exist rather than one that is empty.
pub fn user(slot: usize) -> Scoped {
    assert!(
        slot < USER_SLOTS,
        "user slot {slot} is past the {USER_SLOTS} the table holds"
    );
    Scoped((Metric::ALL.len() + slot) as u8)
}

static USER_NAMES: [OnceLock<&'static str>; USER_SLOTS] = [const { OnceLock::new() }; USER_SLOTS];

/// Names user slot `slot` `name`, once. A second call with the same name is the
/// ordinary case -- the caller names the slot wherever it opens the scope --
/// and a second call with a *different* one is a slot claimed twice, which
/// would put two scopes' time in one row, so it panics.
pub fn name_user(slot: usize, name: &'static str) {
    assert!(
        slot < USER_SLOTS,
        "user slot {slot} is past the {USER_SLOTS} the table holds"
    );
    let held = USER_NAMES[slot].get_or_init(|| name);
    assert_eq!(
        *held, name,
        "user slot {slot} is already {held}, so naming it {name} would put two scopes in one row"
    );
}

/// What the row at `index` is called: a kernel metric's own name, a user slot's
/// registered one, or `None` for a slot nothing has claimed.
fn row_name(index: usize) -> Option<&'static str> {
    assert!(index < N, "row {index} is past the {N} the table holds");
    match Metric::ALL.get(index) {
        Some(m) => Some(m.name()),
        None => USER_NAMES[index - Metric::ALL.len()].get().copied(),
    }
}

static ENABLED: AtomicBool = AtomicBool::new(false);
#[allow(clippy::declare_interior_mutable_const)]
const ZERO: AtomicU64 = AtomicU64::new(0);
static CALLS: [AtomicU64; N] = [ZERO; N];
static NANOS: [AtomicU64; N] = [ZERO; N];

static ALLOC_CALLS_BY: [AtomicU64; N] = [ZERO; N];
static ALLOC_BYTES_BY: [AtomicU64; N] = [ZERO; N];

const STACK_MAX: usize = 32;

#[derive(Clone, Copy)]
struct ScopeStack {
    len: usize,
    items: [u8; STACK_MAX],
}

impl ScopeStack {
    const fn new() -> ScopeStack {
        ScopeStack {
            len: 0,
            items: [0; STACK_MAX],
        }
    }
}

thread_local! {
    static SCOPES: Cell<ScopeStack> = const { Cell::new(ScopeStack::new()) };
}

fn push_scope(m: Scoped) {
    SCOPES.with(|s| {
        let mut st = s.get();
        if st.len < STACK_MAX {
            st.items[st.len] = m.0;
            st.len += 1;
            s.set(st);
        }
    });
}

fn pop_scope() {
    SCOPES.with(|s| {
        let mut st = s.get();
        if st.len > 0 {
            st.len -= 1;
            s.set(st);
        }
    });
}

fn innermost_scope() -> Option<usize> {
    SCOPES.with(|s| {
        let st = s.get();
        (st.len > 0).then(|| st.items[st.len - 1] as usize)
    })
}

static ALLOCS: AtomicU64 = AtomicU64::new(0);
static ALLOC_BYTES: AtomicU64 = AtomicU64::new(0);
static PEAK_LIVE: AtomicU64 = AtomicU64::new(0);
static LIVE: AtomicU64 = AtomicU64::new(0);

#[inline]
pub fn enabled() -> bool {
    ENABLED.load(Relaxed)
}

pub fn set_enabled(on: bool) {
    ENABLED.store(on, Relaxed);
}

pub fn reset() {
    for i in 0..N {
        CALLS[i].store(0, Relaxed);
        NANOS[i].store(0, Relaxed);
        ALLOC_CALLS_BY[i].store(0, Relaxed);
        ALLOC_BYTES_BY[i].store(0, Relaxed);
    }
    ALLOCS.store(0, Relaxed);
    ALLOC_BYTES.store(0, Relaxed);
    LIVE.store(0, Relaxed);
    PEAK_LIVE.store(0, Relaxed);
}

#[inline]
pub fn count(m: impl Into<Scoped>) {
    if enabled() {
        CALLS[m.into().0 as usize].fetch_add(1, Relaxed);
    }
}

#[inline]
pub fn scope(m: impl Into<Scoped>) -> Scope {
    let m = m.into();
    if enabled() {
        push_scope(m);
        Scope {
            m,
            start: Some(Instant::now()),
        }
    } else {
        Scope { m, start: None }
    }
}

pub struct Scope {
    m: Scoped,
    start: Option<Instant>,
}

impl Drop for Scope {
    fn drop(&mut self) {
        if let Some(t) = self.start {
            pop_scope();
            CALLS[self.m.0 as usize].fetch_add(1, Relaxed);
            NANOS[self.m.0 as usize].fetch_add(t.elapsed().as_nanos() as u64, Relaxed);
        }
    }
}

pub struct Row {
    pub name: &'static str,
    pub calls: u64,
    pub nanos: u64,
    pub alloc_calls: u64,
    pub alloc_bytes: u64,
}

pub struct Allocs {
    pub count: u64,
    pub bytes: u64,
    pub peak_live_bytes: u64,
}

pub fn snapshot() -> Vec<Row> {
    let mut rows: Vec<Row> = (0..N)
        .filter_map(|i| {
            Some(Row {
                name: row_name(i)?,
                calls: CALLS[i].load(Relaxed),
                nanos: NANOS[i].load(Relaxed),
                alloc_calls: ALLOC_CALLS_BY[i].load(Relaxed),
                alloc_bytes: ALLOC_BYTES_BY[i].load(Relaxed),
            })
        })
        .filter(|r| r.calls > 0)
        .collect();
    rows.sort_by(|a, b| b.nanos.cmp(&a.nanos).then(b.calls.cmp(&a.calls)));
    rows
}

pub fn allocs() -> Allocs {
    Allocs {
        count: ALLOCS.load(Relaxed),
        bytes: ALLOC_BYTES.load(Relaxed),
        peak_live_bytes: PEAK_LIVE.load(Relaxed),
    }
}

pub struct CountingAlloc<A> {
    inner: A,
}

impl<A> CountingAlloc<A> {
    pub const fn new(inner: A) -> CountingAlloc<A> {
        CountingAlloc { inner }
    }
}

unsafe impl<A: GlobalAlloc> GlobalAlloc for CountingAlloc<A> {
    unsafe fn alloc(&self, layout: Layout) -> *mut u8 {
        if enabled() {
            let size = layout.size() as u64;
            ALLOCS.fetch_add(1, Relaxed);
            ALLOC_BYTES.fetch_add(size, Relaxed);
            let live = LIVE.fetch_add(size, Relaxed).saturating_add(size);
            PEAK_LIVE.fetch_max(live, Relaxed);
            if let Some(mi) = innermost_scope() {
                ALLOC_CALLS_BY[mi].fetch_add(1, Relaxed);
                ALLOC_BYTES_BY[mi].fetch_add(size, Relaxed);
            }
        }
        unsafe { self.inner.alloc(layout) }
    }

    unsafe fn dealloc(&self, ptr: *mut u8, layout: Layout) {
        if enabled() {
            let size = layout.size() as u64;
            let _ = LIVE.fetch_update(Relaxed, Relaxed, |v| Some(v.saturating_sub(size)));
        }
        unsafe { self.inner.dealloc(ptr, layout) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    #[test]
    fn disabled_records_nothing() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        set_enabled(false);
        reset();
        count(Metric::PointInSegs);
        {
            let _s = scope(Metric::Tessellate);
        }
        assert!(snapshot().is_empty(), "counters moved while disabled");
    }

    #[test]
    fn enabled_counts_and_times() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        set_enabled(true);
        reset();
        count(Metric::PointInSegs);
        count(Metric::PointInSegs);
        {
            let _s = scope(Metric::Tessellate);
            std::thread::sleep(std::time::Duration::from_millis(2));
        }
        let rows = snapshot();
        set_enabled(false);

        let pis = rows
            .iter()
            .find(|r| r.name == Metric::PointInSegs.name())
            .expect("counted");
        assert!(pis.calls >= 2, "want >=2 calls, got {}", pis.calls);
        let tess = rows
            .iter()
            .find(|r| r.name == Metric::Tessellate.name())
            .expect("timed");
        assert!(tess.calls >= 1);
        assert!(
            tess.nanos >= 1_000_000,
            "expected >=1ms, got {}ns",
            tess.nanos
        );
        assert!(
            rows.windows(2).all(|w| w[0].nanos >= w[1].nanos),
            "snapshot must be sorted heaviest first"
        );
    }

    #[test]
    fn allocations_are_charged_to_the_innermost_scope() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        set_enabled(true);
        reset();
        {
            let _outer = scope(Metric::SplitRegions);
            let a: Vec<u8> = Vec::with_capacity(4096);
            std::hint::black_box(&a);
            {
                let _inner = scope(Metric::SegSegPoints);
                let b: Vec<u8> = Vec::with_capacity(8192);
                std::hint::black_box(&b);
            }
        }
        let rows = snapshot();
        set_enabled(false);

        let outer = rows
            .iter()
            .find(|r| r.name == Metric::SplitRegions.name())
            .expect("charged");
        let inner = rows
            .iter()
            .find(|r| r.name == Metric::SegSegPoints.name())
            .expect("charged");
        assert!(
            outer.alloc_bytes >= 4096,
            "outer got {} B",
            outer.alloc_bytes
        );
        assert!(
            inner.alloc_bytes >= 8192,
            "inner got {} B",
            inner.alloc_bytes
        );
        assert!(outer.alloc_calls >= 1 && inner.alloc_calls >= 1);
    }
}
