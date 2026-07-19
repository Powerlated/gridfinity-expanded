//! Lightweight instrumentation for the heavy parts of the pipeline.
//!
//! The point is to answer "where did that rebuild go?" with numbers rather than
//! intuition — which is why the metric set is deliberately small and names the
//! operations that are actually expensive (region booleans, the closed-form
//! seg/seg solve, builder interning, blending, tessellation) instead of trying
//! to be a general profiler.
//!
//! **Off by default.** Every entry point begins with one relaxed load of
//! [`ENABLED`], so an uninstrumented build pays a predictable-branch atomic read
//! and nothing else — no timer, no allocation, no contention. The debugger
//! turns it on around a rebuild and off again.
//!
//! Counters are global relaxed atomics. Geometry is built on one thread, so
//! relaxed is sufficient and there is no synchronisation to pay for; if a build
//! is ever parallelised the totals stay correct (sums of per-thread work), only
//! the wall-clock readings would need revisiting.

use std::alloc::{GlobalAlloc, Layout};
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering::Relaxed};
use std::time::Instant;

/// What gets counted. Keep this short: each variant is a row in the debugger,
/// and a metric nobody reads is overhead with a UI cost attached.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Metric {
    /// 2D boolean classification — the core of every region op.
    SplitRegions,
    /// Closed-form seg/seg intersection, the inner loop of the above.
    SegSegPoints,
    /// Closed-form loop/loop distance (island clearance tests).
    MinLoopDistance,
    /// Point-in-loop, called per classified piece.
    PointInSegs,
    /// Vertex interning (hash + weld).
    BuilderVertex,
    /// Arc-edge interning.
    BuilderArc,
    /// Face construction.
    BuilderFace,
    /// Rolling-ball blending, including its solid rebuild.
    BlendEdges,
    /// Analytic faces to triangles.
    Tessellate,
    /// Slab stack resolution.
    BuildSlabs,
    /// Slab stack emission into an existing builder.
    EmitSlabs,
    /// Surface/surface intersection dispatch.
    IntersectSurfaces,
}

impl Metric {
    pub const ALL: [Metric; 12] = [
        Metric::SplitRegions,
        Metric::SegSegPoints,
        Metric::MinLoopDistance,
        Metric::PointInSegs,
        Metric::BuilderVertex,
        Metric::BuilderArc,
        Metric::BuilderFace,
        Metric::BlendEdges,
        Metric::Tessellate,
        Metric::BuildSlabs,
        Metric::EmitSlabs,
        Metric::IntersectSurfaces,
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
            Metric::BlendEdges => "fillet::blend_edges",
            Metric::Tessellate => "tess::tessellate",
            Metric::BuildSlabs => "slab::build_slabs",
            Metric::EmitSlabs => "slab::emit_slabs",
            Metric::IntersectSurfaces => "isect::intersect_surfaces",
        }
    }
}

const N: usize = Metric::ALL.len();

static ENABLED: AtomicBool = AtomicBool::new(false);
#[allow(clippy::declare_interior_mutable_const)]
const ZERO: AtomicU64 = AtomicU64::new(0);
static CALLS: [AtomicU64; N] = [ZERO; N];
static NANOS: [AtomicU64; N] = [ZERO; N];

/// Allocations credited to each metric, **exclusive**: an allocation is charged
/// to the innermost open [`scope`] at the moment it happens, so — unlike
/// [`NANOS`], which nest — these columns partition the attributed allocations
/// rather than double-counting them. Allocations made outside every scope
/// (most construction, e.g. the transient `Loop` Vecs) land in neither array;
/// the shortfall against the global [`ALLOCS`]/[`ALLOC_BYTES`] total is exactly
/// that unattributed churn, which is what the SoA rework is meant to remove.
static ALLOC_CALLS_BY: [AtomicU64; N] = [ZERO; N];
static ALLOC_BYTES_BY: [AtomicU64; N] = [ZERO; N];

/// Fixed-depth stack of the currently-open metric scopes, innermost last, one
/// per thread. **Deliberately allocation-free** (a plain `Copy` array in a
/// `Cell`, never a `Vec`): it is read and written from inside the global
/// allocator, so pushing a scope must not itself allocate or the allocator
/// re-enters. Overflow past `STACK_MAX` silently stops nesting deeper — an
/// attribution gap, never a crash.
const STACK_MAX: usize = 32;

#[derive(Clone, Copy)]
struct ScopeStack {
    len: usize,
    items: [u8; STACK_MAX],
}

impl ScopeStack {
    const fn new() -> ScopeStack {
        ScopeStack { len: 0, items: [0; STACK_MAX] }
    }
}

thread_local! {
    static SCOPES: Cell<ScopeStack> = const { Cell::new(ScopeStack::new()) };
}

fn push_scope(m: Metric) {
    SCOPES.with(|s| {
        let mut st = s.get();
        if st.len < STACK_MAX {
            st.items[st.len] = m as u8;
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

/// Index of the innermost open scope, if any.
fn innermost_scope() -> Option<usize> {
    SCOPES.with(|s| {
        let st = s.get();
        (st.len > 0).then(|| st.items[st.len - 1] as usize)
    })
}

/// Allocations and bytes since the last [`reset`], filled in by
/// [`CountingAlloc`]. Separate from the metric table because allocation is a
/// property of the whole rebuild, not of one operation.
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

/// Count a call without timing it.
///
/// For leaves hot enough that reading the clock would dominate what is being
/// measured — `point_in_segs` runs millions of times in a rebuild, and two
/// `Instant::now()` calls around it would cost more than the function.
#[inline]
pub fn count(m: Metric) {
    if enabled() {
        CALLS[m as usize].fetch_add(1, Relaxed);
    }
}

/// Time a call for as long as the returned guard lives.
///
/// Timings **nest**: `split_regions` includes the `seg_seg_points` beneath it,
/// so the columns do not sum to the total and should be read as "time spent
/// anywhere under this operation".
#[inline]
pub fn scope(m: Metric) -> Scope {
    if enabled() {
        push_scope(m);
        Scope { m, start: Some(Instant::now()) }
    } else {
        Scope { m, start: None }
    }
}

pub struct Scope {
    m: Metric,
    start: Option<Instant>,
}

impl Drop for Scope {
    fn drop(&mut self) {
        if let Some(t) = self.start {
            // Pop before recording so a re-entrant allocation here is not
            // mis-credited to a scope that is already closing.
            pop_scope();
            CALLS[self.m as usize].fetch_add(1, Relaxed);
            NANOS[self.m as usize].fetch_add(t.elapsed().as_nanos() as u64, Relaxed);
        }
    }
}

/// One metric's totals since the last [`reset`].
///
/// `nanos` nest (time spent anywhere under the op); `alloc_calls`/`alloc_bytes`
/// are exclusive (charged to the innermost open scope) — see [`ALLOC_CALLS_BY`].
pub struct Row {
    pub name: &'static str,
    pub calls: u64,
    pub nanos: u64,
    pub alloc_calls: u64,
    pub alloc_bytes: u64,
}

/// Allocation totals since the last [`reset`].
pub struct Allocs {
    pub count: u64,
    pub bytes: u64,
    pub peak_live_bytes: u64,
}

/// Every metric with a non-zero call count, heaviest first (by time, then
/// calls, so untimed leaves still sort sensibly).
pub fn snapshot() -> Vec<Row> {
    let mut rows: Vec<Row> = Metric::ALL
        .iter()
        .map(|&m| Row {
            name: m.name(),
            calls: CALLS[m as usize].load(Relaxed),
            nanos: NANOS[m as usize].load(Relaxed),
            alloc_calls: ALLOC_CALLS_BY[m as usize].load(Relaxed),
            alloc_bytes: ALLOC_BYTES_BY[m as usize].load(Relaxed),
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

/// A `GlobalAlloc` wrapper that counts allocations while [`enabled`].
///
/// Install it in the *binary* (a library must not choose the allocator for its
/// dependents):
///
/// ```ignore
/// #[global_allocator]
/// static ALLOC: CountingAlloc<std::alloc::System> = CountingAlloc::new(std::alloc::System);
/// ```
///
/// `peak_live_bytes` tracks allocated-minus-freed, so it reports the high-water
/// mark of a rebuild rather than its churn — the two answer different questions
/// and the churn (`bytes`) is usually the actionable one here.
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
            // `saturating_add`: geometry is single-threaded in production, but the
            // test harness installs this allocator and runs tests in parallel, so
            // the live/peak counters can race. Saturating keeps it panic-free and
            // bounded (peak is then approximate under concurrency); the churn
            // totals above use fetch_add and stay exact regardless of ordering.
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
            // CAS loop so a concurrent dealloc can't drive LIVE below zero and
            // wrap to a huge value (which would then overflow the add above).
            let size = layout.size() as u64;
            let _ = LIVE.fetch_update(Relaxed, Relaxed, |v| Some(v.saturating_sub(size)));
        }
        unsafe { self.inner.dealloc(ptr, layout) }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The counters are global, so these tests cannot run concurrently — one
    /// toggling `ENABLED` would corrupt the other's reading.
    static LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// Disabled is the default and must record nothing, so a normal build is
    /// unaffected by the presence of the instrumentation.
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

        // `ENABLED` is global, so tests running in parallel are also counting
        // into these totals. Assert what pollution cannot break: our own calls
        // are included, and the ordering contract holds.
        let pis = rows.iter().find(|r| r.name == Metric::PointInSegs.name()).expect("counted");
        assert!(pis.calls >= 2, "want >=2 calls, got {}", pis.calls);
        let tess = rows.iter().find(|r| r.name == Metric::Tessellate.name()).expect("timed");
        assert!(tess.calls >= 1);
        assert!(tess.nanos >= 1_000_000, "expected >=1ms, got {}ns", tess.nanos);
        assert!(
            rows.windows(2).all(|w| w[0].nanos >= w[1].nanos),
            "snapshot must be sorted heaviest first"
        );
    }

    /// An allocation made inside a scope is charged to that scope; the
    /// attribution is exclusive, so a deeper nested scope keeps its own bytes.
    #[test]
    fn allocations_are_charged_to_the_innermost_scope() {
        let _g = LOCK.lock().unwrap_or_else(|e| e.into_inner());
        set_enabled(true);
        reset();
        {
            let _outer = scope(Metric::SplitRegions);
            // Attributed to SplitRegions.
            let a: Vec<u8> = Vec::with_capacity(4096);
            std::hint::black_box(&a);
            {
                let _inner = scope(Metric::SegSegPoints);
                // Attributed to SegSegPoints, not SplitRegions.
                let b: Vec<u8> = Vec::with_capacity(8192);
                std::hint::black_box(&b);
            }
        }
        let rows = snapshot();
        set_enabled(false);

        let outer = rows.iter().find(|r| r.name == Metric::SplitRegions.name()).expect("charged");
        let inner = rows.iter().find(|r| r.name == Metric::SegSegPoints.name()).expect("charged");
        // The stack is global; parallel tests may add to these, so assert our
        // own contribution is present rather than an exact byte count.
        assert!(outer.alloc_bytes >= 4096, "outer got {} B", outer.alloc_bytes);
        assert!(inner.alloc_bytes >= 8192, "inner got {} B", inner.alloc_bytes);
        assert!(outer.alloc_calls >= 1 && inner.alloc_calls >= 1);
    }
}
