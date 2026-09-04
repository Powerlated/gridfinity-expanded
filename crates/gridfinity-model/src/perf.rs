//! The model's own rows in `gridfinity_sketch::perf`'s one table.
//!
//! The kernel names its own scopes and knows nothing about a bin, so the five
//! scopes a bin's plan passes through are claimed as *user slots*: `Scope`
//! addresses one by index and names it on the way in, and the row appears in
//! `perf::snapshot()` beside the kernel's. Sharing the kernel's table rather
//! than keeping a second one is what lets `CountingAlloc` attribute an
//! allocation made inside `plan_piece` to `plan_piece` -- there is one scope
//! stack, and a second table would not be on it.

use gridfinity_sketch::perf;

/// A scope of the model's own, in the order it claims the kernel's user slots.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Scope {
    PlanPiece,
    PlanOuter,
    PlanCavity,
    PlanOps,
    PlanStitch,
}

impl Scope {
    /// The slot this scope occupies, which is its position in the enum.
    fn slot(self) -> usize {
        self as usize
    }

    /// What the row reads as in the profile. The two-space indent marks a step
    /// inside `plan_piece` rather than a scope of its own.
    fn name(self) -> &'static str {
        match self {
            Scope::PlanPiece => "gridfinity::plan_piece",
            Scope::PlanOuter => "  plan: outer loops",
            Scope::PlanCavity => "  plan: cavity",
            Scope::PlanOps => "  plan: peg loop",
            Scope::PlanStitch => "  plan: rings+stitch",
        }
    }
}

/// Opens `s`, naming its slot if this is the first time the process has, and
/// returns the guard whose drop records the elapsed time. Timing is off unless
/// `perf::set_enabled(true)`, in which case this costs one `OnceLock` read.
pub fn scope(s: Scope) -> perf::Scope {
    perf::name_user(s.slot(), s.name());
    perf::scope(perf::user(s.slot()))
}
