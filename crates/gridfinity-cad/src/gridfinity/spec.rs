//! The normative Gridfinity dimensions, and the clamps the model holds itself
//! to when a `Params` asks for something they cannot carry.
//!
//! The first half is the standard: pitch, heights, the outer corner radius, the
//! three peg profiles and where their arcs are struck, the fastener bores. Those
//! are facts about the format and change only if the format does. The second
//! half is this implementation's: the tolerances that decide when two points are
//! one point, when a turn is a corner, when a radius is too small to be worth
//! building -- each named for the quantity it bounds -- and the two functions
//! that map a requested wall thickness or fillet radius onto one the geometry
//! admits. Nothing here builds anything.

pub const GRID_PITCH: f64 = 42.0;

pub const HEIGHT_PER_UNIT: f64 = 7.0;

pub const BASE_TOTAL_HEIGHT: f64 = 7.0;

pub const PEG_HEIGHT: f64 = 4.75;

pub const PEG_Z1: f64 = 0.8;

pub const PEG_Z2: f64 = 2.6;

pub const OUTER_R: f64 = 3.75;

pub const FLOOR_THICKNESS: f64 = 1.2;

pub const HALF_TOL: f64 = 0.25;

pub(super) const MAGNET_RADIUS: f64 = 3.25;

pub(super) const MAGNET_DEPTH: f64 = 2.4;

pub(super) const SCREW_RADIUS: f64 = 1.5;

pub(super) const SCREW_DEPTH: f64 = 6.0;

pub(super) const FASTENER_INSET: f64 = 13.0;

pub(super) const PEG_W_BOTTOM: f64 = 35.6;

pub(super) const PEG_W_MID: f64 = 37.2;

pub(super) const PEG_W_TOP: f64 = 41.5;

pub(super) const PEG_R_BOTTOM: f64 = 0.8;

pub(super) const PEG_R_MID: f64 = 1.6;

pub(super) const PEG_TANGENT: f64 = HALF_TOL + OUTER_R;

pub(super) const REENTRANT_FILLET_OVERHANG: f64 = 8.0;

/// The thinnest wall a *square* cavity corner can carry inside the outer arc.
/// A sharp corner of a cavity inset `wt` sits `sqrt(2) * (OUTER_R - wt)` from
/// the outer arc's centre, so `OUTER_R * (1 - 1/sqrt(2))` is where it reaches
/// exactly as far as the arc itself -- tangency, which leaves *zero* wall at
/// that point and still fails containment. The 0.05 is the clearance that
/// makes it a wall rather than a touch; measured, 1.0983 fails and 1.10 builds.
/// Only sloped bins need this: every other bin rounds its cavity corner
/// concentric with the outer arc instead.
pub(super) const SLOPED_MIN_WALL: f64 = OUTER_R * (1.0 - std::f64::consts::FRAC_1_SQRT_2) + 0.05;

/// The smallest blend radius worth emitting an op for, in millimetres. A
/// rolling ball this small is under one extrusion width, so the blend it would
/// build is invisible in the print and costs a torus face per edge to carry.
pub(super) const MIN_USEFUL_BLEND: f64 = 0.01;

/// The smallest corner radius worth rounding a rectangular quad by, in
/// millimetres -- an inner wall's end or a peg profile's corner. Below it the
/// arc is shorter than the weld quantum and the quad is emitted square.
pub(super) const MIN_QUAD_ROUND: f64 = 0.02;

/// How far apart two of a rounded quad's tangent points must be for the run
/// between them to be a segment rather than a coincidence, in millimetres. At
/// or under it the two arcs meet and no straight piece stands between them.
pub(super) const MIN_STRAIGHT_RUN: f64 = 1e-4;

/// How near two insets must be to be the same inset, in millimetres. Insets are
/// composed from `HALF_TOL`, a wall thickness and a divider half-thickness, so
/// two that should agree agree to a few ulps and two that should not differ by
/// at least `MIN_WALL / 2`.
pub(super) const INSET_SAME: f64 = 1e-6;

/// The magnitude that separates a turn from a straight run in the cross product
/// of two unit *axis-aligned* directions, which is exactly 0, +1 or -1. Half way
/// between is the robust discriminator, not a tolerance on a near miss.
pub(super) const TURN_SIGN: f64 = 0.5;

/// How much of the cavity's depth a sloped floor must leave below the rim, in
/// millimetres, so the high end of the ramp still stands inside the bin.
pub(super) const SLOPE_RIM_HEADROOM: f64 = 0.5;

/// How far above the sloped floor beneath it an island's own top must stand to
/// be a top at all, in millimetres. Closer than this the ramp has already risen
/// past the island and the island is capped at the rim instead.
pub(super) const SLOPE_ISLAND_HEADROOM: f64 = 0.2;

/// The smallest slope run, in millimetres, over which a gradient is meaningful.
/// Below it the piece has no extent along the uphill direction and the floor is
/// built flat.
pub(super) const MIN_SLOPE_SPAN: f64 = 1e-6;

/// The straight run a rounded corner must leave beside an opening's suppressed
/// corner, in millimetres. The arc is shortened to keep it, so the two corners
/// of one short run cannot consume the whole of it and meet.
pub(super) const OPEN_CORNER_CLEARANCE: f64 = 0.35;

/// The shortest a partial-height inner wall is built, in millimetres. A wall the
/// user set to nothing still has to be a solid with a top face for the ramp to
/// blend against; below this it is a sliver no chain can run out along.
pub(super) const MIN_PARTIAL_WALL_HEIGHT: f64 = 0.5;

/// The steepest floor slope the model builds, as a gradient rather than an
/// angle. Past this the ramp is nearer a wall than a floor and the tilted plane
/// meets the cavity's own walls at a grazing angle nothing prints.
pub(super) const MAX_SLOPE_GRADIENT: f64 = 3.0;

/// The four corners of a cell a fastener bore is sunk in, as signed unit offsets
/// from the cell centre to be scaled by `FASTENER_INSET`.
pub(super) const FASTENER_QUADRANTS: [(f64, f64); 4] =
    [(-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)];

/// The thinnest wall the model will build at all, in millimetres -- under half a
/// typical 0.4 mm nozzle's bead either side of the cavity, so a thinner one is
/// not a wall a printer can produce.
pub(super) const MIN_WALL: f64 = 0.4;

/// How far an opened bin's wall must stay clear of `PEG_TANGENT`, in
/// millimetres. An opening lets the cavity run out to the pitch line, so the
/// cavity's inset `HALF_TOL + wt` at the walls either side of it must still
/// leave a straight stub of outline between the wall and the corner arc's
/// tangent point; at `PEG_TANGENT` exactly there is no stub and the wall meets
/// the arc at a point.
pub(super) const OPEN_WALL_HEADROOM: f64 = 0.6;

/// How far below the rim a floor fillet must stop, in millimetres. A blend of
/// the full cavity depth reaches the rim and leaves the wall no straight face
/// above it for the chain to run out against.
pub(super) const FILLET_DEPTH_HEADROOM: f64 = 0.05;

/// How far inside the convex cavity corner a concave floor fillet must stay, in
/// millimetres, so the corner arc remains a curve the blend rolls *along*
/// rather than one it degenerates onto -- the same degeneracy `MIN_TORUS_MAJOR`
/// bounds per contact segment, applied to the corner radius up front.
pub(super) const FILLET_CORNER_HEADROOM: f64 = 0.02;

/// The smallest cavity corner radius that counts as a rounded corner rather
/// than a square one, in millimetres. Below it the cavity is built sharp and
/// takes no floor fillet at all, because a blend has no corner arc to turn.
pub(super) const MIN_ROUNDED_CORNER: f64 = 0.05;

/// The wall thickness one piece is actually built with, given the thickness
/// `Params` asked for and what the bin is: `MIN_WALL` at the thin end always,
/// `PEG_TANGENT - OPEN_WALL_HEADROOM` at the thick end when any edge is open,
/// and `SLOPED_MIN_WALL` at the thin end when the floor is sloped.
///
/// The sloped floor is why the last clamp exists. It builds its cavity square,
/// and a sharp convex corner sits `sqrt(2) * (OUTER_R - wt)` from the outer
/// arc's centre while the arc itself reaches only `OUTER_R`; below
/// `SLOPED_MIN_WALL` the cavity escapes the rounded corner entirely, is no
/// longer inside the rim face it is a hole of, and panicked `plan_piece` with
/// `total_h hole without a containing face`. A flat bin keeps its wall by
/// rounding the cavity concentric with the outer arc; a sloped one cannot,
/// because `ring_on_plane` names an arc on a tilted plane with a Z-axis circle
/// while the true section is an ellipse.
pub(super) fn buildable_wall_thickness(want: f64, openish: bool, sloped: bool) -> f64 {
    let wt = want.max(MIN_WALL);
    let wt = if openish {
        wt.min(PEG_TANGENT - OPEN_WALL_HEADROOM)
    } else {
        wt
    };
    let wt = if sloped { wt.max(SLOPED_MIN_WALL) } else { wt };
    assert!(
        wt >= MIN_WALL && wt.is_finite(),
        "a piece's wall thickness is at least MIN_WALL, got {wt} from a requested {want}"
    );
    wt
}

/// The floor fillet radius one piece actually asks for, given the radius
/// `Params` wanted, the cavity's depth, the convex corner radius `rc` the
/// cavity is rounded by, and whether the floor is sloped.
///
/// Zero on a sloped floor, which takes no blend, and zero where the cavity's
/// corners are square (`rc <= MIN_ROUNDED_CORNER`), since a blend rolling into a
/// sharp corner has no arc to turn. Otherwise the request, held below the rim by
/// `FILLET_DEPTH_HEADROOM` and inside the corner by `FILLET_CORNER_HEADROOM`.
/// The result is a radius the compartment's *depth and corners* admit; whether
/// its width admits it is `max_inward_radius`'s question, asked per loop later.
pub(super) fn buildable_floor_fillet(want: f64, cavity_depth: f64, rc: f64, sloped: bool) -> f64 {
    if sloped || rc <= MIN_ROUNDED_CORNER {
        return 0.0;
    }
    let fr = want
        .min(cavity_depth - FILLET_DEPTH_HEADROOM)
        .max(0.0)
        .min(rc - FILLET_CORNER_HEADROOM);
    assert!(
        fr < rc && fr < cavity_depth,
        "a floor fillet stays inside the corner it turns and the cavity it sits in, but {fr} was \
         settled against a corner radius {rc} and a depth {cavity_depth}"
    );
    fr.max(0.0)
}

/// How far, in radians, a point's angle about a circle's centre may sit outside
/// an arc's stored range and still be a point of it -- and, equally, how near
/// two cut angles must be to be one cut. At the profile's `OUTER_R` this is
/// 3.8e-4 mm of arc, comfortably inside `COINCIDENT` so the angular and positional
/// tests agree about which points coincide.
pub(super) const ARC_ENDPOINT_ANGLE: f64 = 1e-4;
