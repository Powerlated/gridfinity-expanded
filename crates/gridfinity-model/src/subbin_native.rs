//! OCCT-built open-top inserts for fitted compartments.

use gridfinity_sketch::round::MIN_ARC_R;
use gridfinity_sketch::sketch::{Seg, Sketch, ccw_segs};

const MIN_STRAIGHT_RUN: f64 = 0.05;

/// One insert, stated outright in the millimetres of the frame it is built in.
///
/// `x`/`y`/`z` are the minimum corner of the **outer** box, so the solid comes
/// back already standing where it belongs and nothing downstream transforms it.
/// The interior is centred in the outer box, which is what makes `wall` the
/// thickness on all four sides.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SubbinSpec {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub outer_width: f64,
    pub outer_depth: f64,
    /// The interior's own measurements: the void's bounding box is exactly this,
    /// from the top of the floor to the rim. `interior_corner_r` rounds its four
    /// vertical corners *within* that box, so the stated width and depth are
    /// still what the interior measures across its faces.
    pub interior_width: f64,
    pub interior_depth: f64,
    pub interior_height: f64,
    /// How thick the floor under the interior is. Never less than `chamfer`, so
    /// the chamfer band stands in solid material and never eats into a wall.
    pub floor: f64,
    /// The radius the outer box's four vertical corners are rounded by. Never
    /// less than the corner radius of the compartment this drops into -- a
    /// rounded rectangle only shrinks as its radius grows, so rounding at least
    /// as much is what keeps the insert inside that corner.
    pub corner_r: f64,
    /// The radius the interior's four vertical corners are rounded by, zero for
    /// a square-cornered void.
    ///
    /// **`corner_r - wall`, which is what makes the walls a constant width.**
    /// Offsetting a rounded rectangle inward by `wall` gives radius
    /// `corner_r - wall`, so the two arcs are *concentric* and the material
    /// between them measures `wall` all the way round the corner -- unlike a
    /// Gridfinity bin, whose cavity radius is stated independently of its
    /// outline and whose corner wall is therefore whatever the two leave. A
    /// narrower interior may clamp it further (`buildable_interior_corner`),
    /// which only ever makes the corner *thicker* than the runs; `check`
    /// refuses one that would make it thinner.
    ///
    /// It takes nothing off the stated interior -- the box is still
    /// `interior_width` by `interior_depth` across its faces -- and the **floor
    /// stays sharp**: a corner fillet is a vertical blend an object sits beside,
    /// where a floor blend is one it would sink into, which is the whole reason
    /// an insert exists.
    pub interior_corner_r: f64,
    /// The leg of the 45 degree chamfer around the bottom outer edge, which is
    /// the compartment's own built floor fillet. Zero for a compartment with no
    /// blend, which builds the same body with a flat bottom edge.
    pub chamfer: f64,
}

impl SubbinSpec {
    /// The insert's overall height: its floor plus the interior above it.
    pub fn height(&self) -> f64 {
        self.floor + self.interior_height
    }

    /// The wall thickness the outer and interior boxes imply, on each axis in
    /// turn. Equal on both axes for an insert derived by
    /// `outer = interior + 2 * wall`, and legitimately unequal for one whose
    /// compartment grew on one axis only.
    pub fn walls(&self) -> (f64, f64) {
        (
            (self.outer_width - self.interior_width) / 2.0,
            (self.outer_depth - self.interior_depth) / 2.0,
        )
    }

    /// The top of the insert, in the frame it is built in.
    pub fn top(&self) -> f64 {
        self.z + self.height()
    }
}

/// How far the void inside a compartment blended by `fillet` stands in from the
/// compartment's wall at height `z` above its floor: the rolling ball's own
/// profile, and zero once `z` is past the blend.
///
/// This is the quantity the chamfer is measured against, and it is stated here
/// rather than in a comment so the test that holds the chamfer clear of it is
/// checking the same arithmetic the model builds the blend from.
pub fn blend_inset(fillet: f64, z: f64) -> f64 {
    assert!(
        fillet >= 0.0 && z >= 0.0,
        "a blend of {fillet} mm read at height {z} mm is not a blend inside a compartment"
    );
    if z >= fillet {
        return 0.0;
    }
    fillet - (fillet * fillet - (fillet - z) * (fillet - z)).sqrt()
}

/// The outer profile at height `inset` in from the outer box on every side: the
/// box drawn back by that much, its corners rounded by whatever the corner
/// radius has left after the same retreat.
///
/// One producer for both rings of the chamfer band, so the two agree segment for
/// segment by construction: a rounded rectangle inset by `d` is the same
/// rectangle less `2d` on each axis with radius `r - d`, and its straight runs
/// are `w - 2r` and `d - 2r` whatever `d` is.
fn outer_profile(spec: &SubbinSpec, inset: f64) -> Vec<Seg> {
    assert!(
        inset >= 0.0 && inset < spec.corner_r,
        "the outer profile of an insert is drawn back by {inset} mm, which its {} mm corner \
         cannot turn",
        spec.corner_r
    );
    let sketch = Sketch::rounded_rect(
        spec.x + spec.outer_width / 2.0,
        spec.y + spec.outer_depth / 2.0,
        spec.outer_width - 2.0 * inset,
        spec.outer_depth - 2.0 * inset,
        spec.corner_r - inset,
    );
    let segs = ccw_segs(&sketch);
    assert_eq!(
        segs.len(),
        8,
        "an insert's outer profile is four runs and four corner arcs, but drawn back {inset} mm \
         it came to {} segment(s)",
        segs.len()
    );
    segs
}

/// The interior profile: the rectangle the void is, centred in the outer box,
/// with its four vertical corners rounded by `interior_corner_r`.
///
/// Four segments when the corners are square and eight when they are rounded,
/// either way a closed loop whose bounding box is exactly the stated interior.
fn interior_profile(spec: &SubbinSpec) -> Vec<Seg> {
    let sketch = Sketch::rounded_rect(
        spec.x + spec.outer_width / 2.0,
        spec.y + spec.outer_depth / 2.0,
        spec.interior_width,
        spec.interior_depth,
        spec.interior_corner_r,
    );
    let segs = ccw_segs(&sketch);
    assert!(
        segs.len() == if spec.interior_corner_r > 0.0 { 8 } else { 4 },
        "an insert's interior is four runs and, where it is rounded by {} mm, four corner arcs, \
         but it came to {} segment(s)",
        spec.interior_corner_r,
        segs.len()
    );
    segs
}

/// The interior corner radius an insert of this interior can actually turn: the
/// radius its outer corner implies, held to a quarter of the smaller side so a
/// narrow insert keeps a straight run on every face rather than being refused.
///
/// `want` is `corner_r - wall`, the radius that puts the two corner arcs on one
/// centre and makes the wall a constant width; the clamp can only reduce it, and
/// a smaller interior radius is a *thicker* corner, never a thinner one.
///
/// The same shape of clamp `buildable_floor_fillet` applies to a bin: the caller
/// states what it wants and the geometry states what it can carry.
pub fn buildable_interior_corner(want: f64, interior_width: f64, interior_depth: f64) -> f64 {
    assert!(
        interior_width > 0.0 && interior_depth > 0.0,
        "an interior of {interior_width} x {interior_depth} mm is not a box to round the corners of"
    );
    let most = interior_width.min(interior_depth) / 4.0;
    let r = want.max(0.0).min(most);
    if r < MIN_ARC_R { 0.0 } else { r }
}

/// Everything the spec must satisfy before any of it is built, or the first
/// reason it describes no insert.
///
/// Refusals rather than assertions: every one of them is a property of what the
/// caller asked for -- an interior wider than the box around it, a chamfer no
/// corner can turn -- and the `optimize` command turns them into a named error
/// against the object that asked.
fn check(spec: &SubbinSpec) -> Result<(), String> {
    let (wall_x, wall_y) = spec.walls();
    for (axis, outer, interior, wall) in [
        ("width", spec.outer_width, spec.interior_width, wall_x),
        ("depth", spec.outer_depth, spec.interior_depth, wall_y),
    ] {
        if interior <= 0.0 || outer <= 0.0 {
            return Err(format!(
                "an insert of {outer} x {interior} mm in {axis} is not a box"
            ));
        }
        if wall <= 0.0 {
            return Err(format!(
                "an insert whose interior is {interior} mm across a {outer} mm outer box has no \
                 {axis} wall left"
            ));
        }
        if outer - 2.0 * spec.corner_r < MIN_STRAIGHT_RUN {
            return Err(format!(
                "an insert {outer} mm in {axis} cannot turn two {} mm corners and keep a \
                 straight run between them",
                spec.corner_r
            ));
        }
    }
    check_corners(spec, wall_x.min(wall_y))?;
    if spec.interior_height <= 0.0 || spec.floor <= 0.0 {
        return Err(format!(
            "an insert {} mm deep inside on a {} mm floor holds nothing",
            spec.interior_height, spec.floor
        ));
    }
    if spec.chamfer < 0.0 || spec.chamfer > spec.floor {
        return Err(format!(
            "an insert's {} mm bottom chamfer stands in its floor, which is {} mm thick",
            spec.chamfer, spec.floor
        ));
    }
    Ok(())
}

/// Everything the two corner radii must satisfy against walls no thinner than
/// `thinnest`, or the first reason they describe no insert.
///
/// The middle one is the whole of "constant width": a wall is `thinnest` along
/// every run by construction, and it is that at the corners exactly when the two
/// arcs sit on one centre, which is `corner_r - interior_corner_r == wall`. A
/// clamped interior radius leaves the corner *thicker*, which is why this is
/// one-sided.
fn check_corners(spec: &SubbinSpec, thinnest: f64) -> Result<(), String> {
    assert!(
        thinnest > 0.0,
        "a corner is checked against {thinnest} mm walls, which is not a wall"
    );
    if spec.interior_corner_r < 0.0
        || spec.interior_width.min(spec.interior_depth) - 2.0 * spec.interior_corner_r
            < MIN_STRAIGHT_RUN
    {
        return Err(format!(
            "an interior of {} x {} mm cannot turn two {} mm corners and keep a straight run \
             between them",
            spec.interior_width, spec.interior_depth, spec.interior_corner_r
        ));
    }
    if spec.interior_corner_r > 0.0 && spec.corner_r - spec.interior_corner_r < thinnest - 1e-9 {
        return Err(format!(
            "an insert rounded {} mm outside and {} mm inside has a {} mm corner wall against \
             {thinnest} mm runs, so its walls are not a constant width",
            spec.corner_r,
            spec.interior_corner_r,
            spec.corner_r - spec.interior_corner_r
        ));
    }
    if spec.chamfer > 0.0 && spec.corner_r - spec.chamfer < MIN_ARC_R {
        return Err(format!(
            "an insert chamfered by {} mm needs an outer corner of more than {} mm to turn at \
             its base, but has {}",
            spec.chamfer,
            spec.chamfer + MIN_ARC_R,
            spec.corner_r
        ));
    }
    Ok(())
}

/// One insert as a closed solid, standing where its spec puts it: an outer box
/// with a chamfered bottom edge, hollowed to the square-cornered interior the
/// spec states, open at the top.
///
/// Six faces over five rings -- bottom cap, chamfer band, outer wall, rim
/// annulus, interior wall, interior floor -- so every ring is used exactly
/// twice and the result is a closed manifold by construction. A spec with no
/// chamfer builds the same body with the bottom cap directly under the outer
/// wall.
///
/// The returned solid is closed, geometrically sound, bounded by one shell with
/// material inside it, and carries no vertex or edge nothing names -- the four
/// properties `gridfinity::carve_to_cells` holds a printable piece to, asserted
/// here because this is where an insert is produced and there is no later place
/// to state them.
/// Builds the insert as one native OCCT body: the same insert stated as one native
/// kernel body. The outer skin is a prism when its bottom is square and a
/// three-section loft when it is chamfered; subtracting the interior prism
/// opens the top and leaves the stated floor beneath it.
///
/// This deliberately lives beside the analytic builder during migration. A
/// caller can compare the two bodies without changing the type returned by
/// `build_subbin`, while every operation in this path is owned by OCCT.
pub fn build_subbin_occt(spec: &SubbinSpec) -> Result<gridfinity_occt::Shape, String> {
    use gridfinity_occt::{Boolean, Shape};

    check(spec)?;
    let outer = crate::occt::profile(&outer_profile(spec, 0.0));
    let body = if spec.chamfer > 0.0 {
        let bottom = crate::occt::profile(&outer_profile(spec, spec.chamfer));
        Shape::loft(&[
            (&bottom, spec.z),
            (&outer, spec.z + spec.chamfer),
            (&outer, spec.top()),
        ])
    } else {
        Shape::prism(&outer, spec.z, spec.height())
    }
    .map_err(|e| format!("OCCT could not build the insert's outside: {e}"))?;

    let interior = crate::occt::profile(&interior_profile(spec));
    let void = Shape::prism(&interior, spec.z + spec.floor, spec.interior_height)
        .map_err(|e| format!("OCCT could not build the insert's interior: {e}"))?;
    let body = body
        .boolean(&void, Boolean::Cut)
        .map_err(|e| format!("OCCT could not hollow the insert: {e}"))?;
    if !body
        .is_valid()
        .map_err(|e| format!("OCCT could not validate the insert: {e}"))?
    {
        return Err("OCCT built an invalid insert".to_string());
    }
    Ok(body)
}

#[cfg(all(test, feature = "occt"))]
mod tests {
    use super::*;
    #[test]
    fn native_insert_is_valid_and_hollow() {
        let spec = SubbinSpec {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            outer_width: 30.0,
            outer_depth: 24.0,
            interior_width: 26.0,
            interior_depth: 20.0,
            interior_height: 12.0,
            floor: 2.0,
            corner_r: 3.0,
            interior_corner_r: 1.0,
            chamfer: 1.0,
        };
        let body = build_subbin_occt(&spec).expect("native insert");
        assert!(
            body.is_valid().expect("validity"),
            "an OCCT insert must be valid"
        );
        assert_eq!(
            body.shell_volumes().expect("shells").len(),
            1,
            "an OCCT insert must contain one material shell"
        );
        let b = body.bounds().expect("bounds");
        assert!(
            (b.min[2] - spec.z).abs() < 1e-5 && (b.max[2] - spec.top()).abs() < 1e-5,
            "an OCCT insert must occupy its stated vertical extent"
        );
    }
}
