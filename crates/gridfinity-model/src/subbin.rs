//! The insert that drops into a fitted compartment: an open-top box, printed as
//! its own body, whose interior is a strictly bounded rectangular void.
//!
//! `SubbinSpec` states one outright -- where its outer box stands in the frame it
//! is built in, what its interior measures, and how thick the material around
//! that interior is -- and `build_subbin` is the whole of the geometry. There is
//! no walk and no boolean: five rings and six faces, each ring used exactly
//! twice, which is the manifold argument.
//!
//! Two things distinguish it from a Gridfinity bin, and both are the point of it.
//! Its **interior is square**: no corner radius and no floor blend, so an object
//! standing in it is held at its own size at every height rather than sinking
//! into a fillet. And its **outer bottom edge is chamfered at 45 degrees**, so it
//! nests into the concave floor blend of the compartment it drops into instead of
//! standing clear of it: a blend of radius `fr` leaves the void inset by
//! `fr - sqrt(fr^2 - (fr - z)^2)` at height `z`, a chamfer of leg `fr` insets by
//! `fr - z`, and the second is the larger for every `z` in `0..=fr`
//! (`a_chamfer_clears_the_blend_of_its_own_radius` restates it as a sweep). The
//! floor is at least as thick as the chamfer is tall, so the chamfer lives
//! wholly in solid material and the walls are their stated thickness the whole
//! way up.

use gridfinity_brep::build::{cap, loop_of, ring, wall_between};
use gridfinity_brep::geom::Surface;
use gridfinity_brep::math::{Vec3, vec3_of};
use gridfinity_brep::round::MIN_ARC_R;
use gridfinity_brep::sketch::{Seg, Sketch, ccw_segs};
use gridfinity_brep::topo::{Builder, Solid};

/// The shortest straight run a rounded outer profile must keep on each side, so
/// that the chamfer's two rings are the same eight segments and
/// `wall_between` pairs them one for one. `Sketch::rounded_rect` drops a
/// straight run shorter than 1e-6 mm, which would leave the two rings with
/// different segment counts and a silently twisted band.
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
pub fn build_subbin(spec: &SubbinSpec) -> Result<Solid, String> {
    check(spec)?;
    let top = spec.top();
    let floor_z = spec.z + spec.floor;
    let outer = outer_profile(spec, 0.0);
    let inner = interior_profile(spec);

    let mut b = Builder::new();
    let base_z = spec.z + spec.chamfer;
    let outer_lo = ring(&mut b, &outer, base_z);
    if spec.chamfer > 0.0 {
        let chamfered = outer_profile(spec, spec.chamfer);
        let bottom = ring(&mut b, &chamfered, spec.z);
        wall_between(
            &mut b, &chamfered, &outer, &bottom, &outer_lo, spec.z, base_z, true,
        );
        cap(&mut b, spec.z, false, &bottom, &[]);
    } else {
        cap(&mut b, spec.z, false, &outer_lo, &[]);
    }
    let outer_hi = ring(&mut b, &outer, top);
    wall_between(
        &mut b, &outer, &outer, &outer_lo, &outer_hi, base_z, top, true,
    );

    let inner_lo = ring(&mut b, &inner, floor_z);
    let inner_hi = ring(&mut b, &inner, top);
    wall_between(
        &mut b, &inner, &inner, &inner_lo, &inner_hi, floor_z, top, false,
    );
    b.face(
        Surface::plane(vec3_of(0.0, 0.0, floor_z), -Vec3::Z),
        false,
        loop_of(&inner_lo, false),
        vec![],
    );
    cap(&mut b, top, true, &outer_hi, &[&inner_hi]);

    let solid = b.build();
    let bands = outer.len() * if spec.chamfer > 0.0 { 2 } else { 1 };
    assert_eq!(
        solid.faces.len(),
        bands + inner.len() + 3,
        "an insert is one face per outer segment per band, one per interior segment, and the \
         bottom, floor and rim caps, which is {} and not {}",
        bands + inner.len() + 3,
        solid.faces.len()
    );
    assert_subbin_is_sound(&solid, spec);
    Ok(solid)
}

/// Asserts that a built insert is the sound, printable body its spec describes:
/// closed, sound under `audit`, one shell with material inside it, nothing
/// orphaned, and standing in exactly the box the spec stated.
fn assert_subbin_is_sound(solid: &Solid, spec: &SubbinSpec) {
    if let Err(e) = solid.validate() {
        panic!("a built insert is not a closed manifold: {e}");
    }
    let audited = crate::audit(solid);
    assert!(
        audited.is_ok(),
        "a built insert is not geometrically sound:\n{audited}"
    );
    assert!(
        solid.orphan_vertices().is_empty() && solid.orphan_edges().is_empty(),
        "a built insert carries {} vertex(es) and {} edge(s) nothing names",
        solid.orphan_vertices().len(),
        solid.orphan_edges().len()
    );
    let shells = solid.shells();
    assert!(
        shells.len() == 1 && shells[0].encloses_material,
        "a built insert is one shell with its material inside it, but this one has {} shell(s)",
        shells.len()
    );
    let (lo, hi) = bounds(solid);
    let want_lo = vec3_of(spec.x, spec.y, spec.z);
    let want_hi = vec3_of(
        spec.x + spec.outer_width,
        spec.y + spec.outer_depth,
        spec.top(),
    );
    assert!(
        (lo - want_lo).length() < 1e-6 && (hi - want_hi).length() < 1e-6,
        "an insert stated as {want_lo:?}..{want_hi:?} was built as {lo:?}..{hi:?}"
    );
}

/// The minimum and maximum corner of a solid's vertices, which for a body whose
/// every face is a plane, a cylinder or a cone through its own vertices is the
/// body's own bounding box.
fn bounds(solid: &Solid) -> (Vec3, Vec3) {
    assert!(
        !solid.verts.is_empty(),
        "a solid with no vertices has no bounding box"
    );
    let mut lo = solid.verts[0].point;
    let mut hi = lo;
    for v in &solid.verts {
        lo = lo.min(v.point);
        hi = hi.max(v.point);
    }
    (lo, hi)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec() -> SubbinSpec {
        SubbinSpec {
            x: 10.0,
            y: 20.0,
            z: 8.2,
            outer_width: 47.4,
            outer_depth: 126.4,
            interior_width: 45.0,
            interior_depth: 124.0,
            interior_height: 30.0,
            floor: 2.48,
            corner_r: 2.58,
            interior_corner_r: 2.58 - 1.2,
            chamfer: 2.48,
        }
    }

    /// The chamfer's whole reason for existing: at every height it spans, a 45
    /// degree chamfer of leg `fr` stands further in than the concave blend of
    /// radius `fr` it has to clear, so the insert never meets the blend.
    #[test]
    fn a_chamfer_clears_the_blend_of_its_own_radius() {
        let fr = 2.48;
        for i in 0..=200 {
            let z = fr * f64::from(i) / 200.0;
            let chamfer = fr - z;
            let blend = blend_inset(fr, z);
            assert!(
                chamfer >= blend - 1e-12,
                "at {z} mm above the floor a chamfer stands {chamfer} mm in and the blend \
                 {blend} mm, so the insert would meet it"
            );
        }
        assert_eq!(
            blend_inset(fr, fr),
            0.0,
            "the blend is spent at its own radius"
        );
        assert!(
            (blend_inset(fr, 0.0) - fr).abs() < 1e-12,
            "the blend takes its whole radius of floor at the wall"
        );
    }

    /// A built insert is sound and stands in the box it stated. `build_subbin`
    /// asserts both itself, so this is the fixture that exercises them.
    #[test]
    fn an_insert_is_a_sound_body_in_the_box_it_states() {
        let built = build_subbin(&spec()).expect("the fixture is a buildable insert");
        assert_eq!(
            built.shells().len(),
            1,
            "an insert is one shell, which is what makes it one printable part"
        );
    }

    /// The interior measures exactly the box it states: its corners are rounded
    /// *within* that box, and its floor is a plane, so an object is held at its
    /// stated size at every height and sinks into nothing.
    #[test]
    fn the_interior_is_the_box_it_states_with_rounded_corners_inside_it() {
        use gridfinity_brep::geom::Surface;
        let s = spec();
        let built = build_subbin(&s).expect("the fixture is a buildable insert");
        let floor_z = s.z + s.floor;
        let inside = |p: Vec3| {
            p.z >= floor_z - 1e-9
                && p.x >= s.x + s.walls().0 - 1e-9
                && p.x <= s.x + s.outer_width - s.walls().0 + 1e-9
                && p.y >= s.y + s.walls().1 - 1e-9
                && p.y <= s.y + s.outer_depth - s.walls().1 + 1e-9
        };
        let (mut planes, mut arcs) = (0, 0);
        let mut reach = (f64::INFINITY, f64::NEG_INFINITY);
        for fid in 0..built.faces.len() {
            let pts: Vec<Vec3> = built
                .outer_edges(fid)
                .iter()
                .map(|&(e, _)| built.verts[built.edges[e].v0].point)
                .collect();
            if !pts.iter().all(|p| inside(*p)) {
                continue;
            }
            match built.faces[fid].surface {
                Surface::Plane { .. } => planes += 1,
                Surface::Cylinder { radius, .. } => {
                    assert!(
                        (radius - s.interior_corner_r).abs() < 1e-9,
                        "an interior corner is rounded to {radius} mm, not the {} mm stated",
                        s.interior_corner_r
                    );
                    arcs += 1;
                }
                other => panic!("an interior face is a {other:?}, not a wall or a corner"),
            }
            for p in &pts {
                reach = (reach.0.min(p.x), reach.1.max(p.x));
            }
        }
        assert_eq!(
            (planes, arcs),
            (5, 4),
            "the interior is a floor, four walls and four corner arcs"
        );
        let want = (s.x + s.walls().0, s.x + s.outer_width - s.walls().0);
        assert!(
            (reach.0 - want.0).abs() < 1e-9 && (reach.1 - want.1).abs() < 1e-9,
            "the rounded interior measures {reach:?} across, not the {want:?} it states"
        );
    }

    /// The walls are a **constant width**, corners included: every interior
    /// corner cylinder is coaxial with an outer one and smaller by exactly the
    /// wall thickness.
    ///
    /// That is the whole content of `interior_corner_r = corner_r - wall`, and
    /// it is what a Gridfinity bin does *not* do -- its cavity radius is stated
    /// independently of its outline, so the material at its corners is whatever
    /// the two radii leave. Read off the built solid rather than the spec, so it
    /// is the geometry that is checked and not the arithmetic that produced it.
    #[test]
    fn the_walls_are_one_thickness_all_the_way_round_the_corners() {
        use gridfinity_brep::geom::Surface;
        let s = spec();
        let built = build_subbin(&s).expect("the fixture is a buildable insert");
        let mut axes: Vec<(Vec3, f64)> = Vec::new();
        for face in &built.faces {
            if let Surface::Cylinder { base, radius, .. } = face.surface {
                axes.push((base, radius));
            }
        }
        let (wall, _) = s.walls();
        let mut paired = 0;
        for (centre, radius) in &axes {
            if (*radius - s.corner_r).abs() > 1e-9 {
                continue;
            }
            let inner = axes
                .iter()
                .find(|(c, r)| {
                    (c.x - centre.x).abs() < 1e-9
                        && (c.y - centre.y).abs() < 1e-9
                        && (*r - s.interior_corner_r).abs() < 1e-9
                })
                .unwrap_or_else(|| {
                    panic!(
                        "the outer corner at ({}, {}) has no interior corner on its own axis, so \
                         the wall there is not the {wall} mm it is everywhere else",
                        centre.x, centre.y
                    )
                });
            assert!(
                (radius - inner.1 - wall).abs() < 1e-9,
                "a corner stands {} mm of material against {wall} mm runs",
                radius - inner.1
            );
            paired += 1;
        }
        assert_eq!(paired, 4, "an insert turns four corners");
    }

    /// A square-cornered interior is still buildable, and is what a run asking
    /// for no rounding gets.
    #[test]
    fn an_interior_may_be_left_square() {
        let s = SubbinSpec {
            interior_corner_r: 0.0,
            ..spec()
        };
        build_subbin(&s).expect("a square interior is an interior");
        assert_eq!(
            buildable_interior_corner(2.5, 8.0, 40.0),
            2.0,
            "a narrow interior turns the corner it can, not the one that was asked for"
        );
        assert_eq!(
            buildable_interior_corner(0.05, 40.0, 40.0),
            0.0,
            "a radius below the shortest arc worth building is a square corner"
        );
    }

    /// An insert whose compartment has no floor blend builds the same body with
    /// no chamfer at all.
    #[test]
    fn an_unchamfered_insert_builds() {
        let s = SubbinSpec {
            chamfer: 0.0,
            floor: 1.2,
            ..spec()
        };
        build_subbin(&s).expect("an insert with no chamfer is still an insert");
    }

    /// Every refusal is a statement about what was asked for, so each names the
    /// part of the spec at fault rather than failing inside the builder.
    #[test]
    fn a_spec_that_describes_no_insert_is_refused() {
        let thin = SubbinSpec {
            interior_width: 47.4,
            ..spec()
        };
        let err = build_subbin(&thin).expect_err("an interior as wide as its box has no wall");
        assert!(err.contains("width wall"), "{err}");

        let deep = SubbinSpec {
            chamfer: 4.0,
            ..spec()
        };
        let err = build_subbin(&deep).expect_err("a chamfer taller than the floor eats the wall");
        assert!(err.contains("chamfer"), "{err}");
    }
}
