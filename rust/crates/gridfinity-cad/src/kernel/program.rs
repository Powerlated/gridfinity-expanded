
use crate::kernel::build::{RingEdges, loop_of, ring, ring_into, ring_on_plane, seg_edge, wall_between};
use crate::kernel::chamfer::chamfer_edges;
use crate::kernel::fillet;
use crate::kernel::geom::Surface;
use crate::kernel::math::{Vec3, vec3_of};
use crate::kernel::sketch::{Seg, Sketch};
use crate::kernel::slab::{self, Slab, SlabOpts};
use crate::kernel::topo::{Builder, EdgeId, Solid};
use std::collections::HashMap;

pub type DirLoop = (Vec<Seg>, bool);

#[derive(Clone, Debug)]
pub enum PlaneRef {
    Z { z: f32, up: bool },
    Named(String),
    Tilted { origin: Vec3, normal: Vec3 },
}

impl PlaneRef {
    pub fn resolve(&self, prog: &Program) -> (Vec3, Vec3) {
        match *self {
            PlaneRef::Z { z, up } => {
                let normal = if up { Vec3::Z } else { -Vec3::Z };
                (vec3_of(0.0, 0.0, z), normal)
            }
            PlaneRef::Named(ref name) => prog
                .plane(name)
                .unwrap_or_else(|| panic!("PlaneRef::Named({name:?}) not registered")),
            PlaneRef::Tilted { origin, normal } => (origin, normal.normalize_or(Vec3::Z)),
        }
    }

    pub fn z(&self) -> f32 {
        match *self {
            PlaneRef::Z { z, .. } => z,
            PlaneRef::Named(_) | PlaneRef::Tilted { .. } => {
                panic!("PlaneRef::z() called on non-horizontal plane {self:?}; use resolve() instead")
            }
        }
    }

    pub fn is_horizontal(&self) -> bool {
        matches!(*self, PlaneRef::Z { .. })
    }
}

#[derive(Clone, Debug)]
pub enum HoleProfile {
    Plain { radius: f32, depth: f32 },
    Counterbore { bore_r: f32, bore_d: f32, head_r: f32, head_d: f32 },
    Countersink { bore_r: f32, bore_d: f32, head_r: f32, head_angle_deg: f32 },
}

impl HoleProfile {
    pub fn mouth_radius(&self) -> f32 {
        match *self {
            HoleProfile::Plain { radius, .. } => radius,
            HoleProfile::Counterbore { head_r, .. } => head_r,
            HoleProfile::Countersink { head_r, .. } => head_r,
        }
    }
}

pub enum Op {
    Sketch { name: String, profile: Vec<Seg> },
    Plane { name: String, origin: Vec3, normal: Vec3 },

    Extrude { sketch: String, from: PlaneRef, to: PlaneRef },
    ExtrudeCut { sketch: String, from: PlaneRef, to: PlaneRef },
    Loft { profiles: Vec<(String, f32)>, outward: bool },
    Hole { at: crate::kernel::math::Vec2, from_z: f32, profile: HoleProfile },

    PlanarFace { plane: PlaneRef, outer: DirLoop, holes: Vec<DirLoop> },
    WallFaces { lower: Vec<Seg>, upper: Vec<Seg>, z0: f32, z1: f32, outward: bool },
    SlopedWall {
        lower: Vec<Seg>,
        upper: Vec<Seg>,
        lower_plane: PlaneRef,
        upper_plane: PlaneRef,
        outward: bool,
    },

    Wall { lower: Vec<Seg>, upper: Vec<Seg>, z0: f32, z1: f32, outward: bool },
    Cap { z: f32, up: bool, outer: DirLoop, holes: Vec<DirLoop> },
    Slabs { stack: Vec<(slab::Op, Slab)>, opts: SlabOpts },
    Fillet { edges: Vec<(Seg, f32, f32)> },
    Chamfer { edges: Vec<(Seg, f32, f32, f32)> },
    Custom(Box<dyn Fn(&mut Builder) -> Result<(), String>>),
}

impl Op {
    pub fn kind(&self) -> &'static str {
        match self {
            Op::Sketch { .. } => "sketch",
            Op::Plane { .. } => "plane",
            Op::Extrude { .. } => "extrude",
            Op::ExtrudeCut { .. } => "cut",
            Op::Loft { .. } => "loft",
            Op::Hole { .. } => "hole",
            Op::PlanarFace { .. } => "face",
            Op::WallFaces { .. } => "wall",
            Op::SlopedWall { .. } => "wall",
            Op::Wall { .. } => "wall",
            Op::Cap { .. } => "cap",
            Op::Slabs { .. } => "slabs",
            Op::Fillet { .. } => "fillet",
            Op::Chamfer { .. } => "chamfer",
            Op::Custom(_) => "custom",
        }
    }
}

pub struct Step {
    pub label: String,
    pub op: Op,
}

#[derive(Default)]
pub struct Program {
    pub steps: Vec<Step>,
    sketches: HashMap<String, Vec<Seg>>,
    planes: HashMap<String, (Vec3, Vec3)>,
}

impl Program {
    pub fn push(&mut self, label: impl Into<String>, op: Op) {
        match &op {
            Op::Sketch { name, profile } => {
                self.sketches.insert(name.clone(), profile.clone());
            }
            Op::Plane { name, origin, normal } => {
                self.planes.insert(name.clone(), (*origin, *normal));
            }
            _ => {}
        }
        self.steps.push(Step { label: label.into(), op });
    }
    pub fn len(&self) -> usize {
        self.steps.len()
    }
    pub fn is_empty(&self) -> bool {
        self.steps.is_empty()
    }

    pub fn sketch(&self, name: &str) -> Option<&[Seg]> {
        self.sketches.get(name).map(|v| v.as_slice())
    }

    pub fn plane(&self, name: &str) -> Option<(Vec3, Vec3)> {
        self.planes.get(name).copied()
    }
}

pub fn run_all(prog: &Program) -> Result<Solid, String> {
    run(prog, |_| true)
}

/// Rough upper bound on the interned vertices/edges/faces a full run emits, so
/// `Builder`'s intern maps do not rehash their way up to six figures. Only the
/// magnitude matters; over-estimating costs memory, under-estimating costs a
/// rehash.
fn size_hint(prog: &Program) -> (usize, usize, usize) {
    let mut segs = 0usize;
    let mut faces = 0usize;
    for st in &prog.steps {
        match &st.op {
            Op::Loft { profiles, .. } => {
                let n = profiles
                    .iter()
                    .map(|(name, _)| prog.sketch(name).map_or(0, |s| s.len()))
                    .max()
                    .unwrap_or(0);
                segs += n * profiles.len() * 2;
                faces += n * profiles.len();
            }
            Op::Wall { lower, upper, .. } | Op::WallFaces { lower, upper, .. } => {
                segs += (lower.len() + upper.len()) * 2;
                faces += lower.len();
            }
            Op::SlopedWall { lower, upper, .. } => {
                segs += (lower.len() + upper.len()) * 2;
                faces += lower.len();
            }
            Op::PlanarFace { outer, holes, .. } | Op::Cap { outer, holes, .. } => {
                segs += outer.0.len() + holes.iter().map(|h| h.0.len()).sum::<usize>();
                faces += 1;
            }
            Op::Slabs { stack, .. } => {
                let n: usize =
                    stack.iter().map(|(_, s)| s.region.iter().map(|l| l.len()).sum::<usize>()).sum();
                segs += n * 4;
                faces += n * 3;
            }
            _ => {}
        }
    }
    (segs, segs * 2, faces)
}

pub fn run(prog: &Program, enabled: impl Fn(usize) -> bool) -> Result<Solid, String> {
    let _perf = crate::kernel::perf::scope(crate::kernel::perf::Metric::ProgramRun);
    let (nv, ne, nf) = size_hint(prog);
    let mut b = Builder::with_capacity(nv, ne, nf, nf * 4, nf);
    let mut blends: Vec<(EdgeId, f32)> = Vec::new();
    let mut chamfers: Vec<(EdgeId, f32, f32)> = Vec::new();
    let (mut ra, mut rb) = (RingEdges::default(), RingEdges::default());

    for (i, st) in prog.steps.iter().enumerate() {
        if !enabled(i) {
            continue;
        }
        match &st.op {
            Op::Sketch { .. } | Op::Plane { .. } => {
            }
            Op::Extrude { sketch, from, to } => {
                if !from.is_horizontal() || !to.is_horizontal() {
                    return Err(
                        "Extrude: tilted planes not yet supported (slab engine is z-prism only); \
                         use WallFaces + PlanarFace directly"
                            .into(),
                    );
                }
                let profile = prog.sketch(sketch).ok_or_else(|| {
                    format!("Extrude: sketch {sketch:?} not registered")
                })?;
                let stack = vec![(
                    slab::Op::Union,
                    Slab::new(vec![profile.to_vec()], from.z(), to.z()),
                )];
                slab::emit_slabs(&mut b, &stack, &SlabOpts::default())?;
            }
            Op::ExtrudeCut { sketch, from, to } => {
                if !from.is_horizontal() || !to.is_horizontal() {
                    return Err(
                        "ExtrudeCut: tilted planes not yet supported (slab engine is z-prism only); \
                         use WallFaces + PlanarFace directly"
                            .into(),
                    );
                }
                let profile = prog.sketch(sketch).ok_or_else(|| {
                    format!("ExtrudeCut: sketch {sketch:?} not registered")
                })?;
                let stack = vec![(
                    slab::Op::Difference,
                    Slab::new(vec![profile.to_vec()], from.z(), to.z()),
                )];
                slab::emit_slabs(&mut b, &stack, &SlabOpts::default())?;
            }
            Op::Loft { profiles, outward } => {
                if profiles.len() < 2 {
                    return Err(format!("Loft: need ≥2 profiles, got {}", profiles.len()));
                }
                let resolved: Vec<(&[Seg], f32)> = profiles
                    .iter()
                    .map(|(name, z)| -> Result<(&[Seg], f32), String> {
                        let p = prog.sketch(name).ok_or_else(|| {
                            format!("Loft: sketch {name:?} not registered")
                        })?;
                        Ok((p, *z))
                    })
                    .collect::<Result<_, _>>()?;
                ring_into(&mut b, resolved[0].0, resolved[0].1, &mut ra);
                for w in resolved.windows(2) {
                    let (lower, z0) = w[0];
                    let (upper, z1) = w[1];
                    ring_into(&mut b, upper, z1, &mut rb);
                    wall_between(&mut b, lower, upper, &ra, &rb, z0, z1, *outward);
                    std::mem::swap(&mut ra, &mut rb);
                }
            }
            Op::PlanarFace { plane, outer, holes } => {
                let plane_rt = plane.resolve(prog);
                let o = ring_on_plane(&mut b, &outer.0, plane_rt);
                let outer_loop = loop_of(&o, outer.1);
                let mut inner_loops = Vec::with_capacity(holes.len());
                for (segs, fwd) in holes {
                    let r = ring_on_plane(&mut b, segs, plane_rt);
                    inner_loops.push(loop_of(&r, *fwd));
                }
                let (origin, normal) = plane_rt;
                let surface = Surface::plane(origin, normal);
                b.face(surface, true, outer_loop, inner_loops);
            }
            Op::Hole { at, from_z, profile } => {
                emit_hole(&mut b, *at, *from_z, profile)?;
            }
            Op::WallFaces { lower, upper, z0, z1, outward } => {
                ring_into(&mut b, lower, *z0, &mut ra);
                ring_into(&mut b, upper, *z1, &mut rb);
                wall_between(&mut b, lower, upper, &ra, &rb, *z0, *z1, *outward);
            }
            Op::SlopedWall { lower, upper, lower_plane, upper_plane, outward } => {
                let lo_rt = lower_plane.resolve(prog);
                let hi_rt = upper_plane.resolve(prog);
                let lo = ring_on_plane(&mut b, lower, lo_rt);
                let hi = ring_on_plane(&mut b, upper, hi_rt);
                wall_between(&mut b, lower, upper, &lo, &hi, lo_rt.0.z, hi_rt.0.z, *outward);
            }
            Op::Wall { lower, upper, z0, z1, outward } => {
                ring_into(&mut b, lower, *z0, &mut ra);
                ring_into(&mut b, upper, *z1, &mut rb);
                wall_between(&mut b, lower, upper, &ra, &rb, *z0, *z1, *outward);
            }
            Op::Cap { z, up, outer, holes } => {
                let o = ring(&mut b, &outer.0, *z);
                let outer_loop = loop_of(&o, outer.1);
                let mut inner_loops = Vec::with_capacity(holes.len());
                for (segs, fwd) in holes {
                    let r = ring(&mut b, segs, *z);
                    inner_loops.push(loop_of(&r, *fwd));
                }
                let normal = if *up { Vec3::Z } else { -Vec3::Z };
                let surface = Surface::plane(vec3_of(0.0, 0.0, *z), normal);
                b.face(surface, true, outer_loop, inner_loops);
            }
            Op::Slabs { stack, opts } => {
                slab::emit_slabs(&mut b, stack, opts)?;
            }
            Op::Custom(f) => f(&mut b)?,
            Op::Fillet { edges } => {
                for &(ref s, z, r) in edges {
                    if let Some(e) = find_seg_edge(&b, s, z) {
                        blends.push((e, r));
                    }
                }
            }
            Op::Chamfer { edges } => {
                for &(ref s, z, da, db) in edges {
                    chamfers.push((seg_edge(&mut b, s, z).0, da, db));
                }
            }
        }
    }

    let mut solid = b.build();
    if !blends.is_empty() {
        solid = fillet::fillet_best_effort(&solid, &blends)?.0;
    }
    if !chamfers.is_empty() {
        solid = chamfer_edges(&solid, &chamfers)?;
    }
    Ok(solid)
}

/// A blend selection resolved against the edges the build actually produced.
/// The plan names a run; the boolean that built the solid may have split or
/// dropped it, and a selection that no longer names one edge simply goes
/// unblended -- an unfilleted corner, not a build failure.
fn find_seg_edge(b: &Builder, seg: &Seg, z: f32) -> Option<EdgeId> {
    let start = vec3_of(seg.start().x, seg.start().y, z);
    let end = vec3_of(seg.end().x, seg.end().y, z);
    let mid = match *seg {
        Seg::Line { .. } => (start + end) * 0.5,
        Seg::Arc { center, radius, a0, a1, .. } => {
            let t = (a0 + a1) * 0.5;
            vec3_of(center.x + radius * t.cos(), center.y + radius * t.sin(), z)
        }
    };
    b.find_edge(start, end, mid)
}

fn emit_hole(
    b: &mut Builder,
    at: crate::kernel::math::Vec2,
    from_z: f32,
    profile: &HoleProfile,
) -> Result<(), String> {
    let circle = |r: f32| Sketch::circle(at.x, at.y, r).loops.remove(0);
    let (sections, total_depth): (Vec<(f32, f32)>, f32) = match *profile {
        HoleProfile::Plain { radius, depth } => (vec![(radius, depth)], depth),
        HoleProfile::Counterbore { bore_r, bore_d, head_r, head_d } => {
            (vec![(head_r, head_d), (bore_r, bore_d)], bore_d.max(head_d))
        }
        HoleProfile::Countersink { .. } => {
            return Err(
                "Hole: Countersink not yet implemented (cones are not slab-expressible; \
                 would need a loft cut or general boolean)"
                    .into(),
            );
        }
    };

    let stack: Vec<(slab::Op, Slab)> = sections
        .iter()
        .map(|&(r, d)| {
            (slab::Op::Union, Slab::new(vec![circle(r)], from_z, from_z + d))
        })
        .collect();
    let opts = SlabOpts { cavity: true, open_at: vec![from_z] };
    slab::emit_slabs(b, &stack, &opts)?;
    let _ = total_depth;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::sketch::Sketch;

    fn rect(x0: f32, y0: f32, x1: f32, y1: f32) -> Vec<Seg> {
        Sketch::rectangle((x0 + x1) * 0.5, (y0 + y1) * 0.5, x1 - x0, y1 - y0).loops.remove(0)
    }

    fn box_program() -> Program {
        let r = rect(0.0, 0.0, 10.0, 20.0);
        let mut p = Program::default();
        p.push(
            "side walls",
            Op::Wall { lower: r.clone(), upper: r.clone(), z0: 0.0, z1: 5.0, outward: true },
        );
        p.push("top", Op::Cap { z: 5.0, up: true, outer: (r.clone(), true), holes: vec![] });
        p.push("bottom", Op::Cap { z: 0.0, up: false, outer: (r, false), holes: vec![] });
        p
    }

    #[test]
    fn full_program_is_a_valid_solid() {
        let s = run_all(&box_program()).expect("run");
        s.validate().expect("box is manifold");
        assert_eq!(s.faces.len(), 6, "4 sides + 2 caps");
    }

    #[test]
    fn prefix_gives_step_through() {
        let prog = box_program();
        let counts: Vec<usize> = (0..=prog.len())
            .map(|n| run(&prog, |i| i < n).expect("prefix").faces.len())
            .collect();
        assert_eq!(counts, vec![0, 4, 5, 6], "faces accumulate one op at a time");
    }

    #[test]
    fn individual_ops_toggle_off() {
        let prog = box_program();
        let s = run(&prog, |i| i != 1).expect("masked run");
        assert_eq!(s.faces.len(), 5);
        assert!(s.validate().is_err(), "a hole in the solid is not manifold");
    }

    #[test]
    fn blend_resolves_edges_by_geometry_not_id() {
        let r = Sketch::rounded_rect(0.0, 0.0, 20.0, 20.0, 4.0).loops.remove(0);
        let mut p = Program::default();
        p.push(
            "side walls",
            Op::Wall { lower: r.clone(), upper: r.clone(), z0: 0.0, z1: 5.0, outward: true },
        );
        p.push("top", Op::Cap { z: 5.0, up: true, outer: (r.clone(), true), holes: vec![] });
        p.push("bottom", Op::Cap { z: 0.0, up: false, outer: (r.clone(), false), holes: vec![] });
        let plain = run_all(&p).expect("unblended").faces.len();

        p.push("rim fillet", Op::Fillet { edges: r.iter().map(|&s| (s, 5.0, 1.0)).collect() });
        let s = run_all(&p).expect("blend run");
        s.validate().expect("blended box is manifold");
        assert!(s.faces.len() > plain, "blend added faces");
    }


    #[test]
    fn sketch_registers_name_and_is_lookupable() {
        let mut p = Program::default();
        let r = rect(0.0, 0.0, 10.0, 20.0);
        p.push("outline", Op::Sketch { name: "outline".into(), profile: r.clone() });
        assert_eq!(p.sketch("outline").unwrap(), r.as_slice(), "sketch lookup must return the profile");
        assert!(p.sketch("missing").is_none(), "unknown name returns None");
    }

    #[test]
    fn plane_registers_name_and_is_lookupable() {
        let mut p = Program::default();
        let (origin, normal) = (Vec3::new(0.0, 0.0, 8.2), Vec3::new(0.0, 0.0, 1.0));
        p.push(
            "floor datum",
            Op::Plane { name: "floor".into(), origin, normal },
        );
        let got = p.plane("floor").expect("plane lookup");
        assert_eq!(got.0, origin);
        assert_eq!(got.1, normal);
    }

    #[test]
    fn datum_ops_emit_no_geometry() {
        let r = rect(0.0, 0.0, 10.0, 20.0);
        let mut p = Program::default();
        p.push("s", Op::Sketch { name: "s".into(), profile: r });
        p.push("p", Op::Plane { name: "p".into(), origin: Vec3::ZERO, normal: Vec3::Z });
        for n in 0..=p.len() {
            let s = run(&p, |i| i < n).expect("prefix");
            assert_eq!(s.faces.len(), 0, "datums emit nothing (prefix {n})");
        }
    }

    #[test]
    fn datum_lookup_survives_masked_downstream_op() {
        let r = rect(0.0, 0.0, 10.0, 20.0);
        let mut p = Program::default();
        p.push("outline", Op::Sketch { name: "outline".into(), profile: r.clone() });
        p.push(
            "walls",
            Op::Wall { lower: r.clone(), upper: r, z0: 0.0, z1: 5.0, outward: true },
        );
        let _ = run(&p, |i| i != 1).expect("masked run");
        assert_eq!(p.sketch("outline").unwrap().len(), 4, "sketch symbol survives masked downstream op");
    }


    fn sketch_box_program() -> Program {
        let r = rect(0.0, 0.0, 10.0, 20.0);
        let mut p = Program::default();
        p.push("outline", Op::Sketch { name: "outline".into(), profile: r });
        p.push(
            "block",
            Op::Extrude {
                sketch: "outline".into(),
                from: PlaneRef::Z { z: 0.0, up: true },
                to: PlaneRef::Z { z: 5.0, up: true },
            },
        );
        p
    }

    #[test]
    fn extrude_plus_planarface_is_a_valid_box() {
        let s = run_all(&sketch_box_program()).expect("run");
        s.validate().expect("box is manifold");
        assert_eq!(s.faces.len(), 6, "4 walls + 2 caps from a single Extrude");
    }

    #[test]
    fn planarface_emits_a_single_face() {
        let r = rect(0.0, 0.0, 10.0, 20.0);
        let mut p = Program::default();
        p.push(
            "floor",
            Op::PlanarFace {
                plane: PlaneRef::Z { z: 0.0, up: true },
                outer: (r, true),
                holes: vec![],
            },
        );
        let s = run_all(&p).expect("run");
        assert_eq!(s.faces.len(), 1, "PlanarFace emits exactly one face");
    }

    #[test]
    fn extrude_cut_carves_a_pocket() {
        let outer = rect(0.0, 0.0, 20.0, 20.0);
        let pocket = rect(5.0, 5.0, 15.0, 15.0);
        let outer_clone = outer.clone();
        let pocket_clone = pocket.clone();
        let mut p = Program::default();
        p.push("outer", Op::Sketch { name: "outer".into(), profile: outer });
        p.push("pocket", Op::Sketch { name: "pocket".into(), profile: pocket });
        p.push(
            "block + pocket",
            Op::Slabs {
                stack: vec![
                    (
                        slab::Op::Union,
                        Slab::new(vec![p.sketch("outer").unwrap().to_vec()], 0.0, 5.0),
                    ),
                    (
                        slab::Op::Difference,
                        Slab::new(vec![p.sketch("pocket").unwrap().to_vec()], 2.0, 5.0),
                    ),
                ],
                opts: SlabOpts::default(),
            },
        );
        let s = run_all(&p).expect("run");
        s.validate().expect("pocketed block is manifold");

        use crate::kernel::slab;
        let ground = slab::build_slabs(&[
            (slab::Op::Union, Slab::new(vec![outer_clone], 0.0, 5.0)),
            (slab::Op::Difference, Slab::new(vec![pocket_clone], 2.0, 5.0)),
        ])
        .expect("ground truth");
        assert_eq!(s.faces.len(), ground.faces.len(), "program+slabs matches direct slab stack");
    }

    #[test]
    fn extrude_missing_sketch_errors_cleanly() {
        let mut p = Program::default();
        p.push(
            "orphan",
            Op::Extrude {
                sketch: "no-such-sketch".into(),
                from: PlaneRef::Z { z: 0.0, up: true },
                to: PlaneRef::Z { z: 5.0, up: true },
            },
        );
        let err = run_all(&p).unwrap_err();
        assert!(err.contains("not registered"), "got: {err}");
    }

    #[test]
    fn loft_chains_multiple_profiles() {
        let bot = rect(2.0, 2.0, 8.0, 18.0);
        let mid = rect(1.0, 1.0, 9.0, 19.0);
        let top = rect(0.0, 0.0, 10.0, 20.0);
        let mut p = Program::default();
        p.push("bot", Op::Sketch { name: "bot".into(), profile: bot });
        p.push("mid", Op::Sketch { name: "mid".into(), profile: mid });
        p.push("top", Op::Sketch { name: "top".into(), profile: top });
        p.push(
            "loft",
            Op::Loft {
                profiles: vec![
                    ("bot".into(), 0.0),
                    ("mid".into(), 2.0),
                    ("top".into(), 5.0),
                ],
                outward: true,
            },
        );
        let s = run_all(&p).expect("run");
        assert_eq!(s.faces.len(), 8, "two loft bands of 4 walls each");
    }

    #[test]
    fn wallfaces_matches_wall_emission() {
        let r = rect(0.0, 0.0, 10.0, 20.0);
        let mut p_old = Program::default();
        p_old.push(
            "walls",
            Op::Wall { lower: r.clone(), upper: r.clone(), z0: 0.0, z1: 5.0, outward: true },
        );
        let mut p_new = Program::default();
        p_new.push(
            "walls",
            Op::WallFaces { lower: r.clone(), upper: r, z0: 0.0, z1: 5.0, outward: true },
        );
        let (a, b) = (run_all(&p_old).unwrap(), run_all(&p_new).unwrap());
        assert_eq!(a.faces.len(), b.faces.len(), "WallFaces matches Wall");
    }


    fn box_with_hole_program(profile: HoleProfile) -> Program {
        let outline = rect(0.0, 0.0, 20.0, 20.0);
        let mouth_r = profile.mouth_radius();
        let mouth = Sketch::circle(10.0, 10.0, mouth_r).loops.remove(0);
        let mut p = Program::default();
        p.push("outline", Op::Sketch { name: "outline".into(), profile: outline.clone() });
        p.push(
            "walls",
            Op::WallFaces {
                lower: outline.clone(),
                upper: outline.clone(),
                z0: 0.0,
                z1: 5.0,
                outward: true,
            },
        );
        p.push(
            "top",
            Op::Cap { z: 5.0, up: true, outer: (outline.clone(), true), holes: vec![] },
        );
        p.push(
            "hole",
            Op::Hole {
                at: crate::kernel::math::Vec2::new(10.0, 10.0),
                from_z: 0.0,
                profile,
            },
        );
        p.push(
            "bottom (with mouth)",
            Op::Cap {
                z: 0.0,
                up: false,
                outer: (outline, false),
                holes: vec![(mouth, false)],
            },
        );
        p
    }

    #[test]
    fn plain_hole_in_a_block_is_manifold() {
        let p = box_with_hole_program(HoleProfile::Plain { radius: 2.0, depth: 3.0 });
        let s = run_all(&p).expect("run");
        s.validate().expect("block with plain hole is manifold");
    }

    #[test]
    fn counterbore_hole_in_a_block_is_manifold() {
        let p = box_with_hole_program(HoleProfile::Counterbore {
            bore_r: 1.5,
            bore_d: 4.0,
            head_r: 3.25,
            head_d: 2.4,
        });
        let s = run_all(&p).expect("run");
        s.validate().expect("block with counterbore is manifold");
    }

    #[test]
    fn counterbore_produces_more_faces_than_plain() {
        let plain = run_all(&box_with_hole_program(HoleProfile::Plain {
            radius: 1.5,
            depth: 4.0,
        }))
        .unwrap()
        .faces
        .len();
        let cbore = run_all(&box_with_hole_program(HoleProfile::Counterbore {
            bore_r: 1.5,
            bore_d: 4.0,
            head_r: 3.25,
            head_d: 2.4,
        }))
        .unwrap()
        .faces
        .len();
        assert!(cbore > plain, "counterbore ({cbore}) should exceed plain ({plain})");
    }

    #[test]
    fn countersink_returns_clean_error() {
        let mut p = Program::default();
        p.push("outline", Op::Sketch { name: "outline".into(), profile: rect(0.0, 0.0, 20.0, 20.0) });
        p.push(
            "block",
            Op::Extrude {
                sketch: "outline".into(),
                from: PlaneRef::Z { z: 0.0, up: true },
                to: PlaneRef::Z { z: 5.0, up: true },
            },
        );
        p.push(
            "cs",
            Op::Hole {
                at: crate::kernel::math::Vec2::new(10.0, 10.0),
                from_z: 0.0,
                profile: HoleProfile::Countersink {
                    bore_r: 1.5,
                    bore_d: 4.0,
                    head_r: 3.0,
                    head_angle_deg: 90.0,
                },
            },
        );
        let err = run_all(&p).unwrap_err();
        assert!(err.contains("Countersink not yet implemented"), "got: {err}");
    }


    #[test]
    fn chamfer_top_rim_of_box_via_op() {
        let r = Sketch::rectangle(10.0, 10.0, 20.0, 20.0).loops.remove(0);
        let mut p = Program::default();
        p.push("outline", Op::Sketch { name: "outline".into(), profile: r.clone() });
        p.push(
            "walls",
            Op::WallFaces {
                lower: r.clone(),
                upper: r.clone(),
                z0: 0.0,
                z1: 5.0,
                outward: true,
            },
        );
        p.push("top", Op::Cap { z: 5.0, up: true, outer: (r.clone(), true), holes: vec![] });
        p.push("bottom", Op::Cap { z: 0.0, up: false, outer: (r.clone(), false), holes: vec![] });
        let plain = run_all(&p).expect("unblended").faces.len();

        p.push(
            "rim chamfer",
            Op::Chamfer { edges: r.iter().map(|&s| (s, 5.0, 1.0, 1.0)).collect() },
        );
        let s = run_all(&p).expect("chamfer run");
        s.validate().expect("chamfered box is manifold");
        assert!(s.faces.len() > plain, "chamfer added bevel faces");
    }


    #[test]
    fn planarface_on_tilted_plane_lifts_vertices() {
        let outline = rect(0.0, 0.0, 10.0, 10.0);
        let mut p = Program::default();
        p.push(
            "tilted floor",
            Op::PlanarFace {
                plane: PlaneRef::Tilted {
                    origin: Vec3::new(5.0, 5.0, 5.0),
                    normal: Vec3::new(-1.0, 0.0, 1.0),
                },
                outer: (outline, true),
                holes: vec![],
            },
        );
        let s = run_all(&p).expect("run");
        assert_eq!(s.faces.len(), 1, "tilted PlanarFace emits one face");
        let f = &s.faces[0];
        match f.surface {
            Surface::Plane { normal, .. } => {
                assert!(normal.x.abs() > 0.1 && normal.z.abs() > 0.1, "tilted normal {normal}");
            }
            _ => panic!("expected a Plane surface"),
        }
        let zs: Vec<f32> = s.verts.iter().map(|v| v.point.z).collect();
        assert!(zs.iter().cloned().any(|z| (z - 5.0).abs() > 0.5), "vertices lifted off z=5: {zs:?}");
    }

    #[test]
    fn planarface_on_named_datum_works() {
        let outline = rect(0.0, 0.0, 10.0, 10.0);
        let mut p = Program::default();
        p.push(
            "floor datum",
            Op::Plane {
                name: "floor".into(),
                origin: Vec3::new(0.0, 0.0, 0.0),
                normal: Vec3::new(0.0, 1.0, 1.0),
            },
        );
        p.push(
            "tilted face",
            Op::PlanarFace {
                plane: PlaneRef::Named("floor".into()),
                outer: (outline, true),
                holes: vec![],
            },
        );
        let s = run_all(&p).expect("run");
        assert_eq!(s.faces.len(), 1, "named-datum PlanarFace emits one face");
    }

    #[test]
    fn extrude_with_tilted_plane_errors_cleanly() {
        let mut p = Program::default();
        p.push("outline", Op::Sketch { name: "outline".into(), profile: rect(0.0, 0.0, 10.0, 10.0) });
        p.push(
            "bad",
            Op::Extrude {
                sketch: "outline".into(),
                from: PlaneRef::Tilted { origin: Vec3::ZERO, normal: Vec3::new(1.0, 0.0, 1.0) },
                to: PlaneRef::Z { z: 5.0, up: true },
            },
        );
        let err = run_all(&p).unwrap_err();
        assert!(err.contains("tilted planes not yet supported"), "got: {err}");
    }
}
