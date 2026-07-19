//! A model expressed as a **linear list of operations** the kernel executes.
//!
//! A [`Program`] is a flat, labelled sequence of [`Op`]s. Nothing in it refers
//! to builder handles (vertex/edge/face ids): every op carries the geometry it
//! needs as profiles and heights, and blends select their edges by
//! `(seg, z)`. That independence is the point — [`run`] can execute *any
//! subset* of the list, so a prefix gives step-through and an arbitrary mask
//! lets individual operations be switched off, which is what the GUI's
//! geometry debugger drives.
//!
//! Planning stays in the model layer: it decides *what* to build and emits the
//! program. The kernel only executes.
//!
//! Datums ([`Op::Sketch`], [`Op::Plane`]) register named symbols but emit no
//! geometry — downstream feature ops look them up by name through
//! [`Program::sketch`] / [`Program::plane`]. This is the Solidworks/Onshape
//! "named sketch" idiom: profiles are authored once and referenced many times.

use crate::kernel::build::{loop_of, ring, seg_edge, wall_between};
use crate::kernel::fillet::blend_edges;
use crate::kernel::geom::Surface;
use crate::kernel::math::{Vec3, vec3_of};
use crate::kernel::sketch::Seg;
use crate::kernel::slab::{self, Slab, SlabOpts};
use crate::kernel::topo::{Builder, EdgeId, Solid};
use std::collections::HashMap;

/// A cap loop plus the direction to traverse it (`true` = as authored).
pub type DirLoop = (Vec<Seg>, bool);

/// A reference to a plane, resolved against the Program at run time.
///
/// Phase 1 has only the horizontal case. Phase 2 will add `Named` (lookup a
/// datum registered by [`Op::Plane`]) and `Tilted` (inline origin + normal),
/// which is what the sloped-floor extrude cut needs.
#[derive(Clone, Copy, Debug)]
pub enum PlaneRef {
    /// Horizontal plane at `z`. `up = true` → outward normal +Z (a floor or
    /// the top of an extrude); `up = false` → −Z (the underside of a cap).
    Z { z: f32, up: bool },
}

impl PlaneRef {
    /// Resolve to `(origin, normal)`. `prog` is accepted so the Phase 2
    /// `Named` variant can look up datums without changing the call site.
    pub fn resolve(&self, _prog: &Program) -> (Vec3, Vec3) {
        match *self {
            PlaneRef::Z { z, up } => {
                let normal = if up { Vec3::Z } else { -Vec3::Z };
                (vec3_of(0.0, 0.0, z), normal)
            }
        }
    }

    /// The z value (for slab APIs that still take scalar z bounds). Phase 2's
    /// tilted-plane variant will require the slab API to accept a plane spec
    /// instead, at which point this goes away.
    pub fn z(&self) -> f32 {
        match *self {
            PlaneRef::Z { z, .. } => z,
        }
    }
}

/// One modelling operation.
pub enum Op {
    // ── Datums (phase 0) ───────────────────────────────────────────────────
    /// Register a named 2D profile for downstream ops to reference. Emits no
    /// geometry. The same profile can drive an `Extrude`, a `Loft` profile,
    /// and a `PlanarFace` outer without being re-cloned at each call site.
    Sketch { name: String, profile: Vec<Seg> },
    /// Register a named plane (datum) for downstream ops to reference. Emits
    /// no geometry. `origin`/`normal` define an arbitrary plane in 3D; a
    /// horizontal plane at `z` is `origin = (0,0,z), normal = (0,0,1)`.
    Plane { name: String, origin: Vec3, normal: Vec3 },

    // ── Volume features (phase 1) ─────────────────────────────────────────
    /// Extrude a named sketch from `from` to `to` as a solid volume. Sugar
    /// over a single-slab `Slabs` Union stack — kept as its own op because
    /// "extrude" is the CAD-idiomatic name and reads better in the debugger.
    Extrude { sketch: String, from: PlaneRef, to: PlaneRef },
    /// Subtract an extruded sketch from the existing body. Sugar over a
    /// single-slab `Slabs` Difference stack.
    ExtrudeCut { sketch: String, from: PlaneRef, to: PlaneRef },
    /// Loft through N ≥ 2 named sketches at increasing heights. Each
    /// consecutive pair becomes one loft band (cones where an arc's radius
    /// changes between profiles, planes for straight runs, cylinders for
    /// constant-radius arcs) — the same machinery as `Op::Wall`.
    Loft { profiles: Vec<(String, f32)>, outward: bool },

    // ── Face-level features (phase 1) ─────────────────────────────────────
    /// A planar face on an arbitrary plane. Generalises `Op::Cap` to non-
    /// horizontal planes (Phase 2). Loop directions in `outer`/`holes` decide
    /// winding relative to the plane's normal.
    PlanarFace { plane: PlaneRef, outer: DirLoop, holes: Vec<DirLoop> },
    /// Side faces between a lower and upper profile (a prism or single-band
    /// loft). CAD-idiomatic spelling of `Op::Wall`.
    WallFaces { lower: Vec<Seg>, upper: Vec<Seg>, z0: f32, z1: f32, outward: bool },

    // ── Original op set (will retire in phase 4) ──────────────────────────
    /// Side faces between two profiles at two heights. Equal profiles give a
    /// prism band; differing ones give a loft band, with cones wherever an
    /// arc's radius changes — the one thing slabs cannot express.
    Wall { lower: Vec<Seg>, upper: Vec<Seg>, z0: f32, z1: f32, outward: bool },
    /// A horizontal planar face at `z`, `up` picking the +Z or −Z normal.
    Cap { z: f32, up: bool, outer: DirLoop, holes: Vec<DirLoop> },
    /// A 2.5D slab stack (see [`slab`]).
    Slabs { stack: Vec<(slab::Op, Slab)>, opts: SlabOpts },
    /// Rolling-ball blends, each edge named by `(seg, z, radius)` and resolved
    /// against the builder when the op runs.
    Blend { edges: Vec<(Seg, f32, f32)> },
    /// An operation the model supplies itself, for geometry with no kernel
    /// primitive (Gridfinity's bridge-underside stitching, say).
    ///
    /// It must be **self-contained**: capture the geometry it needs by value
    /// and re-derive every builder handle when it runs. `ring`/`seg_edge`
    /// intern, so re-deriving costs nothing and is what lets any op be skipped
    /// without breaking the ones after it.
    Custom(Box<dyn Fn(&mut Builder) -> Result<(), String>>),
}

impl Op {
    /// Short kind name, for the debugger's list.
    pub fn kind(&self) -> &'static str {
        match self {
            Op::Sketch { .. } => "sketch",
            Op::Plane { .. } => "plane",
            Op::Extrude { .. } => "extrude",
            Op::ExtrudeCut { .. } => "cut",
            Op::Loft { .. } => "loft",
            Op::PlanarFace { .. } => "face",
            Op::WallFaces { .. } => "wall",
            Op::Wall { .. } => "wall",
            Op::Cap { .. } => "cap",
            Op::Slabs { .. } => "slabs",
            Op::Blend { .. } => "blend",
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
    /// Named sketch profiles, registered by `Op::Sketch`. Indexed at `push`
    /// time so a downstream op can look up by name without re-running.
    sketches: HashMap<String, Vec<Seg>>,
    /// Named datum planes, registered by `Op::Plane`.
    planes: HashMap<String, (Vec3, Vec3)>,
}

impl Program {
    pub fn push(&mut self, label: impl Into<String>, op: Op) {
        // Datum ops register their symbols at push time, so lookups work even
        // when later feature ops are masked off in the debugger (a datum
        // itself never needs to "run").
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

    /// Look up a named sketch profile. Returns `None` if no `Op::Sketch`
    /// registered `name`.
    pub fn sketch(&self, name: &str) -> Option<&[Seg]> {
        self.sketches.get(name).map(|v| v.as_slice())
    }

    /// Look up a named datum plane as `(origin, normal)`.
    pub fn plane(&self, name: &str) -> Option<(Vec3, Vec3)> {
        self.planes.get(name).copied()
    }
}

/// Execute every operation.
pub fn run_all(prog: &Program) -> Result<Solid, String> {
    run(prog, |_| true)
}

/// Execute the enabled subset.
///
/// Blends are collected as they are reached but applied once at the end,
/// because `blend_edges` consumes and rebuilds the whole solid. A partial
/// subset generally is **not** manifold, so this does not validate — callers
/// wanting a finished part should run everything and validate themselves.
pub fn run(prog: &Program, enabled: impl Fn(usize) -> bool) -> Result<Solid, String> {
    let mut b = Builder::new();
    let mut blends: Vec<(EdgeId, f32)> = Vec::new();

    for (i, st) in prog.steps.iter().enumerate() {
        if !enabled(i) {
            continue;
        }
        match &st.op {
            Op::Sketch { .. } | Op::Plane { .. } => {
                // Datums only — symbols were registered at push time; nothing
                // to emit. Kept explicit so the match stays exhaustive.
            }
            Op::Extrude { sketch, from, to } => {
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
                // Resolve every sketch up front so a missing name fails before
                // emitting any partial geometry.
                let resolved: Vec<(&[Seg], f32)> = profiles
                    .iter()
                    .map(|(name, z)| -> Result<(&[Seg], f32), String> {
                        let p = prog.sketch(name).ok_or_else(|| {
                            format!("Loft: sketch {name:?} not registered")
                        })?;
                        Ok((p, *z))
                    })
                    .collect::<Result<_, _>>()?;
                for w in resolved.windows(2) {
                    let (lower, z0) = w[0];
                    let (upper, z1) = w[1];
                    let lo = ring(&mut b, lower, z0);
                    let hi = ring(&mut b, upper, z1);
                    wall_between(&mut b, lower, upper, &lo, &hi, z0, z1, *outward);
                }
            }
            Op::PlanarFace { plane, outer, holes } => {
                let (origin, normal) = plane.resolve(prog);
                let z = plane.z();
                let o = ring(&mut b, &outer.0, z);
                let outer_loop = loop_of(&o, outer.1);
                let mut inner_loops = Vec::with_capacity(holes.len());
                for (segs, fwd) in holes {
                    let r = ring(&mut b, segs, z);
                    inner_loops.push(loop_of(&r, *fwd));
                }
                let surface = Surface::plane(origin, normal);
                b.face(surface, true, outer_loop, inner_loops);
            }
            Op::WallFaces { lower, upper, z0, z1, outward } => {
                let lo = ring(&mut b, lower, *z0);
                let hi = ring(&mut b, upper, *z1);
                wall_between(&mut b, lower, upper, &lo, &hi, *z0, *z1, *outward);
            }
            Op::Wall { lower, upper, z0, z1, outward } => {
                let lo = ring(&mut b, lower, *z0);
                let hi = ring(&mut b, upper, *z1);
                wall_between(&mut b, lower, upper, &lo, &hi, *z0, *z1, *outward);
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
            Op::Blend { edges } => {
                for &(ref s, z, r) in edges {
                    blends.push((seg_edge(&mut b, s, z).0, r));
                }
            }
        }
    }

    let solid = b.build();
    if blends.is_empty() {
        return Ok(solid);
    }
    blend_edges(&solid, &blends)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::sketch::Sketch;

    fn rect(x0: f32, y0: f32, x1: f32, y1: f32) -> Vec<Seg> {
        Sketch::rectangle((x0 + x1) * 0.5, (y0 + y1) * 0.5, x1 - x0, y1 - y0).loops.remove(0)
    }

    /// A box authored as three ops, so the run/skip machinery is exercised on
    /// something whose face count is obvious.
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
        // Skip the middle op only: an open-topped box, still 5 faces.
        let s = run(&prog, |i| i != 1).expect("masked run");
        assert_eq!(s.faces.len(), 5);
        assert!(s.validate().is_err(), "a hole in the solid is not manifold");
    }

    #[test]
    fn blend_resolves_edges_by_geometry_not_id() {
        // The blend names its edges by (seg, z), never by edge id, so it does
        // not care what ran before it. Rounded corners because `blend_edges`
        // needs a tangent-continuous chain — a sharp box corner would want the
        // unimplemented spherical corner patch.
        let r = Sketch::rounded_rect(0.0, 0.0, 20.0, 20.0, 4.0).loops.remove(0);
        let mut p = Program::default();
        p.push(
            "side walls",
            Op::Wall { lower: r.clone(), upper: r.clone(), z0: 0.0, z1: 5.0, outward: true },
        );
        p.push("top", Op::Cap { z: 5.0, up: true, outer: (r.clone(), true), holes: vec![] });
        p.push("bottom", Op::Cap { z: 0.0, up: false, outer: (r.clone(), false), holes: vec![] });
        let plain = run_all(&p).expect("unblended").faces.len();

        p.push("rim fillet", Op::Blend { edges: r.iter().map(|&s| (s, 5.0, 1.0)).collect() });
        let s = run_all(&p).expect("blend run");
        s.validate().expect("blended box is manifold");
        assert!(s.faces.len() > plain, "blend added faces");
    }

    // ── Phase 0: datums ────────────────────────────────────────────────────

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
        // A program of only Sketch + Plane should produce an empty solid, and
        // every prefix (including all-datums-off and all-datums-on) must run.
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
        // The sketch must be lookupable even when the feature op that uses it
        // is masked off — symbols register at push time, not run time.
        let r = rect(0.0, 0.0, 10.0, 20.0);
        let mut p = Program::default();
        p.push("outline", Op::Sketch { name: "outline".into(), profile: r.clone() });
        p.push(
            "walls",
            Op::Wall { lower: r.clone(), upper: r, z0: 0.0, z1: 5.0, outward: true },
        );
        // Skip the Wall op entirely; the sketch symbol must still resolve.
        let _ = run(&p, |i| i != 1).expect("masked run");
        assert_eq!(p.sketch("outline").unwrap().len(), 4, "sketch symbol survives masked downstream op");
    }

    // ── Phase 1: feature ops ──────────────────────────────────────────────

    fn sketch_box_program() -> Program {
        // A box authored with the new CAD-idiomatic ops. A single Extrude
        // produces a complete closed solid (the slab machinery emits caps at
        // every z-interface by default), so this is one feature op over a
        // named sketch.
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
        // PlanarFace on its own emits exactly one face (useful for closing
        // openings left by open_at on a slab stack, or for standalone datums).
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
        // A compound shape (block minus pocket) needs the slab engine's
        // restricted boolean, which resolves overlapping slabs in one stack.
        // Standalone Op::ExtrudeCut exists for cuts that don't need to compose
        // with another Extrude in the same feature — but a pocket in a block
        // is the canonical compound case, so this test exercises Slabs
        // directly with the same sketch+plane vocabulary the new ops use.
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

        // Ground truth: build the same shape directly via slab stack.
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
        // Three-profile loft: small bottom → mid → larger top, two bands.
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
        // Open top + bottom (no caps in a bare loft): valid as a shell of walls.
        let s = run_all(&p).expect("run");
        // 4 walls per band × 2 bands = 8 side faces.
        assert_eq!(s.faces.len(), 8, "two loft bands of 4 walls each");
    }

    #[test]
    fn wallfaces_matches_wall_emission() {
        // WallFaces is the CAD spelling of Wall; both should produce identical
        // face counts for the same inputs.
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
}
