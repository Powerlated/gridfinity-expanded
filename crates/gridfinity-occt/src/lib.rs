//! Ownership-safe Rust boundary for Open CASCADE.
//!
//! Enable `occt` and set `OCCT_ROOT` to an installed OCCT 8.0.1 prefix. No
//! OCCT C++ type crosses this ABI: that keeps exceptions, RTTI, and allocator
//! ownership on the C++ side and permits the same bridge to link into an
//! Emscripten final module.

#[derive(Debug, Clone, PartialEq)]
pub struct Error(pub String);
impl std::fmt::Display for Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(&self.0)
    }
}
impl std::error::Error for Error {}

/// One segment of a profile loop: a line from `a` to `b`, or an arc from `a` to
/// `b` about `center`, swept from `a0` to `a1` and counter-clockwise exactly
/// when `a1 >= a0`. This is the kernel-neutral half of `gridfinity_brep::Seg`,
/// restated here so the model can hand a profile to either backend.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Seg {
    Line {
        a: [f64; 2],
        b: [f64; 2],
    },
    Arc {
        a: [f64; 2],
        b: [f64; 2],
        center: [f64; 2],
        radius: f64,
        a0: f64,
        a1: f64,
    },
}

impl Seg {
    /// The ten doubles this segment crosses the ABI as.
    fn encode(&self) -> [f64; 10] {
        match *self {
            Seg::Line { a, b } => [0.0, a[0], a[1], b[0], b[1], 0.0, 0.0, 0.0, 0.0, 0.0],
            Seg::Arc {
                a,
                b,
                center,
                radius,
                a0,
                a1,
            } => [
                1.0, a[0], a[1], b[0], b[1], center[0], center[1], radius, a0, a1,
            ],
        }
    }
}

/// A profile: the outer boundary first, then the loops holed in it.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct Profile {
    pub loops: Vec<Vec<Seg>>,
}

impl Profile {
    /// The profile of one closed loop.
    pub fn of(outer: Vec<Seg>) -> Profile {
        Profile { loops: vec![outer] }
    }

    /// The flat segment array and per-loop counts this profile crosses the ABI
    /// as, in the order the loops are held.
    fn encode(&self) -> (Vec<f64>, Vec<usize>) {
        let mut segments = Vec::new();
        let mut lengths = Vec::new();
        for one in &self.loops {
            lengths.push(one.len());
            for seg in one {
                segments.extend_from_slice(&seg.encode());
            }
        }
        (segments, lengths)
    }
}

/// Which shape a boolean leaves behind.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Boolean {
    Cut,
    Fuse,
    Common,
}

impl Boolean {
    fn code(self) -> i32 {
        match self {
            Boolean::Cut => 0,
            Boolean::Fuse => 1,
            Boolean::Common => 2,
        }
    }
}

/// An edge to round, named by the midpoint it runs through rather than by an
/// index: the operation before a blend renumbers every edge of the shape.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FilletEdge {
    pub midpoint: [f64; 3],
    pub radius: f64,
}

/// How many doubles one record of the topology export carries. The bridge
/// header fixes both, and reading one short would silently take the next
/// record's first field for this one's last.
pub const EDGE_STRIDE: usize = 16;
pub const FACE_STRIDE: usize = 14;

/// How much of each thing a shape's B-rep holds.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counts {
    pub vertices: usize,
    pub edges: usize,
    pub faces: usize,
    pub loops: usize,
    pub fins: usize,
    pub chart_points: usize,
}

/// The axis-aligned box a shape occupies, as `[min, max]` corners.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Bounds {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

#[derive(Debug, Clone, Default)]
pub struct Mesh {
    pub positions: Vec<[f64; 3]>,
    pub normals: Vec<[f64; 3]>,
    pub indices: Vec<u32>,
}

#[cfg(feature = "occt")]
mod enabled {
    use super::{
        Boolean, Bounds, Counts, EDGE_STRIDE, Error, FACE_STRIDE, FilletEdge, Mesh, Profile,
    };
    use std::{
        ffi::{CStr, c_char},
        ptr::NonNull,
    };
    #[repr(C)]
    struct RawShape {
        _private: [u8; 0],
    }
    unsafe extern "C" {
        fn gf_occt_last_error() -> *const c_char;
        fn gf_occt_make_box(dx: f64, dy: f64, dz: f64) -> *mut RawShape;
        fn gf_occt_make_rounded_box(dx: f64, dy: f64, dz: f64, r: f64) -> *mut RawShape;
        fn gf_occt_make_cone(r0: f64, r1: f64, h: f64) -> *mut RawShape;
        fn gf_occt_shape_free(p: *mut RawShape);
        fn gf_occt_shape_is_valid(p: *const RawShape) -> i32;
        fn gf_occt_prism_from_loops(
            segments: *const f64,
            loops: *const usize,
            loop_count: usize,
            z: f64,
            dz: f64,
        ) -> *mut RawShape;
        fn gf_occt_loft(
            segments: *const f64,
            loops: *const usize,
            loops_per_section: *const usize,
            zs: *const f64,
            section_count: usize,
        ) -> *mut RawShape;
        fn gf_occt_boolean(a: *const RawShape, b: *const RawShape, op: i32) -> *mut RawShape;
        fn gf_occt_fillet_edges(
            p: *const RawShape,
            edges: *const f64,
            count: usize,
            tolerance: f64,
        ) -> *mut RawShape;
        fn gf_occt_volume(p: *const RawShape, volume: *mut f64) -> i32;
        fn gf_occt_bounds(p: *const RawShape, bounds: *mut f64) -> i32;
        fn gf_occt_shell_count(p: *const RawShape, shells: *mut usize) -> i32;
        fn gf_occt_topology_counts(
            p: *const RawShape,
            vertices: *mut usize,
            edges: *mut usize,
            faces: *mut usize,
            loops: *mut usize,
            fins: *mut usize,
            chart_points: *mut usize,
        ) -> i32;
        #[allow(clippy::too_many_arguments)]
        fn gf_occt_topology_copy(
            p: *const RawShape,
            vertices: *mut f64,
            edges: *mut f64,
            faces: *mut f64,
            loops_per_face: *mut usize,
            fins_per_loop: *mut usize,
            fins: *mut i64,
            charts: *mut f64,
            vertex_count: usize,
            edge_count: usize,
            face_count: usize,
            loop_count: usize,
            fin_count: usize,
            chart_point_count: usize,
        ) -> i32;
        fn gf_occt_mesh_counts(p: *const RawShape, d: f64, nv: *mut usize, ni: *mut usize) -> i32;
        fn gf_occt_mesh_copy(
            p: *const RawShape,
            d: f64,
            pos: *mut f64,
            norm: *mut f64,
            idx: *mut u32,
            nv: usize,
            ni: usize,
        ) -> i32;
    }
    fn error() -> Error {
        unsafe {
            let p = gf_occt_last_error();
            Error(if p.is_null() {
                "OCCT operation failed".into()
            } else {
                CStr::from_ptr(p).to_string_lossy().into_owned()
            })
        }
    }
    pub struct Shape(NonNull<RawShape>);
    impl Shape {
        fn from_raw(p: *mut RawShape) -> Result<Self, Error> {
            NonNull::new(p).map(Self).ok_or_else(error)
        }
        pub fn box_solid(dx: f64, dy: f64, dz: f64) -> Result<Self, Error> {
            Self::from_raw(unsafe { gf_occt_make_box(dx, dy, dz) })
        }
        pub fn rounded_box(dx: f64, dy: f64, dz: f64, r: f64) -> Result<Self, Error> {
            Self::from_raw(unsafe { gf_occt_make_rounded_box(dx, dy, dz, r) })
        }

        /// The truncated cone about +Z with radius `r0` at its base and `r1` at
        /// height `h`. The two radii must differ; equal ones are a cylinder.
        pub fn cone(r0: f64, r1: f64, h: f64) -> Result<Self, Error> {
            Self::from_raw(unsafe { gf_occt_make_cone(r0, r1, h) })
        }
        /// The solid swept from `profile`, laid in the plane `z` and extruded
        /// `dz` along it. Every loop must close, and the first bounds the rest.
        pub fn prism(profile: &Profile, z: f64, dz: f64) -> Result<Self, Error> {
            let (segments, loops) = profile.encode();
            Self::from_raw(unsafe {
                gf_occt_prism_from_loops(
                    segments.as_ptr(),
                    loops.as_ptr(),
                    loops.len(),
                    z,
                    dz,
                )
            })
        }

        /// The solid lofted through `sections`, each a profile and the height
        /// it is laid at, in the order given. Every section must hold the same
        /// number of loops; a Gridfinity peg is four of them.
        pub fn loft(sections: &[(&Profile, f64)]) -> Result<Self, Error> {
            let mut segments = Vec::new();
            let mut loops = Vec::new();
            let mut per_section = Vec::new();
            let mut zs = Vec::new();
            for (profile, z) in sections {
                let (section_segments, section_loops) = profile.encode();
                segments.extend_from_slice(&section_segments);
                per_section.push(section_loops.len());
                loops.extend_from_slice(&section_loops);
                zs.push(*z);
            }
            Self::from_raw(unsafe {
                gf_occt_loft(
                    segments.as_ptr(),
                    loops.as_ptr(),
                    per_section.as_ptr(),
                    zs.as_ptr(),
                    sections.len(),
                )
            })
        }

        /// This shape combined with `other`: `self - other`, `self + other` or
        /// the part they share.
        pub fn boolean(&self, other: &Shape, op: Boolean) -> Result<Self, Error> {
            Self::from_raw(unsafe {
                gf_occt_boolean(self.0.as_ptr(), other.0.as_ptr(), op.code())
            })
        }

        /// This shape with every edge running through one of `edges`' midpoints
        /// rounded to that edge's radius. An edge the shape does not have, and a
        /// blend the kernel refuses, are both errors -- a fillet that does not
        /// land is never a corner quietly left sharp.
        pub fn fillet(&self, edges: &[FilletEdge], tolerance: f64) -> Result<Self, Error> {
            let flat: Vec<f64> = edges
                .iter()
                .flat_map(|e| {
                    [
                        e.midpoint[0],
                        e.midpoint[1],
                        e.midpoint[2],
                        e.radius,
                        0.0,
                        0.0,
                    ]
                })
                .collect();
            Self::from_raw(unsafe {
                gf_occt_fillet_edges(self.0.as_ptr(), flat.as_ptr(), edges.len(), tolerance)
            })
        }

        /// The volume this shape encloses, in the millimetres it was built in.
        pub fn volume(&self) -> Result<f64, Error> {
            let mut volume = 0.0;
            if unsafe { gf_occt_volume(self.0.as_ptr(), &mut volume) } == 0 {
                return Err(error());
            }
            Ok(volume)
        }

        /// The axis-aligned box this shape occupies.
        pub fn bounds(&self) -> Result<Bounds, Error> {
            let mut b = [0.0; 6];
            if unsafe { gf_occt_bounds(self.0.as_ptr(), b.as_mut_ptr()) } == 0 {
                return Err(error());
            }
            Ok(Bounds {
                min: [b[0], b[1], b[2]],
                max: [b[3], b[4], b[5]],
            })
        }

        /// How many shells bound this shape. A printable body has exactly one
        /// per island of material: a shell too many is a detached lump.
        pub fn shell_count(&self) -> Result<usize, Error> {
            let mut shells = 0;
            if unsafe { gf_occt_shell_count(self.0.as_ptr(), &mut shells) } == 0 {
                return Err(error());
            }
            Ok(shells)
        }

        /// How many vertices, edges, faces, loops and fins this shape's B-rep
        /// has, which is what a caller allocates its buffers from.
        pub fn topology_counts(&self) -> Result<Counts, Error> {
            let mut c = Counts::default();
            if unsafe {
                gf_occt_topology_counts(
                    self.0.as_ptr(),
                    &mut c.vertices,
                    &mut c.edges,
                    &mut c.faces,
                    &mut c.loops,
                    &mut c.fins,
                    &mut c.chart_points,
                )
            } == 0
            {
                return Err(error());
            }
            Ok(c)
        }

        /// This shape's B-rep copied into buffers the caller sized from
        /// `topology_counts`. See the bridge header for what each record holds;
        /// a shape carrying geometry the analytic forms cannot state is refused
        /// rather than approximated.
        #[allow(clippy::too_many_arguments)]
        pub fn topology_copy(
            &self,
            vertices: &mut [f64],
            edges: &mut [f64],
            faces: &mut [f64],
            loops_per_face: &mut [usize],
            fins_per_loop: &mut [usize],
            fins: &mut [i64],
            charts: &mut [f64],
        ) -> Result<(), Error> {
            if unsafe {
                gf_occt_topology_copy(
                    self.0.as_ptr(),
                    vertices.as_mut_ptr(),
                    edges.as_mut_ptr(),
                    faces.as_mut_ptr(),
                    loops_per_face.as_mut_ptr(),
                    fins_per_loop.as_mut_ptr(),
                    fins.as_mut_ptr(),
                    charts.as_mut_ptr(),
                    vertices.len() / 3,
                    edges.len() / EDGE_STRIDE,
                    faces.len() / FACE_STRIDE,
                    fins_per_loop.len(),
                    fins.len() / 2,
                    charts.len() / 3,
                )
            } == 0
            {
                return Err(error());
            }
            Ok(())
        }

        pub fn is_valid(&self) -> Result<bool, Error> {
            let v = unsafe { gf_occt_shape_is_valid(self.0.as_ptr()) };
            if v < 0 { Err(error()) } else { Ok(v != 0) }
        }
        pub fn tessellate(&self, deflection: f64) -> Result<Mesh, Error> {
            let (mut nv, mut ni) = (0, 0);
            if unsafe { gf_occt_mesh_counts(self.0.as_ptr(), deflection, &mut nv, &mut ni) } == 0 {
                return Err(error());
            }
            let mut p = vec![[0.0; 3]; nv];
            let mut n = vec![[0.0; 3]; nv];
            let mut i = vec![0; ni];
            if unsafe {
                gf_occt_mesh_copy(
                    self.0.as_ptr(),
                    deflection,
                    p.as_mut_ptr().cast(),
                    n.as_mut_ptr().cast(),
                    i.as_mut_ptr(),
                    nv,
                    ni,
                )
            } == 0
            {
                return Err(error());
            }
            Ok(Mesh {
                positions: p,
                normals: n,
                indices: i,
            })
        }
    }
    impl Drop for Shape {
        fn drop(&mut self) {
            unsafe { gf_occt_shape_free(self.0.as_ptr()) }
        }
    }
}

#[cfg(feature = "occt")]
pub use enabled::Shape;

#[cfg(not(feature = "occt"))]
#[derive(Debug)]
pub struct Shape;
#[cfg(not(feature = "occt"))]
impl Shape {
    fn unavailable<T>() -> Result<T, Error> {
        Err(Error(
            "OCCT backend is disabled; enable gridfinity-occt/occt and set OCCT_ROOT".into(),
        ))
    }
    pub fn box_solid(_: f64, _: f64, _: f64) -> Result<Self, Error> {
        Self::unavailable()
    }
    pub fn rounded_box(_: f64, _: f64, _: f64, _: f64) -> Result<Self, Error> {
        Self::unavailable()
    }
    pub fn cone(_: f64, _: f64, _: f64) -> Result<Self, Error> {
        Self::unavailable()
    }
    pub fn prism(_: &Profile, _: f64, _: f64) -> Result<Self, Error> {
        Self::unavailable()
    }
    pub fn loft(_: &[(&Profile, f64)]) -> Result<Self, Error> {
        Self::unavailable()
    }
    pub fn boolean(&self, _: &Shape, _: Boolean) -> Result<Self, Error> {
        Self::unavailable()
    }
    pub fn fillet(&self, _: &[FilletEdge], _: f64) -> Result<Self, Error> {
        Self::unavailable()
    }
    pub fn volume(&self) -> Result<f64, Error> {
        Self::unavailable()
    }
    pub fn bounds(&self) -> Result<Bounds, Error> {
        Self::unavailable()
    }
    pub fn shell_count(&self) -> Result<usize, Error> {
        Self::unavailable()
    }
    pub fn topology_counts(&self) -> Result<Counts, Error> {
        Self::unavailable()
    }
    pub fn topology_copy(
        &self,
        _: &mut [f64],
        _: &mut [f64],
        _: &mut [f64],
        _: &mut [usize],
        _: &mut [usize],
        _: &mut [i64],
        _: &mut [f64],
    ) -> Result<(), Error> {
        Self::unavailable()
    }
    pub fn is_valid(&self) -> Result<bool, Error> {
        Self::unavailable()
    }
    pub fn tessellate(&self, _: f64) -> Result<Mesh, Error> {
        Self::unavailable()
    }
}

#[cfg(all(test, not(feature = "occt")))]
mod tests {
    #[test]
    fn disabled_backend_is_explicit() {
        assert!(
            super::Shape::box_solid(1.0, 2.0, 3.0)
                .unwrap_err()
                .to_string()
                .contains("disabled")
        );
    }
}

#[cfg(all(test, feature = "occt"))]
mod occt_tests {
    use super::{Boolean, FilletEdge, Profile, Seg, Shape};
    use std::f64::consts::PI;

    /// The counter-clockwise loop of the `w` by `h` rectangle at the origin
    /// with all four corners rounded to `r`: a Gridfinity outline in miniature,
    /// and the profile every prism below is swept from.
    fn rounded_rect(w: f64, h: f64, r: f64) -> Profile {
        let arc = |a: [f64; 2], b: [f64; 2], center: [f64; 2], a0: f64, a1: f64| Seg::Arc {
            a,
            b,
            center,
            radius: r,
            a0,
            a1,
        };
        Profile::of(vec![
            Seg::Line {
                a: [r, 0.0],
                b: [w - r, 0.0],
            },
            arc([w - r, 0.0], [w, r], [w - r, r], -PI / 2.0, 0.0),
            Seg::Line {
                a: [w, r],
                b: [w, h - r],
            },
            arc([w, h - r], [w - r, h], [w - r, h - r], 0.0, PI / 2.0),
            Seg::Line {
                a: [w - r, h],
                b: [r, h],
            },
            arc([r, h], [0.0, h - r], [r, h - r], PI / 2.0, PI),
            Seg::Line {
                a: [0.0, h - r],
                b: [0.0, r],
            },
            arc([0.0, r], [r, 0.0], [r, r], PI, 1.5 * PI),
        ])
    }

    /// The area of that rectangle: the four rounded corners take a square of
    /// side `r` each and give back a quarter disc.
    fn rounded_rect_area(w: f64, h: f64, r: f64) -> f64 {
        w * h - (4.0 - PI) * r * r
    }

    #[track_caller]
    fn assert_close(measured: f64, expected: f64, what: &str) {
        assert!(
            (measured - expected).abs() <= 1e-6 * expected.abs().max(1.0),
            "{what}: measured {measured}, expected {expected}"
        );
    }

    #[test]
    fn a_rounded_prism_encloses_the_area_it_sweeps() {
        let profile = rounded_rect(41.5, 41.5, 4.0);
        let shape = Shape::prism(&profile, 0.0, 7.0).expect("OCCT prism");
        assert!(shape.is_valid().expect("validity check"));
        assert_eq!(shape.shell_count().expect("shells"), 1);
        assert_close(
            shape.volume().expect("volume"),
            rounded_rect_area(41.5, 41.5, 4.0) * 7.0,
            "a prism's volume is its profile's area times its height",
        );
        let bounds = shape.bounds().expect("bounds");
        assert_close(bounds.min[2], 0.0, "a prism starts in the plane it is drawn in");
        assert_close(bounds.max[2], 7.0, "a prism reaches the height it is swept");
    }

    #[test]
    fn a_cavity_takes_exactly_its_own_volume() {
        let outer = Shape::prism(&rounded_rect(41.5, 41.5, 4.0), 0.0, 20.0).expect("outer");
        let cavity = Shape::prism(&inset(rounded_rect(35.5, 35.5, 2.0), 3.0), 3.0, 20.0)
            .expect("cavity");
        let hollow = outer.boolean(&cavity, Boolean::Cut).expect("cut");
        assert!(hollow.is_valid().expect("validity check"));
        assert_eq!(
            hollow.shell_count().expect("shells"),
            1,
            "a cavity opening through the top leaves one shell"
        );
        assert_close(
            hollow.volume().expect("volume"),
            rounded_rect_area(41.5, 41.5, 4.0) * 20.0 - rounded_rect_area(35.5, 35.5, 2.0) * 17.0,
            "a cut removes the part of the tool inside the body",
        );
    }

    #[test]
    fn a_fillet_takes_the_corner_it_names() {
        let shape = Shape::box_solid(20.0, 20.0, 10.0).expect("box");
        let rounded = shape
            .fillet(
                &[FilletEdge {
                    midpoint: [0.0, 0.0, 5.0],
                    radius: 3.0,
                }],
                1e-6,
            )
            .expect("fillet");
        assert!(rounded.is_valid().expect("validity check"));
        assert_close(
            rounded.volume().expect("volume"),
            20.0 * 20.0 * 10.0 - (1.0 - PI / 4.0) * 9.0 * 10.0,
            "rounding one vertical edge takes the square outside its quarter disc",
        );
    }

    #[test]
    fn an_edge_the_shape_does_not_have_is_refused() {
        let shape = Shape::box_solid(20.0, 20.0, 10.0).expect("box");
        let refused = shape.fillet(
            &[FilletEdge {
                midpoint: [7.0, 7.0, 7.0],
                radius: 1.0,
            }],
            1e-6,
        );
        assert!(
            refused.is_err(),
            "a fillet that does not land is an error, not a corner left sharp"
        );
    }

    #[test]
    fn a_loft_between_two_profiles_is_a_frustum() {
        let lower = rounded_rect(20.0, 20.0, 2.0);
        let upper = inset(rounded_rect(16.0, 16.0, 2.0), 2.0);
        let shape = Shape::loft(&[(&lower, 0.0), (&upper, 5.0)]).expect("loft");
        assert!(shape.is_valid().expect("validity check"));
        let bounds = shape.bounds().expect("bounds");
        assert_close(bounds.min[2], 0.0, "a loft starts at its lower profile");
        assert_close(bounds.max[2], 5.0, "a loft ends at its upper profile");
        assert!(
            shape.volume().expect("volume") > 0.0,
            "a loft between two closed profiles encloses material"
        );
    }

    #[test]
    fn a_peg_lofts_through_all_four_of_its_rings() {
        let bottom = rounded_rect(35.6, 35.6, 0.8);
        let mid = inset(rounded_rect(37.2, 37.2, 1.6), -0.8);
        let top = inset(rounded_rect(41.5, 41.5, 4.0), -2.95);
        let peg = Shape::loft(&[
            (&bottom, 0.0),
            (&mid, 0.7),
            (&mid, 2.5),
            (&top, 4.75),
        ])
        .expect("peg loft");
        assert!(peg.is_valid().expect("validity check"));
        assert_eq!(
            peg.shell_count().expect("shells"),
            1,
            "a peg is one closed body, whatever its ring count"
        );
        let bounds = peg.bounds().expect("bounds");
        assert_close(bounds.min[2], 0.0, "a peg starts at the bed");
        assert_close(bounds.max[2], 4.75, "a peg reaches the height its top ring is at");
    }

    /// `profile` moved `by` along both axes, which centres a smaller profile
    /// inside the larger one it is cut from or lofted to.
    fn inset(profile: Profile, by: f64) -> Profile {
        Profile {
            loops: profile
                .loops
                .iter()
                .map(|one| one.iter().map(|seg| shift(*seg, by)).collect())
                .collect(),
        }
    }

    /// `seg` moved `by` along both axes.
    fn shift(seg: Seg, by: f64) -> Seg {
        let move2 = |p: [f64; 2]| [p[0] + by, p[1] + by];
        match seg {
            Seg::Line { a, b } => Seg::Line {
                a: move2(a),
                b: move2(b),
            },
            Seg::Arc {
                a,
                b,
                center,
                radius,
                a0,
                a1,
            } => Seg::Arc {
                a: move2(a),
                b: move2(b),
                center: move2(center),
                radius,
                a0,
                a1,
            },
        }
    }

    #[test]
    fn box_is_valid_and_tessellates() {
        let shape = Shape::box_solid(10.0, 20.0, 5.0).expect("OCCT box");
        assert!(shape.is_valid().expect("validity check"));
        let mesh = shape.tessellate(0.25).expect("OCCT mesh");
        assert!(!mesh.positions.is_empty());
        assert_eq!(mesh.positions.len(), mesh.normals.len());
        assert_eq!(mesh.indices.len() % 3, 0);
    }

    #[test]
    fn rounded_box_is_valid() {
        let shape = Shape::rounded_box(20.0, 20.0, 10.0, 1.0).expect("OCCT fillet");
        assert!(shape.is_valid().expect("validity check"));
    }
}
