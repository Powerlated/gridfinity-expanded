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

impl Mesh {
    /// The number of indexed triangles in this mesh.
    pub fn tri_count(&self) -> usize {
        self.indices.len() / 3
    }

    /// This mesh as binary STL, with one geometric facet normal and three
    /// positions per indexed triangle in the millimetres OCCT produced.
    pub fn to_stl_binary(&self) -> Vec<u8> {
        let mut bytes = Vec::with_capacity(84 + self.tri_count() * 50);
        bytes.extend_from_slice(&[0; 80]);
        bytes.extend_from_slice(&(self.tri_count() as u32).to_le_bytes());
        for triangle in self.indices.chunks_exact(3) {
            let a = self.positions[triangle[0] as usize];
            let b = self.positions[triangle[1] as usize];
            let c = self.positions[triangle[2] as usize];
            let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
            let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
            let mut normal = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            let length =
                (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
            if length > 0.0 {
                normal.iter_mut().for_each(|component| *component /= length);
            }
            for vector in [normal, a, b, c] {
                for component in vector {
                    bytes.extend_from_slice(&(component as f32).to_le_bytes());
                }
            }
            bytes.extend_from_slice(&0u16.to_le_bytes());
        }
        assert_eq!(
            bytes.len(),
            84 + self.tri_count() * 50,
            "a binary STL is 84 header bytes plus 50 bytes per triangle"
        );
        bytes
    }

    /// Non-indexed `[position, normal]` vertices as `f32`, one record per
    /// triangle corner for the renderer's kernel-buffer boundary.
    pub fn render_buffer(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(self.indices.len() * 6);
        for &index in &self.indices {
            let position = self.positions[index as usize];
            let normal = self.normals[index as usize];
            out.extend(position.into_iter().chain(normal).map(|value| value as f32));
        }
        assert_eq!(
            out.len(),
            self.indices.len() * 6,
            "each indexed OCCT triangle corner becomes one six-float render vertex"
        );
        out
    }
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
        fn gf_occt_cut_half_space(
            p: *const RawShape,
            ox: f64,
            oy: f64,
            oz: f64,
            nx: f64,
            ny: f64,
            nz: f64,
        ) -> *mut RawShape;
        fn gf_occt_fillet_edges(
            p: *const RawShape,
            edges: *const f64,
            count: usize,
            tolerance: f64,
        ) -> *mut RawShape;
        fn gf_occt_volume(p: *const RawShape, volume: *mut f64) -> i32;
        fn gf_occt_bounds(p: *const RawShape, bounds: *mut f64) -> i32;
        fn gf_occt_shell_count(p: *const RawShape, shells: *mut usize) -> i32;
        fn gf_occt_shell_volumes(p: *const RawShape, volumes: *mut f64, count: usize) -> i32;
        fn gf_occt_edge_count(p: *const RawShape, edges: *mut usize) -> i32;
        fn gf_occt_edge_midpoints(p: *const RawShape, midpoints: *mut f64, count: usize) -> i32;
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
                gf_occt_prism_from_loops(segments.as_ptr(), loops.as_ptr(), loops.len(), z, dz)
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
            Self::from_raw(unsafe { gf_occt_boolean(self.0.as_ptr(), other.0.as_ptr(), op.code()) })
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

        /// This shape with the material on the side `normal` points to removed,
        /// keeping what lies behind the plane through `origin`. `normal` need
        /// not be unit; it need not be axis aligned either, which is the whole
        /// reason this exists beside `prism` -- a sloped floor is a tilted
        /// plane, and a z-prism cannot state one.
        pub fn cut_half_space(&self, origin: [f64; 3], normal: [f64; 3]) -> Result<Self, Error> {
            Self::from_raw(unsafe {
                gf_occt_cut_half_space(
                    self.0.as_ptr(),
                    origin[0],
                    origin[1],
                    origin[2],
                    normal[0],
                    normal[1],
                    normal[2],
                )
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

        /// The signed volume each shell of this shape bounds, in shell order:
        /// positive where the shell has the material inside it, negative where
        /// it bounds a void sealed within the material. `shell_count` says how
        /// many there are, and every entry being positive is the statement that
        /// the body has no internal cavity.
        pub fn shell_volumes(&self) -> Result<Vec<f64>, Error> {
            let count = self.shell_count()?;
            let mut volumes = vec![0.0; count];
            if unsafe { gf_occt_shell_volumes(self.0.as_ptr(), volumes.as_mut_ptr(), count) } == 0 {
                return Err(error());
            }
            Ok(volumes)
        }

        /// The midpoint of every edge this shape carries, which is the name
        /// `fillet` selects on. A blend that reports an edge the shape does not
        /// have is asking about this list, and nothing else can answer it.
        pub fn edge_midpoints(&self) -> Result<Vec<[f64; 3]>, Error> {
            let mut count = 0usize;
            if unsafe { gf_occt_edge_count(self.0.as_ptr(), &mut count) } == 0 {
                return Err(error());
            }
            let mut flat = vec![0.0; count * 3];
            if unsafe { gf_occt_edge_midpoints(self.0.as_ptr(), flat.as_mut_ptr(), count) } == 0 {
                return Err(error());
            }
            Ok(flat.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect())
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
    pub fn cut_half_space(&self, _: [f64; 3], _: [f64; 3]) -> Result<Self, Error> {
        Self::unavailable()
    }
    pub fn shell_volumes(&self) -> Result<Vec<f64>, Error> {
        Self::unavailable()
    }
    pub fn edge_midpoints(&self) -> Result<Vec<[f64; 3]>, Error> {
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

    /// The same loop translated to `(x, y)`, for a profile that must stand
    /// clear of the body it is cut out of rather than in its corner.
    fn offset_rect(x: f64, y: f64, w: f64, h: f64, r: f64) -> Profile {
        let shift = |p: [f64; 2]| [p[0] + x, p[1] + y];
        Profile {
            loops: rounded_rect(w, h, r)
                .loops
                .into_iter()
                .map(|one| {
                    one.into_iter()
                        .map(|seg| match seg {
                            Seg::Line { a, b } => Seg::Line {
                                a: shift(a),
                                b: shift(b),
                            },
                            Seg::Arc {
                                a,
                                b,
                                center,
                                radius,
                                a0,
                                a1,
                            } => Seg::Arc {
                                a: shift(a),
                                b: shift(b),
                                center: shift(center),
                                radius,
                                a0,
                                a1,
                            },
                        })
                        .collect()
                })
                .collect(),
        }
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

    /// A half-space cut through the middle of a box, square on, leaves exactly
    /// half of it. The volume is the check because it is the one number a cut
    /// cannot get right by accident: a plane placed anywhere else, or facing
    /// the other way, gives a different answer.
    #[test]
    fn a_square_on_half_space_cut_halves_the_body() {
        let cube = Shape::box_solid(10.0, 10.0, 10.0).expect("OCCT box");
        let kept = cube
            .cut_half_space([0.0, 0.0, 5.0], [0.0, 0.0, 1.0])
            .expect("OCCT half-space cut");
        assert!(kept.is_valid().expect("validity check"));
        assert_close(kept.volume().expect("volume"), 500.0, "half a 10mm cube");
        let bounds = kept.bounds().expect("bounds");
        assert_close(bounds.max[2], 5.0, "the cut plane is the new top");
        assert_close(bounds.min[2], 0.0, "the bottom is untouched");
    }

    /// The cut keeps what lies *behind* the normal, so flipping the normal
    /// keeps the other half and the two halves sum back to the whole. Stated as
    /// the sum rather than as two separate volumes because that is the property
    /// a sloped floor relies on: a plane divides the material, it consumes none.
    #[test]
    fn the_two_sides_of_one_plane_sum_back_to_the_whole_body() {
        let cube = Shape::box_solid(8.0, 6.0, 4.0).expect("OCCT box");
        let whole = cube.volume().expect("volume");
        let origin = [3.0, 2.0, 1.0];
        let below = cube.cut_half_space(origin, [1.0, 0.0, 0.0]).expect("cut");
        let above = cube.cut_half_space(origin, [-1.0, 0.0, 0.0]).expect("cut");
        assert_close(
            below.volume().expect("volume") + above.volume().expect("volume"),
            whole,
            "a plane divides the material without consuming any",
        );
    }

    /// A tilted plane is the case a z-prism cannot express and the reason this
    /// call exists. Cutting a square column through its centre at 45 degrees
    /// leaves a wedge of exactly half the volume that still reaches the top at
    /// one wall and the floor at the other, so the tilt itself is pinned rather
    /// than merely that something was removed.
    #[test]
    fn a_tilted_plane_cuts_a_ramp_a_z_prism_cannot_state() {
        let column = Shape::box_solid(10.0, 10.0, 10.0).expect("OCCT box");
        let ramp = column
            .cut_half_space([5.0, 0.0, 5.0], [1.0, 0.0, 1.0])
            .expect("OCCT tilted cut");
        assert!(ramp.is_valid().expect("validity check"));
        assert_close(ramp.volume().expect("volume"), 500.0, "half the column");
        let bounds = ramp.bounds().expect("bounds");
        assert_close(bounds.max[2], 10.0, "the ramp still reaches the top at x=0");
        assert_close(bounds.max[0], 10.0, "and still reaches x=10 at the floor");
    }

    /// One body of material is one shell with the material inside it, so its
    /// signed volume is its own volume: positive, and equal to what the whole
    /// shape measures.
    #[test]
    fn a_solid_body_is_one_shell_whose_signed_volume_is_its_own() {
        let shape = Shape::prism(&rounded_rect(41.5, 41.5, 4.0), 0.0, 7.0).expect("OCCT prism");
        let volumes = shape.shell_volumes().expect("shell volumes");
        assert_eq!(volumes.len(), 1, "one lump of material is one shell");
        assert!(
            volumes[0] > 0.0,
            "its material is inside it: {}",
            volumes[0]
        );
        assert_close(
            volumes[0],
            shape.volume().expect("volume"),
            "the body's own volume",
        );
    }

    /// A void sealed inside material is a second shell with the material
    /// *outside* it, and the sign is what says so. Nothing downstream can see
    /// this otherwise -- a sealed cavity tessellates like any other closed
    /// surface -- so the negative entry is the whole point of the call.
    #[test]
    fn a_void_sealed_inside_material_is_a_shell_of_negative_volume() {
        let block = Shape::box_solid(20.0, 20.0, 20.0).expect("OCCT box");
        let bubble =
            Shape::prism(&offset_rect(7.0, 7.0, 6.0, 6.0, 1.0), 6.0, 2.0).expect("OCCT prism");
        let hollow = block
            .boolean(&bubble, Boolean::Cut)
            .expect("a bubble cut out of the middle of a block");
        let volumes = hollow.shell_volumes().expect("shell volumes");
        assert_eq!(volumes.len(), 2, "the outer surface and the cavity's");
        assert_eq!(
            volumes.iter().filter(|v| **v > 0.0).count(),
            1,
            "exactly one of the two has material inside it: {volumes:?}"
        );
        let void = volumes.iter().copied().fold(f64::INFINITY, f64::min);
        assert_close(
            -void,
            rounded_rect_area(6.0, 6.0, 1.0) * 2.0,
            "the void's own volume",
        );
    }

    /// A box has twelve edges and the midpoint list names all twelve, each on
    /// the box. It is the list `fillet` matches against, so the property worth
    /// stating is that every entry really is a point of the shape's boundary --
    /// a diagnostic reporting points off the body would send its reader the
    /// wrong way -- and that a reported midpoint is one `fillet` accepts.
    #[test]
    fn the_edge_midpoints_are_the_edges_a_blend_selects_on() {
        let cube = Shape::box_solid(4.0, 6.0, 8.0).expect("OCCT box");
        let mids = cube.edge_midpoints().expect("edge midpoints");
        assert_eq!(mids.len(), 12, "a box has twelve edges");
        for m in &mids {
            let on_face = [(m[0], 4.0), (m[1], 6.0), (m[2], 8.0)]
                .iter()
                .filter(|(v, extent)| v.abs() < 1e-9 || (v - extent).abs() < 1e-9)
                .count();
            assert_eq!(
                on_face, 2,
                "an edge midpoint lies on exactly two of the box's faces, not {on_face}: {m:?}"
            );
        }
        let rounded = cube
            .fillet(
                &[FilletEdge {
                    midpoint: mids[0],
                    radius: 0.5,
                }],
                1e-6,
            )
            .expect("a midpoint the list reported is one the blend accepts");
        assert!(rounded.volume().expect("volume") < cube.volume().expect("volume"));
    }

    /// A curved face's mesh normals come from the surface, not from the
    /// triangles: on a cylindrical wall the normal is exactly radial, which an
    /// average of two chords never is.
    ///
    /// Stated over the nodes at each *position* rather than over every node,
    /// because a wall's rim and the cap it meets there stand at the same point
    /// and carry different normals -- which is the second property this pins.
    /// Each face owns its own nodes, so the corner between them stays sharp;
    /// one node shared there would round the rim of every body the model
    /// builds, and no other check in this file can see either fact.
    #[test]
    fn a_curved_faces_mesh_normals_are_the_surfaces_own() {
        let radius = 5.0;
        let column = Shape::cone(radius, radius * 0.999_999, 10.0).expect("OCCT near-cylinder");
        let mesh = column.tessellate(0.02).expect("OCCT mesh");
        let mut walls = 0usize;
        let mut rims = 0usize;
        for (i, p) in mesh.positions.iter().enumerate() {
            let r = (p[0] * p[0] + p[1] * p[1]).sqrt();
            if r < radius * 0.98 {
                continue;
            }
            let here: Vec<[f64; 3]> = mesh
                .positions
                .iter()
                .zip(&mesh.normals)
                .filter(|(q, _)| {
                    (q[0] - p[0]).abs() < 1e-9
                        && (q[1] - p[1]).abs() < 1e-9
                        && (q[2] - p[2]).abs() < 1e-9
                })
                .map(|(_, n)| *n)
                .collect();
            for n in &here {
                let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
                assert!(
                    (len - 1.0).abs() < 1e-9,
                    "every mesh normal is unit, this one is {len}"
                );
            }
            let radial = here
                .iter()
                .filter(|n| (p[0] * n[0] + p[1] * n[1]) / r > 1.0 - 1e-9)
                .count();
            assert!(
                radial >= 1,
                "the node at {p:?} carries the wall's own outward normal exactly; none of the {} there does",
                here.len()
            );
            walls += 1;
            if here.len() > 1 {
                assert!(
                    here.iter().any(|n| n[2].abs() > 0.999),
                    "a node sharing a rim point with the wall is the cap's, and points along z"
                );
                rims += 1;
            }
            if i > 40 {
                break;
            }
        }
        assert!(walls > 0, "the mesh must actually reach the curved wall");
        assert!(
            rims > 0,
            "and must reach the rim, where the wall and the cap stand at one point"
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
        assert_close(
            bounds.min[2],
            0.0,
            "a prism starts in the plane it is drawn in",
        );
        assert_close(bounds.max[2], 7.0, "a prism reaches the height it is swept");
    }

    #[test]
    fn a_cavity_takes_exactly_its_own_volume() {
        let outer = Shape::prism(&rounded_rect(41.5, 41.5, 4.0), 0.0, 20.0).expect("outer");
        let cavity =
            Shape::prism(&inset(rounded_rect(35.5, 35.5, 2.0), 3.0), 3.0, 20.0).expect("cavity");
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
        let peg = Shape::loft(&[(&bottom, 0.0), (&mid, 0.7), (&mid, 2.5), (&top, 4.75)])
            .expect("peg loft");
        assert!(peg.is_valid().expect("validity check"));
        assert_eq!(
            peg.shell_count().expect("shells"),
            1,
            "a peg is one closed body, whatever its ring count"
        );
        let bounds = peg.bounds().expect("bounds");
        assert_close(bounds.min[2], 0.0, "a peg starts at the bed");
        assert_close(
            bounds.max[2],
            4.75,
            "a peg reaches the height its top ring is at",
        );
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
        let stl = mesh.to_stl_binary();
        assert_eq!(
            stl.len(),
            84 + 50 * mesh.tri_count(),
            "the OCCT mesh writes one complete binary STL facet per triangle"
        );
    }

    #[test]
    fn rounded_box_is_valid() {
        let shape = Shape::rounded_box(20.0, 20.0, 10.0, 1.0).expect("OCCT fillet");
        assert!(shape.is_valid().expect("validity check"));
    }
}
