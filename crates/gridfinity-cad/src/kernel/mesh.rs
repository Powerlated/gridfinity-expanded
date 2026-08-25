use crate::kernel::math::{Vec3, weld_key};
use std::collections::HashMap;

#[derive(Clone, Debug, Default)]
pub struct Mesh {
    pub positions: Vec<Vec3>,
    pub indices: Vec<u32>,
}

impl Mesh {
    pub fn tri_count(&self) -> usize {
        self.indices.len() / 3
    }

    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    pub fn bounds(&self) -> (Vec3, Vec3) {
        if self.positions.is_empty() {
            return (Vec3::ZERO, Vec3::ZERO);
        }
        let mut min = Vec3::splat(f64::INFINITY);
        let mut max = Vec3::splat(f64::NEG_INFINITY);
        for &p in &self.positions {
            min = min.min(p);
            max = max.max(p);
        }
        (min, max)
    }

    pub fn triangles(&self) -> impl Iterator<Item = [Vec3; 3]> + '_ {
        self.indices.chunks_exact(3).map(move |t| {
            [
                self.positions[t[0] as usize],
                self.positions[t[1] as usize],
                self.positions[t[2] as usize],
            ]
        })
    }

    pub fn flat_vertices(&self) -> Vec<f32> {
        let mut out = Vec::with_capacity(self.tri_count() * 3 * 6);
        for [a, b, c] in self.triangles() {
            let n = (b - a).cross(c - a);
            let len = n.length();
            if len < 1e-12 {
                continue;
            }
            let n = n / len;
            for p in [a, b, c] {
                out.extend_from_slice(&[
                    p.x as f32,
                    p.y as f32,
                    p.z as f32,
                    n.x as f32,
                    n.y as f32,
                    n.z as f32,
                ]);
            }
        }
        out
    }

    pub fn to_stl_binary(&self) -> Vec<u8> {
        let tris = self.tri_count();
        let mut buf = Vec::with_capacity(84 + tris * 50);
        buf.extend_from_slice(&[0u8; 80]);
        buf.extend_from_slice(&(tris as u32).to_le_bytes());
        for [a, b, c] in self.triangles() {
            let mut n = (b - a).cross(c - a);
            let len = n.length();
            n = if len > 1e-12 { n / len } else { Vec3::ZERO };
            for v in [n, a, b, c] {
                buf.extend_from_slice(&(v.x as f32).to_le_bytes());
                buf.extend_from_slice(&(v.y as f32).to_le_bytes());
                buf.extend_from_slice(&(v.z as f32).to_le_bytes());
            }
            buf.extend_from_slice(&0u16.to_le_bytes());
        }
        assert_eq!(
            buf.len(),
            84 + 50 * tris,
            "binary STL is 84 bytes of header plus 50 per facet"
        );
        buf
    }
}

pub fn weld_triangles(tris: impl IntoIterator<Item = [Vec3; 3]>) -> Mesh {
    let mut positions: Vec<Vec3> = Vec::new();
    let mut index: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut indices: Vec<u32> = Vec::new();

    let mut id_of = |p: Vec3| -> u32 {
        *index.entry(weld_key(p)).or_insert_with(|| {
            let id = positions.len() as u32;
            positions.push(p);
            id
        })
    };

    for [a, b, c] in tris {
        let (ia, ib, ic) = (id_of(a), id_of(b), id_of(c));
        if ia != ib && ib != ic && ia != ic {
            indices.extend_from_slice(&[ia, ib, ic]);
        }
    }

    Mesh { positions, indices }
}
