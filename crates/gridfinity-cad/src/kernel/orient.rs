use crate::kernel::geom::Surface;
use crate::kernel::math::{Vec3, wrap_pi};
use crate::kernel::topo::{EdgeId, Solid};

const SAMPLES_PER_EDGE: usize = 4;

fn loop_points(solid: &Solid, lp: &[(EdgeId, bool)]) -> Vec<Vec3> {
    let mut pts = Vec::with_capacity(lp.len() * SAMPLES_PER_EDGE);
    for &(e, fwd) in lp {
        let ed = solid.edges[e];
        let (ta, tb) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
        for k in 0..SAMPLES_PER_EDGE {
            let f = k as f64 / SAMPLES_PER_EDGE as f64;
            pts.push(ed.curve.point(ta + (tb - ta) * f));
        }
    }
    pts
}

fn area_vector(pts: &[Vec3]) -> Vec3 {
    let mut a = Vec3::ZERO;
    for i in 0..pts.len() {
        a += pts[i].cross(pts[(i + 1) % pts.len()]);
    }
    a * 0.5
}

fn wraps(solid: &Solid, surface: &Surface, lp: &[(EdgeId, bool)]) -> bool {
    if matches!(surface, Surface::Plane { .. }) {
        return false;
    }
    let prep = surface.prepare();
    let mut drift = 0.0f64;
    let mut prev: Option<f64> = None;
    for &(e, fwd) in lp {
        let ed = solid.edges[e];
        let (ta, tb) = if fwd { (ed.t0, ed.t1) } else { (ed.t1, ed.t0) };
        for k in 0..=SAMPLES_PER_EDGE {
            let f = k as f64 / SAMPLES_PER_EDGE as f64;
            let (u, _) = prep.project(ed.curve.point(ta + (tb - ta) * f));
            if let Some(p) = prev {
                drift += wrap_pi(u - p);
            }
            prev = Some(u);
        }
    }
    drift.abs() > std::f64::consts::PI
}

fn outward_area(solid: &Solid, fid: usize, lid: u32) -> Option<f64> {
    let face = &solid.faces[fid];
    let lp = solid.loop_by_id(lid);
    if lp.len() < 2 || wraps(solid, &face.surface, lp) {
        return None;
    }
    let pts = loop_points(solid, lp);
    if pts.len() < 3 {
        return None;
    }
    let centroid = pts.iter().fold(Vec3::ZERO, |a, &p| a + p) / pts.len() as f64;
    let sign = if face.sense { 1.0 } else { -1.0 };
    let normal = face.surface.normal(face.surface.project(centroid)) * sign;
    let a = area_vector(&pts).dot(normal);
    (a.abs() > 1e-9).then_some(a)
}

fn components(solid: &Solid) -> (Vec<usize>, usize) {
    let loops: Vec<(usize, u32)> = (0..solid.faces.len())
        .flat_map(|f| solid.loop_ids(f).map(move |l| (f, l)))
        .collect();
    let index: std::collections::HashMap<u32, usize> = loops
        .iter()
        .enumerate()
        .map(|(i, &(_, l))| (l, i))
        .collect();
    let mut parent: Vec<usize> = (0..loops.len()).collect();
    fn find(parent: &mut Vec<usize>, mut x: usize) -> usize {
        while parent[x] != x {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        x
    }
    let mut by_edge: std::collections::HashMap<EdgeId, Vec<usize>> =
        std::collections::HashMap::new();
    for &(_, lid) in &loops {
        for &(e, _) in solid.loop_by_id(lid) {
            by_edge.entry(e).or_default().push(index[&lid]);
        }
    }
    for users in by_edge.values() {
        for w in users.windows(2) {
            let (a, b) = (find(&mut parent, w[0]), find(&mut parent, w[1]));
            parent[a] = b;
        }
    }
    let mut label = vec![usize::MAX; loops.len()];
    let mut next = 0;
    for i in 0..loops.len() {
        let r = find(&mut parent, i);
        if label[r] == usize::MAX {
            label[r] = next;
            next += 1;
        }
        label[i] = label[r];
    }
    (label, next)
}

pub fn misoriented_loops(solid: &Solid) -> Vec<(usize, u32)> {
    let loops: Vec<(usize, u32)> = (0..solid.faces.len())
        .flat_map(|f| solid.loop_ids(f).map(move |l| (f, l)))
        .collect();
    let (label, n) = components(solid);
    let mut vote = vec![0.0f64; n];
    for (i, &(fid, lid)) in loops.iter().enumerate() {
        let Some(a) = outward_area(solid, fid, lid) else {
            continue;
        };
        let outer = solid.loop_ids(fid).next() == Some(lid);
        let want = if outer { 1.0 } else { -1.0 };
        vote[label[i]] += a * want;
    }
    loops
        .iter()
        .enumerate()
        .filter(|(i, _)| vote[label[*i]] < 0.0)
        .map(|(_, &l)| l)
        .collect()
}

pub fn normalize(solid: &mut Solid) {
    for (_, lid) in misoriented_loops(solid) {
        solid.reverse_loop(lid);
    }
    let left = misoriented_loops(solid);
    assert!(
        left.is_empty(),
        "normalize left {} loop(s) misoriented, first face {}",
        left.len(),
        left[0].0
    );
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::build::extrude;
    use crate::kernel::sketch::Sketch;
    use crate::kernel::tess::tessellate;

    fn signed_volume(solid: &Solid) -> f64 {
        let mut v = 0.0f64;
        for [a, b, c] in tessellate(solid, 16).to_mesh().triangles() {
            v += a.dot(b.cross(c)) as f64;
        }
        v / 6.0
    }

    #[test]
    fn every_solid_the_builder_produces_is_already_oriented() {
        let solids = [
            extrude(&Sketch::rectangle(0.0, 0.0, 10.0, 20.0), 0.0, 5.0),
            extrude(&Sketch::rounded_rect(0.0, 0.0, 40.0, 30.0, 5.0), 0.0, 7.0),
            crate::gridfinity::build(&crate::gridfinity::Params::rect(1, 1)),
            crate::gridfinity::build(&crate::gridfinity::Params::rect(3, 2)),
        ];
        for solid in &solids {
            assert!(
                misoriented_loops(solid).is_empty(),
                "build() must leave every loop material-consistent"
            );
        }
    }

    #[test]
    fn a_cavity_shell_is_the_part_that_needs_reorienting() {
        let mut raw = crate::gridfinity::build(&crate::gridfinity::Params::rect(2, 1));
        let all: Vec<u32> = (0..raw.faces.len()).flat_map(|f| raw.loop_ids(f)).collect();
        for lid in all {
            raw.reverse_loop(lid);
        }
        let flipped = misoriented_loops(&raw).len();
        assert!(flipped > 0, "reversing every loop must break the invariant");
        normalize(&mut raw);
        assert!(misoriented_loops(&raw).is_empty());
        raw.validate()
            .expect("renormalising restores a manifold solid");
    }

    #[test]
    fn normalising_changes_no_geometry() {
        let solid = crate::gridfinity::build(&crate::gridfinity::Params::rect(2, 1));
        let before = signed_volume(&solid);
        let mut flipped = solid.clone();
        let all: Vec<u32> = (0..flipped.faces.len())
            .flat_map(|f| flipped.loop_ids(f))
            .collect();
        for lid in all {
            flipped.reverse_loop(lid);
        }
        normalize(&mut flipped);
        let after = signed_volume(&flipped);
        assert!(
            (before - after).abs() < 1e-3,
            "orientation must not move geometry: {before} vs {after}"
        );
    }
}
