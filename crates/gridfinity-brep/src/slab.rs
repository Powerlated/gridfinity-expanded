use crate::build::{RingEdges, ring, wall_seg};
use crate::geom::Surface;
use crate::math::{Vec3, vec3_of};
use crate::region2d::{presplit_regions, region_difference, region_union};
use crate::sketch::{Seg, loop_area, point_in_segs};
use crate::topo::{Builder, Loop, Solid};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Op {
    Union,
    Difference,
}

#[derive(Clone, Debug)]
pub struct Slab {
    pub region: Vec<Vec<Seg>>,
    pub z0: f64,
    pub z1: f64,
}

impl Slab {
    pub fn new(region: Vec<Vec<Seg>>, z0: f64, z1: f64) -> Slab {
        let (z0, z1) = if z0 <= z1 { (z0, z1) } else { (z1, z0) };
        Slab { region, z0, z1 }
    }
}

const Z_EPS: f64 = 1e-5;

#[derive(Clone, Debug, Default)]
pub struct SlabOpts {
    pub cavity: bool,
    pub open_at: Vec<f64>,
}

pub fn build_slabs(ops: &[(Op, Slab)]) -> Result<Solid, String> {
    let _perf = crate::perf::scope(crate::perf::Metric::BuildSlabs);
    let mut b = Builder::new();
    emit_slabs(&mut b, ops, &SlabOpts::default())?;
    let solid = b.build();
    solid.validate().map_err(|e| format!("slab: {e}"))?;
    Ok(solid)
}

pub fn plan_bands(ops: &[(Op, Slab)]) -> Result<(Vec<f64>, Vec<Vec<Vec<Seg>>>), String> {
    if ops.is_empty() {
        return Err("slab: empty stack".into());
    }

    let split: Vec<Vec<Vec<Seg>>> = presplit_regions(
        &ops.iter()
            .map(|(_, s)| s.region.clone())
            .collect::<Vec<_>>(),
    );

    let mut zs: Vec<f64> = ops.iter().flat_map(|(_, s)| [s.z0, s.z1]).collect();
    zs.sort_by(f64::total_cmp);
    zs.dedup_by(|a, b| (*a - *b).abs() < Z_EPS);
    if zs.len() < 2 {
        return Err("slab: stack has no height".into());
    }

    let bands: Vec<Vec<Vec<Seg>>> = zs
        .windows(2)
        .map(|w| {
            let mid = (w[0] + w[1]) * 0.5;
            let mut acc: Vec<Vec<Seg>> = Vec::new();
            for (i, (op, s)) in ops.iter().enumerate() {
                if mid <= s.z0 || mid >= s.z1 {
                    continue;
                }
                acc = match op {
                    Op::Union => region_union(&acc, &split[i]),
                    Op::Difference => region_difference(&acc, &split[i]),
                };
            }
            acc
        })
        .collect();

    Ok((zs, bands))
}

pub fn emit_slabs(
    b: &mut Builder,
    ops: &[(Op, Slab)],
    opts: &SlabOpts,
) -> Result<Vec<Vec<Vec<Seg>>>, String> {
    let _perf = crate::perf::scope(crate::perf::Metric::EmitSlabs);
    let (zs, bands) = plan_bands(ops)?;

    let solid_side = !opts.cavity;
    for (k, band) in bands.iter().enumerate() {
        let (za, zb) = (zs[k], zs[k + 1]);
        for lp in band {
            for s in lp {
                wall_seg(b, s, za, zb, &[], &[], solid_side);
            }
        }
    }

    let empty: Vec<Vec<Seg>> = Vec::new();
    for (k, &z) in zs.iter().enumerate() {
        if opts.open_at.iter().any(|&o| (o - z).abs() < Z_EPS) {
            continue;
        }
        let below = if k == 0 { &empty } else { &bands[k - 1] };
        let above = if k == bands.len() { &empty } else { &bands[k] };
        for (up, region) in [
            (true, region_difference(below, above)),
            (false, region_difference(above, below)),
        ] {
            for (outer, holes) in group_loops(&region) {
                let o = ring(b, &outer, z);
                let hs: Vec<RingEdges> = holes.iter().map(|h| ring(b, h, z)).collect();
                emit_cap(b, z, up, solid_side, &o, &hs);
            }
        }
    }

    Ok(bands)
}

fn emit_cap(
    b: &mut Builder,
    z: f64,
    up: bool,
    sense: bool,
    outer: &RingEdges,
    holes: &[RingEdges],
) {
    let surface = if up {
        Surface::plane_z(z)
    } else {
        Surface::plane(vec3_of(0.0, 0.0, z), -Vec3::Z)
    };
    let mk = |r: &RingEdges| -> Loop {
        if up {
            Loop::new(r.edges.clone())
        } else {
            Loop::new(r.edges.iter().rev().map(|&(e, d)| (e, !d)).collect())
        }
    };
    b.face(surface, sense, mk(outer), holes.iter().map(mk).collect());
}

fn group_loops(loops: &[Vec<Seg>]) -> Vec<(Vec<Seg>, Vec<Vec<Seg>>)> {
    let mut out: Vec<(Vec<Seg>, Vec<Vec<Seg>>)> = loops
        .iter()
        .filter(|l| loop_area(l) > 0.0)
        .map(|l| (l.clone(), Vec::new()))
        .collect();
    for h in loops.iter().filter(|l| loop_area(l) < 0.0) {
        let pt = h[0].start();
        let mut best: Option<usize> = None;
        for (i, (o, _)) in out.iter().enumerate() {
            if point_in_segs(pt, o)
                && best.is_none_or(|bi| loop_area(o).abs() < loop_area(&out[bi].0).abs())
            {
                best = Some(i);
            }
        }
        if let Some(bi) = best {
            out[bi].1.push(h.clone());
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sketch::Sketch;
    use crate::tess::tessellate;

    fn rect(x0: f64, y0: f64, x1: f64, y1: f64) -> Vec<Vec<Seg>> {
        vec![
            Sketch::rectangle((x0 + x1) * 0.5, (y0 + y1) * 0.5, x1 - x0, y1 - y0)
                .loops
                .remove(0),
        ]
    }

    fn circ(cx: f64, cy: f64, r: f64) -> Vec<Vec<Seg>> {
        vec![Sketch::circle(cx, cy, r).loops.remove(0)]
    }

    fn volume(s: &Solid) -> f64 {
        let mesh = tessellate(s, 32).to_mesh();
        let mut v = 0.0f64;
        for [a, b, c] in mesh.triangles() {
            v += a.dot(b.cross(c)) as f64;
        }
        v / 6.0
    }

    fn watertight(s: &Solid) {
        let mesh = tessellate(s, 8).to_mesh();
        let mut dir = std::collections::HashMap::new();
        for t in mesh.indices.chunks_exact(3) {
            for &(a, b) in &[(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *dir.entry((a, b)).or_insert(0) += 1;
            }
        }
        for (&(a, b), &n) in dir.iter() {
            assert_eq!(n, 1, "edge ({a},{b}) used {n}x");
            assert_eq!(
                dir.get(&(b, a)).copied().unwrap_or(0),
                1,
                "edge ({a},{b}) unpaired"
            );
        }
    }

    #[test]
    fn single_slab_is_a_box() {
        let s = build_slabs(&[(Op::Union, Slab::new(rect(0.0, 0.0, 10.0, 20.0), 0.0, 5.0))])
            .expect("single slab");
        watertight(&s);
        assert!((volume(&s) - 1000.0).abs() < 1e-2, "vol {}", volume(&s));
    }

    #[test]
    fn stacked_union_different_footprints() {
        let s = build_slabs(&[
            (Op::Union, Slab::new(rect(0.0, 0.0, 20.0, 20.0), 0.0, 5.0)),
            (Op::Union, Slab::new(rect(5.0, 5.0, 15.0, 15.0), 5.0, 10.0)),
        ])
        .expect("stacked union");
        watertight(&s);
        assert!(
            (volume(&s) - (2000.0 + 500.0)).abs() < 1e-2,
            "vol {}",
            volume(&s)
        );
    }

    #[test]
    fn overlapping_union_merges_in_z() {
        let s = build_slabs(&[
            (Op::Union, Slab::new(rect(0.0, 0.0, 10.0, 10.0), 0.0, 4.0)),
            (Op::Union, Slab::new(rect(5.0, 5.0, 15.0, 15.0), 0.0, 4.0)),
        ])
        .expect("overlapping union");
        watertight(&s);
        assert!(
            (volume(&s) - 175.0 * 4.0).abs() < 1e-2,
            "vol {}",
            volume(&s)
        );
    }

    #[test]
    fn pocket_difference_makes_walls() {
        let s = build_slabs(&[
            (Op::Union, Slab::new(rect(0.0, 0.0, 20.0, 20.0), 0.0, 10.0)),
            (
                Op::Difference,
                Slab::new(rect(2.0, 2.0, 18.0, 18.0), 2.0, 10.0),
            ),
        ])
        .expect("pocket");
        watertight(&s);
        assert!(
            (volume(&s) - (4000.0 - 256.0 * 8.0)).abs() < 1e-2,
            "vol {}",
            volume(&s)
        );
    }

    #[test]
    fn through_bore_is_a_hole() {
        let s = build_slabs(&[
            (Op::Union, Slab::new(rect(0.0, 0.0, 20.0, 20.0), 0.0, 5.0)),
            (Op::Difference, Slab::new(circ(10.0, 10.0, 3.0), 0.0, 5.0)),
        ])
        .expect("bore");
        watertight(&s);
        let expect = (400.0 - std::f64::consts::PI as f64 * 9.0) * 5.0;
        assert!(
            (volume(&s) - expect).abs() < 0.2,
            "vol {} vs {expect}",
            volume(&s)
        );
    }

    #[test]
    fn cavity_mode_carves_into_caller_geometry() {
        use crate::build::{cap, ring, wall_between};

        let outer = rect(0.0, 0.0, 20.0, 20.0).remove(0);
        let pocket = rect(2.0, 2.0, 18.0, 18.0);
        let mut b = Builder::new();

        let o_lo = ring(&mut b, &outer, 0.0);
        let o_hi = ring(&mut b, &outer, 10.0);
        wall_between(&mut b, &outer, &outer, &o_lo, &o_hi, 0.0, 10.0, true);
        cap(&mut b, 0.0, false, &o_lo, &[]);

        emit_slabs(
            &mut b,
            &[(Op::Union, Slab::new(pocket.clone(), 2.0, 10.0))],
            &SlabOpts {
                cavity: true,
                open_at: vec![10.0],
            },
        )
        .expect("carve pocket");

        let p_hi = ring(&mut b, &pocket[0], 10.0);
        cap(&mut b, 10.0, true, &o_hi, &[&p_hi]);

        let s = b.build();
        s.validate().expect("carved solid is manifold");
        watertight(&s);
        assert!(
            (volume(&s) - (4000.0 - 2048.0)).abs() < 1e-2,
            "vol {}",
            volume(&s)
        );
    }

    #[test]
    fn blind_bore_leaves_a_floor() {
        let s = build_slabs(&[
            (Op::Union, Slab::new(rect(0.0, 0.0, 20.0, 20.0), 0.0, 10.0)),
            (Op::Difference, Slab::new(circ(10.0, 10.0, 3.0), 4.0, 10.0)),
        ])
        .expect("blind bore");
        watertight(&s);
        let expect = 4000.0 - std::f64::consts::PI as f64 * 9.0 * 6.0;
        assert!(
            (volume(&s) - expect).abs() < 0.2,
            "vol {} vs {expect}",
            volume(&s)
        );
    }
}
