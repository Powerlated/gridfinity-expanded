//! Free-form inner walls, as the islands a compartment's cavity is carved
//! around.
//!
//! A wall the user drew anywhere in the bin is not a boundary edge and cannot be
//! walked; it is a rectangle of material standing in the cavity, so
//! `inner_wall_quad` gives it its rounded outline and the cavity subtracts it.
//! `inner_wall_quad_in` additionally refuses one that does not lie inside the
//! compartment it was offered to. Height decides which of the three shapes it
//! becomes: a full-height wall is an `Island` the cavity is differenced by, a
//! partial-height one crossing the compartment boundary is a `Notch` of a
//! `Banded` slab stack, and one wholly inside the compartment is an `Island`
//! with a `top`.

use super::*;
use crate::kernel::math::Vec2;
use crate::kernel::round::short_arc;
use crate::kernel::sketch::{Seg, loop_area, point_in_segs, reverse_loop};

#[derive(Clone, Debug)]
pub(super) struct Island {
    pub(super) segs: Vec<Seg>,
    pub(super) top: Option<f64>,
    pub(super) fr: f64,
}

#[derive(Clone, Debug)]
pub(super) struct Notch {
    pub(super) quad: Vec<Seg>,
    pub(super) contact: Vec<Seg>,
    pub(super) top: f64,
}

#[derive(Clone, Debug)]
pub(super) struct Banded {
    pub(super) outline_a: Vec<Vec<(Seg, Option<usize>)>>,
    pub(super) outline_b: Vec<Seg>,
    pub(super) notches: Vec<Notch>,
}

pub(super) fn inner_wall_quad(w: &InnerWall, r: f64) -> Option<Vec<Seg>> {
    let a = Vec2::new(w.x1, w.y1);
    let b = Vec2::new(w.x2, w.y2);
    let d = b - a;
    let len = d.length();
    if len < 0.1 {
        return None;
    }
    let u = d / len;
    let n = Vec2::new(-u.y, u.x);
    let hw = w.width.max(0.4) / 2.0;
    let (p0, p1, p2, p3) = (a - n * hw, b - n * hw, b + n * hw, a + n * hw);
    let sharp = vec![
        Seg::Line { a: p0, b: p1 },
        Seg::Line { a: p1, b: p2 },
        Seg::Line { a: p2, b: p3 },
        Seg::Line { a: p3, b: p0 },
    ];
    let mut corners = vec![p0, p1, p2, p3];
    let sharp = if loop_area(&sharp) < 0.0 {
        corners.reverse();
        reverse_loop(&sharp)
    } else {
        sharp
    };
    let r = r.min(hw).min(len / 2.0);
    if r < MIN_QUAD_ROUND {
        return Some(sharp);
    }
    let n_c = corners.len();
    let tangents: Vec<(Vec2, Vec2, Vec2)> = (0..n_c)
        .map(|i| {
            let v = corners[i];
            let din = (v - corners[(i + n_c - 1) % n_c]).normalize();
            let dout = (corners[(i + 1) % n_c] - v).normalize();
            let t_in = v - din * r;
            (t_in, v + dout * r, t_in + Vec2::new(-din.y, din.x) * r)
        })
        .collect();
    let mut out = Vec::with_capacity(n_c * 2);
    for i in 0..n_c {
        let (t_in, t_out, center) = tangents[i];
        let a0 = f64::atan2(t_in.y - center.y, t_in.x - center.x);
        let a1 = f64::atan2(t_out.y - center.y, t_out.x - center.x);
        let (a0, a1) = short_arc(a0, a1);
        out.push(Seg::Arc {
            a: t_in,
            b: t_out,
            center,
            radius: r,
            a0,
            a1,
        });
        let next_in = tangents[(i + 1) % n_c].0;
        if (next_in - t_out).length() > MIN_STRAIGHT_RUN {
            out.push(Seg::Line {
                a: t_out,
                b: next_in,
            });
        }
    }
    Some(out)
}

pub(super) fn inner_wall_quad_in(w: &InnerWall, r: f64, outer: &[Seg]) -> Option<Vec<Seg>> {
    let sharp = inner_wall_quad(w, 0.0)?;
    if r < MIN_QUAD_ROUND {
        return Some(sharp);
    }
    let floats_free = sharp.iter().all(|s| point_in_segs(s.start(), outer));
    if floats_free {
        inner_wall_quad(w, r)
    } else {
        Some(sharp)
    }
}
