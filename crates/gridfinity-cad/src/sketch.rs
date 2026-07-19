//! 2D analytic profiles: closed loops of `Line` / `Arc` segments in the XY
//! plane. Corner radii are carried as real arcs (not faceted), so an extruded
//! rounded rectangle becomes true cylindrical corner faces. Outer loops are
//! authored CCW; `build.rs` reorients hole loops to CW as needed.

use crate::math::Vec2;
use std::f32::consts::PI;

/// One boundary segment. `a`/`b` are the endpoints in traversal order; for an
/// `Arc`, `a0`/`a1` are the (unwrapped) angles at `a`/`b` about `center`.
#[derive(Clone, Copy, Debug)]
pub enum Seg {
    Line {
        a: Vec2,
        b: Vec2,
    },
    Arc {
        a: Vec2,
        b: Vec2,
        center: Vec2,
        radius: f32,
        a0: f32,
        a1: f32,
    },
}

impl Seg {
    pub fn start(&self) -> Vec2 {
        match *self {
            Seg::Line { a, .. } | Seg::Arc { a, .. } => a,
        }
    }
    pub fn end(&self) -> Vec2 {
        match *self {
            Seg::Line { b, .. } | Seg::Arc { b, .. } => b,
        }
    }
    /// Reverse traversal direction of this segment.
    pub fn reversed(&self) -> Seg {
        match *self {
            Seg::Line { a, b } => Seg::Line { a: b, b: a },
            Seg::Arc {
                a,
                b,
                center,
                radius,
                a0,
                a1,
            } => Seg::Arc {
                a: b,
                b: a,
                center,
                radius,
                a0: a1,
                a1: a0,
            },
        }
    }
}

/// A closed profile: one or more loops (only the builders here make single-loop
/// sketches; multi-loop regions are assembled in `build.rs`).
#[derive(Clone, Debug, Default)]
pub struct Sketch {
    pub loops: Vec<Vec<Seg>>,
}

impl Sketch {
    pub fn single(loop_: Vec<Seg>) -> Sketch {
        Sketch { loops: vec![loop_] }
    }

    /// Signed area of the first loop (positive = CCW).
    pub fn area(&self) -> f32 {
        loop_area(&self.loops[0])
    }

    /// Axis-aligned rectangle centred at `(cx, cy)`.
    pub fn rectangle(cx: f32, cy: f32, w: f32, h: f32) -> Sketch {
        let (hw, hh) = (w / 2.0, h / 2.0);
        let p = |x, y| Vec2::new(cx + x, cy + y);
        let (bl, br, tr, tl) = (p(-hw, -hh), p(hw, -hh), p(hw, hh), p(-hw, hh));
        Sketch::single(vec![
            Seg::Line { a: bl, b: br },
            Seg::Line { a: br, b: tr },
            Seg::Line { a: tr, b: tl },
            Seg::Line { a: tl, b: bl },
        ])
    }

    /// Rounded rectangle centred at `(cx, cy)`. Falls back to a plain rectangle
    /// for `r <= 0`. CCW: bottom → BR arc → right → TR arc → top → TL arc →
    /// left → BL arc.
    pub fn rounded_rect(cx: f32, cy: f32, w: f32, h: f32, r: f32) -> Sketch {
        let r = r.min(w / 2.0).min(h / 2.0);
        if r <= 1e-4 {
            return Sketch::rectangle(cx, cy, w, h);
        }
        let (hw, hh) = (w / 2.0, h / 2.0);
        let (ix, iy) = (hw - r, hh - r);
        let p = |x, y| Vec2::new(cx + x, cy + y);
        // Straight-segment endpoints on each side.
        let b_l = p(-ix, -hh);
        let b_r = p(ix, -hh);
        let r_b = p(hw, -iy);
        let r_t = p(hw, iy);
        let t_r = p(ix, hh);
        let t_l = p(-ix, hh);
        let l_t = p(-hw, iy);
        let l_b = p(-hw, -iy);
        let arc = |a, b, cx2, cy2, a0, a1| Seg::Arc {
            a,
            b,
            center: Vec2::new(cx2, cy2),
            radius: r,
            a0,
            a1,
        };
        Sketch::single(vec![
            Seg::Line { a: b_l, b: b_r },
            arc(b_r, r_b, cx + ix, cy - iy, -PI / 2.0, 0.0),
            Seg::Line { a: r_b, b: r_t },
            arc(r_t, t_r, cx + ix, cy + iy, 0.0, PI / 2.0),
            Seg::Line { a: t_r, b: t_l },
            arc(t_l, l_t, cx - ix, cy + iy, PI / 2.0, PI),
            Seg::Line { a: l_t, b: l_b },
            arc(l_b, b_l, cx - ix, cy - iy, PI, 1.5 * PI),
        ])
    }

    /// Circle centred at `(cx, cy)`, built from two semicircular arcs so each
    /// edge is a bounded curve with two endpoints (no full-circle seam).
    pub fn circle(cx: f32, cy: f32, r: f32) -> Sketch {
        let c = Vec2::new(cx, cy);
        let right = Vec2::new(cx + r, cy);
        let left = Vec2::new(cx - r, cy);
        Sketch::single(vec![
            Seg::Arc {
                a: right,
                b: left,
                center: c,
                radius: r,
                a0: 0.0,
                a1: PI,
            },
            Seg::Arc {
                a: left,
                b: right,
                center: c,
                radius: r,
                a0: PI,
                a1: 2.0 * PI,
            },
        ])
    }
}

/// Signed area of a loop, exact for arcs: the chord shoelace plus each arc's
/// circular-segment area r²(Δ − sin Δ)/2, where Δ = a1 − a0 is the signed
/// sweep (a CCW arc bulges left of its chord and adds positive area).
pub fn loop_area(segs: &[Seg]) -> f32 {
    let mut s = 0.0;
    for seg in segs {
        let a = seg.start();
        let b = seg.end();
        s += a.x * b.y - b.x * a.y;
        if let Seg::Arc { radius, a0, a1, .. } = *seg {
            let d = a1 - a0;
            s += radius * radius * (d - d.sin());
        }
    }
    s / 2.0
}

/// Exact point-in-loop test (even–odd cast toward +x). Lines use the standard
/// half-open crossing rule; each arc is split at its y-extreme angles
/// (cos θ = 0) into y-monotone pieces so the same rule applies, with the
/// crossing x solved on the circle itself — a y-monotone piece lies entirely
/// on one side of its centre, so the ± of `c.x ± √(r² − dy²)` is the sign of
/// cos θ anywhere on the piece.
pub fn point_in_segs(pt: Vec2, segs: &[Seg]) -> bool {
    let mut inside = false;
    let mut cross = |y0: f32, y1: f32, x: f32| {
        if (y0 > pt.y) != (y1 > pt.y) && x > pt.x {
            inside = !inside;
        }
    };
    for seg in segs {
        match *seg {
            Seg::Line { a, b } => {
                if (a.y > pt.y) != (b.y > pt.y) {
                    cross(a.y, b.y, a.x + (pt.y - a.y) / (b.y - a.y) * (b.x - a.x));
                }
            }
            Seg::Arc { a, b, center, radius, a0, a1 } => {
                // y-extreme angles π/2 + kπ strictly inside the sweep, in
                // traversal order.
                let (lo, hi) = (a0.min(a1), a0.max(a1));
                let k0 = ((lo - PI / 2.0) / PI).floor() as i32 + 1;
                let k1 = ((hi - PI / 2.0) / PI).ceil() as i32 - 1;
                let mut stops: Vec<f32> = (k0..=k1).map(|k| PI / 2.0 + k as f32 * PI).collect();
                if a1 < a0 {
                    stops.reverse();
                }
                let mut t_prev = a0;
                let mut p_prev = a;
                for t in stops.into_iter().chain(std::iter::once(a1)) {
                    let p = if t == a1 {
                        b // exact endpoint, shared with the next seg
                    } else {
                        center + Vec2::new(t.cos(), t.sin()) * radius
                    };
                    if (p_prev.y > pt.y) != (p.y > pt.y) {
                        let dy = pt.y - center.y;
                        let dx = (radius * radius - dy * dy).max(0.0).sqrt();
                        let side = ((t_prev + t) / 2.0).cos();
                        cross(p_prev.y, p.y, center.x + dx.copysign(side));
                    }
                    t_prev = t;
                    p_prev = p;
                }
            }
        }
    }
    inside
}

/// Reverse a loop (order + each segment).
pub fn reverse_loop(segs: &[Seg]) -> Vec<Seg> {
    segs.iter().rev().map(|s| s.reversed()).collect()
}

/// The sketch's first loop, oriented counter-clockwise.
pub fn ccw_segs(s: &Sketch) -> Vec<Seg> {
    let segs = s.loops[0].clone();
    if loop_area(&segs) < 0.0 {
        reverse_loop(&segs)
    } else {
        segs
    }
}
