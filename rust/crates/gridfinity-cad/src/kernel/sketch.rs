use crate::kernel::math::Vec2;
use std::f32::consts::PI;

#[derive(Clone, Copy, Debug, PartialEq)]
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

#[derive(Clone, Debug, Default)]
pub struct Sketch {
    pub loops: Vec<Vec<Seg>>,
}

impl Sketch {
    pub fn single(loop_: Vec<Seg>) -> Sketch {
        Sketch { loops: vec![loop_] }
    }

    pub fn area(&self) -> f32 {
        loop_area(&self.loops[0])
    }

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

    pub fn rounded_rect(cx: f32, cy: f32, w: f32, h: f32, r: f32) -> Sketch {
        let r = r.min(w / 2.0).min(h / 2.0);
        if r <= 1e-4 {
            return Sketch::rectangle(cx, cy, w, h);
        }
        let (hw, hh) = (w / 2.0, h / 2.0);
        let (ix, iy) = (hw - r, hh - r);
        let p = |x, y| Vec2::new(cx + x, cy + y);
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
        let mut segs = vec![
            Seg::Line { a: b_l, b: b_r },
            arc(b_r, r_b, cx + ix, cy - iy, -PI / 2.0, 0.0),
            Seg::Line { a: r_b, b: r_t },
            arc(r_t, t_r, cx + ix, cy + iy, 0.0, PI / 2.0),
            Seg::Line { a: t_r, b: t_l },
            arc(t_l, l_t, cx - ix, cy + iy, PI / 2.0, PI),
            Seg::Line { a: l_t, b: l_b },
            arc(l_b, b_l, cx - ix, cy - iy, PI, 1.5 * PI),
        ];
        segs.retain(|s| !matches!(*s, Seg::Line { a, b } if (b - a).length() < 1e-6));
        Sketch::single(segs)
    }

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

const MAX_ARC_STOPS: usize = 8;

#[derive(Clone, Copy, PartialEq)]
pub struct Aabb {
    pub min: Vec2,
    pub max: Vec2,
}

impl Aabb {
    pub const EMPTY: Aabb = Aabb {
        min: Vec2::new(f32::INFINITY, f32::INFINITY),
        max: Vec2::new(f32::NEG_INFINITY, f32::NEG_INFINITY),
    };

    #[inline]
    pub fn contains(&self, p: Vec2) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }

    #[inline]
    pub fn area(&self) -> f32 {
        ((self.max.x - self.min.x) * (self.max.y - self.min.y)).max(0.0)
    }

    #[inline]
    pub fn union(self, o: Aabb) -> Aabb {
        Aabb {
            min: self.min.min(o.min),
            max: self.max.max(o.max),
        }
    }
}

impl Seg {
    pub fn bbox(&self) -> Aabb {
        match *self {
            Seg::Line { a, b } => Aabb {
                min: a.min(b),
                max: a.max(b),
            },
            Seg::Arc {
                a,
                b,
                center,
                radius,
                a0,
                a1,
            } => {
                let mut bb = Aabb {
                    min: a.min(b),
                    max: a.max(b),
                };
                let (lo, hi) = (a0.min(a1), a0.max(a1));
                let q = PI / 2.0;
                let k0 = (lo / q).ceil() as i32;
                let k1 = (hi / q).floor() as i32;
                for k in k0..=k1 {
                    let t = k as f32 * q;
                    let p = center + Vec2::new(t.cos(), t.sin()) * radius;
                    bb = bb.union(Aabb { min: p, max: p });
                }
                bb
            }
        }
    }
}

pub fn segs_bbox(segs: &[Seg]) -> Aabb {
    segs.iter().fold(Aabb::EMPTY, |acc, s| acc.union(s.bbox()))
}

pub fn point_in_segs(pt: Vec2, segs: &[Seg]) -> bool {
    crate::kernel::perf::count(crate::kernel::perf::Metric::PointInSegs);
    let mut n = 0u32;
    for seg in segs {
        n += seg_crossings(pt, seg);
    }
    n % 2 == 1
}

/// How many times a rightward ray from `pt` crosses `seg`. Parity over a closed
/// loop's segments is the containment test, so a caller holding a spatial index
/// can sum this over candidates instead of walking every segment.
pub fn seg_crossings(pt: Vec2, seg: &Seg) -> u32 {
    let mut hits = 0u32;
    let mut cross = |y0: f32, y1: f32, x: f32| {
        if (y0 > pt.y) != (y1 > pt.y) && x > pt.x {
            hits += 1;
        }
    };
    {
        match *seg {
            Seg::Line { a, b } => {
                if (a.y > pt.y) != (b.y > pt.y) {
                    cross(a.y, b.y, a.x + (pt.y - a.y) / (b.y - a.y) * (b.x - a.x));
                }
            }
            Seg::Arc {
                a,
                b,
                center,
                radius,
                a0,
                a1,
            } => {
                let (lo, hi) = (a0.min(a1), a0.max(a1));
                let k0 = ((lo - PI / 2.0) / PI).floor() as i32 + 1;
                let k1 = ((hi - PI / 2.0) / PI).ceil() as i32 - 1;
                assert!(
                    k1 - k0 < MAX_ARC_STOPS as i32,
                    "arc spans more than 2pi: a0={a0} a1={a1}"
                );
                let mut buf = [0.0f32; MAX_ARC_STOPS];
                let mut n = 0;
                for k in k0..=k1 {
                    if n == MAX_ARC_STOPS {
                        break;
                    }
                    buf[n] = PI / 2.0 + k as f32 * PI;
                    n += 1;
                }
                let stops = &mut buf[..n];
                if a1 < a0 {
                    stops.reverse();
                }
                let mut t_prev = a0;
                let mut p_prev = a;
                for t in stops.iter().copied().chain(std::iter::once(a1)) {
                    let p = if t == a1 {
                        b
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
    hits
}

pub fn reverse_loop(segs: &[Seg]) -> Vec<Seg> {
    segs.iter().rev().map(|s| s.reversed()).collect()
}

pub fn ccw_segs(s: &Sketch) -> Vec<Seg> {
    let segs = s.loops[0].clone();
    if loop_area(&segs) < 0.0 {
        reverse_loop(&segs)
    } else {
        segs
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::build::extrude;

    #[test]
    fn stadium_rounded_rect_extrudes_to_a_valid_solid() {
        let sk = Sketch::rounded_rect(20.0, 20.0, 40.0, 2.4, 1.2);
        assert_eq!(sk.loops[0].len(), 6, "two straight runs + four arcs");
        assert!(
            sk.loops[0]
                .iter()
                .all(|s| (s.end() - s.start()).length() > 1e-6)
        );
        let solid = extrude(&sk, 0.0, 15.0);
        solid.validate().expect("stadium prism must be manifold");
    }

    #[test]
    fn stadium_loop_closes_and_keeps_its_area() {
        let (w, h) = (40.0f32, 2.4f32);
        let sk = Sketch::rounded_rect(0.0, 0.0, w, h, 5.0);
        let segs = &sk.loops[0];
        for i in 0..segs.len() {
            let gap = segs[(i + 1) % segs.len()].start() - segs[i].end();
            assert!(
                gap.length() < 1e-6,
                "loop breaks between seg {i} and the next"
            );
        }
        let r = h / 2.0;
        let want = (w - h) * h + std::f32::consts::PI * r * r;
        assert!(
            (loop_area(segs) - want).abs() < 1e-3,
            "area {}",
            loop_area(segs)
        );
    }

    #[test]
    fn ordinary_rounded_rect_keeps_all_eight_segments() {
        let sk = Sketch::rounded_rect(0.0, 0.0, 40.0, 30.0, 5.0);
        assert_eq!(sk.loops[0].len(), 8);
    }

    fn brute_bbox(s: &Seg) -> Aabb {
        let mut bb = Aabb::EMPTY;
        for i in 0..=2048 {
            let t = i as f32 / 2048.0;
            let p = match *s {
                Seg::Line { a, b } => a + (b - a) * t,
                Seg::Arc {
                    center,
                    radius,
                    a0,
                    a1,
                    ..
                } => {
                    let ang = a0 + (a1 - a0) * t;
                    center + Vec2::new(ang.cos(), ang.sin()) * radius
                }
            };
            bb = bb.union(Aabb { min: p, max: p });
        }
        bb
    }

    /// The bbox must be tight, not merely conservative: `stitch_loops_2d`
    /// orders containment candidates by bbox area, which is only sound when a
    /// containing loop's box really does enclose the contained one's.
    #[test]
    fn arc_bbox_is_tight_against_dense_sampling() {
        let c = Vec2::new(3.0, -2.0);
        let r = 7.0;
        for k in 0..24 {
            let a0 = k as f32 * 0.31 - 3.0;
            for span in [0.4, 1.2, PI, 4.0, 2.0 * PI, -1.2, -4.0] {
                let a1 = a0 + span;
                let seg = Seg::Arc {
                    a: c + Vec2::new(a0.cos(), a0.sin()) * r,
                    b: c + Vec2::new(a1.cos(), a1.sin()) * r,
                    center: c,
                    radius: r,
                    a0,
                    a1,
                };
                let (got, want) = (seg.bbox(), brute_bbox(&seg));
                let tol = 1e-2;
                assert!(
                    got.min.x <= want.min.x + tol
                        && got.min.y <= want.min.y + tol
                        && got.max.x >= want.max.x - tol
                        && got.max.y >= want.max.y - tol,
                    "bbox does not contain arc a0={a0} span={span}"
                );
                assert!(
                    got.min.x >= want.min.x - tol
                        && got.min.y >= want.min.y - tol
                        && got.max.x <= want.max.x + tol
                        && got.max.y <= want.max.y + tol,
                    "bbox is loose for a0={a0} span={span}"
                );
            }
        }
    }

    /// A loop's box encloses any loop inside it, so sorting by area cannot skip
    /// a real container.
    #[test]
    fn containment_implies_bbox_area_ordering() {
        let outer = Sketch::rounded_rect(0.0, 0.0, 40.0, 30.0, 5.0).loops[0].clone();
        let inner = Sketch::circle(10.0, 10.0, 3.0).loops[0].clone();
        let (bo, bi) = (segs_bbox(&outer), segs_bbox(&inner));
        assert!(point_in_segs(inner[0].start(), &outer));
        assert!(bo.area() > bi.area());
        assert!(bo.contains(inner[0].start()));
    }

    /// The arc branch of `point_in_segs` used to heap-allocate its stop list on
    /// every call; the fixed buffer must give identical answers.
    #[test]
    fn point_in_circle_matches_radius_test() {
        let circle = Sketch::circle(1.0, 2.0, 5.0).loops[0].clone();
        for i in -12..12 {
            for j in -12..12 {
                let p = Vec2::new(1.0 + i as f32 * 0.7, 2.0 + j as f32 * 0.7);
                let want = (p - Vec2::new(1.0, 2.0)).length() < 5.0;
                if ((p - Vec2::new(1.0, 2.0)).length() - 5.0).abs() < 1e-3 {
                    continue;
                }
                assert_eq!(point_in_segs(p, &circle), want, "at {p:?}");
            }
        }
    }
}
