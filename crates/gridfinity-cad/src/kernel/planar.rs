//! Triangulation of a planar face: loops of 2D points in, triangles out.
//!
//! This is the one place the kernel turns exact geometry into triangles, and it
//! is a monotone decomposition rather than ear clipping. The difference that
//! matters is not speed but what the output guarantees: every boundary vertex
//! the caller supplies appears in the result, holes are handled by the sweep
//! itself, and each interior edge is shared by exactly two triangles. An ear
//! clipper gives none of those -- it drops collinear boundary vertices, so the
//! neighbouring face's samples no longer line up and the chords have to be
//! fanned back in afterwards, and it eliminates holes by rescanning the merged
//! ring per hole, which is quadratic.
//!
//! The algorithm is the textbook one (de Berg et al., chapter 3): sweep top to
//! bottom, add diagonals at split and merge vertices to cut the region into
//! y-monotone pieces, then triangulate each piece off a stack.
//!
//! Degeneracy is the whole difficulty here, because bin cavity floors are
//! rectilinear and full of exactly-equal coordinates. Two rules keep it sound.
//! The sweep order is lexicographic -- decreasing y, then increasing x -- which
//! is exactly an infinitesimal shear, so no two distinct vertices tie and no
//! edge is horizontal in the sweep's frame. And the orientation predicate is
//! shear invariant (a shear preserves signed area), so the same integer-clean
//! cross product decides left-of-edge before and after; only genuinely
//! collinear cases need the along-the-line tie-break.

use crate::kernel::math::Vec2;

/// Sweep order: strictly decreasing y, ties broken by increasing x.
#[inline]
fn above(a: Vec2, b: Vec2) -> bool {
    a.y > b.y || (a.y == b.y && a.x < b.x)
}

#[inline]
fn cross(o: Vec2, a: Vec2, b: Vec2) -> f64 {
    (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x)
}

/// Which of `atan2`'s four pieces a direction falls in, so that comparing
/// sectors first and then a single cross product reproduces `atan2` order
/// exactly. `atan2` returns 0 along +x, rises to π along -x through +y, and
/// runs -π to 0 through -y.
#[inline]
fn sector(d: Vec2) -> u8 {
    if d.y < 0.0 {
        0
    } else if d.y > 0.0 {
        2
    } else if d.x > 0.0 {
        1
    } else {
        3
    }
}

/// Order two directions by the angle `atan2` would give them, without going
/// through `atan2`.
///
/// The angle itself cannot be trusted here. Two directions that differ by a
/// part in 10^8 -- a diagonal running the length of a face, a hair off a
/// boundary edge it passes over -- round to the *same* f64 angle, and the sort
/// then orders them arbitrarily, which sends `next_in_face` into a neighbouring
/// face and the triangulation covers parts of the region twice. Within one
/// half-plane the cross product answers the same question and is exact to the
/// last bit the inputs justify: the terms it subtracts are the products that
/// went into them, not the output of a transcendental function.
#[inline]
fn angular_cmp(a: Vec2, b: Vec2) -> std::cmp::Ordering {
    let (sa, sb) = (sector(a), sector(b));
    if sa != sb {
        return sa.cmp(&sb);
    }
    // Same sector, so the two span less than half a turn and the sign of the
    // cross product is the order: positive means `b` is counter-clockwise of
    // `a`, which is the larger angle.
    (a.x * b.y - a.y * b.x).total_cmp(&0.0).reverse()
}

/// What the stack in `monotone` holds. It is seeded with two vertices and every
/// pop is guarded by a length test, so a miss means the walk left the chain it
/// was following, not that the polygon was too small.
const STACK: &str = "the monotone stack keeps the chain walked so far";

/// Signed area of `p[s..e]` traversed in order; positive is counter-clockwise.
pub fn span_ccw(p: &[Vec2], s: usize, e: usize) -> bool {
    let mut a = 0.0f64;
    for i in s..e {
        let (u, v) = (p[i], p[if i + 1 == e { s } else { i + 1 }]);
        a += u.x * v.y - v.x * u.y;
    }
    a > 0.0
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Start,
    End,
    Split,
    Merge,
    Regular,
}

#[derive(Default)]
pub struct Planar {
    pt: Vec<Vec2>,
    orig: Vec<u32>,
    nxt: Vec<u32>,
    prv: Vec<u32>,
    order: Vec<u32>,
    kind: Vec<Kind>,
    status: Vec<u32>,
    helper: Vec<u32>,
    diagonals: Vec<(u32, u32)>,

    he_from: Vec<u32>,
    he_to: Vec<u32>,
    out_off: Vec<u32>,
    out_he: Vec<u32>,
    he_pos: Vec<u32>,
    used: Vec<bool>,
    face: Vec<u32>,
    seq: Vec<u32>,
    left: Vec<bool>,
    stack: Vec<usize>,
}

impl Planar {
    /// Triangulate the region bounded by `spans` (the first is the outer
    /// boundary, the rest are holes) over the points in `uv`, appending
    /// triangles as triples of `uv` indices. Returns false if the input is
    /// degenerate enough that no triangulation was produced.
    pub fn run(
        &mut self,
        uv: &[Vec2],
        spans: &[(usize, usize)],
        out: &mut Vec<[usize; 3]>,
    ) -> bool {
        if spans.is_empty() || spans[0].1 - spans[0].0 < 3 {
            return false;
        }
        self.load(uv, spans);
        if self.pt.len() < 3 {
            return false;
        }
        self.sweep();
        self.dedup_diagonals();
        self.build_half_edges();
        self.emit(out)
    }

    fn load(&mut self, uv: &[Vec2], spans: &[(usize, usize)]) {
        self.pt.clear();
        self.orig.clear();
        self.nxt.clear();
        self.prv.clear();
        for (si, &(s, e)) in spans.iter().enumerate() {
            if e - s < 3 {
                continue;
            }
            // Outer counter-clockwise, holes clockwise, so the material is on
            // the left of every directed boundary edge.
            let fwd = span_ccw(uv, s, e) == (si == 0);
            let first = self.pt.len() as u32;
            for k in 0..e - s {
                let i = if fwd { s + k } else { e - 1 - k };
                self.pt.push(uv[i]);
                self.orig.push(i as u32);
            }
            let last = self.pt.len() as u32 - 1;
            for n in first..=last {
                self.nxt.push(if n == last { first } else { n + 1 });
                self.prv.push(if n == first { last } else { n - 1 });
            }
        }
    }

    fn sweep(&mut self) {
        let n = self.pt.len();
        self.kind.clear();
        self.kind.resize(n, Kind::Regular);
        for v in 0..n {
            let (p, q) = (self.prv[v] as usize, self.nxt[v] as usize);
            let (pa, qa) = (above(self.pt[p], self.pt[v]), above(self.pt[q], self.pt[v]));
            let reflex = cross(self.pt[p], self.pt[v], self.pt[q]) < 0.0;
            self.kind[v] = match (pa, qa) {
                (false, false) if reflex => Kind::Split,
                (false, false) => Kind::Start,
                (true, true) if reflex => Kind::Merge,
                (true, true) => Kind::End,
                _ => Kind::Regular,
            };
        }

        self.order.clear();
        self.order.extend(0..n as u32);
        let pt = &self.pt;
        self.order.sort_unstable_by(|&a, &b| {
            let (u, v) = (pt[a as usize], pt[b as usize]);
            v.y.total_cmp(&u.y).then(u.x.total_cmp(&v.x))
        });

        self.status.clear();
        self.helper.clear();
        self.helper.resize(n, u32::MAX);
        self.diagonals.clear();

        for i in 0..self.order.len() {
            let v = self.order[i];
            let p = self.prv[v as usize];
            match self.kind[v as usize] {
                Kind::Start => {
                    self.insert(v);
                    self.helper[v as usize] = v;
                }
                Kind::End => {
                    self.close(p, v);
                    self.remove(p);
                }
                Kind::Split => {
                    if let Some(j) = self.left_of(v) {
                        self.diagonals.push((v, self.helper[j as usize]));
                        self.helper[j as usize] = v;
                    }
                    self.insert(v);
                    self.helper[v as usize] = v;
                }
                Kind::Merge => {
                    self.close(p, v);
                    self.remove(p);
                    if let Some(j) = self.left_of(v) {
                        self.close(j, v);
                        self.helper[j as usize] = v;
                    }
                }
                Kind::Regular => {
                    if above(self.pt[p as usize], self.pt[v as usize]) {
                        self.close(p, v);
                        self.remove(p);
                        self.insert(v);
                        self.helper[v as usize] = v;
                    } else if let Some(j) = self.left_of(v) {
                        self.close(j, v);
                        self.helper[j as usize] = v;
                    }
                }
            }
        }
    }

    /// Add the diagonal that resolves edge `e`'s helper if it was a merge
    /// vertex, which is the only case that leaves the region non-monotone.
    fn close(&mut self, e: u32, v: u32) {
        let h = self.helper[e as usize];
        if h != u32::MAX && self.kind[h as usize] == Kind::Merge {
            self.diagonals.push((v, h));
        }
    }

    /// Is `q` to the right of the downward edge leaving `e`? Ties (q exactly on
    /// the edge's line) fall back to position along it, which is what the
    /// sheared frame would compare.
    fn right_of(&self, e: u32, q: Vec2) -> bool {
        let (a, b) = (self.pt[e as usize], self.pt[self.nxt[e as usize] as usize]);
        if a.y == b.y {
            // Sheared, this edge occupies no sweep height at all, so it is only
            // ever compared against points on its own line; order by x past its
            // lower (larger-x) end.
            return q.x > b.x;
        }
        let c = cross(a, b, q);
        if c != 0.0 {
            // q left of the downward edge means q sits at greater x, so the
            // edge is the one on the left.
            return c > 0.0;
        }
        above(b, q)
    }

    fn insert(&mut self, v: u32) {
        let q = self.pt[v as usize];
        let at = self.status.partition_point(|&e| self.right_of(e, q));
        self.status.insert(at, v);
    }

    fn remove(&mut self, e: u32) {
        if let Some(i) = self.status.iter().position(|&x| x == e) {
            self.status.remove(i);
        }
    }

    /// The status edge immediately left of `v`.
    fn left_of(&self, v: u32) -> Option<u32> {
        let q = self.pt[v as usize];
        let at = self.status.partition_point(|&e| self.right_of(e, q));
        at.checked_sub(1).map(|i| self.status[i])
    }

    /// A vertex can be the helper resolved from two different edges, which asks
    /// for the same diagonal twice; emitting it twice would give the piece
    /// either side of it a doubled boundary.
    fn dedup_diagonals(&mut self) {
        let (nxt, prv) = (&self.nxt, &self.prv);
        self.diagonals
            .retain(|&(a, b)| a != b && nxt[a as usize] != b && prv[a as usize] != b);
        for d in &mut self.diagonals {
            if d.0 > d.1 {
                *d = (d.1, d.0);
            }
        }
        self.diagonals.sort_unstable();
        self.diagonals.dedup();
    }

    /// Every edge gets both directions, paired as `h` and `h ^ 1`. The material
    /// side is the even half of each boundary pair and both halves of each
    /// diagonal; the odd boundary halves exist only so that "the edge we came
    /// in on" can be named by index rather than looked up by angle, which is
    /// what a diagonal lying along a boundary edge would break.
    fn build_half_edges(&mut self) {
        let n = self.pt.len();
        self.he_from.clear();
        self.he_to.clear();
        for v in 0..n as u32 {
            self.he_from.push(v);
            self.he_to.push(self.nxt[v as usize]);
            self.he_from.push(self.nxt[v as usize]);
            self.he_to.push(v);
        }
        for k in 0..self.diagonals.len() {
            let (a, b) = self.diagonals[k];
            self.he_from.push(a);
            self.he_to.push(b);
            self.he_from.push(b);
            self.he_to.push(a);
        }

        let m = self.he_from.len();
        let mut count = vec![0u32; n + 1];
        for &f in &self.he_from {
            count[f as usize + 1] += 1;
        }
        for i in 0..n {
            count[i + 1] += count[i];
        }
        self.out_off = count;
        self.out_he.clear();
        self.out_he.resize(m, 0);
        let mut cursor = self.out_off[..n].to_vec();
        for h in 0..m as u32 {
            let f = self.he_from[h as usize] as usize;
            self.out_he[cursor[f] as usize] = h;
            cursor[f] += 1;
        }
        let (pt, he_from, he_to) = (&self.pt, &self.he_from, &self.he_to);
        let dir =
            |h: u32| -> Vec2 { pt[he_to[h as usize] as usize] - pt[he_from[h as usize] as usize] };
        for v in 0..n {
            let (s, e) = (self.out_off[v] as usize, self.out_off[v + 1] as usize);
            self.out_he[s..e].sort_unstable_by(|&a, &b| angular_cmp(dir(a), dir(b)));
            // Two half-edges leaving one vertex in exactly the same direction
            // have no angular order, so `next_in_face` would pick between them
            // arbitrarily and the walk could leave the face it is tracing. It
            // means a diagonal was laid along a boundary edge -- geometry the
            // sweep must not produce, not a case to break the tie on.
            for w in self.out_he[s..e].windows(2) {
                let (d0, d1) = (dir(w[0]), dir(w[1]));
                assert!(
                    d0.x * d1.y - d0.y * d1.x != 0.0 || d0.dot(d1) <= 0.0,
                    "half-edges {} and {} leave vertex {v} ({:?}) in the same direction: {d0:?}, {d1:?}",
                    w[0],
                    w[1],
                    pt[v]
                );
            }
        }
        self.he_pos.clear();
        self.he_pos.resize(m, 0);
        for (slot, &h) in self.out_he.iter().enumerate() {
            self.he_pos[h as usize] = slot as u32;
        }
    }

    /// Half-edges on the material side: the even half of each boundary pair,
    /// and both halves of every diagonal.
    #[inline]
    fn is_material(&self, h: u32) -> bool {
        h as usize >= 2 * self.pt.len() || h % 2 == 0
    }

    /// Walk each monotone piece and triangulate it.
    fn emit(&mut self, out: &mut Vec<[usize; 3]>) -> bool {
        let m = self.he_from.len();
        self.used.clear();
        self.used.resize(m, false);
        let before = out.len();
        for h0 in 0..m as u32 {
            if self.used[h0 as usize] || !self.is_material(h0) {
                continue;
            }
            self.face.clear();
            let mut h = h0;
            loop {
                if self.used[h as usize] {
                    return false;
                }
                self.used[h as usize] = true;
                self.face.push(self.he_from[h as usize]);
                h = self.next_in_face(h);
                if h == h0 {
                    break;
                }
                if self.face.len() > m {
                    return false;
                }
            }
            let face = std::mem::take(&mut self.face);
            self.monotone(&face, out);
            self.face = face;
        }
        out.len() > before
    }

    /// The half-edge continuing the face on the left of `h`: at `h`'s head, the
    /// outgoing edge one step clockwise from `h`'s twin.
    #[inline]
    fn next_in_face(&self, h: u32) -> u32 {
        let twin = h ^ 1;
        let v = self.he_from[twin as usize] as usize;
        let (s, e) = (self.out_off[v] as usize, self.out_off[v + 1] as usize);
        let slot = self.he_pos[twin as usize] as usize;
        self.out_he[if slot == s { e - 1 } else { slot - 1 }]
    }

    fn monotone(&mut self, face: &[u32], out: &mut Vec<[usize; 3]>) {
        let n = face.len();
        if n < 3 {
            return;
        }
        let o = |v: u32| self.orig[v as usize] as usize;
        if n == 3 {
            out.push([o(face[0]), o(face[1]), o(face[2])]);
            return;
        }

        let pt = &self.pt;
        let top = (0..n).max_by(|&a, &b| {
            if above(pt[face[a] as usize], pt[face[b] as usize]) {
                std::cmp::Ordering::Greater
            } else {
                std::cmp::Ordering::Less
            }
        });
        let Some(top) = top else { return };

        // Walking forward from the top vertex descends the left chain.
        self.seq.clear();
        self.left.clear();
        let (mut i, mut j) = (top, top);
        let step_fwd = |k: usize| (k + 1) % n;
        let step_bwd = |k: usize| (k + n - 1) % n;
        self.seq.push(face[top]);
        self.left.push(true);
        i = step_fwd(i);
        j = step_bwd(j);
        while self.seq.len() < n {
            let (a, b) = (face[i], face[j]);
            if i == j {
                self.seq.push(a);
                self.left.push(true);
                break;
            }
            if above(pt[a as usize], pt[b as usize]) {
                self.seq.push(a);
                self.left.push(true);
                i = step_fwd(i);
            } else {
                self.seq.push(b);
                self.left.push(false);
                j = step_bwd(j);
            }
        }
        if self.seq.len() != n {
            return;
        }

        let seq = std::mem::take(&mut self.seq);
        let left = std::mem::take(&mut self.left);
        let mut st = std::mem::take(&mut self.stack);
        st.clear();
        st.push(0);
        st.push(1);
        for k in 2..n - 1 {
            let top_of = *st.last().expect(STACK);
            if left[k] != left[top_of] {
                while st.len() > 1 {
                    let a = st.pop().expect(STACK);
                    let b = *st.last().expect(STACK);
                    let t = if left[a] {
                        [seq[k], seq[b], seq[a]]
                    } else {
                        [seq[k], seq[a], seq[b]]
                    };
                    out.push([o(t[0]), o(t[1]), o(t[2])]);
                }
                st.pop();
                st.push(k - 1);
                st.push(k);
            } else {
                let mut last = st.pop().expect(STACK);
                while let Some(&t) = st.last() {
                    let c = cross(
                        pt[seq[k] as usize],
                        pt[seq[last] as usize],
                        pt[seq[t] as usize],
                    );
                    let ok = if left[k] { c < 0.0 } else { c > 0.0 };
                    if !ok {
                        break;
                    }
                    let tri = if left[last] {
                        [seq[k], seq[t], seq[last]]
                    } else {
                        [seq[k], seq[last], seq[t]]
                    };
                    out.push([o(tri[0]), o(tri[1]), o(tri[2])]);
                    last = st.pop().expect(STACK);
                }
                st.push(last);
                st.push(k);
            }
        }
        let k = n - 1;
        while st.len() > 1 {
            let a = st.pop().expect(STACK);
            let b = *st.last().expect(STACK);
            let t = if left[a] {
                [seq[k], seq[b], seq[a]]
            } else {
                [seq[k], seq[a], seq[b]]
            };
            out.push([o(t[0]), o(t[1]), o(t[2])]);
        }

        self.seq = seq;
        self.left = left;
        self.stack = st;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::kernel::hash::FxHashMap;

    /// The three properties a face triangulation has to have for the mesh
    /// around it to stay closed: it covers the region exactly (equal signed
    /// area), every interior edge is shared by two triangles while every
    /// boundary edge is used once in the boundary's own direction, and no
    /// supplied vertex is dropped (a dropped one would leave the neighbouring
    /// face emitting a sample this face never meets).
    fn check(uv: &[Vec2], spans: &[(usize, usize)]) -> Vec<[usize; 3]> {
        let mut tris = Vec::new();
        let mut p = Planar::default();
        assert!(
            p.run(uv, spans, &mut tris),
            "triangulation produced nothing"
        );

        let mut want = 0.0f64;
        let mut boundary: FxHashMap<(usize, usize), i32> = FxHashMap::default();
        for (si, &(s, e)) in spans.iter().enumerate() {
            let mut a = 0.0f64;
            for i in s..e {
                let j = if i + 1 == e { s } else { i + 1 };
                a += uv[i].x as f64 * uv[j].y as f64 - uv[j].x as f64 * uv[i].y as f64;
            }
            // Outer counter-clockwise, holes clockwise, whatever came in.
            let fwd = (a > 0.0) == (si == 0);
            for i in s..e {
                let j = if i + 1 == e { s } else { i + 1 };
                let (u, v) = if fwd { (i, j) } else { (j, i) };
                *boundary.entry((u, v)).or_default() += 1;
            }
            want += if fwd { a } else { -a };
        }
        want /= 2.0;

        let mut got = 0.0f64;
        let mut edges: FxHashMap<(usize, usize), i32> = FxHashMap::default();
        let mut seen = vec![false; uv.len()];
        for &[a, b, c] in &tris {
            got += (uv[b].x as f64 - uv[a].x as f64) * (uv[c].y as f64 - uv[a].y as f64)
                - (uv[b].y as f64 - uv[a].y as f64) * (uv[c].x as f64 - uv[a].x as f64);
            for &(u, v) in &[(a, b), (b, c), (c, a)] {
                *edges.entry((u, v)).or_default() += 1;
                seen[u] = true;
            }
        }
        got /= 2.0;
        assert!(
            (got - want).abs() <= 1e-3 * want.abs().max(1.0),
            "area {got} != {want}"
        );

        for (&(u, v), &n) in &edges {
            let rev = edges.get(&(v, u)).copied().unwrap_or(0);
            if boundary.contains_key(&(u, v)) {
                assert_eq!(
                    (n, rev),
                    (1, 0),
                    "boundary edge {u}->{v} used {n}x, {rev}x back"
                );
            } else {
                assert_eq!(
                    (n, rev),
                    (1, 1),
                    "interior edge {u}->{v} used {n}x, {rev}x back"
                );
            }
        }
        for (&(u, v), _) in &boundary {
            assert_eq!(
                edges.get(&(u, v)).copied().unwrap_or(0),
                1,
                "boundary {u}->{v} missing"
            );
        }
        for &(s, e) in spans {
            for (i, ok) in seen.iter().enumerate().take(e).skip(s) {
                assert!(*ok, "vertex {i} dropped");
            }
        }
        tris
    }

    fn ring(out: &mut Vec<Vec2>, pts: &[(f64, f64)]) -> (usize, usize) {
        let s = out.len();
        out.extend(pts.iter().map(|&(x, y)| Vec2::new(x, y)));
        (s, out.len())
    }

    #[test]
    fn square() {
        let mut uv = Vec::new();
        let a = ring(
            &mut uv,
            &[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)],
        );
        assert_eq!(check(&uv, &[a]).len(), 2);
    }

    #[test]
    fn square_with_a_square_hole() {
        let mut uv = Vec::new();
        let a = ring(
            &mut uv,
            &[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)],
        );
        let h = ring(&mut uv, &[(3.0, 3.0), (3.0, 7.0), (7.0, 7.0), (7.0, 3.0)]);
        check(&uv, &[a, h]);
    }

    #[test]
    fn many_holes_on_a_lattice() {
        let mut uv = Vec::new();
        let mut spans = vec![ring(
            &mut uv,
            &[(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)],
        )];
        for i in 0..6 {
            for j in 0..6 {
                let (x, y) = (5.0 + i as f64 * 15.0, 5.0 + j as f64 * 15.0);
                spans.push(ring(
                    &mut uv,
                    &[(x, y), (x, y + 10.0), (x + 10.0, y + 10.0), (x + 10.0, y)],
                ));
            }
        }
        check(&uv, &spans);
    }

    /// The rim of a 1x3 bin cut into four compartments by two tidy inner
    /// walls, the vertical one on the bin's exact centre line. Verbatim
    /// `tess.rs` output -- one outer loop and four rounded-rectangle holes,
    /// all simple, disjoint and properly nested. In this face's uv frame the
    /// sweep runs across the bin's 42 mm width, so the two holes of a column
    /// begin and end at exactly the same sweep height.
    const FOUR_COMPARTMENT_RIM: [&[(f64, f64)]; 5] = [
        &[
            (-0.25, 4.0),
            (-0.25, 38.0),
            (-0.3777783, 38.970573),
            (-0.7524047, 39.875),
            (-1.3483496, 40.65165),
            (-2.125, 41.247597),
            (-3.0294285, 41.622223),
            (-4.0, 41.75),
            (-38.0, 41.75),
            (-46.0, 41.75),
            (-80.0, 41.75),
            (-88.0, 41.75),
            (-122.0, 41.75),
            (-122.97057, 41.622223),
            (-123.875, 41.247597),
            (-124.65165, 40.65165),
            (-125.2476, 39.875),
            (-125.62222, 38.970573),
            (-125.75, 38.0),
            (-125.75, 4.0),
            (-125.62222, 3.029428),
            (-125.2476, 2.1249998),
            (-124.65165, 1.3483496),
            (-123.875, 0.7524047),
            (-122.97057, 0.3777783),
            (-122.0, 0.25),
            (-88.0, 0.25),
            (-80.0, 0.25),
            (-46.0, 0.25),
            (-38.0, 0.25),
            (-4.0, 0.25),
            (-3.0294285, 0.3777783),
            (-2.1249993, 0.75240517),
            (-1.3483491, 1.34835),
            (-0.7524047, 2.1250005),
            (-0.37777805, 3.029429),
        ],
        &[
            (-1.45, 17.100002),
            (-1.45, 4.0),
            (-1.5368888, 3.3400116),
            (-1.7916348, 2.7250004),
            (-2.1968772, 2.196878),
            (-2.7249994, 1.7916355),
            (-3.3400111, 1.5368893),
            (-3.9999998, 1.45),
            (-80.3, 1.45),
            (-80.94705, 1.5351856),
            (-81.55, 1.7849367),
            (-82.06777, 2.182233),
            (-82.465065, 2.6999998),
            (-82.71482, 3.3029523),
            (-82.8, 3.95),
            (-82.8, 17.1),
            (-82.71482, 17.747047),
            (-82.465065, 18.35),
            (-82.06777, 18.867767),
            (-81.55, 19.265064),
            (-80.94705, 19.514814),
            (-80.3, 19.6),
            (-3.95, 19.600002),
            (-3.3029523, 19.514816),
            (-2.7, 19.265066),
            (-2.182233, 18.86777),
            (-1.7849364, 18.350002),
            (-1.5351856, 17.74705),
        ],
        &[
            (-85.2, 17.1),
            (-85.2, 3.95),
            (-85.28518, 3.3029525),
            (-85.534935, 2.7000003),
            (-85.93223, 2.1822333),
            (-86.45, 1.7849369),
            (-87.05295, 1.5351856),
            (-87.7, 1.45),
            (-122.0, 1.45),
            (-122.65999, 1.5368893),
            (-123.275, 1.7916353),
            (-123.80312, 2.1968777),
            (-124.20837, 2.725),
            (-124.46311, 3.3400111),
            (-124.55, 4.0),
            (-124.55, 17.1),
            (-124.46482, 17.747047),
            (-124.215065, 18.35),
            (-123.81777, 18.867767),
            (-123.3, 19.265064),
            (-122.69705, 19.514814),
            (-122.05, 19.6),
            (-87.7, 19.6),
            (-87.05295, 19.514814),
            (-86.45, 19.265064),
            (-85.93223, 18.867767),
            (-85.534935, 18.35),
            (-85.28518, 17.747047),
        ],
        &[
            (-124.55, 24.899998),
            (-124.55, 38.0),
            (-124.46311, 38.65999),
            (-124.20837, 39.275),
            (-123.80312, 39.803123),
            (-123.275, 40.208366),
            (-122.65999, 40.46311),
            (-122.0, 40.55),
            (-87.7, 40.55),
            (-87.05295, 40.464813),
            (-86.45, 40.21506),
            (-85.93223, 39.817764),
            (-85.534935, 39.3),
            (-85.28518, 38.69705),
            (-85.2, 38.05),
            (-85.2, 24.9),
            (-85.28518, 24.252953),
            (-85.534935, 23.65),
            (-85.93223, 23.132233),
            (-86.45, 22.734936),
            (-87.05295, 22.485186),
            (-87.7, 22.4),
            (-122.05, 22.399998),
            (-122.69705, 22.485184),
            (-123.3, 22.734934),
            (-123.81777, 23.13223),
            (-124.215065, 23.649998),
            (-124.46482, 24.25295),
        ],
        &[
            (-82.8, 24.9),
            (-82.8, 38.05),
            (-82.71482, 38.69705),
            (-82.465065, 39.3),
            (-82.06777, 39.817764),
            (-81.55, 40.21506),
            (-80.94705, 40.464813),
            (-80.3, 40.55),
            (-3.9999998, 40.55),
            (-3.3400111, 40.46311),
            (-2.725, 40.208366),
            (-2.1968775, 39.803123),
            (-1.7916348, 39.275),
            (-1.5368891, 38.65999),
            (-1.45, 38.0),
            (-1.45, 24.9),
            (-1.5351853, 24.252953),
            (-1.7849364, 23.65),
            (-2.1822329, 23.132233),
            (-2.6999996, 22.734936),
            (-3.3029523, 22.485186),
            (-3.95, 22.4),
            (-80.3, 22.4),
            (-80.94705, 22.485186),
            (-81.55, 22.734936),
            (-82.06777, 23.132233),
            (-82.465065, 23.65),
            (-82.71482, 24.252953),
        ],
    ];

    #[test]
    fn four_compartments_either_side_of_the_centre_line() {
        let mut p = Vec::new();
        let spans: Vec<_> = FOUR_COMPARTMENT_RIM
            .iter()
            .map(|l| ring(&mut p, l))
            .collect();
        check(&p, &spans);
    }

    /// Reentrant corners of a staircase cavity are exactly collinear, which is
    /// what made ear clipping emit a zero-area triangle spanning them.
    #[test]
    fn collinear_reentrant_staircase() {
        // Outline of the polyomino (0,0),(1,0),(1,1),(2,1),(2,2),(3,2),(3,3):
        // the reentrant corners (10,10), (20,20) and (30,30) are exactly
        // collinear, which is the shape that made ear clipping emit a zero-area
        // triangle spanning all three.
        let mut uv = Vec::new();
        let a = ring(
            &mut uv,
            &[
                (0.0, 0.0),
                (20.0, 0.0),
                (20.0, 10.0),
                (30.0, 10.0),
                (30.0, 20.0),
                (40.0, 20.0),
                (40.0, 40.0),
                (30.0, 40.0),
                (30.0, 30.0),
                (20.0, 30.0),
                (20.0, 20.0),
                (10.0, 20.0),
                (10.0, 10.0),
                (0.0, 10.0),
            ],
        );
        check(&uv, &[a]);
    }

    /// Rectilinear input shares y coordinates everywhere, which is the sweep's
    /// hard case.
    #[test]
    fn comb_with_shared_y_coordinates() {
        let mut pts: Vec<(f64, f64)> = vec![(0.0, 0.0)];
        for k in 0..8 {
            let x = k as f64 * 10.0;
            pts.push((x + 2.0, 0.0));
            pts.push((x + 2.0, 20.0));
            pts.push((x + 8.0, 20.0));
            pts.push((x + 8.0, 0.0));
        }
        pts.push((80.0, 0.0));
        pts.push((80.0, -5.0));
        pts.push((0.0, -5.0));
        let mut uv = Vec::new();
        let a = ring(&mut uv, &pts);
        check(&uv, &[a]);
    }

    #[test]
    fn hole_orientation_is_normalised_either_way() {
        let mut uv = Vec::new();
        let a = ring(
            &mut uv,
            &[(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0)],
        );
        let h = ring(&mut uv, &[(3.0, 3.0), (7.0, 3.0), (7.0, 7.0), (3.0, 7.0)]);
        check(&uv, &[a, h]);
    }

    #[test]
    fn random_star_polygons() {
        let mut seed = 0x2545_f491_4f6c_dd1du64;
        let mut rnd = || {
            seed ^= seed << 13;
            seed ^= seed >> 7;
            seed ^= seed << 17;
            (seed >> 11) as f64 / (1u64 << 53) as f64
        };
        for case in 0..200 {
            let n = 5 + (case % 20);
            let pts: Vec<(f64, f64)> = (0..n)
                .map(|k| {
                    let a = k as f64 / n as f64 * std::f64::consts::TAU;
                    let r = 5.0 + 15.0 * rnd();
                    (r * a.cos(), r * a.sin())
                })
                .collect();
            let mut uv = Vec::new();
            let s = ring(&mut uv, &pts);
            check(&uv, &[s]);
        }
    }
}
