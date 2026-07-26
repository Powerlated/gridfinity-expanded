//! "Bad Apple!!" as a live analytic-B-rep stress test.
//!
//! The famous shadow-art music video, pre-decoded to a stack of 1-bit frames
//! (64×48, 30 fps, bit-packed MSB-first — one bit per pixel, set = white
//! silhouette), embedded straight into the binary. Every displayed frame is
//! rebuilt *through the real kernel*: the white pixels are grouped into
//! 4-connected components, and **each contiguous blob becomes one full
//! parametric Gridfinity bin** — a polyomino of grid cells with shared walls,
//! one cavity, chamfered connector pegs and bridge stitching, exactly as if
//! those cells had been painted into a single logical bin. Nothing is cached
//! and nothing is instanced; the whole B-rep pipeline runs from scratch. A
//! frame's triangle count over the wall time it took to build is a live
//! "triangles/second" number for the kernel.
//!
//! Building is expensive enough (a large white region is a large multi-cell
//! bin) that it runs on a [`Worker`] thread: the UI stays responsive and simply
//! shows the newest finished frame, dropping any the kernel was too slow to
//! reach so playback stays locked to the music's timeline.
//!
//! The per-blob bins are independent solids that merely abut; each is
//! watertight on its own, so there is no cross-solid manifold to maintain — we
//! just concatenate their render buffers.

use crate::MESH_STRIDE;
use gridfinity_cad::gridfinity::{self, Params};
use gridfinity_cad::layout::GridCell;
use gridfinity_cad::tessellate;

/// Frame dimensions (must match the `ffmpeg` scale used to build the asset).
pub const W: usize = 64;
pub const H: usize = 48;
/// Source frame rate the asset was sampled at.
pub const FPS: f64 = 30.0;

/// One grid cell (42 mm pitch) per white pixel.
const PITCH: f32 = gridfinity::GRID_PITCH;

const ROW_BYTES: usize = W / 8;
const FRAME_BYTES: usize = ROW_BYTES * H;

/// Bit-packed frames: `frames × (W·H/8)` bytes, MSB-first within each byte.
///
/// The asset is git-ignored (see `.gitignore`); regenerate it from the source
/// video with:
///
/// ```text
/// ffmpeg -y -i <video> \
///   -vf "fps=30,scale=64:48,format=gray,lut=y='if(gt(val,110),255,0)'" \
///   -pix_fmt monob -f rawvideo crates/gridfinity-gui/assets/badapple.raw
/// ```
static FRAMES: &[u8] = include_bytes!("../assets/badapple.raw");

/// Number of frames in the embedded clip.
pub fn frame_count() -> usize {
    FRAMES.len() / FRAME_BYTES
}

/// The overall footprint the plate occupies, for framing the camera.
pub fn bounds() -> ([f32; 3], [f32; 3]) {
    let h = 2.0 * gridfinity::HEIGHT_PER_UNIT;
    ([0.0, 0.0, 0.0], [W as f32 * PITCH, H as f32 * PITCH, h])
}

#[inline]
fn white(frame: &[u8], x: usize, y: usize) -> bool {
    (frame[y * ROW_BYTES + x / 8] >> (7 - (x % 8))) & 1 == 1
}

/// The bin every white pixel is built from: a shallow, plain 1×1 bin. Kept
/// deliberately cheap (two units tall, no floor fillet, no fasteners) so a
/// frame with hundreds of pixels still rebuilds every one from scratch in a
/// fraction of a second.
fn cell_params() -> Params {
    Params {
        height_units: 2,
        floor_fillet: 0.0,
        magnet_holes: false,
        screw_holes: false,
        ..Params::default()
    }
}

/// Label every white pixel into 4-connected components, each a set of grid
/// cells (Y flipped so the image is upright — video rows run top-down). A
/// contiguous blob of pixels becomes **one** multi-cell Gridfinity bin, exactly
/// like painting those cells into a single logical bin in the editor.
fn components(f: &[u8]) -> Vec<Vec<GridCell>> {
    let mut seen = vec![false; W * H];
    let mut out = Vec::new();
    let mut stack = Vec::new();
    for sy in 0..H {
        for sx in 0..W {
            if seen[sy * W + sx] || !white(f, sx, sy) {
                continue;
            }
            let mut cells = Vec::new();
            seen[sy * W + sx] = true;
            stack.push((sx, sy));
            while let Some((x, y)) = stack.pop() {
                cells.push(GridCell { x: x as i32, y: (H - 1 - y) as i32 });
                let mut nb = |nx: usize, ny: usize, stack: &mut Vec<(usize, usize)>| {
                    if !seen[ny * W + nx] && white(f, nx, ny) {
                        seen[ny * W + nx] = true;
                        stack.push((nx, ny));
                    }
                };
                if x > 0 { nb(x - 1, y, &mut stack); }
                if x + 1 < W { nb(x + 1, y, &mut stack); }
                if y > 0 { nb(x, y - 1, &mut stack); }
                if y + 1 < H { nb(x, y + 1, &mut stack); }
            }
            out.push(cells);
        }
    }
    out
}

/// Tessellate a solid and append it to `verts`, returning its triangle count.
fn emit(solid: &gridfinity_cad::kernel::topo::Solid, verts: &mut Vec<f32>) -> usize {
    let src = tessellate(solid, 1).render_buffer();
    for v in src.chunks_exact(6) {
        verts.extend_from_slice(v);
        verts.push(0.0); // never a failed bin
    }
    src.len() / (6 * 3)
}

/// Build one frame as a field of **real Gridfinity bins**: each contiguous
/// group of white pixels is one full parametric bin (shared walls, one cavity,
/// chamfered connector pegs, bridge stitching), run fresh through the kernel.
/// Returns the shaded vertex buffer (`MESH_STRIDE` floats/vertex, unflagged)
/// and the triangle count.
///
/// Nothing is instanced: this is the stress test, so every frame is rebuilt
/// from scratch. The triangle count over the wall time is the kernel's live
/// throughput.
pub fn build_frame(frame: usize) -> (Vec<f32>, usize) {
    let f = &FRAMES[frame * FRAME_BYTES..(frame + 1) * FRAME_BYTES];
    let p = cell_params();
    let mut verts: Vec<f32> = Vec::new();
    let mut tris = 0usize;

    for cells in components(f) {
        match gridfinity::build_piece(&p, &cells, &cells, None) {
            Ok(solid) => tris += emit(&solid, &mut verts),
            // A complex blob (holes, awkward reentrant corners) the model won't
            // build as one bin falls back to a bin per cell, so the frame is
            // never missing pixels.
            Err(_) => {
                for c in &cells {
                    let one = [*c];
                    if let Ok(s) = gridfinity::build_piece(&p, &one, &one, None) {
                        tris += emit(&s, &mut verts);
                    }
                }
            }
        }
    }
    debug_assert_eq!(verts.len() % MESH_STRIDE, 0);
    (verts, tris)
}

// ── Background build worker ───────────────────────────────────────────────
use std::sync::mpsc::{Receiver, Sender, channel};
use std::thread::JoinHandle;
use std::time::Instant;

/// A built frame handed back from the worker thread.
pub struct FrameResult {
    pub frame: usize,
    pub verts: Vec<f32>,
    pub tris: usize,
    pub build_secs: f64,
}

/// Owns a thread that builds requested frames off the UI thread. The UI keeps
/// at most one request in flight and uploads whatever the worker returns, so a
/// frame slower than the display interval simply drops later frames rather than
/// stalling the window.
pub struct Worker {
    req: Option<Sender<usize>>,
    res: Receiver<FrameResult>,
    handle: Option<JoinHandle<()>>,
}

impl Worker {
    pub fn spawn() -> Worker {
        let (req_tx, req_rx) = channel::<usize>();
        let (res_tx, res_rx) = channel::<FrameResult>();
        let handle = std::thread::spawn(move || {
            while let Ok(frame) = req_rx.recv() {
                let t = Instant::now();
                let (verts, tris) = build_frame(frame);
                let build_secs = t.elapsed().as_secs_f64();
                if res_tx.send(FrameResult { frame, verts, tris, build_secs }).is_err() {
                    break;
                }
            }
        });
        Worker { req: Some(req_tx), res: res_rx, handle: Some(handle) }
    }

    /// Queue a frame to build. Cheap; the heavy work happens on the worker.
    pub fn request(&self, frame: usize) {
        if let Some(req) = &self.req {
            let _ = req.send(frame);
        }
    }

    /// Non-blocking: the most recently finished frame, if any is ready.
    pub fn try_recv(&self) -> Option<FrameResult> {
        let mut last = None;
        while let Ok(r) = self.res.try_recv() {
            last = Some(r);
        }
        last
    }
}

impl Drop for Worker {
    fn drop(&mut self) {
        // Close the request channel so the worker's `recv` returns `Err` and the
        // loop exits, then join. It may be mid-build; join waits that out.
        self.req = None;
        if let Some(h) = self.handle.take() {
            let _ = h.join();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn frames_decode_and_build() {
        let n = frame_count();
        assert!(n > 1000, "asset should hold the whole clip, got {n}");
        // A black frame builds nothing; a silhouette frame builds boxes.
        let (v0, t0) = build_frame(0);
        assert_eq!(t0, 0, "frame 0 is a black screen");
        assert!(v0.is_empty());
        let (v, t) = build_frame(n / 2);
        assert!(t > 0 && !v.is_empty(), "mid clip has geometry");
    }

    #[test]
    #[ignore]
    fn bin_cost() {
        use gridfinity_cad::gridfinity::{self, Params};
        use gridfinity_cad::layout::GridCell;
        use gridfinity_cad::tessellate;
        let p = Params { height_units: 2, floor_fillet: 0.0, ..Params::default() };
        let cells = vec![GridCell { x: 0, y: 0 }];
        // warm
        let _ = gridfinity::build_piece(&p, &cells, &cells, None).unwrap();
        let t = Instant::now();
        let iters = 200;
        let mut tris = 0;
        for _ in 0..iters {
            let s = gridfinity::build_piece(&p, &cells, &cells, None).unwrap();
            tris = tessellate(&s, 1).render_buffer().len() / 18;
        }
        let per = t.elapsed().as_secs_f64() / iters as f64;
        println!("one bin: {tris} tris, {:.3} ms build+tess", per * 1e3);
        println!("=> 500 bins/frame = {:.1} ms/frame = {:.1} fps", per * 500.0 * 1e3, 1.0 / (per * 500.0));
    }

    /// Where a frame's time actually goes: per-metric table over a sample of
    /// frames, plus the blob-size distribution that drives it.
    #[test]
    #[ignore]
    fn profile() {
        use gridfinity_cad::kernel::perf;
        // The first 10 seconds at the source rate: a real playback window, so
        // "ms/frame" here is directly the number that has to reach 33.3.
        let sample: Vec<usize> = (0..(10.0 * FPS) as usize).collect();

        // Blob distribution first (uninstrumented).
        let (mut blobs, mut cells, mut biggest) = (0usize, 0usize, 0usize);
        for &f in &sample {
            for c in components(&FRAMES[f * FRAME_BYTES..(f + 1) * FRAME_BYTES]) {
                blobs += 1;
                cells += c.len();
                biggest = biggest.max(c.len());
            }
        }
        println!(
            "{} frames: {blobs} blobs ({:.1}/frame), {cells} cells ({:.1}/frame), biggest blob {biggest} cells",
            sample.len(),
            blobs as f64 / sample.len() as f64,
            cells as f64 / sample.len() as f64,
        );

        for &f in &sample {
            let _ = build_frame(f);
        } // warm

        perf::set_enabled(true);
        perf::reset();
        let t = Instant::now();
        let mut tris = 0;
        let mut each: Vec<f64> = Vec::new();
        for &f in &sample {
            let ft = Instant::now();
            tris += build_frame(f).1;
            each.push(ft.elapsed().as_secs_f64() * 1e3);
        }
        let secs = t.elapsed().as_secs_f64();
        let mut sorted = each.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let pct = |p: f64| sorted[((sorted.len() - 1) as f64 * p) as usize];
        println!(
            "\nper-frame ms: p50 {:.1}  p90 {:.1}  p99 {:.1}  max {:.1}  ({} of {} frames over 33.3)",
            pct(0.50),
            pct(0.90),
            pct(0.99),
            sorted[sorted.len() - 1],
            each.iter().filter(|&&m| m > 33.3).count(),
            each.len(),
        );
        let rows = perf::snapshot();
        let allocs = perf::allocs();
        perf::set_enabled(false);

        println!(
            "\n{:.1} ms/frame, {tris} tris ({:.0}/frame), {:.2}M tri/s",
            secs / sample.len() as f64 * 1e3,
            tris as f64 / sample.len() as f64,
            tris as f64 / secs / 1e6
        );
        println!("{:<28} {:>10} {:>9} {:>6}  {:>10} {:>9}", "metric", "calls", "ms", "%", "allocs", "MB");
        for r in &rows {
            println!(
                "{:<28} {:>10} {:>9.1} {:>5.1}%  {:>10} {:>9.1}",
                r.name,
                r.calls,
                r.nanos as f64 / 1e6,
                r.nanos as f64 / 1e9 / secs * 100.0,
                r.alloc_calls,
                r.alloc_bytes as f64 / 1e6,
            );
        }
        println!(
            "total allocs {} ({:.1} MB churn, {:.1} MB peak)",
            allocs.count,
            allocs.bytes as f64 / 1e6,
            allocs.peak_live_bytes as f64 / 1e6
        );
    }

    /// Not a gate — prints the live throughput the GUI will show.
    #[test]
    #[ignore]
    fn throughput() {
        let n = frame_count();
        let (mut tris, mut secs) = (0usize, 0.0f64);
        for frame in (0..n).step_by(37) {
            let t = Instant::now();
            let (_, tc) = build_frame(frame);
            secs += t.elapsed().as_secs_f64();
            tris += tc;
        }
        println!(
            "built {tris} triangles in {secs:.3}s = {:.2}M tri/s",
            tris as f64 / secs / 1e6
        );
    }
}
