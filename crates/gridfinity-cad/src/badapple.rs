use crate::gridfinity::{self, Params};
use crate::layout::GridCell;
use std::io::Read;
use std::sync::OnceLock;

pub const W: usize = 64;
pub const H: usize = 48;
pub const FPS: f64 = 30.0;

const ROW_BYTES: usize = W / 8;
pub const FRAME_BYTES: usize = ROW_BYTES * H;

static COMPRESSED: &[u8] = include_bytes!("../assets/badapple.raw.gz");
static FRAMES: OnceLock<Vec<u8>> = OnceLock::new();

pub fn frames() -> &'static [u8] {
    FRAMES.get_or_init(|| {
        let mut out = Vec::new();
        flate2::read::GzDecoder::new(COMPRESSED)
            .read_to_end(&mut out)
            .expect("badapple.raw.gz must inflate");
        assert!(
            out.len() % FRAME_BYTES == 0,
            "asset is {} bytes, not a whole number of {FRAME_BYTES}-byte frames",
            out.len()
        );
        out
    })
}

pub fn frame_count() -> usize {
    frames().len() / FRAME_BYTES
}

pub fn frame(index: usize) -> &'static [u8] {
    &frames()[index * FRAME_BYTES..(index + 1) * FRAME_BYTES]
}

pub fn bounds() -> ([f32; 3], [f32; 3]) {
    let h = 2.0 * gridfinity::HEIGHT_PER_UNIT;
    (
        [0.0, 0.0, 0.0],
        [
            W as f32 * gridfinity::GRID_PITCH,
            H as f32 * gridfinity::GRID_PITCH,
            h,
        ],
    )
}

#[inline]
fn white(frame: &[u8], x: usize, y: usize) -> bool {
    (frame[y * ROW_BYTES + x / 8] >> (7 - (x % 8))) & 1 == 1
}

pub fn cell_params() -> Params {
    Params {
        height_units: 2,
        floor_fillet: 0.0,
        magnet_holes: false,
        screw_holes: false,
        ..Params::default()
    }
}

pub fn components(frame: &[u8]) -> Vec<Vec<GridCell>> {
    let mut seen = vec![false; W * H];
    let mut out = Vec::new();
    let mut stack = Vec::new();
    for sy in 0..H {
        for sx in 0..W {
            if seen[sy * W + sx] || !white(frame, sx, sy) {
                continue;
            }
            let mut cells = Vec::new();
            seen[sy * W + sx] = true;
            stack.push((sx, sy));
            while let Some((x, y)) = stack.pop() {
                cells.push(GridCell {
                    x: x as i32,
                    y: (H - 1 - y) as i32,
                });
                let mut nb = |nx: usize, ny: usize, stack: &mut Vec<(usize, usize)>| {
                    if !seen[ny * W + nx] && white(frame, nx, ny) {
                        seen[ny * W + nx] = true;
                        stack.push((nx, ny));
                    }
                };
                if x > 0 {
                    nb(x - 1, y, &mut stack);
                }
                if x + 1 < W {
                    nb(x + 1, y, &mut stack);
                }
                if y > 0 {
                    nb(x, y - 1, &mut stack);
                }
                if y + 1 < H {
                    nb(x, y + 1, &mut stack);
                }
            }
            out.push(cells);
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn asset_inflates_to_whole_frames() {
        let n = frame_count();
        assert!(n > 1000, "asset should hold the whole clip, got {n}");
        assert_eq!(frames().len(), n * FRAME_BYTES);
    }

    #[test]
    fn first_frame_is_black_and_mid_clip_is_not() {
        assert!(components(frame(0)).is_empty(), "frame 0 is a black screen");
        assert!(
            !components(frame(frame_count() / 2)).is_empty(),
            "mid clip has cells"
        );
    }
}
