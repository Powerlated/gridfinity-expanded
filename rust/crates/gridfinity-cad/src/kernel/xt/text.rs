//! The XT text encoding: how a graph of nodes becomes the byte stream a
//! Parasolid frustrum reads.
//!
//! `Writer` owns the three things the rest of `xt/` never touches. It hands out
//! the node indices every pointer field is written as, so a node can be
//! referenced before it is emitted. It encodes fields by the text format's own
//! rules -- a number is followed by exactly one space, a `char` or `logical` is
//! not, an unset value is a lone `?`. And it splits the stream into records near
//! 80 characters, moving a pending separator space to the *start* of the next
//! record rather than leaving it at the end of one, because a reader strips
//! trailing spaces and two numbers either side of a break would otherwise run
//! together. Lengths cross from the kernel's millimetres to the metres the file
//! declares, here and nowhere else: `dist` and `pos` convert, `real` and `dir`
//! do not.

use crate::kernel::math::Vec3;

/// A node's position in the file's node sequence, which is what every pointer
/// field is written as. Index 0 is the null pointer and index 1 is the root.
pub type Index = u32;

pub const BODY: u16 = 12;
pub const SHELL: u16 = 13;
pub const FACE: u16 = 14;
pub const LOOP: u16 = 15;
pub const EDGE: u16 = 16;
pub const FIN: u16 = 17;
pub const VERTEX: u16 = 18;
pub const REGION: u16 = 19;
pub const POINT: u16 = 29;
pub const LINE: u16 = 30;
pub const CIRCLE: u16 = 31;
pub const ELLIPSE: u16 = 32;
pub const INTERSECTION: u16 = 38;
pub const CHART: u16 = 40;
pub const LIMIT: u16 = 41;
pub const PLANE: u16 = 50;
pub const CYLINDER: u16 = 51;
pub const CONE: u16 = 52;
pub const SPHERE: u16 = 53;
pub const TORUS: u16 = 54;
pub const POINTER_LIS_BLOCK: u16 = 74;
pub const GEOMETRIC_OWNER: u16 = 141;

/// Millimetres in one metre. The kernel models in millimetres; a transmit file
/// carries no units of its own, and every application that reads one -- as the
/// file's own `res_size` of 1000 and `res_linear` of 1e-8 imply -- reads metres.
pub const MM_PER_M: f64 = 1000.0;

/// The modeller version an application writing XT files must claim, and the
/// schema its field sequences follow.
const MODELLER_VERSION: &str = ": TRANSMIT FILE created by modeller version 1200000";
const SCHEMA: &str = "SCH_1200000_12006";

/// Where a record is broken. The format puts no requirement on the number
/// beyond the 80-character ceiling on keyword values in the header, and a
/// reader ignores the newlines entirely.
const RECORD_WIDTH: usize = 76;

pub struct Writer {
    out: String,
    col: usize,
    pending_space: bool,
    next: Index,
    emitted: usize,
}

impl Writer {
    /// A writer positioned at the first node of an empty file: the human header,
    /// the format flag sequence and the userfield size are already written, and
    /// index 1 is the next index `alloc` hands out, which the caller must give
    /// to the root node.
    pub fn new() -> Writer {
        let mut w = Writer {
            out: String::new(),
            col: 0,
            pending_space: false,
            next: 1,
            emitted: 0,
        };
        w.header();
        w
    }

    /// The next unused node index, reserved for the caller. Indices are handed
    /// out in ascending order from 1 and never reused, so a node may be
    /// referenced by any number of fields written before it is emitted.
    pub fn alloc(&mut self) -> Index {
        let i = self.next;
        self.next += 1;
        i
    }

    /// How many indices have been handed out, which is the number of nodes the
    /// file must contain for every pointer written to resolve.
    pub fn allocated(&self) -> usize {
        (self.next - 1) as usize
    }

    /// Opens a fixed-length node of type `ty` at `index`, leaving the caller to
    /// write its fields in schema order.
    pub fn begin(&mut self, ty: u16, index: Index) {
        assert!(index > 0 && index < self.next, "node index {index} was never allocated");
        self.emitted += 1;
        self.token(&ty.to_string(), true);
        self.token(&index.to_string(), true);
    }

    /// Opens a variable-length node, whose entry carries the length of its final
    /// field between the node type and the index.
    pub fn begin_var(&mut self, ty: u16, len: usize, index: Index) {
        assert!(index > 0 && index < self.next, "node index {index} was never allocated");
        self.emitted += 1;
        self.token(&ty.to_string(), true);
        self.token(&len.to_string(), true);
        self.token(&index.to_string(), true);
    }

    /// An `int` or `short int` field.
    pub fn int(&mut self, v: i64) {
        self.token(&v.to_string(), true);
    }

    /// A `pointer-index` field: the referenced node's index, or 0 for null.
    pub fn ptr(&mut self, i: Index) {
        assert!(i < self.next, "pointer to node index {i}, which was never allocated");
        self.token(&i.to_string(), true);
    }

    /// A `double` field carrying a pure number -- a ratio, a sine, a count --
    /// with no length in it, so no unit conversion applies.
    pub fn real(&mut self, v: f64) {
        assert!(v.is_finite(), "a transmit file cannot carry the non-finite value {v}");
        self.token(&fmt_real(v), true);
    }

    /// A `double` field carrying a length in the kernel's millimetres, written
    /// in the metres the file is in.
    pub fn dist(&mut self, mm: f32) {
        assert!(mm.is_finite(), "a transmit file cannot carry the non-finite length {mm}");
        self.real(mm as f64 / MM_PER_M);
    }

    /// A `vector` field carrying a position in the kernel's millimetres, written
    /// in metres.
    pub fn pos(&mut self, p: Vec3) {
        assert!(p.is_finite(), "a transmit file cannot carry the non-finite point {p:?}");
        for c in [p.x, p.y, p.z] {
            self.real(c as f64 / MM_PER_M);
        }
    }

    /// A `vector` field carrying a direction, which the format requires to be a
    /// unit vector and which carries no length to convert.
    pub fn dir(&mut self, d: Vec3) {
        let len = d.length();
        assert!(
            (len - 1.0).abs() < 1e-4,
            "a direction field must be a unit vector, got {d:?} of length {len}"
        );
        let d = d / len;
        for c in [d.x, d.y, d.z] {
            self.real(c as f64);
        }
    }

    /// A `char` field, written as itself and not followed by a space.
    pub fn ch(&mut self, c: char) {
        assert!(c.is_ascii_graphic(), "a char field must be an ASCII printing character, got {c:?}");
        self.token(&c.to_string(), false);
    }

    /// A `logical` field, written `T` or `F` and not followed by a space.
    pub fn logical(&mut self, b: bool) {
        self.token(if b { "T" } else { "F" }, false);
    }

    /// An unset field of any numeric type -- the format's null int, null double
    /// and null vector are one `?` each, and like a char it takes no space after.
    pub fn null(&mut self) {
        self.token("?", false);
    }

    /// The finished file: the node sequence the caller emitted, closed by the
    /// terminator (a node type of 1 and an index of 0) and a final newline.
    /// Every index handed out must have become exactly one node, or the
    /// pointers to the missing ones dangle.
    pub fn finish(mut self) -> String {
        assert_eq!(
            self.emitted,
            self.allocated(),
            "every allocated index must be emitted as a node, or the pointers to it dangle"
        );
        self.int(1);
        self.int(0);
        self.out.push('\n');
        self.out
    }

    /// Appends one field token, breaking the record first if it would overrun
    /// and carrying any pending separator space to the new record's start.
    fn token(&mut self, text: &str, space_after: bool) {
        let width = text.len() + usize::from(self.pending_space);
        if self.col > 0 && self.col + width > RECORD_WIDTH {
            self.out.push('\n');
            self.col = 0;
        }
        if self.pending_space {
            self.out.push(' ');
            self.col += 1;
            self.pending_space = false;
        }
        self.out.push_str(text);
        self.col += text.len();
        self.pending_space = space_after;
    }

    /// Writes one complete record, used for the header lines the format
    /// prescribes literally rather than as fields.
    fn line(&mut self, s: &str) {
        assert!(!self.pending_space, "a header line cannot interrupt a field's separator");
        self.out.push_str(s);
        self.out.push('\n');
        self.col = 0;
    }

    /// The human header, the text flag sequence and the userfield size, after
    /// which the stream is positioned for the first node.
    fn header(&mut self) {
        self.line("**ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz***********");
        self.line("**PARASOLID !\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~0123456789***********");
        self.line("**PART1;APPL=gridfinity-expanded;FORMAT=text;GUISE=transmit;");
        self.line(&format!("**PART2;SCH={SCHEMA};USFLD_SIZE=0;"));
        self.line("**PART3;");
        self.line("**END_OF_HEADER*****************************************************");
        self.line("T");
        self.line(&format!("{} {}", MODELLER_VERSION.len(), MODELLER_VERSION));
        self.line(&format!("{} {}", SCHEMA.len(), SCHEMA));
        self.int(0);
    }
}

/// A double in the shortest decimal form that reads back as the same value,
/// with an exponent-free spelling for the magnitudes a model is made of.
fn fmt_real(v: f64) -> String {
    if v == 0.0 {
        return "0".to_string();
    }
    let s = format!("{v}");
    if s.contains('e') { format!("{v:.17}") } else { s }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_separator_space_moves_to_the_next_record_rather_than_ending_one() {
        let mut w = Writer::new();
        for _ in 0..200 {
            let i = w.alloc();
            w.int(i as i64);
        }
        let out = std::mem::take(&mut w.out);
        for line in out.lines() {
            assert!(
                !line.ends_with(' '),
                "a record ending in a space loses its separator: {line:?}"
            );
            assert!(
                !line.contains("  "),
                "adjacent spaces are not allowed inside a record: {line:?}"
            );
        }
        let rejoined: String = out.lines().collect::<Vec<_>>().join("");
        for i in 1..200 {
            assert!(
                rejoined.contains(&format!(" {i} ")),
                "value {i} lost its separators once the records were rejoined"
            );
        }
        assert!(
            rejoined.ends_with(" 200"),
            "the final value keeps its leading separator and takes no trailing one"
        );
    }

    #[test]
    fn a_length_crosses_to_metres_and_a_direction_does_not() {
        let mut w = Writer::new();
        let i = w.alloc();
        w.begin(POINT, i);
        w.pos(Vec3::new(42.0, -1.5, 0.0));
        w.dir(Vec3::X);
        let out = w.out;
        assert!(out.contains("0.042 -0.0015 0"), "positions convert to metres:\n{out}");
        assert!(out.contains("1 0 0"), "directions stay unit vectors:\n{out}");
    }
}
