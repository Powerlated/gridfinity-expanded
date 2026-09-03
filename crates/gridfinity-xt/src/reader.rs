//! The transmit-file reader: the text a writer emits, back into the node graph
//! it claims to hold.
//!
//! Parsing is schema-driven and character-level, because a `char` or null
//! field takes no trailing space and the next number runs straight into it --
//! `?10` in the manual's own example -- so splitting on whitespace desyncs at
//! the first such field. `parse` walks the node sequence field by field with
//! the schema table below, which states the same field orders `topo` and
//! `surf` write from, and fails with a located message on the first thing it
//! cannot read -- a validator's finding, not a panic, because it runs on
//! files nobody here wrote.
//!
//! Every node keeps its tokens alongside its typed fields, and `Parsed`
//! can `render` them back to a valid unwrapped stream. That round trip is
//! what lets the validator's tests corrupt a file *as a file* -- flip one
//! sign, move one chart point, renumber one pointer -- and prove the
//! validator catches that corruption rather than something incidental.

use super::text::{CHART, LIMIT, POINTER_LIS_BLOCK};

/// What one field of a node is, which says how it sits in the text stream:
/// a number ends at the one space after it, a `char` or `logical` is not
/// followed by a space, a null is one `?` also without a space, and a vector
/// is three numbers.
#[derive(Clone, Copy, PartialEq)]
enum Kind {
    Int,
    Dbl,
    Ptr,
    NullableDbl,
    Chr,
    Vec,
}

/// One lexical token and whether whitespace followed it in the stream, which
/// is what re-emitting it needs: a number is followed by one space, a char or
/// null is not.
pub struct Token {
    pub text: String,
    pub space_after: bool,
}

/// One parsed node: its type, its variable length for the three types that
/// carry one, its fields split by kind so a pointer is a pointer, and the
/// tokens they came from so a caller can rewrite a field in place.
pub struct Node {
    pub ty: u16,
    pub len: Option<usize>,
    pub index: u32,
    pub ints: Vec<i64>,
    pub dbls: Vec<f64>,
    pub ptrs: Vec<u32>,
    pub chars: Vec<char>,
    pub vecs: Vec<[f64; 3]>,
    pub nulls: usize,
    pub tokens: Vec<Token>,
    int_tokens: Vec<usize>,
    dbl_tokens: Vec<usize>,
    ptr_tokens: Vec<usize>,
    chr_tokens: Vec<usize>,
    vec_tokens: Vec<usize>,
}

/// A whole file: the header lines up to and including the schema line, and
/// every node in stream order. The userfield size is not kept -- a transmit
/// file of this schema has none, and `render` writes the zero back.
pub struct Parsed {
    pub header: Vec<String>,
    pub nodes: Vec<Node>,
}

/// The field kinds of node type `ty` in schema order, `len` variable fields
/// expanded -- transcribed from the format's own node structures, the same
/// tables `topo` and `surf` write from.
fn schema(ty: u16, len: usize) -> Vec<Kind> {
    let common = [
        Kind::Int,
        Kind::Ptr,
        Kind::Ptr,
        Kind::Ptr,
        Kind::Ptr,
        Kind::Ptr,
    ];
    let mut k: Vec<Kind> = match ty {
        super::text::BODY => vec![
            Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
            Kind::Dbl, Kind::Dbl, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Int, Kind::Ptr,
            Kind::Int, Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
            Kind::Ptr, Kind::Ptr,
        ],
        super::text::REGION => vec![
            Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Chr,
        ],
        super::text::SHELL => vec![
            Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
            Kind::Ptr, Kind::Ptr,
        ],
        super::text::FACE => vec![
            Kind::Int, Kind::Ptr, Kind::NullableDbl, Kind::Ptr, Kind::Ptr, Kind::Ptr,
            Kind::Ptr, Kind::Ptr, Kind::Chr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
            Kind::Ptr,
        ],
        super::text::LOOP => vec![
            Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
        ],
        super::text::EDGE => vec![
            Kind::Int, Kind::Ptr, Kind::NullableDbl, Kind::Ptr, Kind::Ptr, Kind::Ptr,
            Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
        ],
        super::text::FIN => vec![
            Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
            Kind::Ptr, Kind::Ptr, Kind::Chr,
        ],
        super::text::VERTEX => vec![
            Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr,
            Kind::NullableDbl, Kind::Ptr,
        ],
        super::text::POINT => vec![
            Kind::Int, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Vec,
        ],
        super::text::LINE => common_plus(&common, vec![Kind::Chr, Kind::Vec, Kind::Vec]),
        super::text::CIRCLE => common_plus(
            &common,
            vec![Kind::Chr, Kind::Vec, Kind::Vec, Kind::Vec, Kind::Dbl],
        ),
        super::text::ELLIPSE => common_plus(
            &common,
            vec![
                Kind::Chr,
                Kind::Vec,
                Kind::Vec,
                Kind::Vec,
                Kind::Dbl,
                Kind::Dbl,
            ],
        ),
        super::text::INTERSECTION => common_plus(
            &common,
            vec![
                Kind::Chr,
                Kind::Ptr,
                Kind::Ptr,
                Kind::Ptr,
                Kind::Ptr,
                Kind::Ptr,
            ],
        ),
        super::text::PLANE => common_plus(&common, vec![Kind::Chr, Kind::Vec, Kind::Vec, Kind::Vec]),
        super::text::CYLINDER => common_plus(
            &common,
            vec![Kind::Chr, Kind::Vec, Kind::Vec, Kind::Dbl, Kind::Vec],
        ),
        super::text::CONE => common_plus(
            &common,
            vec![
                Kind::Chr,
                Kind::Vec,
                Kind::Vec,
                Kind::Dbl,
                Kind::Dbl,
                Kind::Dbl,
                Kind::Vec,
            ],
        ),
        super::text::SPHERE => common_plus(
            &common,
            vec![Kind::Chr, Kind::Vec, Kind::Dbl, Kind::Vec, Kind::Vec],
        ),
        super::text::TORUS => common_plus(
            &common,
            vec![Kind::Chr, Kind::Vec, Kind::Vec, Kind::Dbl, Kind::Dbl, Kind::Vec],
        ),
        super::text::GEOMETRIC_OWNER => vec![Kind::Ptr, Kind::Ptr, Kind::Ptr, Kind::Ptr],
        CHART | LIMIT | POINTER_LIS_BLOCK => Vec::new(),
        _ => panic!("this reader knows every node type of SCH_1200000_12006, not {ty}"),
    };
    match ty {
        CHART => {
            assert!(k.is_empty(), "a chart's fields are all its own, with no common prefix");
            k.extend([
                Kind::Dbl,
                Kind::Dbl,
                Kind::Int,
                Kind::NullableDbl,
                Kind::NullableDbl,
                Kind::NullableDbl,
                Kind::NullableDbl,
            ]);
            k.extend(std::iter::repeat_n(Kind::Vec, len));
        }
        LIMIT => {
            assert!(k.is_empty(), "a limit's fields are all its own, with no common prefix");
            k.push(Kind::Chr);
            k.extend(std::iter::repeat_n(Kind::Vec, len));
        }
        POINTER_LIS_BLOCK => {
            assert!(
                k.is_empty(),
                "a pointer list block's fields are all its own, with no common prefix"
            );
            k.extend([Kind::Int, Kind::Ptr]);
            k.extend(std::iter::repeat_n(Kind::Ptr, len));
        }
        _ => assert_eq!(len, 0, "only CHART, LIMIT and POINTER_LIS_BLOCK carry a length"),
    }
    k
}

/// The six fields every curve and surface node begins with, then `own`.
fn common_plus(common: &[Kind], own: Vec<Kind>) -> Vec<Kind> {
    let mut k = common.to_vec();
    k.extend(own);
    k
}

/// Reads the node sequence out of `text`, or a message naming the first thing
/// that is not a transmit file of this schema: no `T` flag line, a userfield
/// size other than zero, an unknown node type, or a field that will not parse.
/// Header lines before the flag sequence are kept verbatim; the node stream
/// after it has its records unwrapped, which the format allows because
/// newlines are not significant.
pub fn parse(text: &str) -> Result<Parsed, String> {
    let lines: Vec<&str> = text.lines().collect();
    let t_line = lines
        .iter()
        .position(|l| *l == "T")
        .filter(|&i| i > 0 && lines[i - 1].starts_with("**END_OF_HEADER"))
        .ok_or_else(|| {
            "no text-format flag line `T` after the header -- a binary XT file, or not XT at \
             all"
                .to_string()
        })?;
    if lines.len() < t_line + 4 {
        return Err("the flag sequence is followed by a version line, a schema line and at \
                    least the userfield size"
            .to_string());
    }
    let header: Vec<String> = lines[..t_line + 3].iter().map(|s| s.to_string()).collect();
    let mut chars: Vec<char> = lines[t_line + 3..].concat().chars().collect();
    if chars.last() == Some(&'\r') {
        chars.pop();
    }
    let mut at = 0usize;
    let mut nodes: Vec<Node> = Vec::new();
    let usfld = read_number(&mut chars, &mut at)?;
    if usfld != 0.0 {
        return Err(format!(
            "a transmit file of this schema carries no user fields, but the userfield size is \
             {usfld}"
        ));
    }
    loop {
        let ty = read_number(&mut chars, &mut at)?.round() as i64;
        if !(0..=65535).contains(&ty) {
            return Err(format!("node type {ty} is not a 2-byte integer"));
        }
        let mut len = None;
        if matches!(ty as u16, CHART | LIMIT | POINTER_LIS_BLOCK) {
            len = Some(read_number(&mut chars, &mut at)?.round() as usize);
        }
        let index = read_int(&mut chars, &mut at)?;
        if ty == 1 && index == 0 {
            if at != chars.len() {
                return Err(format!(
                    "content follows the 1 0 terminator, starting with {:?}",
                    chars[at..(at + 20).min(chars.len())].iter().collect::<String>()
                ));
            }
            return Ok(Parsed { header, nodes });
        }
        if index == 0 {
            return Err("node index 0 is the null pointer, never a node's own index".to_string());
        }
        let ty = ty as u16;
        if !KNOWN.contains(&ty) {
            return Err(format!(
                "node {index} has type {ty}, which this reader's schema SCH_1200000_12006 \
                 does not define"
            ));
        }
        let mut node = Node {
            ty,
            len,
            index: index as u32,
            ints: Vec::new(),
            dbls: Vec::new(),
            ptrs: Vec::new(),
            chars: Vec::new(),
            vecs: Vec::new(),
            nulls: 0,
            tokens: vec![
                Token { text: ty.to_string(), space_after: true },
                Token { text: index.to_string(), space_after: true },
            ],
            int_tokens: Vec::new(),
            dbl_tokens: Vec::new(),
            ptr_tokens: Vec::new(),
            chr_tokens: Vec::new(),
            vec_tokens: Vec::new(),
        };
        if len.is_some() {
            node.tokens.insert(
                1,
                Token {
                    text: len
                        .expect("only a variable-length node reaches this, and it carries one")
                        .to_string(),
                    space_after: true,
                },
            );
        }
        for kind in schema(ty, len.unwrap_or(0)) {
            match kind {
                Kind::Int => {
                    let v = read_int(&mut chars, &mut at)?;
                    node.push_int(node.ints.len(), v.to_string());
                    node.ints.push(v);
                }
                Kind::Dbl => {
                    let v = read_number(&mut chars, &mut at)?;
                    node.push_dbl(node.dbls.len(), v);
                    node.dbls.push(v);
                }
                Kind::Ptr => {
                    let v = read_int(&mut chars, &mut at)?;
                    node.push_ptr(node.ptrs.len(), v.to_string());
                    node.ptrs.push(v as u32);
                }
                Kind::NullableDbl => {
                    if at < chars.len() && chars[at] == '?' {
                        at += 1;
                        let followed = at < chars.len() && chars[at] == ' ';
                        if followed {
                            at += 1;
                        }
                        node.tokens.push(Token { text: "?".to_string(), space_after: followed });
                        node.nulls += 1;
                    } else {
                        let v = read_number(&mut chars, &mut at)?;
                        node.push_dbl(node.dbls.len(), v);
                        node.dbls.push(v);
                    }
                }
                Kind::Chr => {
                    if at >= chars.len() {
                        return Err(format!(
                            "node {index} of type {ty} ends mid-field, wanting a char"
                        ));
                    }
                    let c = chars[at];
                    at += 1;
                    let followed = at < chars.len() && chars[at] == ' ';
                    if followed {
                        at += 1;
                    }
                    node.tokens.push(Token { text: c.to_string(), space_after: followed });
                    node.chr_tokens.push(node.tokens.len() - 1);
                    node.chars.push(c);
                }
                Kind::Vec => {
                    let mut v = [0.0; 3];
                    for c in &mut v {
                        *c = read_number(&mut chars, &mut at)?;
                    }
                    node.push_vec(node.vecs.len(), v);
                    node.vecs.push(v);
                }
            }
        }
        assert_eq!(
            node.tokens.len(),
            2 + usize::from(len.is_some())
                + node.int_tokens.len()
                + node.dbl_tokens.len()
                + node.ptr_tokens.len()
                + node.chr_tokens.len()
                + node.nulls
                + node.vec_tokens.len(),
            "every field of node {} leaves exactly one token behind",
            node.index
        );
        nodes.push(node);
    }
}

/// The node types this reader's schema defines, so an unknown one is a clean
/// refusal instead of a misparse.
const KNOWN: &[u16] = &[
    super::text::BODY,
    super::text::SHELL,
    super::text::FACE,
    super::text::LOOP,
    super::text::EDGE,
    super::text::FIN,
    super::text::VERTEX,
    super::text::REGION,
    super::text::POINT,
    super::text::LINE,
    super::text::CIRCLE,
    super::text::ELLIPSE,
    super::text::INTERSECTION,
    CHART,
    LIMIT,
    super::text::PLANE,
    super::text::CYLINDER,
    super::text::CONE,
    super::text::SPHERE,
    super::text::TORUS,
    POINTER_LIS_BLOCK,
    super::text::GEOMETRIC_OWNER,
];

/// One whitespace-delimited token and whether whitespace followed it, leaving
/// the cursor at the start of the next token or at the end of the stream.
fn read_token(chars: &mut Vec<char>, at: &mut usize) -> (String, bool) {
    let start = *at;
    while *at < chars.len() && chars[*at] != ' ' {
        *at += 1;
    }
    let text: String = chars[start..*at].iter().collect();
    let followed = *at < chars.len();
    if followed {
        *at += 1;
    }
    (text, followed)
}

/// One number and its trailing space, as a validator's located failure when
/// the token is not one.
fn read_number(chars: &mut Vec<char>, at: &mut usize) -> Result<f64, String> {
    let (text, _) = read_token(chars, at);
    parse_double(&text)
        .ok_or_else(|| format!("a field that is not a char must parse as a number, got {text:?}"))
}

/// One integral field: a number with no fractional part in i64 range.
fn read_int(chars: &mut Vec<char>, at: &mut usize) -> Result<i64, String> {
    let v = read_number(chars, at)?;
    if v.fract() != 0.0 || v.abs() >= 9.2e18 {
        return Err(format!("an integer field holds {v}, which is not an i64"));
    }
    Ok(v as i64)
}

/// A token as a finite double, or `None` -- the one parse step every caller
/// funnels through so a non-number is always a located failure.
fn parse_double(text: &str) -> Option<f64> {
    text.parse::<f64>().ok().filter(|v| v.is_finite())
}

impl Node {
    /// Records where int field `slot` keeps its token, so a later rewrite goes
    /// to the stream and the typed field stays in step with it.
    fn push_int(&mut self, slot: usize, text: String) {
        self.tokens.push(Token { text, space_after: true });
        self.int_tokens.push(self.tokens.len() - 1);
        assert_eq!(self.int_tokens.len(), slot + 1, "int tokens land in slot order");
    }

    /// Records where pointer field `slot` keeps its token.
    fn push_ptr(&mut self, slot: usize, text: String) {
        self.tokens.push(Token { text, space_after: true });
        self.ptr_tokens.push(self.tokens.len() - 1);
        assert_eq!(self.ptr_tokens.len(), slot + 1, "pointer tokens land in slot order");
    }

    /// Records where double field `slot` keeps its token.
    fn push_dbl(&mut self, slot: usize, v: f64) {
        self.tokens.push(Token { text: fmt(v), space_after: true });
        self.dbl_tokens.push(self.tokens.len() - 1);
        assert_eq!(self.dbl_tokens.len(), slot + 1, "double tokens land in slot order");
    }

    /// Records where vector field `slot` keeps its three component tokens.
    fn push_vec(&mut self, slot: usize, v: [f64; 3]) {
        for c in v {
            self.tokens.push(Token { text: fmt(c), space_after: true });
            self.vec_tokens.push(self.tokens.len() - 1);
        }
        assert_eq!(self.vec_tokens.len(), 3 * (slot + 1), "vector tokens land in slot order");
    }
}

/// A double spelled the way the writer spells one, so a re-rendered file reads
/// the same as the tokens it came from.
fn fmt(v: f64) -> String {
    if v == 0.0 {
        "0".to_string()
    } else {
        let s = format!("{v}");
        if s.contains('e') { format!("{v:.17}") } else { s }
    }
}

impl Parsed {
    /// The nodes re-serialized as a valid single-record-stream file: the
    /// header verbatim, the zero userfield size, every node's tokens with
    /// their original separators, and the terminator. A rendered file is what
    /// mutation tests feed back through the validator.
    pub fn render(&self) -> String {
        let mut out = self.header.join("\n");
        out.push_str("\n0 ");
        for node in &self.nodes {
            for token in &node.tokens {
                out.push_str(&token.text);
                if token.space_after {
                    out.push(' ');
                }
            }
        }
        out.push_str("1 0");
        out
    }

    /// The node of index `i`, for callers that know it exists.
    pub fn node(&self, i: u32) -> Option<&Node> {
        self.nodes.iter().find(|n| n.index == i)
    }
}

/// Field rewriting for tests: each sets the typed field and rewrites the
/// token it came from, so `render` emits the change.
#[cfg(test)]
impl Parsed {
    /// Points pointer slot `slot` of node `index` at `value`.
    pub(crate) fn set_ptr(&mut self, index: u32, slot: usize, value: u32) {
        let node = self.node_mut(index);
        node.ptrs[slot] = value;
        let token = node.ptr_tokens[slot];
        node.tokens[token].text = value.to_string();
    }

    /// Sets int slot `slot` of node `index` to `value`.
    pub(crate) fn set_int(&mut self, index: u32, slot: usize, value: i64) {
        let node = self.node_mut(index);
        node.ints[slot] = value;
        let token = node.int_tokens[slot];
        node.tokens[token].text = value.to_string();
    }

    /// Flips char slot `slot` of node `index` between `+` and `-`.
    pub(crate) fn flip_sense(&mut self, index: u32, slot: usize) {
        let node = self.node_mut(index);
        let flipped = if node.chars[slot] == '+' { '-' } else { '+' };
        node.chars[slot] = flipped;
        let token = node.chr_tokens[slot];
        node.tokens[token].text = flipped.to_string();
    }

    /// Shifts component `comp` of vector slot `slot` of node `index` by
    /// `delta`, in the file's own units.
    pub(crate) fn offset_vec(&mut self, index: u32, slot: usize, comp: usize, delta: f64) {
        let node = self.node_mut(index);
        node.vecs[slot][comp] += delta;
        let token = node.vec_tokens[3 * slot + comp];
        node.tokens[token].text = fmt(node.vecs[slot][comp]);
    }

    /// Renumber's node `index`'s own index to `new`, in the stream and the
    /// typed field.
    pub(crate) fn set_index(&mut self, index: u32, new: u32) {
        let node = self.node_mut(index);
        node.index = new;
        let token = usize::from(node.len.is_some()) + 1;
        node.tokens[token].text = new.to_string();
    }

    /// Drops the node of index `index` from the stream, leaving a gap behind.
    pub(crate) fn drop_node(&mut self, index: u32) {
        self.nodes.retain(|n| n.index != index);
    }

    /// The node of index `i`, mutable, for the rewriters above.
    fn node_mut(&mut self, i: u32) -> &mut Node {
        self.nodes
            .iter_mut()
            .find(|n| n.index == i)
            .unwrap_or_else(|| panic!("node {i} exists to be rewritten"))
    }
}
