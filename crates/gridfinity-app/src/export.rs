//! Writing an `optimize` run's built pieces out, in whichever of the two wired
//! formats it asked for.
//!
//! `Format` is that choice. STL is triangles and one file per printable piece,
//! so `write_stl_dir` tessellates each piece at `EXPORT_RES` and writes it into
//! a directory under the name the model gave it. Parasolid X_T is the analytic
//! B-rep and one multi-body file, so `write_xt` hands every piece's `Solid`
//! straight to the kernel's transmit writer with nothing tessellated on the way.
//! Either returns one `Written` per file, carrying what the report prints about
//! it. A kernel refusal is returned as an error and leaves no partial file
//! behind.

use gridfinity_cad::gridfinity::BinPiece;
use gridfinity_cad::kernel::topo::Solid;
use gridfinity_cad::{tessellate, to_xt_text};
use std::path::{Path, PathBuf};

/// How finely a piece is sampled on the way to triangles. The same resolution
/// the window's own STL export uses, so a piece exported either way is the same
/// file.
const EXPORT_RES: usize = 48;

/// The two wired export formats.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Format {
    Stl,
    ParasolidXt,
}

impl Format {
    /// The format named by its command-line spelling, or `None` for anything
    /// else.
    pub fn from_name(name: &str) -> Option<Format> {
        match name {
            "stl" => Some(Format::Stl),
            "parasolid_x_t" => Some(Format::ParasolidXt),
            _ => None,
        }
    }

    /// Whether this format could write to `path`, decided before any geometry is
    /// built. STL fills a directory and X_T writes one file, so each is refused
    /// by an existing path of the other kind, and X_T is refused by a parent
    /// directory that does not exist. Passing here does not promise the write
    /// will succeed -- permissions and a full disk are still the writer's to
    /// report -- but it catches the mistakes that would otherwise surface only
    /// after minutes of packing and building.
    pub fn check_output(self, path: &Path) -> Result<(), String> {
        match self {
            Format::Stl if path.is_file() => Err(format!(
                "--format stl writes one file per printable piece into a directory, but {} is a file",
                path.display()
            )),
            Format::ParasolidXt if path.is_dir() => Err(format!(
                "--format parasolid_x_t writes every piece into one file, but {} is a directory",
                path.display()
            )),
            Format::ParasolidXt => match path.parent() {
                Some(dir) if !dir.as_os_str().is_empty() && !dir.is_dir() => Err(format!(
                    "there is no directory {} to write {} into",
                    dir.display(),
                    path.display()
                )),
                _ => Ok(()),
            },
            Format::Stl => Ok(()),
        }
    }
}

/// What one written file holds, in the unit its format counts in.
pub enum Contents {
    Triangles(usize),
    Bodies(usize),
}

/// One file that was written: where it went, how big it is, and what it holds.
pub struct Written {
    pub path: PathBuf,
    pub bytes: usize,
    pub contents: Contents,
}

/// Every piece written into `dir` as its own binary STL, named as the model
/// names it. The directory is created when it does not exist.
///
/// Every piece is tessellated before any file is written, so a piece the
/// tessellator refuses -- `tessellate` asserts its own mesh is watertight --
/// leaves the directory as it found it rather than half a set of parts.
pub fn write_stl_dir(dir: &Path, pieces: &[BinPiece]) -> Result<Vec<Written>, String> {
    assert!(
        !pieces.is_empty(),
        "an export with no pieces reached the writer, which has nothing to write"
    );
    let files: Vec<(PathBuf, Vec<u8>, usize)> = pieces
        .iter()
        .map(|piece| {
            let mesh = tessellate(&piece.solid, EXPORT_RES).to_mesh();
            (dir.join(&piece.name), mesh.to_stl_binary(), mesh.tri_count())
        })
        .collect();
    std::fs::create_dir_all(dir)
        .map_err(|e| format!("could not create the output directory {}: {e}", dir.display()))?;
    let mut out = Vec::with_capacity(pieces.len());
    for (path, bytes, tris) in files {
        std::fs::write(&path, &bytes)
            .map_err(|e| format!("could not write {}: {e}", path.display()))?;
        out.push(Written {
            path,
            bytes: bytes.len(),
            contents: Contents::Triangles(tris),
        });
    }
    Ok(out)
}

/// Every piece written into one Parasolid transmit file as its own body, in the
/// order the model built them.
pub fn write_xt(path: &Path, pieces: &[BinPiece]) -> Result<Written, String> {
    assert!(
        !pieces.is_empty(),
        "an export with no pieces reached the writer, which has nothing to write"
    );
    let bodies: Vec<&Solid> = pieces.iter().map(|p| &p.solid).collect();
    let text = to_xt_text(&bodies)?;
    std::fs::write(path, text.as_bytes())
        .map_err(|e| format!("could not write {}: {e}", path.display()))?;
    Ok(Written {
        path: path.to_path_buf(),
        bytes: text.len(),
        contents: Contents::Bodies(pieces.len()),
    })
}
