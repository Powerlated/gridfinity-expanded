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
//!
//! `Format` also owns the relationship between a format and the path that can
//! hold it -- `EXTENSION`, `inferred_from`, `check_extension` -- because that is
//! one fact and the command line asks it in both directions: which format an
//! output names, and whether an output can hold the format that was named.

use clap::ValueEnum;
use std::path::{Path, PathBuf};

/// How finely a piece is sampled on the way to triangles. The same resolution
/// the window's own STL export uses, so a piece exported either way is the same
/// file.

/// The two wired export formats.
#[derive(Clone, Copy, Debug, PartialEq, Eq, ValueEnum)]
pub enum Format {
    #[value(name = "stl")]
    Stl,
    #[value(name = "parasolid_x_t")]
    ParasolidXt,
}

/// The file extension a Parasolid transmit file carries, lowercase and without
/// its dot. STL has no counterpart here on purpose: its output is a *directory*
/// of one file per piece, so a path that names it carries no extension at all.
const XT_EXTENSION: &str = "x_t";

impl Format {
    /// The format an output path names by its extension, or `None` when it names
    /// none.
    ///
    /// Only `.x_t` names one. An STL run writes a directory, and a directory's
    /// name says nothing about what goes in it, so an output that is not `.x_t`
    /// leaves the format to be stated.
    pub fn inferred_from(path: &Path) -> Option<Format> {
        let ext = path.extension()?.to_str()?;
        ext.eq_ignore_ascii_case(XT_EXTENSION)
            .then_some(Format::ParasolidXt)
    }

    /// `Ok` when `path`'s extension is one this format can be written to, and a
    /// message naming the mismatch otherwise.
    ///
    /// X_T is one file and must be spelled `.x_t`. STL is a directory of pieces,
    /// each already named `.stl` by the model, so its path must carry neither
    /// extension -- `out/drawer.stl` would be a *directory* full of `.stl`
    /// files, which reads as the file it is not.
    pub fn check_extension(self, path: &Path) -> Result<(), String> {
        let ext = path.extension().and_then(|e| e.to_str()).unwrap_or("");
        match self {
            Format::ParasolidXt if !ext.eq_ignore_ascii_case(XT_EXTENSION) => Err(format!(
                "--format parasolid_x_t writes one file, whose name must end in .{XT_EXTENSION}, but {} does not",
                path.display()
            )),
            Format::Stl
                if ext.eq_ignore_ascii_case("stl") || ext.eq_ignore_ascii_case(XT_EXTENSION) =>
            {
                Err(format!(
                    "--format stl writes one .stl file per piece into a directory, so its output names that directory and carries no extension, but {} ends in .{ext}",
                    path.display()
                ))
            }
            _ => Ok(()),
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

/// One owned native body ready for an optimizer export.
#[cfg(feature = "occt")]
pub struct OcctBody<'a> {
    pub name: &'a str,
    pub shape: &'a gridfinity_occt::Shape,
}

/// Native OCCT bodies written as one binary STL apiece.
#[cfg(feature = "occt")]
pub fn write_occt_stl_dir(dir: &Path, bodies: &[OcctBody<'_>]) -> Result<Vec<Written>, String> {
    assert!(
        !bodies.is_empty(),
        "an OCCT STL export contains at least one body"
    );
    let files: Vec<(PathBuf, Vec<u8>, usize)> = bodies
        .iter()
        .map(|body| {
            let mesh = body
                .shape
                .tessellate(0.08)
                .map_err(|e| format!("OCCT could not tessellate {}: {e}", body.name))?;
            let triangles = mesh.tri_count();
            Ok((dir.join(&body.name), mesh.to_stl_binary(), triangles))
        })
        .collect::<Result<_, String>>()?;
    std::fs::create_dir_all(dir).map_err(|e| {
        format!(
            "could not create the output directory {}: {e}",
            dir.display()
        )
    })?;
    files
        .into_iter()
        .map(|(path, bytes, triangles)| {
            std::fs::write(&path, &bytes)
                .map_err(|e| format!("could not write {}: {e}", path.display()))?;
            Ok(Written {
                path,
                bytes: bytes.len(),
                contents: Contents::Triangles(triangles),
            })
        })
        .collect()
}

/// Native OCCT bodies written into one Parasolid transmit file.
#[cfg(feature = "occt")]
pub fn write_occt_xt(path: &Path, bodies: &[OcctBody<'_>]) -> Result<Written, String> {
    assert!(
        !bodies.is_empty(),
        "an OCCT X_T export contains at least one body"
    );
    let shapes: Vec<&gridfinity_occt::Shape> = bodies.iter().map(|body| body.shape).collect();
    let text = gridfinity_xt::occt::to_xt_text(&shapes).map_err(|e| e.to_string())?;
    std::fs::write(path, text.as_bytes())
        .map_err(|e| format!("could not write {}: {e}", path.display()))?;
    Ok(Written {
        path: path.to_path_buf(),
        bytes: text.len(),
        contents: Contents::Bodies(bodies.len()),
    })
}

/// Every piece written into `dir` as its own binary STL, named as the model
/// names it. The directory is created when it does not exist.
///
/// Every piece is tessellated before any file is written, so a piece the
/// tessellator refuses -- `tessellate` asserts its own mesh is watertight --
/// leaves the directory as it found it rather than half a set of parts.
#[cfg(not(feature = "occt"))]
pub fn write_stl_dir(dir: &Path, pieces: &[Body<'_>]) -> Result<Vec<Written>, String> {
    assert!(
        !pieces.is_empty(),
        "an export with no pieces reached the writer, which has nothing to write"
    );
    let files: Vec<(PathBuf, Vec<u8>, usize)> = pieces
        .iter()
        .map(|piece| {
            let mesh = tessellate(piece.solid, EXPORT_RES).to_mesh();
            (dir.join(piece.name), mesh.to_stl_binary(), mesh.tri_count())
        })
        .collect();
    std::fs::create_dir_all(dir).map_err(|e| {
        format!(
            "could not create the output directory {}: {e}",
            dir.display()
        )
    })?;
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
#[cfg(not(feature = "occt"))]
pub fn write_xt(path: &Path, pieces: &[Body<'_>]) -> Result<Written, String> {
    assert!(
        !pieces.is_empty(),
        "an export with no pieces reached the writer, which has nothing to write"
    );
    let bodies: Vec<&Solid> = pieces.iter().map(|p| p.solid).collect();
    let text = to_xt_text(&bodies)?;
    std::fs::write(path, text.as_bytes())
        .map_err(|e| format!("could not write {}: {e}", path.display()))?;
    Ok(Written {
        path: path.to_path_buf(),
        bytes: text.len(),
        contents: Contents::Bodies(pieces.len()),
    })
}
