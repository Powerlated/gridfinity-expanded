//! Builds the browser page: one WebAssembly module and the host that starts it.
//!
//! `cargo run -p gridfinity-web --release` configures the pinned `vendor/occt`
//! submodule if it has not been built for the web yet, links the app, the
//! renderer and OCCT into a single Emscripten module, and stages that beside
//! `crates/gridfinity-app/web/`'s page in `dist/`, which is what the Pages
//! workflow uploads.
//!
//! This stands where a `trunk build` would. Trunk drives
//! `wasm32-unknown-emscripten`, whose libc and C++ runtime OCCT needs; nothing
//! else here is unusual, and the flags the link depends on live in
//! `.cargo/config.toml` beside the reason for each.
//!
//! The toolchain is found rather than demanded: `EMSDK` if it is set, else a
//! local `target/emsdk`, and `target/tools/bin` for `wasm-bindgen`, which emcc
//! runs itself under `-sWASM_BINDGEN`.

use std::{
    env, fs,
    path::{Path, PathBuf},
    process::Command,
};

/// Where the finished page is staged, relative to the workspace root.
const DIST: &str = "dist";

/// The two files the Emscripten link emits: its ES module and the module it
/// loads. The page imports the first by name, so both are copied unrenamed.
const ARTIFACTS: [&str; 2] = ["gridfinity-app.js", "gridfinity_app.wasm"];

fn main() {
    let root = workspace_root();
    let emsdk = env::var_os("EMSDK")
        .map(PathBuf::from)
        .unwrap_or_else(|| root.join("target/emsdk"));
    let path = prepend_path(&[
        root.join("target/tools/bin"),
        emsdk.join("upstream/emscripten"),
        emsdk.clone(),
    ]);

    let occt = env::var_os("OCCT_ROOT")
        .map(PathBuf::from)
        .unwrap_or_else(|| root.join("target/occt-install/emscripten"));
    if !occt.join("include/opencascade/TopoDS_Shape.hxx").is_file() {
        run(Command::new("cmake").args(["--preset", "occt-web"]).current_dir(&root).env("EMSDK", &emsdk));
        run(Command::new("cmake")
            .args(["--build", "--preset", "occt-web-install"])
            .current_dir(&root)
            .env("EMSDK", &emsdk));
    }

    run(Command::new("cargo")
        .args([
            "build",
            "--release",
            "--target",
            "wasm32-unknown-emscripten",
            "-p",
            "gridfinity-app",
            "--features",
            "occt",
        ])
        .current_dir(&root)
        .env("EMSDK", &emsdk)
        .env("OCCT_ROOT", &occt)
        .env("PATH", &path));

    let dist = root.join(DIST);
    if dist.exists() {
        fs::remove_dir_all(&dist).expect("the dist directory is removable");
    }
    fs::create_dir_all(&dist).expect("the dist directory is creatable");

    let built = root.join("target/wasm32-unknown-emscripten/release");
    for artifact in ARTIFACTS {
        let from = built.join(artifact);
        assert!(
            from.is_file(),
            "the Emscripten link emits {artifact}, but {} is not there",
            from.display()
        );
        fs::copy(&from, dist.join(artifact)).expect("each artifact is copyable");
    }
    let page = root.join("crates/gridfinity-app/web");
    for entry in fs::read_dir(&page).expect("the page's directory is readable") {
        let entry = entry.expect("each of the page's files is readable");
        fs::copy(entry.path(), dist.join(entry.file_name())).expect("each page file is copyable");
    }

    let wasm: Vec<_> = fs::read_dir(&dist)
        .expect("the staged page is readable")
        .filter_map(|e| e.ok().map(|e| e.file_name()))
        .filter(|n| Path::new(n).extension().is_some_and(|x| x == "wasm"))
        .collect();
    assert_eq!(
        wasm.len(),
        1,
        "the page carries exactly one WebAssembly module, got {wasm:?}"
    );
    println!("Staged {}", dist.display());
}

/// The workspace root, which is two directories above this crate.
fn workspace_root() -> PathBuf {
    let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    manifest
        .parent()
        .and_then(Path::parent)
        .expect("the crate sits two directories below the workspace root")
        .to_path_buf()
}

/// `PATH` with `heads` in front of it, in the order given.
fn prepend_path(heads: &[PathBuf]) -> std::ffi::OsString {
    let existing = env::var_os("PATH").unwrap_or_default();
    let mut all: Vec<PathBuf> = heads.to_vec();
    all.extend(env::split_paths(&existing));
    env::join_paths(all).expect("no path holds the separator character")
}

/// Runs `command`, failing with what it was and how it ended.
fn run(command: &mut Command) {
    let status = command
        .status()
        .unwrap_or_else(|e| panic!("{:?} could not be started: {e}", command.get_program()));
    assert!(
        status.success(),
        "{:?} failed with {status}",
        command.get_program()
    );
}
