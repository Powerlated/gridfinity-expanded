use std::{
    env,
    path::{Path, PathBuf},
};

const TOOLKITS: &[&str] = &[
    "TKOffset",
    "TKFillet",
    "TKBO",
    "TKMesh",
    "TKBool",
    "TKShHealing",
    "TKPrim",
    "TKTopAlgo",
    "TKGeomAlgo",
    "TKBRep",
    "TKGeomBase",
    "TKG3d",
    "TKG2d",
    "TKMath",
    "TKernel",
];

/// Resolves `value` against the workspace root when it is relative, so the
/// paths this crate documents (`target/occt-install/native`) name the same
/// prefix from a build script whose working directory is the crate.
fn absolute(value: &Path) -> PathBuf {
    if value.is_absolute() {
        return value.to_path_buf();
    }
    let manifest =
        PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("cargo sets CARGO_MANIFEST_DIR"));
    let workspace = manifest
        .parent()
        .and_then(Path::parent)
        .expect("the crate sits two directories below the workspace root");
    workspace.join(value)
}

/// The subdirectories of `dir`, empty when it is not a readable directory.
fn subdirectories(dir: &Path) -> Vec<PathBuf> {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return Vec::new();
    };
    entries
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| path.is_dir())
        .collect()
}

/// The directory of `root` holding the toolkit archives: `lib` if it holds a
/// built `TKernel`, else the first `<platform>/<toolchain>/lib` that does.
/// OCCT installs under a platform triple (`win64/clang/lib`) on every target
/// its own `custom` script covers, so `lib` alone does not locate it.
fn library_dir(root: &Path) -> PathBuf {
    let holds_tkernel = |dir: &Path| {
        [
            "TKernel.lib",
            "libTKernel.a",
            "libTKernel.so",
            "libTKernel.dylib",
        ]
        .iter()
        .any(|name| dir.join(name).is_file())
    };
    let direct = root.join("lib");
    if holds_tkernel(&direct) {
        return direct;
    }
    let mut candidates: Vec<PathBuf> = subdirectories(root)
        .iter()
        .flat_map(|platform| subdirectories(platform))
        .map(|toolchain| toolchain.join("lib"))
        .filter(|dir| holds_tkernel(dir))
        .collect();
    candidates.sort();
    assert!(
        !candidates.is_empty(),
        "OCCT_ROOT holds no TKernel archive under lib/ or <platform>/<toolchain>/lib/; build the install target or set OCCT_LIB_DIR: {}",
        root.display()
    );
    candidates.swap_remove(0)
}

fn main() {
    println!("cargo:rerun-if-changed=cpp/bridge.cpp");
    println!("cargo:rerun-if-changed=include/gridfinity_occt.h");
    println!("cargo:rerun-if-env-changed=OCCT_ROOT");
    println!("cargo:rerun-if-env-changed=OCCT_LIB_DIR");
    println!("cargo:rerun-if-env-changed=OCCT_LINK_KIND");

    if env::var_os("CARGO_FEATURE_OCCT").is_none() {
        return;
    }

    let root = absolute(Path::new(&env::var_os("OCCT_ROOT").expect(
        "the `occt` feature requires OCCT_ROOT to point at an OCCT install prefix; run cmake --preset occt-native-install first",
    )));
    let emscripten = env::var("TARGET").is_ok_and(|t| t == "wasm32-unknown-emscripten");
    let include = root.join("include").join("opencascade");
    if !include.join("TopoDS_Shape.hxx").is_file() {
        panic!(
            "OCCT_ROOT has no include/opencascade/TopoDS_Shape.hxx: {}",
            root.display()
        );
    }

    let mut bridge = cc::Build::new();
    bridge
        .cpp(true)
        .std("c++17")
        .include(&include)
        .include("include")
        .file("cpp/bridge.cpp")
        .warnings(true);
    if emscripten {
        // cc-rs names the Emscripten tools `em++.bat` and `emar.bat` on
        // Windows; emsdk 6 ships `.exe` wrappers and no batch ones, so name the
        // tools themselves and let PATH resolve the extension.
        if env::var_os("CXX_wasm32_unknown_emscripten").is_none() {
            bridge.compiler("em++");
        }
        if env::var_os("AR_wasm32_unknown_emscripten").is_none() {
            bridge.archiver("emar");
        }
        // rustc links this target with `-fwasm-exceptions`, and one module
        // carries one exception model, so the bridge that catches OCCT's
        // exceptions must be compiled with the same one the toolkits were.
        bridge.flag("-fwasm-exceptions");
    }
    bridge.compile("gridfinity_occt_bridge");

    let lib = env::var_os("OCCT_LIB_DIR")
        .map(|dir| absolute(Path::new(&dir)))
        .unwrap_or_else(|| library_dir(&root));
    println!("cargo:rustc-link-search=native={}", lib.display());
    let kind = env::var("OCCT_LINK_KIND").unwrap_or_else(|_| "static".into());
    for toolkit in TOOLKITS {
        println!("cargo:rustc-link-lib={kind}={toolkit}");
    }
    if env::var("CARGO_CFG_TARGET_OS").as_deref() == Ok("windows") {
        println!("cargo:rustc-link-lib=user32");
        println!("cargo:rustc-link-lib=advapi32");
    }
}
