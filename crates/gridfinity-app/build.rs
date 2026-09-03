//! Supplies the EGL 1.5 entry points the Emscripten link needs and Emscripten's
//! own EGL does not define; see `emscripten/egl15_stubs.c` for why they are
//! referenced at all. Every other target builds nothing here.

use std::env;

fn main() {
    println!("cargo:rerun-if-changed=emscripten/egl15_stubs.c");
    if env::var("TARGET").as_deref() != Ok("wasm32-unknown-emscripten") {
        return;
    }
    let mut stubs = cc::Build::new();
    stubs.file("emscripten/egl15_stubs.c").warnings(true);
    if env::var_os("CC_wasm32_unknown_emscripten").is_none() {
        stubs.compiler("emcc");
    }
    if env::var_os("AR_wasm32_unknown_emscripten").is_none() {
        stubs.archiver("emar");
    }
    stubs.compile("gridfinity_egl15_stubs");
}
