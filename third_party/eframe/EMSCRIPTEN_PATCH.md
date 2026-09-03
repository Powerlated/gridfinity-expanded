# Local Emscripten compatibility patch

This is `eframe` 0.35.0 (MIT OR Apache-2.0), copied from its Cargo source
distribution. Target gates treat `wasm32-unknown-emscripten` as a winit/native
runner target, while retaining `WebRunner` for `wasm32-unknown-unknown`.

This is necessary because eframe's WebRunner creates wgpu's browser `Canvas`
surface directly, while wgpu intentionally selects its GLES backend on
Emscripten. Remove the patch when upstream supports this target directly.
