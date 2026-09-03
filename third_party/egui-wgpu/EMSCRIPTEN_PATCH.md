# Local Emscripten compatibility patch

This is `egui-wgpu` 0.35.0 (MIT OR Apache-2.0), copied from the Cargo source
distribution. The only source change gates the WebGPU secure-context probe out
for `target_os = "emscripten"`; wgpu does not expose its `web_sys` re-export on
that backend. Remove this patch when an upstream release carries the fix.
