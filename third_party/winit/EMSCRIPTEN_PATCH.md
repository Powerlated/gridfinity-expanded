# Local Emscripten compatibility patch

This is winit 0.30.13 (Apache-2.0), copied from its Cargo distribution. Its web
platform cfg is enabled for the Emscripten wasm family as well as
`wasm32-unknown-unknown`. The web dependencies were already selected for the
whole wasm target family.
