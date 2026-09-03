# Third-party source distributions

## Open CASCADE Technology 8.0.1

`vendor/occt` is an unmodified Git submodule pinned to OCCT 8.0.1. It is
licensed under GNU LGPL 2.1 with the Open CASCADE exception. The authoritative
notices and source are retained in the submodule (`LICENSE_LGPL_21.txt` and
`OCCT_LGPL_EXCEPTION.txt`). Distributions that link OCCT must include those
notices, identify local OCCT modifications, and provide the corresponding OCCT
source/build materials required by the license. This notice is engineering
guidance, not legal advice.

## eframe 0.35.0, egui-wgpu 0.35.0, and winit 0.30.13

`third_party/` contains patched Cargo source distributions needed to route
Emscripten through winit and wgpu's GLES backend. eframe and egui-wgpu are
MIT OR Apache-2.0; winit is Apache-2.0. Their upstream license files and Cargo
metadata are retained, and each patch is described in an
`EMSCRIPTEN_PATCH.md` beside the source.
