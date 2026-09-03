# Gridfinity Expanded

A Rust/egui parametric CAD application for designing connected Gridfinity bins and exporting printable parts. The same egui application runs on desktop and in the browser; the browser page is only a canvas host.

The editor supports multiple explicitly selected bins, perimeter openings, full-height internal walls, editable printer-aware cuts, magnet recesses, and M3 recesses. Geometry and interface run in one process on the desktop, and in one WebAssembly instance in the browser. OCCT is being introduced behind the `occt` feature while the legacy analytic kernel remains available during migration.

## Development

One cargo workspace, rooted here; every command runs from the repository root.

```sh
cargo run                 # the application
cargo test --workspace    # the suite
```

Required validation commands are documented in [`CLAUDE.md`](./CLAUDE.md).

The OCCT-backed crates are behind the `occt` feature and need OCCT built for the host:

```sh
cmake --preset occt-native
cmake --build --preset occt-native-install
OCCT_ROOT=target/occt-install/native cargo test -p gridfinity-occt --features occt
OCCT_ROOT=target/occt-install/native cargo test -p gridfinity-xt --features occt
```

The browser page is built by its own crate, which links the application, the renderer
and OCCT into a single WebAssembly module and stages the page around it in `dist/`:

```sh
cargo run -p gridfinity-web --release
```

That needs `wasm32-unknown-emscripten`, Emscripten 6.0.5, wasm-bindgen-cli 0.2.126,
CMake and Ninja; it configures and builds the pinned `vendor/occt` submodule on first
use. A local emsdk in `target/emsdk` and wasm-bindgen in `target/tools/bin` are found
automatically. Serve `dist/` with any static file server.

## Fitting a drawer from the command line

The Rust workspace ships one binary, `gridfinity-app`. With no arguments it opens the egui
construction debugger. With `optimize` it fits a drawer headlessly: give it a TOML naming the
drawer's inside measurements and the objects to organise in it, and it packs them, hollows a
compartment for each, splits every body for your printer's bed, writes the geometry, and prints
what it did — packing efficiency, the compartments hollowed, rounding that would not land, and how
many pieces the bed forced.

`--mode` is required and says what to build. `--mode walls` makes the whole drawer **one bin**,
solid everywhere no object was packed and hollowed to a compartment per object. `--mode bins` makes
**one ordinary Gridfinity bin per object**, sized to hold that object's whole quantity as its own
compartments and shaped to the cells its compartments actually reach, with every bin dropping into
the one baseplate. Either way, an object the packer cannot place fails the run rather than being
quietly left out.

```sh
# one binary STL per printable piece, into out/
cargo run -- optimize examples/drawer.toml --mode walls --format stl -o out

# every piece as a body of one Parasolid transmit file, then open it in the debugger
# (.x_t names the format, so --format may be left off)
cargo run -- optimize examples/drawer.toml --mode walls -o drawer.x_t --view

# a Gridfinity bin of its own for every object, fitted and just looked at, writing nothing
cargo run -- optimize examples/drawer-of-bins.toml --mode bins --view
```

`--mode` must be given: the two modes build entirely different sets of parts out of one file.
`-o` names the output and at least one of `-o` and `--view` must be given. `--format` is inferred
from a `.x_t` output and required otherwise, since an STL run writes a directory of one file per
piece.

[`examples/drawer.toml`](./examples/drawer.toml) is a worked `--mode walls` input with every
setting spelled out, and [`examples/drawer-of-bins.toml`](./examples/drawer-of-bins.toml) is a
worked `--mode bins` one. A discrete bin is a whole number of cells on each axis, so the same
drawer holds less as separate bins than it does divided: the first file's object list is more than
`--mode bins` can fit, and the run names the object it ran out of room for rather than dropping
it.

## Geometry documentation

[`CLAUDE.md`](./CLAUDE.md) is the canonical specification and architecture record. It documents the trusted-input contract, shape/wall/cut ownership, solid construction, direct preview, STL export, and printability gates. Normative Gridfinity dimensions live in `crates/gridfinity-model/src/gridfinity/spec.rs`.
