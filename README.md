# Gridfinity Expanded

A React 19 + Vite 6 TypeScript application for designing connected Gridfinity bins and exporting printable STL parts.

The editor supports multiple explicitly selected bins, perimeter openings, full-height internal walls, editable printer-aware cuts, magnet recesses, and M3 recesses. The UI resolves cuts into printable parts, then a background worker builds trusted input into triangle soups used directly by both the preview and STL export. The preview is drawn by the Rust workspace's own WebGL2 renderer, shared with its egui debugger.

## Development

```sh
npm install
npm run dev
```

Required validation commands are documented in [`AGENTS.md`](./AGENTS.md).

## Fitting a drawer from the command line

The Rust workspace ships one binary, `gridfinity-app`. With no arguments it opens the egui
construction debugger. With `optimize` it fits a drawer headlessly: give it a TOML naming the
drawer's inside measurements and the objects to organise in it, and it packs them, generates the
dividers, splits the bin for your printer's bed, writes the geometry, and prints what it did —
packing efficiency, unplaced objects, dividers generated, rounding that would not land, and how
many pieces the bed forced.

```sh
# one binary STL per printable piece, into out/
cargo run -- optimize examples/drawer.toml --format stl out

# every piece as a body of one Parasolid transmit file, then open it in the debugger
cargo run -- optimize examples/drawer.toml --format parasolid_x_t drawer.x_t --view
```

[`examples/drawer.toml`](./examples/drawer.toml) is a worked input with every setting spelled out.

## Geometry documentation

[`AGENTS.md`](./AGENTS.md) is the canonical specification and architecture record. It documents the trusted-input contract, shape/wall/cut ownership, solid construction, direct preview, STL export, and printability gates. Normative Gridfinity dimensions and their sources live in `src/lib/gridfinitySpec.ts`.
