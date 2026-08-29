import { GRIDFINITY_SPEC } from '../../lib/gridfinitySpec';

const GRID_PITCH = GRIDFINITY_SPEC.gridPitch;

// The coordinate system for the Walls and Cuts SVG editors, in one place so
// every overlay (grid, cells, edges, cuts, custom walls) maps the same
// way — changing CELL/PAD (zoom, larger padding) can never desync one of them.

/** SVG units per grid cell / canvas padding. */
export const CELL = 40;
export const PAD = 8;

/** Grid-line index → SVG unit. */
export const gridToSvg = (i: number): number => PAD + i * CELL;
/** Whole-bin mm → SVG unit. */
export const mmToSvg = (mm: number): number => PAD + (mm / GRID_PITCH) * CELL;
/** SVG unit → whole-bin mm. */
export const svgToMm = (u: number): number => ((u - PAD) / CELL) * GRID_PITCH;
/** A length in mm → a length in SVG units. */
export const mmSpanToSvg = (mm: number): number => (mm / GRID_PITCH) * CELL;

/** A pointer event's position in the canvas's own viewBox units. */
export function pointerToUnits(
  svg: SVGSVGElement, e: { clientX: number; clientY: number },
): { x: number; y: number } {
  const rect = svg.getBoundingClientRect();
  const view = svg.viewBox.baseVal;
  return {
    x: ((e.clientX - rect.left) / rect.width) * view.width,
    y: ((e.clientY - rect.top) / rect.height) * view.height,
  };
}

/** A pointer event's position in whole-bin mm, from the canvas's own viewBox. */
export function pointerToMm(
  svg: SVGSVGElement, e: { clientX: number; clientY: number },
): { x: number; y: number } {
  const units = pointerToUnits(svg, e);
  return { x: svgToMm(units.x), y: svgToMm(units.y) };
}
