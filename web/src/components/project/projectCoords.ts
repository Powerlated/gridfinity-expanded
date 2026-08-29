import { pointerToUnits } from '../sidebar/editorCoords';
import { partsBounds } from '../../lib/project/rects';
import type { Point2, Rect } from '../../lib/types';

export const OBJECT_PAD = 8;
export const OBJECT_UNITS_PER_MM = 3;
export const OBJECT_GRID_MM = 10;
export const OBJECT_FIELD_MIN_MM = 120;
export const OBJECT_FIELD_STEP_MM = OBJECT_GRID_MM * 2;

export function objectFieldMm(parts: Rect[]): number {
  const bounds = partsBounds(parts);
  const extent = Math.max(bounds.x + bounds.width, bounds.y + bounds.depth);
  const stepped = Math.ceil(extent / OBJECT_FIELD_STEP_MM) * OBJECT_FIELD_STEP_MM;
  return Math.max(OBJECT_FIELD_MIN_MM, stepped + OBJECT_FIELD_STEP_MM);
}

export function objectMmToSvg(mm: number): number {
  return OBJECT_PAD + mm * OBJECT_UNITS_PER_MM;
}

export function objectSvgToMm(units: number): number {
  return (units - OBJECT_PAD) / OBJECT_UNITS_PER_MM;
}

export function objectPointerToMm(svg: SVGSVGElement, event: { clientX: number; clientY: number }): Point2 {
  const units = pointerToUnits(svg, event);
  return { x: objectSvgToMm(units.x), y: objectSvgToMm(units.y) };
}
