/**
 * Gridfinity dimensions used by the generator. This module deliberately has
 * no imports so the specification can be shared by UI, tests, Node scripts,
 * and the geometry worker without pulling in application or geometry code.
 */

/** Compatibility dimensions from the original design and maintained references. */
export const GRIDFINITY_SPEC = {
  gridPitch: 42,
  heightUnit: 7,
  outerTopWidth: 41.5,
  outerCornerRadius: 3.75,
  baseProfile: {
    height: 4.75,
    lowerChamferHeight: 0.8,
    upperChamferStart: 2.6,
    bottomWidth: 35.6,
    middleWidth: 37.2,
    bottomRadius: 0.8,
    middleRadius: 1.5,
  },
  baseHeight: 7,
  floorThickness: 1.2,
  hardware: {
    magnet: {
      nominalDiameter: 6,
      nominalThickness: 2,
      recessDiameter: 6.5,
      recessDepth: 2.4,
    },
    m3: {
      recessDiameter: 3,
      recessDepth: 6,
    },
    centerOffset: 13,
  },
} as const;

/** Measurements derived from the normative compatibility dimensions above. */
export const GRIDFINITY_DERIVED = {
  baseBridgeHeight:
    GRIDFINITY_SPEC.baseHeight - GRIDFINITY_SPEC.baseProfile.height,
  perimeterClearancePerSide:
    (GRIDFINITY_SPEC.gridPitch - GRIDFINITY_SPEC.outerTopWidth) / 2,
} as const;

/** Product defaults are choices, not Gridfinity compatibility requirements. */
export const DESIGN_DEFAULTS = {
  heightUnits: 3,
  minimumHeightUnits: 2,
  perimeterThickness: 1.2,
  filletRadius: 2.8,
  fasteners: { magnets: false, m3: false },
  printerName: 'Prusa MK4 / MK3S+',
} as const;

/** Planning, manufacturing, and CSG allowances are intentionally non-normative. */
export const IMPLEMENTATION_ALLOWANCES = {
  bedClearancePerSide: 5,
  minimumStraightCavityWall: 0.2,
} as const;

export const GRIDFINITY_SOURCES = [
  {
    title: 'Gridfinity specification',
    url: 'https://gridfinity.xyz/specification/',
    scope: '42 mm grid, 7 mm height units, 6 × 2 mm magnets, and M3 screws',
  },
  {
    title: 'Gridfinity Rebuilt OpenSCAD',
    url: 'https://github.com/kennetek/gridfinity-rebuilt-openscad',
    scope: 'Parametric base-profile and compatibility dimensions',
  },
  {
    title: 'Gridfinity Documentation — original specification',
    url: 'https://stu142.com/Gridfinity-Documentation/',
    scope: 'Original-bin profile drawings and hardware placement',
  },
] as const;

export function gridfinityHeight(heightUnits: number): number {
  return heightUnits * GRIDFINITY_SPEC.heightUnit;
}

/** Largest floor fillet that still leaves a straight cavity wall at this bin height. */
export function maximumFilletRadius(height: number): number {
  const cavityDepth = height - GRIDFINITY_SPEC.baseHeight - GRIDFINITY_SPEC.floorThickness;
  return Math.max(0, cavityDepth - IMPLEMENTATION_ALLOWANCES.minimumStraightCavityWall);
}
