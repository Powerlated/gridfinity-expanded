// Palette for logical bins in the editors (stable id hash → color).
export const BIN_COLORS = [
  '#2563eb', '#16a34a', '#d97706', '#dc2626',
  '#9333ea', '#0d9488', '#db2777', '#65a30d',
];

export function binColor(binId: string | undefined): string {
  let hash = 0;
  for (const char of binId ?? '') hash = (hash * 31 + char.charCodeAt(0)) >>> 0;
  return BIN_COLORS[hash % BIN_COLORS.length];
}
