import { RENDER_FLOATS_PER_TRIANGLE, RENDER_VERTEX_STRIDE } from '../types';

export function downloadBuffer(buffer: ArrayBuffer, filename: string, mimeType: string): void {
  const blob = new Blob([buffer], { type: mimeType });
  const url = URL.createObjectURL(blob);
  const anchor = document.createElement('a');
  anchor.href = url;
  anchor.download = filename;
  anchor.click();
  URL.revokeObjectURL(url);
}

export function verticesToStl(vertices: Float32Array): ArrayBuffer {
  const triangleCount = vertices.length / RENDER_FLOATS_PER_TRIANGLE;
  const buffer = new ArrayBuffer(84 + triangleCount * 50);
  const view = new DataView(buffer);
  view.setUint32(80, triangleCount, true);

  let offset = 84;
  for (let index = 0; index < vertices.length; index += RENDER_FLOATS_PER_TRIANGLE) {
    const b = index + RENDER_VERTEX_STRIDE;
    const c = index + 2 * RENDER_VERTEX_STRIDE;
    const ax = vertices[index], ay = vertices[index + 1], az = vertices[index + 2];
    const bx = vertices[b], by = vertices[b + 1], bz = vertices[b + 2];
    const cx = vertices[c], cy = vertices[c + 1], cz = vertices[c + 2];
    const ux = bx - ax, uy = by - ay, uz = bz - az;
    const vx = cx - ax, vy = cy - ay, vz = cz - az;
    let nx = uy * vz - uz * vy;
    let ny = uz * vx - ux * vz;
    let nz = ux * vy - uy * vx;
    const length = Math.hypot(nx, ny, nz);
    nx /= length;
    ny /= length;
    nz /= length;

    view.setFloat32(offset, nx, true);
    view.setFloat32(offset + 4, ny, true);
    view.setFloat32(offset + 8, nz, true);
    const corners = [ax, ay, az, bx, by, bz, cx, cy, cz];
    for (let component = 0; component < corners.length; component++) {
      view.setFloat32(offset + 12 + component * 4, corners[component], true);
    }
    view.setUint16(offset + 48, 0, true);
    offset += 50;
  }
  return buffer;
}

export function downloadStl(
  vertices: Float32Array,
  filename = 'gridfinity-bin.stl',
): void {
  downloadBuffer(verticesToStl(vertices), filename, 'model/stl');
}
