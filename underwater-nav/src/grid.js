// 3D voxel representation of the configuration space. A voxel is blocked when
// the vehicle box, placed at the voxel center, cannot fit: feasibility is about
// the VEHICLE VOLUME, never the center point alone.
//
// Two-stage culling:
//   1) grid build: per-axis half-extent erosion against axis-aligned SDFs — a
//      necessary (not sufficient) condition for the oriented box, cheap enough
//      to evaluate over the whole volume;
//   2) exact oriented-box swept checks on every search edge (see planner.js),
//      which are the authoritative collision test.
import { worldSdf, boundsSdf, vehicleHalf } from './geometry.js';

// Local box-surface probes used for the axis-aligned erosion.
function localProbes(half) {
  const [hx, hy, hz] = half;
  const corners = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
    corners.push([sx * hx, sy * hy, sz * hz]);
  }
  const faces = [
    [hx, 0, 0], [-hx, 0, 0], [0, hy, 0], [0, -hy, 0], [0, 0, hz], [0, 0, -hz],
  ];
  return [...corners, ...faces];
}

export class VoxelGrid {
  constructor(world, cellSize, dims, margin = 0.05) {
    this.world = world;
    this.cell = cellSize;
    this.margin = margin;
    this.half = vehicleHalf(dims);
    // Inscribed radius: keeps the whole box inside the water volume.
    this.er = Math.min(...this.half) + margin;
    // Axis-aligned surface probes (world frame, yaw = 0). The swept edge test
    // may additionally rotate the box into an angled slot, but rotation can
    // never make an axis-aligned collision disappear — so this erosion is a
    // necessary condition for every orientation and never marks a voxel free
    // that the box cannot occupy.
    this.probes = localProbes(this.half);
    const b = world.bounds;
    this.min = b.min;
    this.nx = Math.round((b.max[0] - b.min[0]) / cellSize);
    this.ny = Math.round((b.max[1] - b.min[1]) / cellSize);
    this.nz = Math.round((b.max[2] - b.min[2]) / cellSize);
    this.n = this.nx * this.ny * this.nz;
    this.blocked = new Uint8Array(this.n);
    this.clearance = new Float32Array(this.n); // obstacle SDF at voxel center
  }

  idx(i, j, k) { return (k * this.ny + j) * this.nx + i; }

  coord(i, j, k) {
    return [
      this.min[0] + (i + 0.5) * this.cell,
      this.min[1] + (j + 0.5) * this.cell,
      this.min[2] + (k + 0.5) * this.cell,
    ];
  }

  nearestIndex(p) {
    const i = Math.round((p[0] - this.min[0]) / this.cell - 0.5);
    const j = Math.round((p[1] - this.min[1]) / this.cell - 0.5);
    const k = Math.round((p[2] - this.min[2]) / this.cell - 0.5);
    if (i < 0 || j < 0 || k < 0 || i >= this.nx || j >= this.ny || k >= this.nz) return -1;
    return this.idx(i, j, k);
  }

  ijkOf(idx) {
    const i = idx % this.nx;
    const j = ((idx - i) / this.nx) % this.ny;
    const k = Math.floor(idx / (this.nx * this.ny));
    return [i, j, k];
  }

  centerOf(idx) {
    const [i, j, k] = this.ijkOf(idx);
    return this.coord(i, j, k);
  }

  // Build the configuration-space occupancy using axis-aligned box erosion:
  // a voxel is free only if the full axis-aligned vehicle box fits at its
  // center. This is necessary for any yawed orientation, so the swept edge
  // test (which can rotate into angled slots) never relies on a voxel this
  // pass marks blocked. Feasibility is a vehicle-volume statement, not a
  // center-point one.
  build() {
    for (let k = 0; k < this.nz; k++) {
      for (let j = 0; j < this.ny; j++) {
        for (let i = 0; i < this.nx; i++) {
          const p = this.coord(i, j, k);
          const s = worldSdf(this.world, p);
          const idx = this.idx(i, j, k);
          this.clearance[idx] = s;
          if (boundsSdf(this.world, p) < this.er) {
            this.blocked[idx] = 1;
            continue;
          }
          for (const [dx, dy, dz] of this.probes) {
            if (worldSdf(this.world, [p[0] + dx, p[1] + dy, p[2] + dz]) < this.margin) {
              this.blocked[idx] = 1;
              break;
            }
          }
        }
      }
    }
    return this;
  }

  // 26-connected neighbours of a voxel, with Euclidean step lengths.
  *neighbors(idx) {
    const [i, j, k] = this.ijkOf(idx);
    for (let dk = -1; dk <= 1; dk++) {
      const k2 = k + dk;
      if (k2 < 0 || k2 >= this.nz) continue;
      for (let dj = -1; dj <= 1; dj++) {
        const j2 = j + dj;
        if (j2 < 0 || j2 >= this.ny) continue;
        for (let di = -1; di <= 1; di++) {
          if (di === 0 && dj === 0 && dk === 0) continue;
          const i2 = i + di;
          if (i2 < 0 || i2 >= this.nx) continue;
          const nIdx = this.idx(i2, j2, k2);
          if (this.blocked[nIdx]) continue;
          const step = this.cell * Math.hypot(di, dj, dk);
          yield [nIdx, step];
        }
      }
    }
  }

  // Flood fill from a start voxel; returns marked reachable voxels (26-conn).
  floodFill(startIdx) {
    const seen = new Uint8Array(this.n);
    if (this.blocked[startIdx]) return seen;
    const queue = [startIdx];
    seen[startIdx] = 1;
    let head = 0;
    while (head < queue.length) {
      const cur = queue[head++];
      for (const [nb] of this.neighbors(cur)) {
        if (!seen[nb]) {
          seen[nb] = 1;
          queue.push(nb);
        }
      }
    }
    return seen;
  }

  // Blocking voxels: blocked cells directly adjacent (26-conn) to the reached
  // free component — the physical shell that stops the vehicle, not every
  // solid voxel in the world. Capped for rendering bandwidth.
  blockingFrontier(reached, cap = 6000) {
    const frontier = [];
    for (let idx = 0; idx < this.n && frontier.length < cap; idx++) {
      if (!reached[idx]) continue;
      const [i, j, k] = this.ijkOf(idx);
      for (let dk = -1; dk <= 1 && frontier.length < cap; dk++) {
        for (let dj = -1; dj <= 1 && frontier.length < cap; dj++) {
          for (let di = -1; di <= 1 && frontier.length < cap; di++) {
            const i2 = i + di, j2 = j + dj, k2 = k + dk;
            if (i2 < 0 || j2 < 0 || k2 < 0 || i2 >= this.nx || j2 >= this.ny || k2 >= this.nz) continue;
            const nIdx = this.idx(i2, j2, k2);
            if (this.blocked[nIdx]) frontier.push(nIdx);
          }
        }
      }
    }
    return [...new Set(frontier)];
  }
}

// Compact integer-array payload for the frontend (positions derived from
// origin + cell * (i,j,k)), plus per-voxel clearance for color coding.
export function voxelPayload(grid, indices) {
  return {
    cell: grid.cell,
    origin: grid.min,
    dims: [grid.nx, grid.ny, grid.nz],
    indices: indices.map((idx) => grid.ijkOf(idx)),
    clearance: indices.map((idx) => Number(grid.clearance[idx].toFixed(3))),
  };
}
