// 3D voxel representation of the configuration space. A voxel is blocked when
// the vehicle box cannot occupy it at ANY candidate yaw — feasibility is about
// the VEHICLE VOLUME AND ORIENTATION, never the center point alone.
//
// The rasterization is orientation-aware: a voxel on an oblique slot is free as
// long as *some* yaw of the vehicle box fits there. This is what an earlier
// axis-aligned erosion got wrong — rotating the box into a 30° slot only reduces
// the footprint on the slot's normal, so an axis-aligned test erased holes the
// vehicle can genuinely thread. The per-voxel free-yaw hint propagated here is
// the orientation the search seeds each cell with.
//
// Two culling tiers keep the whole-volume build cheap:
//   1) inscribed-sphere reject — if even the inscribed sphere penetrates, no
//      yaw can help (necessary condition, exact);
//   2) circumscribed-sphere accept — if the full circumsphere clears obstacles
//      AND the water walls, every yaw is free (sufficient condition, exact);
//   3) otherwise an exact oriented-box pose test per candidate yaw.
// The authoritative per-motion test is still the swept translation/rotation in
// planner.js, evaluated at the same yaw the renderer draws.
import {
  worldSdf, boundsSdf, vehicleHalf, poseClearanceExact, obstacleClearance,
} from './geometry.js';

// Candidate yaws, evenly spaced. 12 bins = 30° includes the axis-aligned and the
// 90°-rotated orientations exactly, plus every 30° oblique slot.
export const CANDIDATE_YAWS = Array.from({ length: 12 }, (_, i) => (i * Math.PI) / 12);

// Axis-aligned local probes (corners + face centers). The axis-aligned box is
// the *largest* footprint the vehicle can project, so "the axis-aligned box has
// no contact" is a SUFFICIENT condition for every yaw — a conservative free-space
// test that never claims a voxel usable when some orientation is blocked.
function axisProbes(half) {
  const [hx, hy, hz] = half;
  const pts = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
    pts.push([sx * hx, sy * hy, sz * hz]);
  }
  pts.push([hx, 0, 0], [-hx, 0, 0], [0, hy, 0], [0, -hy, 0], [0, 0, hz], [0, 0, -hz]);
  return pts;
}

export class VoxelGrid {
  // opts: number margin (legacy positional) OR { margin, origin, mode }
  //   origin: grid min-corner override (diagnostics: hold obstacles fixed, move
  //           only the raster origin); defaults to world.bounds.min.
  //   mode:   'oriented' (default) — per-candidate-yaw, admits oblique slots.
  //           'axis'     — axis-aligned erosion. It is a SUPERSET of the blocked
  //                        set (conservative, never phantom-free) and is what the
  //                        fine independent reference audit uses, where the exact
  //                        oriented test over ~766k voxels would be too slow.
  //           'legacy-axis' — the old code path that documented the bug; kept
  //                        only so the diagnose tool can reproduce the failure.
  constructor(world, cellSize, dims, opts = {}) {
    const o = typeof opts === 'number' ? { margin: opts } : opts;
    this.world = world;
    this.cell = cellSize;
    this.margin = o.margin ?? 0.05;
    this.mode = o.mode ?? 'oriented';
    this.half = vehicleHalf(dims);
    this.er = Math.min(...this.half) + this.margin;
    this.circum = Math.hypot(...this.half);
    this.axisProbes = axisProbes(this.half);
    const b = world.bounds;
    this.min = o.origin ? o.origin.slice() : b.min.slice();
    this.nx = Math.round((b.max[0] - b.min[0]) / cellSize);
    this.ny = Math.round((b.max[1] - b.min[1]) / cellSize);
    this.nz = Math.round((b.max[2] - b.min[2]) / cellSize);
    this.n = this.nx * this.ny * this.nz;
    this.blocked = new Uint8Array(this.n);
    this.clearance = new Float32Array(this.n); // obstacle SDF at voxel center
    this.yawHint = new Float32Array(this.n).fill(NaN); // free orientation (NaN = any)
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

  build() {
    const half = this.half, w = this.world, m = this.margin;
    const inR = Math.min(...half), circum = this.circum;
    for (let k = 0; k < this.nz; k++) {
      for (let j = 0; j < this.ny; j++) {
        for (let i = 0; i < this.nx; i++) {
          const p = this.coord(i, j, k);
          const idx = this.idx(i, j, k);
          const s = worldSdf(w, p);
          this.clearance[idx] = s;
          if (this.mode === 'legacy-axis') {
            // Old behaviour: axis-aligned surface probes only (the bug).
            this.blocked[idx] = this._legacyBlocked(p) ? 1 : 0;
            this.yawHint[idx] = 0;
            continue;
          }
          if (this.mode === 'axis') {
            // Conservative lower-bound raster: free only when the axis-aligned
            // box fits. A cheap inscribed-sphere accept (center clearance beats
            // the circumscribed radius, all probes cleared at once) skips the
            // 14 per-point SDFs in open water; this is the hot path for the
            // fine reference audit.
            if (s > this.circum + this.margin && boundsSdf(w, p) > this.circum + this.margin) {
              this.blocked[idx] = 0; this.yawHint[idx] = NaN;
            } else if (this._axisFree(p)) { this.blocked[idx] = 0; this.yawHint[idx] = NaN; }
            else this.blocked[idx] = 1;
            continue;
          }
          if (s > circum + m && boundsSdf(w, p) > circum + m) {
            this.blocked[idx] = 0; // full circumsphere clear: any yaw
            continue;
          }
          if (s < inR + m) { this.blocked[idx] = 1; continue; } // inscribed sphere fails
          let freeYaw = NaN;
          for (const yaw of CANDIDATE_YAWS) {
            if (poseClearanceExact(w, p, yaw, half, m).clear) { freeYaw = yaw; break; }
          }
          if (Number.isNaN(freeYaw)) this.blocked[idx] = 1;
          else this.yawHint[idx] = freeYaw;
        }
      }
    }
    return this;
  }

  _axisFree(p) {
    if (boundsSdf(this.world, p) < this.er) return false;
    for (const [dx, dy, dz] of this.axisProbes) {
      if (worldSdf(this.world, [p[0] + dx, p[1] + dy, p[2] + dz]) < this.margin) return false;
    }
    return true;
  }

  _legacyBlocked(p) {
    const [hx, hy, hz] = this.half;
    const m = this.margin;
    if (boundsSdf(this.world, p) < this.er) return true;
    for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
      if (worldSdf(this.world, [p[0] + sx * hx, p[1] + sy * hy, p[2] + sz * hz]) < m) return true;
    }
    for (const q of [[hx, 0, 0], [-hx, 0, 0], [0, hy, 0], [0, -hy, 0], [0, 0, hz], [0, 0, -hz]]) {
      if (worldSdf(this.world, [p[0] + q[0], p[1] + q[1], p[2] + q[2]]) < m) return true;
    }
    return false;
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
    if (startIdx < 0 || this.blocked[startIdx]) return seen;
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
// origin + cell * (i,j,k)), plus per-voxel clearance and free-yaw hint.
export function voxelPayload(grid, indices) {
  return {
    cell: grid.cell,
    origin: grid.min,
    dims: [grid.nx, grid.ny, grid.nz],
    mode: grid.mode,
    indices: indices.map((idx) => grid.ijkOf(idx)),
    clearance: indices.map((idx) => Number(grid.clearance[idx].toFixed(3))),
    // yaw hint in degrees, or null when any orientation fits
    yawHint: indices.map((idx) => {
      const y = grid.yawHint[idx];
      return Number.isFinite(y) ? Number(((y * 180) / Math.PI).toFixed(1)) : null;
    }),
  };
}

// Clearance of a free voxel at its stored free yaw (for diagnostics).
export function voxelPoseClearance(grid, idx) {
  const c = grid.centerOf(idx);
  const y = Number.isFinite(grid.yawHint[idx]) ? grid.yawHint[idx] : 0;
  return poseClearanceExact(grid.world, c, y, grid.half, -Infinity);
}

export { obstacleClearance };
