// True 3D A* over the orientation-aware voxel configuration space. Vertical and
// diagonal moves are first-class: all 26 neighbours are expanded with Euclidean
// costs, so routes can climb, dive and weave instead of following a height-tagged
// horizontal map.
//
// MOTION MODEL — "yaw lane". A submersible holds a heading and thrusts in any
// direction (crabbing). We therefore plan one A* per candidate heading ("lane"):
// the vehicle rotates in place at the start to the lane yaw (verified), then
// translates through every hop at that CONSTANT yaw, exactly the pose the
// renderer draws. This is why an oblique slot is threadable at its own angle
// while the gate is crossed along x, and why a wide vehicle can pass a hallway
// broadside (lane 90°). The first lane that reaches the goal wins; the lowest
// lane that fits is usually the most natural heading.
//
// Correctness: every expanded edge is re-checked with the exact oriented-box
// swept translation at the SAME margin the final verification uses, so a lane
// the search accepts is a lane the verifier accepts — the delivered route can
// never be one the collision model rejects.
import {
  dist, vehicleHalf, sweptTranslation, rotationClearance, poseClearanceExact,
  angleDistance, heading, worldSdf,
} from './geometry.js';

class MinHeap {
  constructor() { this.a = []; }
  get size() { return this.a.length; }
  push(node) {
    const a = this.a;
    a.push(node);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (a[p].f <= a[i].f) break;
      [a[p], a[i]] = [a[i], a[p]];
      i = p;
    }
  }
  pop() {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1, r = l + 1;
        let s = i;
        if (l < a.length && a[l].f < a[s].f) s = l;
        if (r < a.length && a[r].f < a[s].f) s = r;
        if (s === i) break;
        [a[s], a[i]] = [a[i], a[s]];
        i = s;
      }
    }
    return top;
  }
}

// Candidate headings, 30° apart. The vehicle box is symmetric under a 180° yaw,
// so θ and θ+180° are the same pose; we normalise every lane to its
// representative in (-90°, 90°] and include 0 (aligned with x) and ±60/±30/90.
export const LANE_YAWS = Array.from({ length: 12 }, (_, i) => {
  let y = (i * Math.PI) / 6;
  // fold into (-pi/2, pi/2]
  while (y <= -Math.PI / 2) y += Math.PI;
  while (y > Math.PI / 2) y -= Math.PI;
  return y;
})

// Plan a single-lane route at a fixed heading. Returns null if this lane cannot
// reach the goal. Edges are verified with the exact swept translation at `yaw`.
function planLane(grid, world, dims, startIdx, goalIdx, yaw, motionStep, safeMargin, maxExpansions) {
  const half = vehicleHalf(dims);
  const goal = grid.centerOf(goalIdx);
  const gScore = new Float64Array(grid.n).fill(Infinity);
  const came = new Int32Array(grid.n).fill(-1);
  const closed = new Uint8Array(grid.n);
  const heap = new MinHeap();
  const edgeCache = new Map();
  const edgeKey = (a, b) => (a < b ? a * grid.n + b : b * grid.n + a);

  const circum = Math.hypot(half[0], half[1], half[2]);
  const inRadius = Math.min(half[0], half[1], half[2]);
  let expansions = 0;
  let sweptRejected = 0;
  let fastAccepted = 0;

  const edgeSafe = (aIdx, bIdx) => {
    const key = edgeKey(aIdx, bIdx);
    const cached = edgeCache.get(key);
    if (cached !== undefined) return cached;
    const ca = grid.centerOf(aIdx), cb = grid.centerOf(bIdx);
    const mid = [(ca[0] + cb[0]) / 2, (ca[1] + cb[1]) / 2, (ca[2] + cb[2]) / 2];
    const midS = worldSdf(world, mid);
    if (midS < inRadius + grid.margin) {
      edgeCache.set(key, false);
      sweptRejected++;
      return false;
    }
    if (grid.clearance[aIdx] > circum + grid.margin
      && grid.clearance[bIdx] > circum + grid.margin
      && midS > circum + grid.margin) {
      edgeCache.set(key, true);
      fastAccepted++;
      return true;
    }
    const res = sweptTranslation(world, ca, cb, yaw, half, motionStep, safeMargin);
    edgeCache.set(key, res.clear);
    if (!res.clear) sweptRejected++;
    return res.clear;
  };

  gScore[startIdx] = 0;
  heap.push({ idx: startIdx, f: dist(grid.centerOf(startIdx), goal) });
  let found = false;
  while (heap.size) {
    const { idx } = heap.pop();
    if (closed[idx]) continue;
    if (idx !== startIdx && came[idx] === -1) continue;
    if (idx !== startIdx && !edgeSafe(came[idx], idx)) continue;
    if (idx === goalIdx) { found = true; break; }
    closed[idx] = 1;
    if (++expansions > maxExpansions) break;
    for (const [nb, step] of grid.neighbors(idx)) {
      if (closed[nb]) continue;
      const tentative = gScore[idx] + step;
      if (tentative < gScore[nb]) {
        gScore[nb] = tentative;
        came[nb] = idx;
        heap.push({ idx: nb, f: tentative + dist(grid.centerOf(nb), goal) });
      }
    }
  }
  if (!found) {
    return { ok: false, expansions, sweptRejected, fastAccepted };
  }
  const idxPath = [];
  let cur = goalIdx;
  while (cur !== -1) { idxPath.push(cur); cur = came[cur]; }
  idxPath.reverse();
  return {
    ok: true, yaw, idxPath, path: idxPath.map((idx) => grid.centerOf(idx)),
    expansions, sweptRejected, fastAccepted,
  };
}

export function planPath(grid, world, startIdx, goalIdx, dims, opts = {}) {
  const motionStep = opts.motionStep ?? grid.cell;
  // SAFE is the guaranteed continuous clearance certified by the sweep; the grid
  // is built at SAFE + motionStep/2 so a free pose clears the sweep's demand.
  const safeMargin = opts.safeMargin ?? Math.max(0, grid.margin - motionStep / 2);
  const maxExpansions = opts.maxExpansions ?? 400000;
  const half = vehicleHalf(dims);
  const startCenter = grid.centerOf(startIdx);

  // Lanes to try, in order of rotation cost from the vehicle's initial heading
  // (0 = long axis along +x). A lane whose in-place rotation at the start is
  // blocked is skipped outright.
  const startYaw = opts.startYaw ?? 0;
  // De-duplicate the folded lanes, then order by rotation cost from the start.
  const uniq = [];
  for (const y of (opts.lanes ?? LANE_YAWS)) {
    if (!uniq.some((u) => angleDistance(u, y) < 1e-6)) uniq.push(y);
  }
  const lanes = uniq.sort((a, b) => angleDistance(a, startYaw) - angleDistance(b, startYaw));

  let agg = { expansions: 0, sweptRejected: 0, fastAccepted: 0 };
  const attempts = [];
  for (const yaw of lanes) {
    const rot = rotationClearance(world, startCenter, startYaw, yaw, half, safeMargin);
    if (!rot.clear) { attempts.push({ yaw, skip: 'rotate_blocked' }); continue; }
    const res = planLane(grid, world, dims, startIdx, goalIdx, yaw, motionStep, safeMargin, maxExpansions);
    agg.expansions += res.expansions;
    agg.sweptRejected += res.sweptRejected;
    agg.fastAccepted += res.fastAccepted;
    attempts.push({ yaw, ok: res.ok, expansions: res.expansions });
    if (res.ok) {
      return {
        ...res,
        startYaw,
        laneYaw: yaw,
        startRotateClear: rot.clearance,
        attempts,
        ...agg,
      };
    }
  }

  // No lane reached the goal: report the failure with the reached component of
  // the lane that got furthest is not meaningful across lanes, so use the grid
  // flood fill (conservative) for the blocking display.
  return { ok: false, reason: 'unreachable', attempts, ...agg };
}

// Shortcutting that keeps the committed lane yaw. A chord is accepted only when
// the vehicle, held at the lane yaw, sweeps it clear — the pose the renderer
// draws. Shortcuts only remove waypoints; the safety criterion never relaxes.
//
// The dense timeline carries the yaw per sample so verification and playback use
// exactly the same orientation.
export function smoothPath(grid, world, path, dims, opts = {}) {
  const half = vehicleHalf(dims);
  const motionStep = opts.motionStep ?? grid.cell;
  const safeMargin = opts.safeMargin ?? Math.max(0, grid.margin - motionStep / 2);
  const yaw = opts.laneYaw ?? opts.yaws?.[0] ?? 0;
  const startYaw = opts.startYaw ?? 0;

  const safe = (a, b) => sweptTranslation(world, a, b, yaw, half, motionStep, safeMargin);

  let waypoints = path.slice();
  let changed = true;
  let passes = 0;
  while (changed && passes < 40) {
    changed = false;
    passes++;
    const out = [waypoints[0]];
    let anchor = 0;
    while (anchor < waypoints.length - 1) {
      let furthest = anchor + 1;
      for (let k = waypoints.length - 1; k > anchor + 1; k--) {
        if (safe(waypoints[anchor], waypoints[k]).clear) { furthest = k; break; }
      }
      out.push(waypoints[furthest]);
      if (furthest !== anchor + 1) changed = true;
      anchor = furthest;
    }
    waypoints = out;
  }

  // Dense timeline: in-place rotation at the first waypoint (start heading ->
  // lane yaw), then constant-yaw translation along each leg.
  const samples = [];
  const sampleYaw = [];
  const push = (p, y) => { samples.push(p.slice()); sampleYaw.push(y); };
  push(waypoints[0], startYaw);
  const rotDelta = yaw - startYaw;
  if (Math.abs(rotDelta) > 1e-9) {
    const nr = Math.max(1, Math.ceil(Math.abs(rotDelta) / (Math.PI / 24)));
    for (let k = 1; k <= nr; k++) push(waypoints[0], startYaw + rotDelta * (k / nr));
  }
  const segLen = [];
  let total = 0;
  for (let i = 0; i < waypoints.length - 1; i++) {
    const l = dist(waypoints[i], waypoints[i + 1]);
    segLen.push(l);
    total += l;
  }
  const samplesN = Math.max(waypoints.length, Math.ceil(total / (motionStep * 0.9)));
  for (let s = 0; s <= samplesN; s++) {
    const target = (s / samplesN) * total;
    let acc = 0, seg = 0;
    while (seg < segLen.length && acc + segLen[seg] < target) { acc += segLen[seg]; seg++; }
    if (seg >= segLen.length) { push(waypoints[waypoints.length - 1], yaw); continue; }
    const t = segLen[seg] === 0 ? 0 : (target - acc) / segLen[seg];
    const a = waypoints[seg], b = waypoints[seg + 1];
    push([
      a[0] + (b[0] - a[0]) * t,
      a[1] + (b[1] - a[1]) * t,
      a[2] + (b[2] - a[2]) * t,
    ], yaw);
  }
  return { waypoints, waypointYaw: waypoints.map(() => yaw), samples, sampleYaw, laneYaw: yaw };
}

export { poseClearanceExact };
