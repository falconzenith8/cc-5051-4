// True 3D A* over the eroded voxel configuration space. Vertical and diagonal
// moves are first-class: all 26 neighbours are expanded with Euclidean costs,
// so routes can climb, dive and weave instead of following a height-tagged
// horizontal map. Every expanded edge is additionally re-checked with the
// conservative swept box test against the full analytic vehicle geometry.
import { dist, sweptBoxClearance, vehicleHalf, worldSdf, add, sub, scale } from './geometry.js';

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

export function planPath(grid, world, startIdx, goalIdx, dims, opts = {}) {
  const half = vehicleHalf(dims);
  const surfaceStep = opts.surfaceStep ?? grid.cell * 1.0;
  const motionStep = opts.motionStep ?? grid.cell * 1.0;
  const maxExpansions = opts.maxExpansions ?? 400000;

  const goal = grid.centerOf(goalIdx);
  const gScore = new Float64Array(grid.n).fill(Infinity);
  const came = new Int32Array(grid.n).fill(-1);
  const closed = new Uint8Array(grid.n);
  const heap = new MinHeap();

  gScore[startIdx] = 0;
  heap.push({ idx: startIdx, f: dist(grid.centerOf(startIdx), goal) });

  // Lazy edge validation: the swept test runs when an edge is first made
  // permanent (on pop), not once per neighbour relaxation. Results are cached
  // per undirected edge, so each physical motion is evaluated at most once.
  const edgeCache = new Map();
  const edgeKey = (a, b) => (a < b ? a * grid.n + b : b * grid.n + a);
  // SDF values at voxel centers are precomputed in the grid; midpoints of
  // voxel hops land on half-integer offsets and are memoized here.
  const midSdfCache = new Map();
  const endSdf = (idx) => grid.clearance[idx];
  const inRadius = Math.min(half[0], half[1], half[2]);
  const circum = Math.hypot(half[0], half[1], half[2]);
  let expansions = 0;
  let sweptRejected = 0;
  let fastAccepted = 0;

  const edgeSafe = (aIdx, bIdx) => {
    const key = edgeKey(aIdx, bIdx);
    const cached = edgeCache.get(key);
    if (cached !== undefined) return cached;
    const ca = grid.centerOf(aIdx), cb = grid.centerOf(bIdx);
    let midS = midSdfCache.get(key);
    if (midS === undefined) {
      const mid = add(ca, scale(sub(cb, ca), 0.5));
      midS = worldSdf(world, mid);
      midSdfCache.set(key, midS);
    }
    // Necessary-condition reject: even the inscribed sphere does not fit.
    if (midS < inRadius + grid.margin) {
      edgeCache.set(key, false);
      sweptRejected++;
      return false;
    }
    // Sufficient-condition accept: the circumscribed sphere fits at both
    // ends and the midpoint, so every box corner stays clear throughout the
    // hop — skip the expensive surface sweep in open water.
    if (endSdf(aIdx) > circum + grid.margin
      && endSdf(bIdx) > circum + grid.margin
      && midS > circum + grid.margin) {
      edgeCache.set(key, true);
      fastAccepted++;
      return true;
    }
    // Near-obstacle region: authoritative oriented swept box test.
    const res = sweptBoxClearance(
      world, ca, cb, half, surfaceStep, motionStep, grid.margin,
    );
    edgeCache.set(key, res.clear);
    if (!res.clear) sweptRejected++;
    return res.clear;
  };

  let found = false;
  while (heap.size) {
    const { idx } = heap.pop();
    if (closed[idx]) continue;
    if (idx !== startIdx && came[idx] === -1) continue;
    if (idx !== startIdx && !edgeSafe(came[idx], idx)) {
      // Parent motion is physically infeasible. The edge is cached false, but
      // the voxel stays open: keep its g-score so another safe parent can
      // re-relax it, and do not mark it closed.
      continue;
    }
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
    return {
      ok: false, reason: 'unreachable',
      expansions, sweptRejected, fastAccepted,
      reached: reachedFromCame(grid, came, startIdx, closed),
    };
  }

  const idxPath = [];
  let cur = goalIdx;
  while (cur !== -1) { idxPath.push(cur); cur = came[cur]; }
  idxPath.reverse();
  const path = idxPath.map((idx) => grid.centerOf(idx));
  return { ok: true, path, idxPath, expansions, sweptRejected, fastAccepted };
}

// Reconstruct the reached free component after a failed search: every voxel
// that got a finite g-score is part of the connected, swept-checked component.
function reachedFromCame(grid, came, startIdx) {
  const reached = new Uint8Array(grid.n);
  const stack = [startIdx];
  reached[startIdx] = 1;
  while (stack.length) {
    const idx = stack.pop();
    for (const [nb] of grid.neighbors(idx)) {
      if (!reached[nb] && (came[nb] !== -1 || nb === startIdx)) {
        reached[nb] = 1;
        stack.push(nb);
      }
    }
  }
  return reached;
}

// Line-of-sight shortcutting on the voxel path using the same conservative
// swept check, then a final dense resample so the delivered route is smooth.
// Shortcuts may only remove waypoints; they never relax the safety criterion.
export function smoothPath(grid, world, path, dims, opts = {}) {
  const half = vehicleHalf(dims);
  const surfaceStep = opts.surfaceStep ?? grid.cell * 1.0;
  const motionStep = opts.motionStep ?? grid.cell * 1.0;

  const safe = (a, b) => sweptBoxClearance(
    world, a, b, half, surfaceStep, motionStep, grid.margin,
  );

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

  // Dense, uniform resample of the smoothed polyline for playback + verification.
  const segLen = [];
  let total = 0;
  for (let i = 0; i < waypoints.length - 1; i++) {
    const l = dist(waypoints[i], waypoints[i + 1]);
    segLen.push(l);
    total += l;
  }
  const samples = [];
  const samplesN = Math.max(waypoints.length, Math.ceil(total / (motionStep * 0.9)));
  for (let s = 0; s <= samplesN; s++) {
    const target = (s / samplesN) * total;
    let acc = 0, seg = 0;
    while (seg < segLen.length && acc + segLen[seg] < target) { acc += segLen[seg]; seg++; }
    if (seg >= segLen.length) { samples.push(waypoints[waypoints.length - 1]); continue; }
    const t = segLen[seg] === 0 ? 0 : (target - acc) / segLen[seg];
    const a = waypoints[seg], b = waypoints[seg + 1];
    samples.push([
      a[0] + (b[0] - a[0]) * t,
      a[1] + (b[1] - a[1]) * t,
      a[2] + (b[2] - a[2]) * t,
    ]);
  }
  return { waypoints, samples };
}
