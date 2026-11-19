// Orchestration layer: builds the configuration space, runs the true-3D search,
// smooths and re-verifies the route, and independently audits reachability and
// clearance on a separate high-resolution reference grid.
import {
  sweptBoxClearance, naiveCenterlineCheck,
  poseClearance, vehicleHalf, erosionRadius, heading, dist,
} from './geometry.js';
import { VoxelGrid, voxelPayload } from './grid.js';
import { planPath, smoothPath } from './planner.js';

const SEARCH_CELL = 0.5;
const REFERENCE_CELL = 0.25;

function roundPath(points) {
  return points.map((p) => p.map((v) => Number(v.toFixed(3))));
}

// High-resolution independent audit: flood fill on a finer erosion grid.
// This grid is never used by the planner, so it is an independent reference
// for reachability; it also reports the bottleneck clearance along the route.
function referenceAudit(world, dims, start, goal, margin) {
  const ref = new VoxelGrid(world, REFERENCE_CELL, dims, margin).build();
  const sIdx = ref.nearestIndex(start);
  const gIdx = ref.nearestIndex(goal);
  const startBlocked = sIdx < 0 || ref.blocked[sIdx] === 1;
  const goalBlocked = gIdx < 0 || ref.blocked[gIdx] === 1;
  let reachable = false;
  let reachedCount = 0;
  let reached = null;
  let enclosedGoal = false;
  if (!startBlocked) {
    reached = ref.floodFill(sIdx);
    for (let i = 0; i < reached.length; i++) if (reached[i]) reachedCount++;
    reachable = !goalBlocked && reached[gIdx] === 1;
    // A goal voxel that is itself free but outside the start's connected free
    // component is enclosed by solids (e.g. a sealed cavity).
    enclosedGoal = !goalBlocked && reached[gIdx] !== 1;
  }
  return {
    cell: REFERENCE_CELL,
    erosionRadius: Number(erosionRadius(dims).toFixed(3)),
    totalVoxels: ref.n,
    reachedVoxels: reachedCount,
    startBlocked,
    goalBlocked,
    enclosedGoal,
    reachable,
  };
}

// Full-length conservative swept re-verification of the dense sample path.
function verifySamples(world, dims, samples, margin, cell) {
  const half = vehicleHalf(dims);
  const surfaceStep = cell * 1.0;
  const motionStep = cell * 1.0;
  let worst = Infinity;
  let worstAt = null;
  const trace = [];
  for (let i = 0; i < samples.length; i++) {
    const yaw = i < samples.length - 1
      ? heading(samples[i], samples[i + 1])
      : heading(samples[Math.max(0, i - 1)], samples[i]);
    const r = poseClearance(world, samples[i], yaw, half, -Infinity, surfaceStep);
    if (r.clearance < worst) { worst = r.clearance; worstAt = samples[i]; }
    // Record closest obstacle distance at every recorded pose (thinned later).
    if (i % 2 === 0 || i === samples.length - 1) {
      trace.push({ at: samples[i].map((v) => Number(v.toFixed(3))), clearance: Number(r.clearance.toFixed(3)) });
    }
  }
  // Segment-by-segment swept check with the Lipschitz conservative bound.
  let segmentFail = null;
  for (let i = 0; i < samples.length - 1; i++) {
    const r = sweptBoxClearance(world, samples[i], samples[i + 1], half, surfaceStep, motionStep, margin);
    if (!r.clear) { segmentFail = { segment: i, at: r.at, clearance: Number(r.clearance.toFixed(3)) }; break; }
    if (r.clearance < worst) worst = r.clearance;
  }
  return {
    continuousSafe: segmentFail === null,
    // Reported clearance: measured surface SDF minimum. The Lipschitz bound
    // (measured - motionStep/2) is what the per-segment accept/reject decision
    // uses; trace entries expose the measured value at every pose.
    conservativeClearance: Number((worst - motionStep / 2).toFixed(3)),
    measuredClearance: Number(worst.toFixed(3)),
    worstAt: worstAt ? worstAt.map((v) => Number(v.toFixed(3))) : null,
    failure: segmentFail,
    trace,
  };
}

export function runNavigation(world, dims, start, goal, opts = {}) {
  const margin = opts.margin ?? 0.05;
  const t0 = Date.now();
  const half = vehicleHalf(dims);

  // Endpoints must themselves fit the vehicle volume.
  const startFit = poseClearance(world, start, 0, half, margin, SEARCH_CELL * 0.6);
  const goalFit = poseClearance(world, goal, 0, half, margin, SEARCH_CELL * 0.6);

  const grid = new VoxelGrid(world, SEARCH_CELL, dims, margin).build();
  const startIdx = grid.nearestIndex(start);
  const goalIdx = grid.nearestIndex(goal);

  const base = {
    scenario: world.id,
    vehicle: { ...dims, erosionRadius: Number(erosionRadius(dims).toFixed(3)) },
    start: start.map((v) => Number(v.toFixed(3))),
    goal: goal.map((v) => Number(v.toFixed(3))),
    grid: { cell: SEARCH_CELL, dims: [grid.nx, grid.ny, grid.nz], total: grid.n, blocked: grid.blocked.reduce((a, b) => a + b, 0) },
    reference: referenceAudit(world, dims, start, goal, margin),
    timings: {},
  };

  if (!startFit.clear || startIdx < 0 || grid.blocked[startIdx]) {
    return { ...base, ok: false, reason: 'start_in_collision', startClearance: Number(startFit.clearance.toFixed(3)) };
  }
  if (!goalFit.clear || goalIdx < 0 || grid.blocked[goalIdx]) {
    return { ...base, ok: false, reason: 'goal_in_collision', goalClearance: Number(goalFit.clearance.toFixed(3)) };
  }

  const search = planPath(grid, world, startIdx, goalIdx, dims, opts);
  base.timings.searchMs = Date.now() - t0;

  if (!search.ok) {
    // Independent flood on the search grid for the blocking frontier.
    const reached = grid.floodFill(startIdx);
    const frontier = grid.blockingFrontier(reached);
    return {
      ...base,
      ok: false,
      reason: search.reason,
      expansions: search.expansions,
      sweptRejected: search.sweptRejected,
      blocking: voxelPayload(grid, frontier),
      reached: voxelPayload(grid, reachedVoxelList(reached, 5000)),
      timings: { ...base.timings, totalMs: Date.now() - t0 },
    };
  }

  const t1 = Date.now();
  const smoothed = smoothPath(grid, world, search.path, dims, opts);
  base.timings.smoothMs = Date.now() - t1;

  const t2 = Date.now();
  const verification = verifySamples(world, dims, smoothed.samples, margin, SEARCH_CELL);
  base.timings.verifyMs = Date.now() - t2;

  // Deliberate naive centerline audit for comparison/education: a large
  // vehicle through a narrow slit has an "empty" centerline even though the
  // swept box collides.
  const naive = naiveCenterlineCheck(world, start, goal, SEARCH_CELL, 0);

  return {
    ...base,
    ok: true,
    reason: 'routed',
    expansions: search.expansions,
    sweptRejected: search.sweptRejected,
    voxelPath: roundPath(search.path),
    waypoints: roundPath(smoothed.waypoints),
    samples: roundPath(smoothed.samples),
    pathLength: Number(pathLength(smoothed.samples).toFixed(3)),
    verification,
    naiveCenterline: { clear: naive.clear, clearance: Number(naive.clearance.toFixed(3)) },
    timings: { ...base.timings, totalMs: Date.now() - t0 },
  };
}

function reachedVoxelList(reached, cap) {
  const out = [];
  for (let i = 0; i < reached.length && out.length < cap; i++) {
    if (reached[i]) out.push(i);
  }
  return out;
}

function pathLength(samples) {
  let total = 0;
  for (let i = 1; i < samples.length; i++) total += dist(samples[i - 1], samples[i]);
  return total;
}

// Grid payload for the UI slice view (blocked + clearance, subsampled on demand).
export function gridPayload(world, dims, margin = 0.05) {
  const grid = new VoxelGrid(world, SEARCH_CELL, dims, margin).build();
  const blockedIdx = [];
  for (let i = 0; i < grid.n; i++) if (grid.blocked[i]) blockedIdx.push(i);
  return {
    ...voxelPayload(grid, blockedIdx),
    eroded: true,
    erosionRadius: Number(erosionRadius(dims).toFixed(3)),
  };
}
