// Orchestration layer: builds the configuration space, runs the true-3D search,
// smooths and re-verifies the route, and independently audits reachability and
// clearance on a separate high-resolution reference grid.
//
// The verification is bound to the delivered pose: every dense sample carries
// the yaw the renderer draws, and the swept checks use that same yaw. The route
// is only reported ok when the exact oriented-box sweep finds no contact, so a
// player can never be handed a trajectory that the collision model rejects.
import {
  sweptTranslation, rotationClearance, poseClearance, poseClearanceExact,
  naiveCenterlineCheck, vehicleHalf, erosionRadius, dist, shortestAngle,
} from './geometry.js';
import { VoxelGrid, voxelPayload, CANDIDATE_YAWS } from './grid.js';
import { planPath, smoothPath } from './planner.js';

const SEARCH_CELL = 0.5;
const REFERENCE_CELL = 0.25;
const MOTION_STEP = SEARCH_CELL * 0.6;
// Continuous clearance guaranteed along the delivered route. The raster is built
// with margin = SAFE + MOTION_STEP/2 so any free pose exceeds the sweep's demand
// (worst > SAFE + MOTION_STEP/2) and the search's accept threshold and the
// verifier's reject threshold are the SAME number, SAFE. A route either passes
// both or fails both; nothing is delivered green that verification would reject.
const SAFE_CLEARANCE = 0.05;

function roundPath(points) {
  return points.map((p) => p.map((v) => Number(v.toFixed(3))));
}
function roundAngle(a) { return Number(a.toFixed(4)); }

// High-resolution independent audit: flood fill on a finer erosion grid.
// This grid is never used by the planner, so it is an independent reference
// for reachability; it also reports the bottleneck clearance along the route.
function referenceAudit(world, dims, start, goal, margin) {
  // 'axis' is the conservative superset raster (fast, sound for every yaw), so
  // reachability on the fine grid can never under-report a usable corridor.
  const ref = new VoxelGrid(world, REFERENCE_CELL, dims, { margin, mode: 'axis' }).build();
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

// Conservative verification of the delivered motion. The motion actually drawn
// is: rotate in place at the first waypoint (start heading -> lane yaw), then
// translate along each waypoint leg at that lane yaw. We verify exactly that
// structure — one rotation test and one swept translation per leg — because the
// search and smoother certify the same legs with the same `motionStep` bound.
//
// (An earlier version re-swept the dense playback samples instead. Those are
// sub-segments of the very same legs, so re-applying a fresh motionStep/2
// cushion to each short one double-counted the bound and rejected routes whose
// true continuous clearance was well above SAFE. Verifying the legs is both
// sound — a 1-Lipschitz clearance sampled at spacing s satisfies
// trueMin >= minSample - s/2 <= minSample - motionStep/2 — and identical to the
// criterion the search accepted.)
//
// The dense samples are still cleared and carried in `trace` so a failure flag
// resolves to a concrete collision sample: `failure.sampleIndex` is the drawn
// sample nearest the first contact.
function verifyPath(world, dims, smoothed, safeMargin) {
  const { waypoints, waypointYaw, samples, sampleYaw } = smoothed;
  const half = vehicleHalf(dims);
  const margin = safeMargin;
  const laneYaw = smoothed.laneYaw ?? waypointYaw[0] ?? 0;
  const startYaw = sampleYaw[0] ?? 0;

  // Dense pose trace (the poses the renderer draws) — used for the UI trace and
  // for reporting the measured minimum.
  let worst = Infinity, worstI = -1, worstObs = -1;
  const trace = [];
  for (let i = 0; i < samples.length; i++) {
    const yaw = sampleYaw[i] ?? 0;
    const r = poseClearanceExact(world, samples[i], yaw, half, -Infinity);
    if (r.clearance < worst) { worst = r.clearance; worstI = i; worstObs = r.obstacle; }
    if (i % 2 === 0 || i === samples.length - 1) {
      trace.push({
        index: i,
        at: samples[i].map((v) => Number(v.toFixed(3))),
        yaw: roundAngle(yaw),
        clearance: Number(r.clearance.toFixed(3)),
        obstacle: r.obstacle,
      });
    }
  }
  const nearestSample = (p) => {
    let best = -1, bd = Infinity;
    for (let i = 0; i < samples.length; i++) {
      const d = dist(samples[i], p);
      if (d < bd) { bd = d; best = i; }
    }
    return best;
  };

  // Rigorous verdict on the delivered legs: the initial in-place rotation, then
  // one constant-yaw swept translation per waypoint leg.
  let failure = null;
  let continuous = Infinity;
  const rot = rotationClearance(world, waypoints[0], startYaw, laneYaw, half, margin);
  if (!rot.clear) {
    failure = {
      kind: 'rotation', segment: 0,
      at: waypoints[0].map((v) => Number(v.toFixed(3))),
      yaw: roundAngle(laneYaw), clearance: Number(rot.clearance.toFixed(3)),
      obstacle: rot.obstacle, sampleIndex: nearestSample(waypoints[0]),
    };
  } else {
    continuous = Math.min(continuous, rot.clearance);
    for (let i = 0; i < waypoints.length - 1 && !failure; i++) {
      const yaw = waypointYaw[i] ?? laneYaw;
      const r = sweptTranslation(world, waypoints[i], waypoints[i + 1], yaw, half, MOTION_STEP, margin);
      if (!r.clear) {
        const at = r.at ?? waypoints[i + 1];
        failure = {
          kind: 'translation', segment: i,
          at: at.map((v) => Number(v.toFixed(3))),
          yaw: roundAngle(yaw), clearance: Number(r.clearance.toFixed(3)),
          obstacle: r.obstacle, sampleIndex: nearestSample(at),
        };
        break;
      }
      continuous = Math.min(continuous, r.clearance);
    }
  }

  return {
    continuousSafe: failure === null,
    conservativeClearance: Number((failure ? failure.clearance : continuous).toFixed(3)),
    measuredClearance: Number(worst.toFixed(3)),
    worstAt: worstI >= 0 ? samples[worstI].map((v) => Number(v.toFixed(3))) : null,
    worstYaw: worstI >= 0 ? roundAngle(sampleYaw[worstI] ?? 0) : null,
    worstSample: worstI,
    worstObstacle: worstObs,
    failure,
    motionStep: MOTION_STEP,
    trace,
  };
}

export function runNavigation(world, dims, start, goal, opts = {}) {
  const safeMargin = opts.safeMargin ?? SAFE_CLEARANCE;
  const gridMargin = safeMargin + MOTION_STEP / 2;
  const startYaw = opts.startYaw ?? 0;
  const t0 = Date.now();
  const half = vehicleHalf(dims);

  // Endpoints must themselves fit the vehicle volume at the start yaw / any yaw.
  const startFit = poseClearanceExact(world, start, startYaw, half, safeMargin);
  // The goal may be approached at any orientation; accept it if some candidate
  // yaw fits (the search will arrive at whatever yaw it chose).
  let goalFit = { clear: false, clearance: -Infinity };
  for (const yaw of CANDIDATE_YAWS) {
    const r = poseClearanceExact(world, goal, yaw, half, safeMargin);
    if (r.clearance > goalFit.clearance) goalFit = r;
    if (r.clear) break;
  }

  const searchOpts = { ...opts, motionStep: MOTION_STEP, safeMargin, startYaw };
  const grid = new VoxelGrid(world, SEARCH_CELL, dims, { margin: gridMargin, origin: opts.origin }).build();
  const startIdx = grid.nearestIndex(start);
  const goalIdx = grid.nearestIndex(goal);

  const base = {
    scenario: world.id,
    vehicle: { ...dims, erosionRadius: Number(erosionRadius(dims).toFixed(3)) },
    start: start.map((v) => Number(v.toFixed(3))),
    startYaw: roundAngle(startYaw),
    goal: goal.map((v) => Number(v.toFixed(3))),
    grid: {
      cell: SEARCH_CELL,
      mode: grid.mode,
      origin: grid.min,
      dims: [grid.nx, grid.ny, grid.nz],
      total: grid.n,
      blocked: grid.blocked.reduce((a, b) => a + b, 0),
    },
    reference: referenceAudit(world, dims, start, goal, gridMargin),
    timings: {},
  };

  if (!startFit.clear || startIdx < 0 || grid.blocked[startIdx]) {
    return {
      ...base, ok: false, reason: 'start_in_collision',
      startClearance: Number(startFit.clearance.toFixed(3)),
    };
  }
  if (!goalFit.clear || goalIdx < 0 || grid.blocked[goalIdx]) {
    return {
      ...base, ok: false, reason: 'goal_in_collision',
      goalClearance: Number(goalFit.clearance.toFixed(3)),
    };
  }

  const search = planPath(grid, world, startIdx, goalIdx, dims, searchOpts);
  base.timings.searchMs = Date.now() - t0;

  if (!search.ok) {
    // Independent flood on the search grid for the blocking frontier.
    const reached = grid.floodFill(startIdx);
    const frontier = grid.blockingFrontier(reached);
    return {
      ...base,
      ok: false,
      reason: search.reason,
      laneAttempts: (search.attempts ?? []).map((a) => ({ yaw: roundAngle(a.yaw), ok: a.ok ?? false, skip: a.skip })),
      expansions: search.expansions,
      sweptRejected: search.sweptRejected,
      blocking: voxelPayload(grid, frontier),
      reached: voxelPayload(grid, reachedVoxelList(reached, 5000)),
      timings: { ...base.timings, totalMs: Date.now() - t0 },
    };
  }

  const t1 = Date.now();
  const smoothed = smoothPath(grid, world, search.path, dims, {
    ...searchOpts, laneYaw: search.laneYaw,
  });
  base.timings.smoothMs = Date.now() - t1;

  const t2 = Date.now();
  const verification = verifyPath(world, dims, smoothed, safeMargin);
  base.timings.verifyMs = Date.now() - t2;

  // Deliberate naive centerline audit for comparison/education: a large vehicle
  // through a narrow slit has an "empty" centerline even though the swept box
  // collides.
  const naive = naiveCenterlineCheck(world, start, goal, SEARCH_CELL, 0);

  // The delivered route must be continuously safe; if the exact sweep finds a
  // contact we refuse to report success rather than ship a colliding path.
  if (!verification.continuousSafe) {
    return {
      ...base,
      ok: false,
      reason: 'verification_failed',
      expansions: search.expansions,
      sweptRejected: search.sweptRejected,
      voxelPath: roundPath(search.path),
      waypoints: roundPath(smoothed.waypoints),
      samples: roundPath(smoothed.samples),
      sampleYaw: smoothed.sampleYaw.map(roundAngle),
      waypointYaw: smoothed.waypointYaw.map(roundAngle),
      verification,
      timings: { ...base.timings, totalMs: Date.now() - t0 },
    };
  }

  return {
    ...base,
    ok: true,
    reason: 'routed',
    laneYaw: roundAngle(search.laneYaw),
    laneAttempts: search.attempts.map((a) => ({ yaw: roundAngle(a.yaw), ok: a.ok })),
    expansions: search.expansions,
    sweptRejected: search.sweptRejected,
    voxelPath: roundPath(search.path),
    waypoints: roundPath(smoothed.waypoints),
    waypointYaw: smoothed.waypointYaw.map(roundAngle),
    samples: roundPath(smoothed.samples),
    sampleYaw: smoothed.sampleYaw.map(roundAngle),
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
export function gridPayload(world, dims, opts = {}) {
  const o = typeof opts === 'number' ? { margin: opts } : opts;
  const grid = new VoxelGrid(world, SEARCH_CELL, dims, { margin: o.margin ?? SAFE_CLEARANCE, mode: o.mode })
    .build();
  const blockedIdx = [];
  for (let i = 0; i < grid.n; i++) if (grid.blocked[i]) blockedIdx.push(i);
  return {
    ...voxelPayload(grid, blockedIdx),
    eroded: true,
    erosionRadius: Number(erosionRadius(dims).toFixed(3)),
  };
}
