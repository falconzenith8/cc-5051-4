// Diagnostic probes for "the raster said no" / "the smoother said yes" failures.
//
// The point is to make each failure from this demo replayable and traceable:
//
//   * probe A ("origin shift") holds the OBSTACLES fixed and only moves the
//     raster origin. If reachability flips under a pure sub-cell origin shift,
//     the rasterization stage is the one that erased the corridor, not the
//     geometry.
//
//   * probe B ("smoother swap") holds the ORIGINAL polyline fixed and only
//     replaces the smoother (greedy shortcut vs no smoothing / chord-cap). If
//     the raw A* polyline verifies but the smoothed one does not, the smoothing
//     stage pushed the safe line into the hull.
//
//   * traceback finds the EARLIEST voxel (raster probe) or segment (smoother
//     probe) at which two configurations disagree on the reachability verdict.
//
//   * minimalLevel strips the scenario down to the geometry bracketing that
//     earliest flip plus the vehicle body, so the failure reproduces on its own.
//
// Every probe returns the vehicle HEADING and the SWEPT BODY (oriented boxes
// sampled along the motion), so two cases with different headings are never
// collapsed into a single voxel screenshot that loses the evidence basis.
import {
  dist, vehicleHalf, poseClearanceExact, sweptTranslation,
} from './geometry.js';
import { VoxelGrid, CANDIDATE_YAWS } from './grid.js';
import { planPath, smoothPath } from './planner.js';
export { planPath, smoothPath };

const SEARCH_CELL = 0.5;
const MOTION_STEP = SEARCH_CELL * 0.6;
const SAFE_CLEARANCE = 0.05;
const GRID_MARGIN = SAFE_CLEARANCE + MOTION_STEP / 2;

function round3(p) { return p.map((v) => Number(v.toFixed(3))); }
function roundAngle(a) { return Number(a.toFixed(4)); }

// Oriented vehicle boxes at the drawn pose(s): the "swept body" evidence.
// A single sample yields one box; a segment yields the box at both ends and the
// mid-point, all at the SAME committed yaw (never re-derived from the tangent).
function sweptBody(points, yaw, half, everyPoint = false) {
  const boxes = [];
  const addBox = (c) => boxes.push({
    center: round3(c),
    yaw: roundAngle(yaw),
    half: half.map((v) => Number(v.toFixed(3))),
  });
  if (points.length === 1) { addBox(points[0]); return boxes; }
  for (let i = 0; i < points.length - 1; i++) {
    addBox(points[i]);
    addBox(points[i].map((v, d) => (v + points[i + 1][d]) / 2));
    if (everyPoint || i === points.length - 2) addBox(points[i + 1]);
  }
  return boxes;
}

// The orientation a renderer must draw: the committed lane yaw. The old UI used
// atan2 of the path tangent, which silently disagreed with the verified pose.
export function drawnHeading(laneYaw, startYaw) {
  return {
    laneYaw: roundAngle(laneYaw),
    laneYawDeg: Number(((laneYaw * 180) / Math.PI).toFixed(1)),
    startYaw: roundAngle(startYaw),
    note: 'vehicle is drawn at laneYaw for the whole route; the start in-place rotation goes startYaw -> laneYaw',
  };
}

// PROBE A — hold obstacles fixed, shift only the raster origin.
// Reports whether the reachability verdict flips across origins, and if so the
// earliest voxel pair (same physical cell, different rasterisation) that differ.
export function probeOriginShift(world, dims, start, goal, opts = {}) {
  const shifts = opts.shifts ?? [0, 0.1, 0.2, 0.3, 0.4];
  const safeMargin = opts.safeMargin ?? SAFE_CLEARANCE;
  const gridMargin = safeMargin + MOTION_STEP / 2;
  const b = world.bounds;
  const rows = [];
  for (const s of shifts) {
    for (const axis of opts.axes ?? [0]) {
      const origin = b.min.slice();
      origin[axis] += s;
      const grid = new VoxelGrid(world, SEARCH_CELL, dims, { margin: gridMargin, origin }).build();
      const sIdx = grid.nearestIndex(start);
      const gIdx = grid.nearestIndex(goal);
      let reachable = false, startBlocked = sIdx < 0 || grid.blocked[sIdx] === 1;
      let goalBlocked = gIdx < 0 || grid.blocked[gIdx] === 1;
      if (!startBlocked) {
        const seen = grid.floodFill(sIdx);
        reachable = !goalBlocked && seen[gIdx] === 1;
      }
      rows.push({
        shift: Number(s.toFixed(2)), axis,
        origin: round3(origin),
        startBlocked, goalBlocked, reachable,
        blockedVoxels: grid.blocked.reduce((a, v) => a + v, 0),
      });
    }
  }
  const verdicts = new Set(rows.map((r) => r.reachable));
  return {
    probe: 'origin_shift',
    obstaclesFixed: true,
    vehicle: dims,
    heading: drawnHeading(opts.laneYaw ?? 0, opts.startYaw ?? 0),
    flipped: verdicts.size > 1,
    // A flip under a pure sub-cell origin move proves rasterisation — not the
    // obstacle geometry — changed the reachability conclusion.
    attribution: verdicts.size > 1 ? 'rasterization' : 'not_rasterization',
    rows,
  };
}

// PROBE B — hold the ORIGINAL polyline fixed, swap only the smoother.
// `path` is the raw A* voxel-centre polyline. We verify the raw polyline and
// each smoother variant with the SAME lane yaw, then report which segment first
// became unsafe — the smoother is the stage that moved the safe line.
//
// `variant: 'legacy_anyyaw'` reproduces the historical bug: a chord was accepted
// if ANY yaw cleared at the endpoints, but playback committed to the travel
// heading. Comparing it to the lane-yaw smoother isolates the smoothing stage.
function legacyAnyYawSmooth(world, dims, path, opts = {}) {
  const half = vehicleHalf(dims);
  const motionStep = opts.motionStep ?? MOTION_STEP;
  const safeMargin = opts.safeMargin ?? SAFE_CLEARANCE;
  const chordSafe = (a, b) => {
    // The old bug: accept if some candidate yaw clears the endpoints.
    for (const y of CANDIDATE_YAWS) {
      const ra = poseClearanceExact(world, a, y, half, safeMargin);
      const rb = poseClearanceExact(world, b, y, half, safeMargin);
      if (ra.clear && rb.clear) {
        const sw = sweptTranslation(world, a, b, y, half, motionStep, safeMargin);
        if (sw.clear) return true;
      }
    }
    return false;
  };
  let wps = path.slice();
  let changed = true, passes = 0;
  while (changed && passes < 40) {
    changed = false; passes++;
    const out = [wps[0]];
    let anchor = 0;
    while (anchor < wps.length - 1) {
      let furthest = anchor + 1;
      for (let k = wps.length - 1; k > anchor + 1; k--) {
        if (chordSafe(wps[anchor], wps[k])) { furthest = k; break; }
      }
      out.push(wps[furthest]);
      if (furthest !== anchor + 1) changed = true;
      anchor = furthest;
    }
    wps = out;
  }
  return wps;
}

export function probeSmootherSwap(world, dims, path, opts = {}) {
  const laneYaw = opts.laneYaw ?? 0;
  const startYaw = opts.startYaw ?? laneYaw;
  const safeMargin = opts.safeMargin ?? SAFE_CLEARANCE;
  const half = vehicleHalf(dims);
  const variants = [{
    name: 'raw_astar',
    waypoints: path,
    // raw A* is not "smoothed"; each hop is already a verified sweep
  }];
  {
    const fakeGrid = { cell: opts.cell ?? SEARCH_CELL, margin: GRID_MARGIN };
    const sm = smoothPath(fakeGrid, world, path, dims, {
      motionStep: MOTION_STEP, safeMargin, laneYaw, startYaw,
    });
    variants.push({ name: 'lane_yaw_shortcut', waypoints: sm.waypoints });
    if (opts.includeLegacy !== false) {
      variants.push({
        name: 'legacy_anyyaw',
        waypoints: legacyAnyYawSmooth(world, dims, path, { motionStep: MOTION_STEP, safeMargin }),
      });
    }
  }

  const results = variants.map((v) => {
    const legs = [];
    let firstBad = null;
    let swept = [];
    for (let i = 0; i < v.waypoints.length - 1; i++) {
      const a = v.waypoints[i], b = v.waypoints[i + 1];
      const r = sweptTranslation(world, a, b, laneYaw, half, MOTION_STEP, safeMargin);
      legs.push({
        segment: i,
        from: round3(a), to: round3(b),
        length: Number(dist(a, b).toFixed(3)),
        clear: r.clear,
        clearance: Number(r.clearance.toFixed(3)),
        at: r.at ? round3(r.at) : null,
        obstacle: r.obstacle,
      });
      if (!r.clear && firstBad === null) {
        firstBad = { segment: i, at: r.at ? round3(r.at) : round3(b), clearance: Number(r.clearance.toFixed(3)) };
      }
      swept.push(...sweptBody([a, b], laneYaw, half));
    }
    return {
      name: v.name,
      waypointCount: v.waypoints.length,
      safe: firstBad === null,
      firstBadSegment: firstBad,
      legs,
      // The swept body evidence: oriented boxes at the drawn heading.
      sweptBody: swept,
      heading: drawnHeading(laneYaw, startYaw),
    };
  });

  const raw = results.find((r) => r.name === 'raw_astar');
  const lane = results.find((r) => r.name === 'lane_yaw_shortcut');
  const legacy = results.find((r) => r.name === 'legacy_anyyaw');
  // Fault is smoothing if the raw polyline is safe at the drawn heading but a
  // smoothed variant is not.
  const smootherFault = raw.safe && (!lane.safe || (legacy && !legacy.safe));
  const badVariant = !lane.safe ? lane : (legacy && !legacy.safe ? legacy : null);
  return {
    probe: 'smoother_swap',
    polylineFixed: true,
    vehicle: dims,
    attribution: smootherFault ? 'smoothing' : (raw.safe ? 'none' : 'search_or_geometry'),
    // The earliest segment at which smoothing left the safe set.
    earliestDivergenceSegment: badVariant ? badVariant.firstBadSegment.segment : null,
    results,
  };
}

// PROBE C — traceback of the earliest voxel where two rasters disagree.
// Given the same obstacles and the same physical cell centre, compare the
// oriented raster against the legacy axis-aligned raster at each voxel; the
// first disagreement along the route (or by index) is the "earliest voxel that
// changes the reachability conclusion".
//
// `anchor` (the raw polyline) orders candidates by distance to it and keeps only
// the ones within `corridorTol`, so "earliest" means earliest on the corridor
// the vehicle would use — not an arbitrary eroded cell in open water.
export function earliestFlipVoxel(world, dims, opts = {}) {
  const margin = opts.margin ?? GRID_MARGIN;
  const oriented = new VoxelGrid(world, SEARCH_CELL, dims, { margin, mode: 'oriented' }).build();
  const legacy = new VoxelGrid(world, SEARCH_CELL, dims, { margin, mode: 'legacy-axis' }).build();
  const corridorTol = opts.corridorTol ?? Infinity;
  const anchorPts = opts.anchor ?? null;
  const nearestAnchor = (c) => {
    if (!anchorPts) return 0;
    let best = Infinity;
    for (const p of anchorPts) { const d = dist(c, p); if (d < best) best = d; }
    return best;
  };
  const cells = [];
  for (let idx = 0; idx < oriented.n; idx++) {
    if (oriented.blocked[idx] !== 0) continue;
    if (legacy.blocked[idx] !== 1) continue;
    const c = oriented.centerOf(idx);
    const da = nearestAnchor(c);
    if (da > corridorTol) continue;
    cells.push({
      index: idx,
      center: round3(c),
      oriented_free: true,
      legacy_blocked: true,
      yawHintDeg: Number.isFinite(oriented.yawHint[idx])
        ? Number(((oriented.yawHint[idx] * 180) / Math.PI).toFixed(1)) : null,
      clearance: Number(oriented.clearance[idx].toFixed(3)),
      anchorDist: anchorPts ? Number(da.toFixed(2)) : null,
    });
  }
  // Earliest on the corridor = closest to the anchor; fall back to index order.
  if (anchorPts) cells.sort((a, b) => (a.anchorDist - b.anchorDist) || (a.index - b.index));
  return {
    probe: 'earliest_flip_voxel',
    cell: SEARCH_CELL,
    modeA: 'oriented',
    modeB: 'legacy-axis',
    // These are cells the axis-aligned raster erased but the vehicle can occupy.
    erasedButTraversable: cells.length,
    earliest: cells[0] ?? null,
    // A short list of the first few flipped cells for drawing/evidence.
    firstFew: cells.slice(0, 12),
    vehicle: dims,
  };
}

// PROBE D — earliest segment where the delivered polyline's pose set diverges
// from the verified set. Given dense samples + per-sample yaw, find the first
// sample whose exact pose clearance drops below SAFE, with its swept body.
export function earliestPoseFailure(world, dims, samples, sampleYaw, opts = {}) {
  const safeMargin = opts.safeMargin ?? SAFE_CLEARANCE;
  const half = vehicleHalf(dims);
  for (let i = 0; i < samples.length; i++) {
    const yaw = sampleYaw[i] ?? 0;
    const r = poseClearanceExact(world, samples[i], yaw, half, safeMargin);
    if (!r.clear) {
      return {
        probe: 'earliest_pose_failure',
        index: i,
        at: round3(samples[i]),
        yaw: roundAngle(yaw),
        clearance: Number(r.clearance.toFixed(3)),
        obstacle: r.obstacle,
        sweptBody: sweptBody([samples[i]], yaw, half),
        heading: drawnHeading(yaw, sampleYaw[0] ?? yaw),
      };
    }
  }
  return { probe: 'earliest_pose_failure', index: null, safe: true };
}

// MINIMAL REPRODUCTION LEVEL — keep only the geometry that brackets the earliest
// flip / failing segment, plus the vehicle body. The level is a first-class
// scenario object (same shape as scenarios.js) so it can be planned directly.
//
// Start and goal are placed INSIDE the level's own tight bounds, on either side
// of the anchor, and only along a direction where both endpoints actually admit
// the vehicle. The mini-level is thus self-contained: it replays the failure
// instead of inheriting endpoints from the full map (which the tight bounds
// would exclude).
export function minimalLevel(world, dims, anchor, opts = {}) {
  const half = vehicleHalf(dims);
  const pad = opts.pad ?? 3.0;
  const a = anchor.center ?? anchor.at ?? anchor.from;
  const tol = opts.tol ?? 4.0;
  const kept = [];
  for (const o of world.obstacles) {
    // Keep obstacles whose analytic surface could be within tol of the anchor.
    const reach = o.type === 'box'
      ? Math.hypot(...o.half)
      : (o.type === 'cylinderZ' ? Math.hypot(o.radius, 17) : o.radius);
    if (dist(o.center, a) <= reach + tol) kept.push(o);
  }
  // Bounds tightened around the anchor so the mini-level is genuinely minimal.
  const bounds = {
    min: [a[0] - pad, a[1] - pad, Math.max(world.bounds.min[2], a[2] - pad)],
    max: [a[0] + pad, a[1] + pad, Math.min(world.bounds.max[2], a[2] + pad)],
  };
  // The mini-level's own world, used to validate endpoints against the same
  // obstacles AND the tightened bounds the replay will use.
  const miniWorld = { ...world, bounds, obstacles: kept };
  const margin = opts.safeMargin ?? SAFE_CLEARANCE;
  const yaw = opts.anchorYaw ?? 0;
  const free = (p) => poseClearanceExact(miniWorld, p, yaw, half, margin).clear;
  const inBounds = (p) => p[0] >= bounds.min[0] && p[0] <= bounds.max[0]
    && p[1] >= bounds.min[1] && p[1] <= bounds.max[1]
    && p[2] >= bounds.min[2] && p[2] <= bounds.max[2];
  const clampPt = (p) => [
    Math.max(bounds.min[0] + 0.25, Math.min(bounds.max[0] - 0.25, p[0])),
    Math.max(bounds.min[1] + 0.25, Math.min(bounds.max[1] - 0.25, p[1])),
    Math.max(bounds.min[2] + 0.25, Math.min(bounds.max[2] - 0.25, p[2])),
  ];

  // Best endpoints: free route points inside the mini-bounds (free in the MINI
  // world, so the body also clears the tightened bounds), on opposite sides of
  // the anchor.
  let start = null, goal = null, usedDir = [1, 0], span = 0;
  const route = opts.route ?? null;
  if (route && route.length >= 2) {
    const inR = route.filter((p) => inBounds(p) && free(p));
    if (inR.length >= 2) {
      // Split around the anchor's projection onto the route.
      let ai = 0, ad = Infinity;
      for (let i = 0; i < route.length; i++) { const d = dist(route[i], a); if (d < ad) { ad = d; ai = i; } }
      const before = route.slice(0, ai + 1).filter((p) => inBounds(p) && free(p));
      const after = route.slice(ai).filter((p) => inBounds(p) && free(p));
      const s = before[0] ?? inR[0];
      const g = after[after.length - 1] ?? inR[inR.length - 1];
      if (s !== g && dist(s, g) > 1e-6) {
        start = s.slice(); goal = g.slice();
        const dx = goal[0] - start[0], dy = goal[1] - start[1];
        const l = Math.hypot(dx, dy) || 1;
        usedDir = [dx / l, dy / l];
        span = Number(dist(start, goal).toFixed(2));
      }
    }
  }

  if (!start) {
    // Flanking search: prefer the route's local direction, then axes/diagonals.
    const dirs = [];
    if (anchor.prev && anchor.next) {
      const dx = anchor.next[0] - anchor.prev[0], dy = anchor.next[1] - anchor.prev[1];
      const l = Math.hypot(dx, dy) || 1;
      dirs.push([dx / l, dy / l]);
    }
    dirs.push([1, 0], [0, 1], [1, 1], [1, -1]);
    dirLoop:
    for (const d of dirs) {
      for (let s = 0.8; s <= pad - 0.3 + 1e-9; s += 0.3) {
        const p1 = clampPt([a[0] - d[0] * s, a[1] - d[1] * s, a[2]]);
        const p2 = clampPt([a[0] + d[0] * s, a[1] + d[1] * s, a[2]]);
        if (free(p1) && free(p2)) { start = p1; goal = p2; usedDir = d; span = s; break dirLoop; }
      }
    }
  }
  // Fallback: the anchor as both endpoints (a free cell that brackets the flip).
  if (!start) { start = clampPt(a); goal = clampPt(a); span = 0; }
  return {
    id: `${world.id}-minimal`,
    name: `最小复现：${world.name}`,
    description: '仅保留最早翻转体素/线段处的几何与载具体型，用于独立复现该失败。',
    bounds,
    obstacles: kept,
    start,
    goals: [goal],
    vehicle: dims,
    anchor: round3(a),
    direction: usedDir.map((v) => Number(v.toFixed(3))),
    span: Number(span.toFixed(2)),
    // Heading the mini-level should be planned at (the lane that fits the hole).
    startYaw: roundAngle(yaw),
    keptObstacles: kept.length,
    // Whether the anchor itself is a true free cell (so a route can cross it).
    anchorFree: free(a),
  };
}

// One-shot report the UI/API can render: runs the requested probes and returns
// a bundle with headings and swept bodies attached to every finding.
export function diagnose(world, dims, start, goal, opts = {}) {
  let plan = null, searchErr = null;
  try {
    const grid = new VoxelGrid(world, SEARCH_CELL, dims, { margin: GRID_MARGIN }).build();
    const sIdx = grid.nearestIndex(start), gIdx = grid.nearestIndex(goal);
    if (sIdx >= 0 && gIdx >= 0 && !grid.blocked[sIdx]) {
      plan = planPath(grid, world, sIdx, gIdx, dims, {
        motionStep: MOTION_STEP, safeMargin: SAFE_CLEARANCE, startYaw: opts.startYaw ?? 0,
      });
    }
  } catch (e) { searchErr = String(e.message || e); }

  const laneYaw = (plan && plan.ok) ? plan.laneYaw : (opts.laneYaw ?? 0);
  const startYaw = opts.startYaw ?? 0;

  const out = {
    scenario: world.id,
    vehicle: dims,
    start: round3(start),
    goal: round3(goal),
    heading: drawnHeading(laneYaw, startYaw),
    searched: !!plan,
    searchOk: !!(plan && plan.ok),
    searchErr,
    originShift: probeOriginShift(world, dims, start, goal, { laneYaw, startYaw, axes: opts.axes ?? [0, 1] }),
    // Corridor-aware earliest flip: anchored on the raw A* polyline when available.
    earliestFlip: earliestFlipVoxel(world, dims, {
      anchor: plan && plan.ok ? plan.path : null,
      corridorTol: plan && plan.ok ? 3.0 : Infinity,
    }),
  };

  if (plan && plan.ok) {
    const swap = probeSmootherSwap(world, dims, plan.path, { laneYaw, startYaw });
    out.smootherSwap = swap;
    const smSafe = swap.results.find((r) => r.name === 'lane_yaw_shortcut').safe;
    // Minimal level anchored on the first divergence / first bad segment. Its
    // endpoints come from free route points inside the mini-bounds.
    const badAxis = !smSafe
      ? { at: swap.results.find((r) => r.name === 'lane_yaw_shortcut').firstBadSegment.at,
          prev: plan.path[0], next: plan.path[plan.path.length - 1] }
      : (out.earliestFlip.earliest
        ? { ...out.earliestFlip.earliest, prev: plan.path[0], next: plan.path[plan.path.length - 1] }
        : { at: plan.path[Math.floor(plan.path.length / 2)], prev: plan.path[0], next: plan.path[plan.path.length - 1] });
    out.minimalLevel = minimalLevel(world, dims, badAxis, { route: plan.path, anchorYaw: laneYaw });
  } else if (out.earliestFlip.earliest) {
    out.minimalLevel = minimalLevel(world, dims, out.earliestFlip.earliest, { anchorYaw: laneYaw });
  }

  // The crisp verdict: which stage actually erred for this case.
  out.attribution = out.originShift.flipped
    ? 'rasterization'
    : (out.smootherSwap && out.smootherSwap.attribution === 'smoothing' ? 'smoothing' : 'none_detected');
  return out;
}
