// Diagnostic-probe validation. Asserts the traceback tool can name the faulty
// stage for each historical failure, that the delivered route is re-derivable
// from the stored heading + swept body, and that the same fix does NOT tunnel a
// genuinely-too-narrow hole.
import { buildScenario, DEFAULT_DIMS } from '../src/scenarios.js';
import { runNavigation } from '../src/engine.js';
import {
  diagnose, probeSmootherSwap, probeOriginShift, earliestFlipVoxel,
  minimalLevel, drawnHeading,
} from '../src/diagnose.js';
import { sweptTranslation, obstacleClearance, vehicleHalf } from '../src/geometry.js';

let pass = 0, fail = 0;
function check(name, cond, detail = '') {
  if (cond) { pass++; console.log(`  PASS  ${name}`); }
  else { fail++; console.log(`  FAIL  ${name} ${detail}`); }
}
const FAT = { length: 2.4, width: 3.2, height: 2.6 };

console.log('--- Probe B (smoother swap) names the smoothing stage ---');
{
  // Fat vehicle in the wreck: the historical any-yaw smoother bent the polyline
  // into the hull while the raw A* polyline is safe at the drawn heading.
  const w = buildScenario('wreck');
  const r = runNavigation(w, FAT, w.start, w.goals[0]);
  check('wreck/fat is routed with a committed heading', r.ok && typeof r.laneYaw === 'number');
  const swap = probeSmootherSwap(w, FAT, r.voxelPath, {
    laneYaw: r.laneYaw, startYaw: r.startYaw,
  });
  const raw = swap.results.find((x) => x.name === 'raw_astar');
  const lane = swap.results.find((x) => x.name === 'lane_yaw_shortcut');
  const legacy = swap.results.find((x) => x.name === 'legacy_anyyaw');
  check('raw A* polyline is safe at the committed heading', raw.safe === true);
  check('lane-yaw smoother keeps the route safe', lane.safe === true);
  check('legacy any-yaw smoother reproduces the collision', legacy.safe === false,
    `legacy firstBad=${JSON.stringify(legacy.firstBadSegment)}`);
  check('the probe attributes the failure to smoothing',
    swap.attribution === 'smoothing', swap.attribution);
  check('earliest divergence segment recorded', typeof swap.earliestDivergenceSegment === 'number');
  // The failing legacy segment carries the exact contact sample.
  check('legacy failure points at a concrete contact pose',
    legacy.firstBadSegment && Number.isFinite(legacy.firstBadSegment.clearance));
}

console.log('--- Probe A (origin shift) isolates rasterisation ---');
{
  // The slit has a corridor the axis-aligned raster erases at every origin; the
  // point here is that shifting ONLY the raster origin cannot change a result
  // produced by the oriented raster — so origin_shift must report no flip while
  // the earliest-flip comparison pinpoints the erased cells.
  const w = buildScenario('slit');
  const origin = probeOriginShift(w, FAT, w.start, w.goals[0], { axes: [0, 1] });
  check('origin shift over all tested offsets has no verdict flip (fixed pipeline)',
    origin.flipped === false);
  check('every shifted origin still finds the fat-slit route',
    origin.rows.every((r) => r.reachable === true));
  const flip = earliestFlipVoxel(w, FAT);
  check('axis-aligned raster erases many traversable cells', flip.erasedButTraversable > 100,
    String(flip.erasedButTraversable));
  check('earliest erased cell has a free orientation that is not axis-aligned',
    flip.earliest && flip.earliest.yawHintDeg !== null && Math.abs(flip.earliest.yawHintDeg) > 1,
    JSON.stringify(flip.earliest));
  // A minimal level keeps the geometry bracketing that cell plus the body.
  const mini = minimalLevel(w, FAT, flip.earliest, { start: w.start, goal: w.goals[0] });
  check('minimal level retains some obstacle geometry', mini.keptObstacles >= 1);
  check('minimal level carries the vehicle body', mini.vehicle.width === FAT.width);
  check('minimal level tightens bounds around the anchor',
    mini.bounds.max[0] - mini.bounds.min[0] < w.bounds.max[0] - w.bounds.min[0]);
  check('minimal level endpoints lie inside its own bounds',
    mini.start.every((v, d) => v >= mini.bounds.min[d] && v <= mini.bounds.max[d])
    && mini.goals[0].every((v, d) => v >= mini.bounds.min[d] && v <= mini.bounds.max[d]));
  // A successfully-routed case yields a minimal level that replays on its own.
  const rSlit = runNavigation(buildScenario('slit'), FAT,
    buildScenario('slit').start, buildScenario('slit').goals[0]);
  const dSlit = diagnose(buildScenario('slit'), FAT, rSlit.start, rSlit.goal);
  const mini2 = dSlit.minimalLevel;
  const replay = runNavigation(mini2, mini2.vehicle, mini2.start, mini2.goals[0], { startYaw: mini2.startYaw });
  check('minimal level from a routed case replays self-contained',
    replay.ok && replay.verification.continuousSafe,
    `replay ${replay.reason ?? ''} ${replay.verification?.measuredClearance ?? ''}`);
}

console.log('--- Failure flag traces to a collision sample ---');
{
  // The engine's verification records the exact pose of any contact.
  const w = buildScenario('cavity');
  const r = runNavigation(w, DEFAULT_DIMS, w.start, w.goals[0]);
  check('sealed cavity is unreachable', !r.ok && r.reason === 'unreachable');
  // With no route there is no verification record, but the reference audit still
  // explains enclosure; the trace from flag -> sample applies to routed cases.
  const okWorld = buildScenario('wreck');
  const ro = runNavigation(okWorld, DEFAULT_DIMS, okWorld.start, okWorld.goals[0]);
  check('routed case exposes a dense verification trace', ro.ok && ro.verification.trace.length > 10);
  check('trace entries carry position + yaw + clearance + obstacle id',
    ro.verification.trace.every((t) => Array.isArray(t.at) && Number.isFinite(t.yaw)
      && Number.isFinite(t.clearance) && Number.isFinite(t.obstacle)));
}

console.log('--- Mirror case: same physics, distinct heading, swept body kept ---');
{
  const w = buildScenario('slitMirror');
  const r = runNavigation(w, DEFAULT_DIMS, w.start, w.goals[0]);
  check('mirrored rotated slit is routed for the nominal vehicle', r.ok, r.reason);
  check('mirror success is continuously safe', r.ok && r.verification.continuousSafe);
  // It must actually cross the slot plane, not detour around the block ends.
  const yaw = Math.PI / 6 + Math.PI / 2;
  const u = [Math.cos(yaw), Math.sin(yaw)], n = [-Math.sin(yaw), Math.cos(yaw)];
  const du = r.samples.map((p) => p[0] * u[0] + p[1] * u[1]);
  const dn = r.samples.map((p) => Math.abs(p[0] * n[0] + p[1] * n[1]));
  check('mirror route crosses the slot centre plane',
    Math.min(...du) < 0 && Math.max(...du) > 0);
  check('mirror route stays inside the gap (|n| < gap/2)',
    Math.max(...dn) < 3.4 / 2, `max|n| ${Math.max(...dn).toFixed(2)}`);
  // Tight but legal: hug the obstacle with a small positive surface clearance.
  const half = vehicleHalf(DEFAULT_DIMS);
  let minSurf = Infinity;
  for (let i = 0; i < r.samples.length; i++) {
    for (const o of w.obstacles) minSurf = Math.min(minSurf, obstacleClearance(o, r.samples[i], r.sampleYaw[i], half));
  }
  check('mirror success hugs the obstacle yet stays legal',
    minSurf > 0 && minSurf < 0.35, `min surface ${minSurf.toFixed(3)}`);

  // Heading is recorded and the two slit directions are NOT collapsed: the
  // original slit's fat heading (approx -60 deg) differs from the mirror's.
  const orig = buildScenario('slit');
  const ro = runNavigation(orig, FAT, orig.start, orig.goals[0]);
  const deg = (a) => Math.round((a * 180) / Math.PI);
  check('original and mirror record different vehicle headings',
    ro.ok && r.ok && Math.abs(((deg(ro.laneYaw) - deg(r.laneYaw) + 540) % 360) - 180) > 10,
    `orig ${deg(ro.laneYaw)}° vs mirror ${deg(r.laneYaw)}°`);
  const hd = drawnHeading(r.laneYaw, r.startYaw);
  check('drawn heading is the committed lane yaw, not the path tangent',
    hd.laneYaw === r.laneYaw && /laneYaw/.test(hd.note));
}

console.log('--- The same patch does NOT tunnel a genuinely-too-narrow hole ---');
{
  // Squeeze the mirror gap to 2.0 m — genuinely narrower than the fat body can
  // thread (its 2.4 m length needs > 2.0 m) yet not obviously closed. The
  // oriented planner must fail rather than report a false pass.
  const w = buildScenario('slitMirror');
  const yaw = Math.PI / 6 + Math.PI / 2;
  const u = [Math.cos(yaw), Math.sin(yaw)], n = [-Math.sin(yaw), Math.cos(yaw)];
  const gap = 2.0, halfBlock = [6.0, 3.4, 7.0], off = halfBlock[1] + gap / 2;
  const squeezed = {
    ...w,
    id: 'slitMirrorNarrow',
    // Rebuild only the two slot blocks at the narrower gap, keep the rest.
    obstacles: [
      { type: 'box', center: [n[0] * off, n[1] * off, 8], half: halfBlock, yaw,
        rot: [[Math.cos(yaw), Math.sin(yaw), 0], [-Math.sin(yaw), Math.cos(yaw), 0], [0, 0, 1]] },
      { type: 'box', center: [-n[0] * off, -n[1] * off, 8], half: halfBlock, yaw,
        rot: [[Math.cos(yaw), Math.sin(yaw), 0], [-Math.sin(yaw), Math.cos(yaw), 0], [0, 0, 1]] },
      ...w.obstacles.filter((o) => o.type === 'box' && o.half[2] === 0.6),
    ],
  };
  const rn = runNavigation(squeezed, FAT, squeezed.start, squeezed.goals[0]);
  check('fat vehicle cannot pass the genuinely-too-narrow slot',
    !rn.ok, `ok=${rn.ok} reason=${rn.reason}`);
  check('the rejection is not a false pass (no delivered route)',
    !rn.ok || rn.verification.continuousSafe === false);
  // Direct sweep proof: fat body at the slot heading collides in the 2.0 m gap.
  const half = vehicleHalf(FAT);
  const a = [-u[0] * 3, -u[1] * 3, 8], b = [u[0] * 3, u[1] * 3, 8];
  const sweep = sweptTranslation(squeezed, a, b, 2 * Math.PI / 3, half, 0.3, 0.05);
  check('swept fat body collides across the 2.0 m gap', sweep.clear === false,
    `clearance ${sweep.clearance.toFixed(3)}`);
}

console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
