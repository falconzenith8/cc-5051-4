// Headless validation of the 3D navigation core. No UI involved.
import { buildScenario, DEFAULT_DIMS } from '../src/scenarios.js';
import { runNavigation, gridPayload } from '../src/engine.js';
import { worldSdf, sweptBoxClearance, naiveCenterlineCheck, vehicleHalf } from '../src/geometry.js';

let pass = 0, fail = 0;
function check(name, cond, detail = '') {
  if (cond) { pass++; console.log(`  PASS  ${name}`); }
  else { fail++; console.log(`  FAIL  ${name} ${detail}`); }
}

function summarize(r) {
  return {
    ok: r.ok, reason: r.reason,
    refReachable: r.reference.reachable,
    length: r.pathLength,
    safe: r.verification && r.verification.continuousSafe,
    clearance: r.verification && r.verification.conservativeClearance,
    naiveClear: r.naiveCenterline && r.naiveCenterline.clear,
    ms: r.timings.totalMs,
  };
}

console.log('--- Scenario 1: wreck (vertical detour mandatory) ---');
{
  const w = buildScenario('wreck');
  const r = runNavigation(w, DEFAULT_DIMS, w.start, w.goals[0]);
  console.log(' ', JSON.stringify(summarize(r)));
  check('wreck route found', r.ok);
  check('reference flood agrees route exists', r.reference.reachable === true);
  check('smoothed route passes continuous swept verification', r.ok && r.verification.continuousSafe);
  check('measured route clearance stays positive', r.ok && r.verification.measuredClearance > 0,
    `measured ${r.verification.measuredClearance}`);
  if (r.ok) {
    const zs = r.samples.map((p) => p[2]);
    const topGap = 10.5; // bulkhead top
    check('route genuinely climbs over the bulkhead (z > 10.5)', Math.max(...zs) > topGap,
      `max z = ${Math.max(...zs).toFixed(2)}`);
    check('route also dives/varies in z rather than flat tagging',
      Math.max(...zs) - Math.min(...zs) > 5);
    check('dense sample path is not axis-locked (contains diagonal motion)',
      r.samples.some((p, i) => i > 0 && Math.abs(p[0] - r.samples[i - 1][0]) > 1e-6
        && Math.abs(p[1] - r.samples[i - 1][1]) > 1e-6
        && Math.abs(p[2] - r.samples[i - 1][2]) > 1e-6));
  }
}

console.log('--- Scenario 2a: oblique slit, nominal vehicle fits through ---');
let slitWorld;
{
  slitWorld = buildScenario('slit');
  const r = runNavigation(slitWorld, DEFAULT_DIMS, slitWorld.start, slitWorld.goals[0]);
  console.log(' ', JSON.stringify(summarize(r)));
  check('nominal vehicle threads the slit', r.ok);
  check('reference agrees', r.reference.reachable === true);
  check('swept verification safe', r.ok && r.verification.continuousSafe);
  check('path actually crosses the gate plane x=0', r.ok &&
    r.samples.some((p) => Math.abs(p[0]) < 1.0));
}

console.log('--- Scenario 2b: widened vehicle cannot fit the slit (volume, not center) ---');
{
  const fat = { length: 2.4, width: 3.2, height: 2.6 };
  const r = runNavigation(slitWorld, fat, slitWorld.start, slitWorld.goals[0]);
  console.log(' ', JSON.stringify(summarize(r)));
  // The centerline down the middle of the free gap stays outside every solid
  // regardless of vehicle size; the volumetric check is what rejects the fat
  // vehicle whose box overlaps the slot walls.
  // Centerline inside the gate corridor only (avoid the block end caps):
  // the vehicle center stays in free space for ANY size, while the fat box
  // volume still collides with the slot walls.
  const gateLine = { a: [-3.0, 0, 8], b: [3.0, 0, 8] };
  const naive = naiveCenterlineCheck(slitWorld, gateLine.a, gateLine.b, 0.5, 0);
  check('naive centerline check wrongly reports the straight line empty',
    naive.clear === true, `naive clearance ${naive.clearance.toFixed(2)}`);
  // The fat vehicle's swept box along that same line must be rejected.
  const half = vehicleHalf(fat);
  const sweep = sweptBoxClearance(slitWorld, gateLine.a, gateLine.b, half, 0.3, 0.4, 0.05);
  check('swept box detects the fat vehicle colliding in the slit', sweep.clear === false);
  check('fat vehicle reported unreachable with blocking volume',
    !r.ok && r.reason === 'unreachable' && r.blocking && r.blocking.indices.length > 0);
  check('high-res reference agrees the fat vehicle is geometrically trapped',
    r.reference.reachable === false);
}

console.log('--- Scenario 3a: sealed cavity goal is unreachable ---');
{
  const w = buildScenario('cavity');
  const r = runNavigation(w, DEFAULT_DIMS, w.start, w.goals[0]);
  console.log(' ', JSON.stringify({ ok: r.ok, reason: r.reason, refReachable: r.reference.reachable,
    blocking: r.blocking && r.blocking.indices.length, reached: r.reached && r.reached.indices.length }));
  check('planner reports unreachable', !r.ok && r.reason === 'unreachable');
  check('high-res reference also says unreachable', r.reference.reachable === false);
  check('high-res reference identifies the enclosed goal',
    r.reference.enclosedGoal === true && r.reference.reachable === false);
  check('blocking volume (shell frontier) returned for display',
    r.blocking && r.blocking.indices.length > 0);
  check('reached free component returned for display',
    r.reached && r.reached.indices.length > 0);
}

console.log('--- Scenario 3b: same cavity world, external goal IS reachable ---');
{
  const w = buildScenario('cavity');
  const r = runNavigation(w, DEFAULT_DIMS, w.start, w.goals[1]);
  console.log(' ', JSON.stringify(summarize(r)));
  check('external control goal routed', r.ok);
  check('reference agrees reachable', r.reference.reachable === true);
  check('route safe under swept verification', r.ok && r.verification.continuousSafe);
}

console.log('--- Sampled-empty does not imply segment-safe demonstration ---');
{
  const w = buildScenario('slit');
  const fat = { length: 2.4, width: 3.2, height: 2.6 };
  const half = vehicleHalf(fat);
  // Pick two free center points straddling a slit wall at coarse spacing such
  // that every sampled center is outside the solid.
  const a = [-3.2, 0, 8], b = [3.2, 0, 8];
  const naive = naiveCenterlineCheck(w, a, b, 1.0, 0);
  const sweep = sweptBoxClearance(w, a, b, half, 0.3, 0.4, 0.05);
  console.log(`  naive centerline clearance=${naive.clearance.toFixed(2)} clear=${naive.clear}; swept clear=${sweep.clear}`);
  check('center samples all empty yet swept box collides',
    naive.clear === true && sweep.clear === false);
}

console.log('--- Eroded grid payload sanity ---');
{
  const w = buildScenario('wreck');
  const p = gridPayload(w, DEFAULT_DIMS);
  check('grid payload carries blocked voxels in config space', p.indices.length > 0);
  check('blocked voxels are reported with their analytic clearance',
    p.clearance.every((c) => Number.isFinite(c)));
  const fat = { length: 5, width: 5, height: 5 };
  const pf = gridPayload(w, fat);
  check('larger vehicle erodes strictly more space', pf.indices.length > p.indices.length,
    `${pf.indices.length} vs ${p.indices.length}`);
}

console.log('--- Grid erosion vs swept-check consistency (no phantom free space) ---');
{
  // The axis-aligned erosion is a necessary condition for every orientation:
  // wherever the eroded grid flood fill cannot go, the swept A* must also fail,
  // so the planner can never claim a corridor the vehicle volume cannot use.
  const fat = { length: 2.4, width: 3.2, height: 2.6 };
  for (const id of ['slit', 'wreck', 'cavity']) {
    const w = buildScenario(id);
    const goal = id === 'cavity' ? w.goals[0] : w.goals[0];
    const r = runNavigation(w, fat, w.start, goal);
    const ref = r.reference;
    check(`${id}: planner reachability never exceeds the eroded reference`,
      r.ok === false ? ref.reachable === false : ref.reachable === true,
      `planner ok=${r.ok} ref=${ref.reachable}`);
  }
}

console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
