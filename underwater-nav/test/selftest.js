// Headless validation of the 3D navigation core. No UI involved.
import { buildScenario, DEFAULT_DIMS } from '../src/scenarios.js';
import { runNavigation, gridPayload } from '../src/engine.js';
import { worldSdf, sweptBoxClearance, sweptTranslation, naiveCenterlineCheck, vehicleHalf } from '../src/geometry.js';

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

console.log('--- Scenario 2b: fat vehicle threads the oblique slit only when aligned ---');
{
  const fat = { length: 2.4, width: 3.2, height: 2.6 };
  const href = vehicleHalf(fat);
  // The gate line stays clear of every solid; a centerline-only check is blind to
  // the body. The vehicle genuinely fits the ~30 deg slot, but ONLY at the slot
  // angle: swept at the slot's own heading it clears, swept axis-aligned it hits
  // the walls. This is the orientation-aware result the axis-aligned erosion
  // used to erase.
  // Corridor runs along the slot's long axis (30 deg from x); a line through the
  // gap centre. The fat box passes only when its long axis is turned to the slot
  // normal (yaw 120 deg ≡ -60 deg): its 2.4 m length then spans the 3.4 m gap.
  const dir = [Math.cos(Math.PI / 6), Math.sin(Math.PI / 6)];
  const gateLine = { a: [-4 * dir[0], -4 * dir[1], 8], b: [4 * dir[0], 4 * dir[1], 8] };
  const naive = naiveCenterlineCheck(slitWorld, gateLine.a, gateLine.b, 0.5, 0);
  check('naive centerline check wrongly reports the straight line empty',
    naive.clear === true, `naive clearance ${naive.clearance.toFixed(2)}`);
  const slotYaw = (2 * Math.PI) / 3; // long axis across the gap
  const aligned = sweptTranslation(slitWorld, gateLine.a, gateLine.b, slotYaw, href, 0.4, 0.05);
  const axis = sweptTranslation(slitWorld, gateLine.a, gateLine.b, 0, href, 0.4, 0.05);
  check('swept axis-aligned body collides in the slit', axis.clear === false,
    `axis clearance ${axis.clearance.toFixed(3)}`);
  check('same body threads the slit at the slot heading',
    aligned.clear === true && aligned.clearance > 0.05,
    `aligned clearance ${aligned.clearance.toFixed(3)}`);

  const r = runNavigation(slitWorld, fat, slitWorld.start, slitWorld.goals[0]);
  console.log(' ', JSON.stringify(summarize(r)));
  check('planner threads the fat vehicle through the slit at an oblique lane',
    r.ok && r.laneYaw !== 0 && r.verification.continuousSafe,
    `ok=${r.ok} lane=${(r.laneYaw * 180 / Math.PI).toFixed(0)}`);
  check('the delivered fat route stays clear (no false pass through the shell)',
    r.ok && r.verification.measuredClearance > 0,
    `measured ${r.verification && r.verification.measuredClearance}`);
  // The conservative axis-aligned reference is a subset of true free space, so it
  // may under-report an oblique corridor; it must never over-report one.
  check('conservative reference does not over-report vs the oriented planner',
    r.ok || r.reference.reachable === false,
    `planner ok=${r.ok} ref=${r.reference.reachable}`);
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

console.log('--- Oriented planner vs conservative reference (soundness direction) ---');
{
  // The fine reference raster is axis-aligned, so its free set is a SUBSET of
  // true free space: it may miss an oblique corridor (as in the fat-slit case)
  // but must never invent one. The oriented planner therefore may reach a goal
  // the reference calls enclosed, but a reference-reachable goal must also be
  // planner-reachable, and every delivered route must be continuously safe.
  const fat = { length: 2.4, width: 3.2, height: 2.6 };
  for (const id of ['slit', 'wreck', 'cavity']) {
    const w = buildScenario(id);
    const goal = w.goals[0];
    const r = runNavigation(w, fat, w.start, goal);
    const ref = r.reference;
    check(`${id}: reference-reachable implies planner-reachable`,
      !ref.reachable || r.ok === true,
      `planner ok=${r.ok} ref=${ref.reachable}`);
    check(`${id}: delivered route (if any) is continuously safe`,
      !r.ok || r.verification.continuousSafe,
      `ok=${r.ok}`);
  }
  // The fat-slit corridor the conservative reference cannot see is exactly the
  // case that used to be misclassified as trapped.
  const slit = buildScenario('slit');
  const rs = runNavigation(slit, fat, slit.start, slit.goals[0]);
  check('conservative reference under-reports the oblique fat-slit corridor',
    rs.ok === true && rs.reference.reachable === false,
    `planner ok=${rs.ok} ref=${rs.reference.reachable}`);
}

console.log(`\n${pass} passed, ${fail} failed`);
process.exit(fail ? 1 : 0);
