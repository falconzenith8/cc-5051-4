// Procedurally generated 3D obstacle scenarios. Every scenario is a list of
// analytic solids (oriented boxes / spheres / cylinders) inside a rectangular
// water volume — obstacles are never removed to make a route fit.
import { makeRng } from './rng.js';

export const DEFAULT_DIMS = { length: 2.4, width: 1.4, height: 1.2 };

// Shared water volume for all scenarios: 34 x 22 x 16 metres, z is up.
const BOUNDS = { min: [-17, -11, 0], max: [17, 11, 16] };

function box(center, half, yaw = 0) {
  return { type: 'box', center, half, rot: yaw ? [
    [Math.cos(yaw), Math.sin(yaw), 0],
    [-Math.sin(yaw), Math.cos(yaw), 0],
    [0, 0, 1],
  ] : null, yaw };
}
function sphere(center, radius) { return { type: 'sphere', center, radius }; }
function cyl(center, radius) { return { type: 'cylinderZ', center, radius }; }

// 1) WRECK HULL: a full-width transverse bulkhead with only an overhead gap,
//    plus an overhang canopy above the start. The direct horizontal route is
//    sealed; the vehicle must climb over the bulkhead and weave under the
//    canopy — a route reachable ONLY by vertical detour. Hull ribs and seabed
//    mounds are procedurally scattered with a fixed seed.
function wreckScenario() {
  const obstacles = [];
  const rng = makeRng(20260815);

  // Seabed mounds (spheres) — undulating terrain.
  for (let i = 0; i < 7; i++) {
    const x = -13 + rng() * 26;
    const y = -9 + rng() * 18;
    const r = 1.2 + rng() * 1.8;
    obstacles.push(sphere([x, y, -r * 0.55], r));
  }

  // Full-width transverse bulkhead at x=2, from seabed to z=10.5.
  // Gap above it (z in 10.5..16) is the ONLY way across.
  obstacles.push(box([2, 0, 5.25], [0.9, 11.2, 5.25]));

  // Overhang canopy above the start side: blocks the straight climb, forcing
  // the route to slide laterally while rising — genuine 3D weaving.
  obstacles.push(box([-2.5, 1.5, 13.6], [4.6, 6.5, 1.1]));

  // Wreck hull ribs: tilted slabs along x on the far side, leaving staggered
  // gaps so the descent is also a 3D problem.
  const ribYaws = [0.5, -0.4, 0.35];
  const ribX = [6, 9, 12];
  for (let i = 0; i < 3; i++) {
    obstacles.push(box([ribX[i], -1 + i * 1.5, 5.5], [0.7, 4.5, 4.0], ribYaws[i]));
  }
  // A hanging cylinder (pipe) narrows the overhead channel.
  obstacles.push(cyl([2, -6.5, 13.4], 0.9));

  return {
    id: 'wreck',
    name: '沉船外壳：仅垂直绕行可达',
    description: '横贯整个水域的舱壁只在顶部留缝，入口上方还有悬垂顶盖，载具必须先侧向横移再向上翻越、再从交错肋骨间下降。',
    bounds: BOUNDS,
    obstacles,
    start: [-14, -8, 4],
    goals: [
      [11.5, 8.5, 5],
      [8, 7.5, 12.5],
      [-6, 7, 12.5],
    ],
  };
}

// 2) OBLIQUE SLIT: two thick blocks form a yawed slot (~30 degrees). The gap
//    width is sized against the vehicle's cross-section: passable only when
//    the vehicle body is aligned with the slot; an over-sized vehicle cannot
//    squeeze through and must go around. Centerline-only checks wrongly say
//    "empty" for any size — the volumetric/swept check is what rejects it.
function slitScenario() {
  const obstacles = [];
  const rng = makeRng(77001);
  const yaw = Math.PI / 6; // 30 degrees

  // Two long blocks, gap of 3.4 m between their faces, both yawed together.
  // Nominal vehicle (1.4 m wide) threads it when aligned; the 3.2 m-wide
  // variant cannot and must detour around the block ends.
  const halfBlock = [7.5, 3.4, 7.0];
  const gap = 3.4;
  const off = halfBlock[1] + gap / 2;
  // Perpendicular offset in world frame.
  const px = -Math.sin(yaw) * off, py = Math.cos(yaw) * off;
  obstacles.push(box([px, py, 8], halfBlock, yaw));
  obstacles.push(box([-px, -py, 8], halfBlock, yaw));

  // Ceiling and floor slabs cap the vertical escape near the slot so an
  // over-sized vehicle is genuinely forced to detour around the block ends.
  obstacles.push(box([0, 0, 15.4], [9.0, 9.0, 0.6]));
  obstacles.push(box([0, 0, 0.6], [9.0, 9.0, 0.6]));

  // Scattered rocks outside the gate for visual/navigation richness.
  for (let i = 0; i < 5; i++) {
    const side = rng() < 0.5 ? -1 : 1;
    const x = side * (10 + rng() * 3.5);
    const y = -8 + rng() * 16;
    const z = 1.5 + rng() * 5;
    obstacles.push(sphere([x, y, z], 0.8 + rng() * 0.9));
  }

  return {
    id: 'slit',
    name: '斜向狭孔：按载具体积判定可通过性',
    description: '两块厚壁形成 30° 斜缝，缝宽按载具横截面给定。标准载具可贴缝穿过；加宽后的载具中心轨迹仍在空区，但体积无法通过，只能绕壁端而行。',
    bounds: BOUNDS,
    obstacles,
    start: [-14, 0, 8],
    goals: [
      [11.5, 0, 8],
      [11, -7, 12],
    ],
  };
}

// 3) SEALED CAVITY: the target sits inside a fully closed shell. No route
//    exists; the planner must report unreachable and return the blocking
//    shell as the blocking volume together with the reached free component.
function cavityScenario() {
  const obstacles = [];
  const rng = makeRng(424242);

  // Closed room centered at (2, 1, 8), interior 9 x 7 x 6, walls 1 m thick.
  const c = [2, 1, 8];
  const interior = [4.5, 3.5, 3.0];
  const t = 1.0;
  // floor / ceiling
  obstacles.push(box([c[0], c[1], c[2] - interior[2] - t / 2], [interior[0] + t, interior[1] + t, t / 2]));
  obstacles.push(box([c[0], c[1], c[2] + interior[2] + t / 2], [interior[0] + t, interior[1] + t, t / 2]));
  // four walls
  obstacles.push(box([c[0] - interior[0] - t / 2, c[1], c[2]], [t / 2, interior[1], interior[2]]));
  obstacles.push(box([c[0] + interior[0] + t / 2, c[1], c[2]], [t / 2, interior[1], interior[2]]));
  obstacles.push(box([c[0], c[1] - interior[1] - t / 2, c[2]], [interior[0], t / 2, interior[2]]));
  obstacles.push(box([c[0], c[1] + interior[1] + t / 2, c[2]], [interior[0], t / 2, interior[2]]));

  // External rocks so the approach itself is a navigation problem.
  for (let i = 0; i < 6; i++) {
    const x = -12 + rng() * 10;
    const y = -8 + rng() * 16;
    const z = 1.5 + rng() * 6;
    obstacles.push(sphere([x, y, z], 0.9 + rng() * 1.1));
  }
  // Overhang between start and the cavity.
  obstacles.push(box([-5.5, -3, 12.5], [3.2, 5.5, 1.0], 0.3));

  return {
    id: 'cavity',
    name: '封闭空腔：不可达目标与阻断体积',
    obstacles,
    bounds: BOUNDS,
    start: [-14, -8, 4],
    goals: [
      c, // inside the sealed shell
      [11, 7, 13.5], // reachable control goal to prove the same planner succeeds elsewhere
    ],
  };
}

// 4) MIRRORED / ROTATED HOLE: the same oblique slit copied to the opposite
//    direction of the map and turned a further 90 degrees (30° -> 120°). The
//    gap width, block thickness and vehicle are identical to scenario 2, so the
//    passability physics is the same — but the failure heading is different.
//    Recording each case with its own committed lane yaw and swept body is what
//    keeps the two directions distinguishable; a single voxel screenshot would
//    collapse them and lose the evidence basis. Start/goal are biased toward the
//    +n slot wall so the delivered route hugs the obstacle while staying legal.
function slitMirrorScenario() {
  const obstacles = [];
  const rng = makeRng(77002);
  const yaw = Math.PI / 6 + Math.PI / 2; // 120 degrees
  const u = [Math.cos(yaw), Math.sin(yaw)]; // corridor (slot long axis)
  const n = [-Math.sin(yaw), Math.cos(yaw)]; // gap normal

  // Same gap (3.4 m) and thickness as scenario 2, but the block LENGTH is
  // shortened so the rotated axis still fits the map and both vehicles keep a
  // legal detour. Physics of the slot itself is unchanged.
  const halfBlock = [6.0, 3.4, 7.0];
  const gap = 3.4;
  const off = halfBlock[1] + gap / 2;
  obstacles.push(box([n[0] * off, n[1] * off, 8], halfBlock, yaw));
  obstacles.push(box([-n[0] * off, -n[1] * off, 8], halfBlock, yaw));

  // Same capping slabs as scenario 2: vertical escape is sealed near the slot.
  obstacles.push(box([0, 0, 15.4], [9.0, 9.0, 0.6]));
  obstacles.push(box([0, 0, 0.6], [9.0, 9.0, 0.6]));

  for (let i = 0; i < 5; i++) {
    const side = rng() < 0.5 ? -1 : 1;
    const x = side * (10 + rng() * 3.5);
    const y = -8 + rng() * 16;
    const z = 1.5 + rng() * 5;
    obstacles.push(sphere([x, y, z], 0.8 + rng() * 0.9));
  }

  // Bias 0.5 m toward the +n wall: the legal route skims that side (min surface
  // clearance ~0.20 m) instead of running down the middle.
  const bias = 0.5, reach = 9;
  const start = [-u[0] * reach + n[0] * bias, -u[1] * reach + n[1] * bias, 8];
  const goal = [u[0] * reach + n[0] * bias, u[1] * reach + n[1] * bias, 8];

  return {
    id: 'slitMirror',
    name: '镜像斜孔：复制到另一方向并旋转 90°',
    description: '把 30° 斜缝复制到地图另一方向并再旋转 90°（120°）。缝宽与载具与场景二一致，通过性物理相同，但失败朝向不同——每个案例分别保存载具朝向与扫掠体，不会把两个方向不同的案例压缩成同一体素截图。起点/目标偏向 +n 侧缝壁，成功轨迹紧贴障碍但始终合法。',
    bounds: BOUNDS,
    obstacles,
    start,
    goals: [goal],
  };
}

const BUILDERS = { wreck: wreckScenario, slit: slitScenario, cavity: cavityScenario, slitMirror: slitMirrorScenario };

export function buildScenario(id) {
  const b = BUILDERS[id];
  if (!b) throw new Error('unknown scenario ' + id);
  return b();
}

export function scenarioIds() {
  return Object.keys(BUILDERS);
}
