// True 3D analytic geometry: oriented SDF primitives and rigid box (vehicle) checks.
// SDF convention: f(point) < 0 means inside the solid obstacle.

export function add(a, b) { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
export function sub(a, b) { return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]; }
export function scale(a, s) { return [a[0] * s, a[1] * s, a[2] * s]; }
export function dot(a, b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
export function len(a) { return Math.hypot(a[0], a[1], a[2]); }
export function dist(a, b) { return len(sub(a, b)); }

export function clamp(v, lo, hi) { return Math.max(lo, Math.min(hi, v)); }

// Rotation about the Z (up) axis. Vehicles/obstacles here are yaw-rotated.
export function rotZ(theta) {
  const c = Math.cos(theta), s = Math.sin(theta);
  return [
    [c, -s, 0],
    [s, c, 0],
    [0, 0, 1],
  ];
}

export function applyRot(R, p) {
  return [
    R[0][0] * p[0] + R[0][1] * p[1] + R[0][2] * p[2],
    R[1][0] * p[0] + R[1][1] * p[1] + R[1][2] * p[2],
    R[2][0] * p[0] + R[2][1] * p[1] + R[2][2] * p[2],
  ];
}

// World -> local frame of an oriented obstacle (inverse rotation = transpose).
function toLocal(p, o) {
  const d = sub(p, o.center);
  if (!o.rot) return d;
  return applyRot([o.rot[0], o.rot[1], o.rot[2]], d); // transpose rows
}

// Signed distance to an axis-aligned box of half extents h, evaluated at local p.
function sdfBoxLocal(p, h) {
  const qx = Math.abs(p[0]) - h[0];
  const qy = Math.abs(p[1]) - h[1];
  const qz = Math.abs(p[2]) - h[2];
  const ox = Math.max(qx, 0), oy = Math.max(qy, 0), oz = Math.max(qz, 0);
  const outside = Math.hypot(ox, oy, oz);
  const inside = Math.min(Math.max(qx, qy, qz), 0);
  return outside + inside;
}

function sdfSphereLocal(p, r) {
  return len(p) - r;
}

// Infinite cylinder with its axis along local Z.
function sdfCylinderZLocal(p, r) {
  return Math.hypot(p[0], p[1]) - r;
}

// SDF of one obstacle at a world-space point.
export function obstacleSdf(o, p) {
  const lp = toLocal(p, o);
  switch (o.type) {
    case 'box': return sdfBoxLocal(lp, o.half);
    case 'sphere': return sdfSphereLocal(lp, o.radius);
    case 'cylinderZ': return sdfCylinderZLocal(lp, o.radius);
    default: throw new Error('unknown obstacle type ' + o.type);
  }
}

// Union SDF of a world of analytic obstacles (outside bounds count as solid too).
export function worldSdf(world, p) {
  let m = Infinity;
  const obs = world.obstacles;
  for (let i = 0; i < obs.length; i++) {
    const d = obstacleSdf(obs[i], p);
    if (d < m) m = d;
  }
  return m;
}

// Signed distance to the rectangular water-volume boundary (negative outside).
export function boundsSdf(world, p) {
  const b = world.bounds;
  const hx = (b.max[0] - b.min[0]) / 2;
  const hy = (b.max[1] - b.min[1]) / 2;
  const hz = (b.max[2] - b.min[2]) / 2;
  const c = [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, (b.min[2] + b.max[2]) / 2];
  return -sdfBoxLocal(sub(p, c), [hx, hy, hz]); // positive inside, negative outside
}

// Vehicle: oriented rigid box. Local axes: x = length, y = width, z = height.
// half = [L/2, W/2, H/2]; yaw rotates about Z.
export function vehicleHalf(dims) {
  return [dims.length / 2, dims.width / 2, dims.height / 2];
}

// Inscribed-sphere radius of the vehicle box: erosion radius used to build the
// configuration-space occupancy (conservative: guarantees no penetration when
// the center keeps at least this clearance).
export function erosionRadius(dims) {
  return Math.min(dims.length, dims.width, dims.height) / 2;
}

// Sparse convex-hull probes for the planning-time pose test: 8 corners plus
// 6 face centers. For any convex obstacle SDF, these bound the minimum over
// the whole box surface; the dense surface grid is reserved for the final
// route verification.
export function sparseProbes(half) {
  const [hx, hy, hz] = half;
  const pts = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
    pts.push([sx * hx, sy * hy, sz * hz]);
  }
  pts.push([hx, 0, 0], [-hx, 0, 0], [0, hy, 0], [0, -hy, 0], [0, 0, hz], [0, 0, -hz]);
  return pts;
}

// Pre-rotate the sparse probes for every candidate yaw once per planning run.
export function rotatedProbes(half, yaw) {
  const R = rotZ(yaw);
  return sparseProbes(half).map((lp) => applyRot(R, lp));
}

// Clearance of an oriented box pose using a fixed pre-rotated probe set.
export function poseClearanceSparse(world, center, probes, margin, inRadius) {
  let m = Infinity;
  for (let i = 0; i < probes.length; i++) {
    const rp = probes[i];
    const d = worldSdf(world, [center[0] + rp[0], center[1] + rp[1], center[2] + rp[2]]);
    if (d < m) m = d;
    if (d <= margin) return { clear: false, clearance: d };
  }
  if (boundsSdf(world, center) <= inRadius + margin) return { clear: false, clearance: m };
  return { clear: true, clearance: m };
}

// Surface sample points of the oriented vehicle box at pose (center, yaw),
// in world coordinates. Step <= stepMax along every edge. Used only for the
// dense final verification of the smoothed route.
export function boxSurfacePoints(center, yaw, half, stepMax) {
  const [hx, hy, hz] = half;
  const corners = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
    corners.push([sx * hx, sy * hy, sz * hz]);
  }
  const set = new Map();
  const put = (lp) => {
    const wp = add(applyRot(rotZ(yaw), lp), center);
    set.set(wp.map((v) => Math.round(v * 1e5)).join(','), wp);
  };
  // 8 corners
  for (const c of corners) put(c);
  // 12 edges, subdivided
  const edges = [];
  for (let i = 0; i < 8; i++) for (let j = i + 1; j < 8; j++) {
    const diff = sub(corners[j], corners[i]);
    const nDiff = diff.reduce((acc, v) => acc + (v !== 0 ? 1 : 0), 0);
    if (nDiff === 1) edges.push([corners[i], corners[j]]);
  }
  for (const [a, b] of edges) {
    const el = dist(a, b);
    const n = Math.max(1, Math.ceil(el / stepMax));
    for (let k = 0; k <= n; k++) {
      put(add(a, scale(sub(b, a), k / n)));
    }
  }
  // Face interior points (6 faces) so a large flat face cannot hide between edges.
  const faceAxes = [
    { axis: 0, sign: -1 }, { axis: 0, sign: 1 },
    { axis: 1, sign: -1 }, { axis: 1, sign: 1 },
    { axis: 2, sign: -1 }, { axis: 2, sign: 1 },
  ];
  for (const f of faceAxes) {
    const others = [0, 1, 2].filter((a) => a !== f.axis);
    const h0 = half[others[0]], h1 = half[others[1]];
    const n0 = Math.max(1, Math.ceil((2 * h0) / stepMax));
    const n1 = Math.max(1, Math.ceil((2 * h1) / stepMax));
    for (let i = 0; i <= n0; i++) for (let j = 0; j <= n1; j++) {
      const lp = [0, 0, 0];
      lp[f.axis] = f.sign * half[f.axis];
      lp[others[0]] = -h0 + (2 * h0 * i) / n0;
      lp[others[1]] = -h1 + (2 * h1 * j) / n1;
      put(lp);
    }
  }
  return [...set.values()];
}

// ---------------------------------------------------------------------------
// EXACT oriented vehicle-box vs analytic primitive clearance.
//
// The oriented box is the vehicle at (center, yaw). For each obstacle we return
// d <= 0 exactly when the two solids intersect, and a positive value otherwise.
// For disjoint convex pairs this is a true (linearised) minimum distance, and for
// overlapping pairs a penetration depth; the box-vs-box value can be the minimum
// over the separating-axes' overlaps, so it is always a LOWER bound on the
// distance to the obstacle surface — never an over-estimate. That direction of
// error is the safe one: the planner rejects at most a few extra poses, it never
// accepts one that the dense surface sampling (geometry sdf) would find inside.
//
// This replaces the probe-based pose test at the authoritative checkpoints.
// It is smooth in yaw, so a committed heading is evaluated honestly instead of
// being silently satisfied by "some other yaw fits".
// ---------------------------------------------------------------------------

// Reply-frame axes of a yaw rotation about Z (unit vectors, world XY).
function yawAxes(yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return [[c, s], [-s, c]];
}

export function obstacleClearance(o, center, yaw, half) {
  const [u0, u1] = yawAxes(yaw);
  const C = center;
  if (o.type === 'box') {
    const [v0, v1] = yawAxes(o.yaw || 0);
    const dx = C[0] - o.center[0], dy = C[1] - o.center[1];
    let minOverlap = Infinity;
    let maxGap = -Infinity;
    // Four candidate separating axes in the horizontal plane: two vehicle
    // face normals and two obstacle face normals (rotated-box SAT in 2D).
    for (const n of [u0, u1, v0, v1]) {
      const proj = Math.abs(dx * n[0] + dy * n[1]);
      const rv = half[0] * Math.abs(u0[0] * n[0] + u0[1] * n[1])
        + half[1] * Math.abs(u1[0] * n[0] + u1[1] * n[1]);
      const ro = o.half[0] * Math.abs(v0[0] * n[0] + v0[1] * n[1])
        + o.half[1] * Math.abs(v1[0] * n[0] + v1[1] * n[1]);
      const overlap = rv + ro - proj;
      if (overlap < minOverlap) minOverlap = overlap;
      if (overlap < 0 && -overlap > maxGap) maxGap = -overlap;
    }
    // Vertical interval (both boxes are z-aligned).
    const overlapZ = half[2] + o.half[2] - Math.abs(C[2] - o.center[2]);
    if (overlapZ < minOverlap) minOverlap = overlapZ;
    if (overlapZ < 0 && -overlapZ > maxGap) maxGap = -overlapZ;
    return minOverlap >= 0 ? -minOverlap : maxGap;
  }
  if (o.type === 'sphere') {
    const dx = o.center[0] - C[0], dy = o.center[1] - C[1], dz = o.center[2] - C[2];
    const l = [dx * u0[0] + dy * u0[1], dx * u1[0] + dy * u1[1], dz];
    const cx = clamp(l[0], -half[0], half[0]);
    const cy = clamp(l[1], -half[1], half[1]);
    const cz = clamp(l[2], -half[2], half[2]);
    const dist = Math.hypot(l[0] - cx, l[1] - cy, l[2] - cz);
    if (dist > 1e-12) return dist - o.radius;
    // Sphere center is inside the box: penetration = distance to the nearest face.
    const inside = Math.min(half[0] - Math.abs(l[0]), half[1] - Math.abs(l[1]), half[2] - Math.abs(l[2]));
    return -(inside + o.radius);
  }
  if (o.type === 'cylinderZ') {
    const dx = o.center[0] - C[0], dy = o.center[1] - C[1];
    const l = [dx * u0[0] + dy * u0[1], dx * u1[0] + dy * u1[1]];
    const cx = clamp(l[0], -half[0], half[0]);
    const cy = clamp(l[1], -half[1], half[1]);
    const dist = Math.hypot(l[0] - cx, l[1] - cy);
    if (dist > 1e-12) return dist - o.radius;
    const inside = Math.min(half[0] - Math.abs(l[0]), half[1] - Math.abs(l[1]));
    return -(inside + o.radius);
  }
  throw new Error('unknown obstacle type ' + o.type);
}

// Clearance of the oriented vehicle box to the water-volume walls (box corners).
export function boundsClearance(world, center, yaw, half) {
  const [u0, u1] = yawAxes(yaw);
  let m = Infinity;
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
    const p = [
      center[0] + sx * half[0] * u0[0] + sy * half[1] * u1[0],
      center[1] + sx * half[0] * u0[1] + sy * half[1] * u1[1],
      center[2] + sz * half[2],
    ];
    const d = boundsSdf(world, p);
    if (d < m) m = d;
  }
  return m;
}

// Exact obstacle clearance of the oriented vehicle at one pose. obstacle = the
// index of the closest obstacle, or -1 for none, -2 for the water boundary.
export function poseClearanceExact(world, center, yaw, half, margin = -Infinity) {
  let m = Infinity, obstacle = -1;
  const obs = world.obstacles;
  for (let i = 0; i < obs.length; i++) {
    const d = obstacleClearance(obs[i], center, yaw, half);
    if (d < m) { m = d; obstacle = i; }
  }
  const b = boundsClearance(world, center, yaw, half);
  if (b < m) { m = b; obstacle = -2; }
  return { clear: m > margin, clearance: m, obstacle };
}

// Rotation-in-place check: sweep the yaw from y0 to y1 at a fixed center. Uses
// the same conservative Lipschitz bound (arcChord/2) as translation sweeps.
export function rotationClearance(world, center, y0, y1, half, margin = 0) {
  const delta = Math.abs(shortestAngle(y1 - y0));
  if (delta < 1e-9) return { clear: true, clearance: Infinity };
  const circum = Math.hypot(half[0], half[1], half[2]);
  const n = Math.max(1, Math.ceil((circum * delta) / ROT_STEP));
  let worst = Infinity, obstacle = -1;
  const dy = y1 - y0;
  for (let i = 0; i <= n; i++) {
    const yaw = y0 + dy * (i / n);
    const r = poseClearanceExact(world, center, yaw, half, -Infinity);
    if (r.clearance < worst) { worst = r.clearance; obstacle = r.obstacle; }
    if (r.clearance <= margin) return { clear: false, clearance: r.clearance, obstacle, yaw };
  }
  const conservative = worst - (circum * delta / n) / 2;
  return { clear: conservative > margin, clearance: conservative, obstacle };
}

// Translation at a CONSTANT yaw: this is exactly the pose the renderer draws for
// a waypoint segment, so the swept verdict refers to the delivered motion rather
// than to "the best of several orientations".
export function sweptTranslation(world, pa, pb, yaw, half, motionStep, margin = 0) {
  const d = dist(pa, pb);
  const n = Math.max(1, Math.ceil(d / motionStep));
  let worst = Infinity, worstPose = 0, obstacle = -1;
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    const c = add(pa, scale(sub(pb, pa), t));
    const r = poseClearanceExact(world, c, yaw, half, -Infinity);
    if (r.clearance < worst) { worst = r.clearance; worstPose = i; obstacle = r.obstacle; }
    if (r.clearance <= margin) {
      return { clear: false, clearance: r.clearance, measured: worst, at: c, yaw, pose: i, obstacle };
    }
  }
  const conservative = worst - motionStep / 2;
  return { clear: conservative > margin, clearance: conservative, measured: worst, at: null, yaw, pose: worstPose, obstacle };
}

export const ROT_STEP = 0.3;

// Signed shortest angular difference in (-pi, pi].
export function shortestAngle(a) {
  let x = a % (Math.PI * 2);
  if (x <= -Math.PI) x += Math.PI * 2;
  if (x > Math.PI) x -= Math.PI * 2;
  return x;
}

export function angleDistance(a, b) {
  return Math.abs(shortestAngle(a - b));
}

// Exact-ish collision of the oriented vehicle box at a single pose:
// minimum obstacle SDF over the box surface <= margin means collision/too close.
export function poseClearance(world, center, yaw, half, margin, stepMax) {
  const pts = boxSurfacePoints(center, yaw, half, stepMax);
  let m = Infinity;
  for (const p of pts) {
    const d = worldSdf(world, p);
    if (d < m) m = d;
    if (d <= margin) return { clear: false, clearance: d };
  }
  const b = boundsSdf(world, center);
  // Inscribed-sphere bound: sufficient to keep the whole box inside the water
  // volume (the box contains this sphere), without the circumscribed sphere
  // wrongly rejecting motions near a boundary.
  const inRadius = Math.min(half[0], half[1], half[2]);
  if (b <= inRadius + margin) return { clear: false, clearance: b };
  return { clear: true, clearance: m };
}

// Heading of a vehicle moving a -> b in the horizontal plane (yaw about Z).
export function heading(a, b) {
  return Math.atan2(b[1] - a[1], b[0] - a[0]);
}

// Planning-time swept test on the sparse convex-hull probes. Candidate yaw
// probe sets are built once per call (14 rotated points each), then each pose
// evaluates 14 SDFs with no per-pose allocation. The Lipschitz conservative
// bound (motionStep/2) still backs every accept decision.
export function sweptBoxClearance(world, pa, pb, half, surfaceStep, motionStep, margin = 0) {
  const d = dist(pa, pb);
  const n = Math.max(1, Math.ceil(d / motionStep));
  const inRadius = Math.min(half[0], half[1], half[2]);
  const h = heading(pa, pb);
  const yaws = [h];
  for (let i = 0; i < 8; i++) {
    const yaw = (i * Math.PI) / 8;
    if (Math.abs(((yaw - h) % Math.PI + Math.PI) % Math.PI) >= 1e-9) yaws.push(yaw);
  }
  let best = null;
  for (const yaw of yaws) {
    const probes = rotatedProbes(half, yaw);
    // Necessary-condition center prune for this heading (inscribed sphere).
    let pruned = false;
    for (let i = 0; i <= n; i++) {
      const t = i / n;
      const center = add(pa, scale(sub(pb, pa), t));
      if (worldSdf(world, center) <= inRadius + margin) { pruned = true; break; }
    }
    if (pruned) {
      if (!best || best.pruned) best = { clear: false, pruned: true, yaw };
      continue;
    }
    let worst = Infinity;
    let failed = false;
    for (let i = 0; i <= n; i++) {
      const t = i / n;
      const center = add(pa, scale(sub(pb, pa), t));
      const r = poseClearanceSparse(world, center, probes, margin, inRadius);
      if (r.clearance < worst) worst = r.clearance;
      if (!r.clear) { failed = true; break; }
    }
    if (!failed) {
      const conservative = worst - motionStep / 2;
      if (conservative > margin) {
        return { clear: true, clearance: conservative, measured: worst, yaw };
      }
    }
    if (!best || !best.pruned) {
      if (!best || best.pruned || worst > (best.clearance ?? -Infinity)) {
        best = { clear: false, clearance: worst, pruned: false, yaw };
      }
    }
  }
  return best;
}

// Naive centerline check: only the vehicle center is sampled. Deliberately
// provided to demonstrate why "all sampled points are empty" is not enough —
// it ignores vehicle extent and between-pose motion.
export function naiveCenterlineCheck(world, pa, pb, sampleStep, margin = 0) {
  const d = dist(pa, pb);
  const n = Math.max(1, Math.ceil(d / sampleStep));
  let worst = Infinity;
  for (let i = 0; i <= n; i++) {
    const t = i / n;
    const c = add(pa, scale(sub(pb, pa), t));
    const s = worldSdf(world, c);
    if (s < worst) worst = s;
    if (s <= margin) return { clear: false, clearance: s, at: c, t };
  }
  return { clear: true, clearance: worst };
}
