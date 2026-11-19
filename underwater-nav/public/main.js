import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const COLORS = {
  obstacle: 0x2e6e8e, obstacleEm: 0x74c7e8,
  path: 0x35e0ff, block: 0xff4d6d, reached: 0x3aa0ff, grid: 0xc98a2e,
  start: 0x39ff9c, goal: 0xffd23f, vehicle: 0xeaf6ff,
};

let state = {
  scenario: null, scenarios: [], vehicle: null,
  goal: null, picking: false, result: null,
  clipAxis: 2, clipAt: 1, clipEnabled: false,
  playing: false, playT: 0, speed: 1,
};

// ---------- renderer / scene ----------
const canvas = $('scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setPixelRatio(Math.min(devicePixelRatio, 2));
renderer.localClippingEnabled = true;
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x06121f);
scene.fog = new THREE.FogExp2(0x06121f, 0.012);

const camera = new THREE.PerspectiveCamera(55, 1, 0.1, 400);
const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;
controls.dampingFactor = 0.08;

scene.add(new THREE.AmbientLight(0x88aacc, 0.8));
const key = new THREE.DirectionalLight(0xffffff, 1.4);
key.position.set(20, 30, 40);
scene.add(key);
const fill = new THREE.DirectionalLight(0x4488cc, 0.6);
fill.position.set(-25, -15, 20);
scene.add(fill);

const obstacleGroup = new THREE.Group();
const voxelGroup = new THREE.Group();
const pathGroup = new THREE.Group();
scene.add(obstacleGroup, voxelGroup, pathGroup);

const clipPlane = new THREE.Plane(new THREE.Vector3(0, 0, -1), 1e6);
const obstacleMaterial = new THREE.MeshStandardMaterial({
  color: COLORS.obstacle, emissive: COLORS.obstacleEm, emissiveIntensity: 0.12,
  roughness: 0.55, metalness: 0.25, transparent: true, opacity: 0.82,
  clippingPlanes: [clipPlane], side: THREE.DoubleSide,
});
const edgeMaterial = new THREE.LineBasicMaterial({ color: 0x9fe0ff, clippingPlanes: [clipPlane] });

// Invisible raycast target covering the whole water volume.
const waterVolume = new THREE.Mesh(
  new THREE.BoxGeometry(1, 1, 1),
  new THREE.MeshBasicMaterial({ transparent: true, opacity: 0, depthWrite: false }),
);
waterVolume.material.colorWrite = false;
scene.add(waterVolume);
const waterFrame = new THREE.LineSegments(
  new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)),
  new THREE.LineBasicMaterial({ color: 0x2b6ea0, transparent: true, opacity: 0.7 }),
);
scene.add(waterFrame);

// ---------- markers / vehicle ----------
const startMarker = new THREE.Mesh(
  new THREE.SphereGeometry(0.45, 18, 18),
  new THREE.MeshStandardMaterial({ color: COLORS.start, emissive: COLORS.start, emissiveIntensity: 0.5 }),
);
const goalMarker = new THREE.Mesh(
  new THREE.SphereGeometry(0.45, 18, 18),
  new THREE.MeshStandardMaterial({ color: COLORS.goal, emissive: COLORS.goal, emissiveIntensity: 0.55 }),
);
scene.add(startMarker, goalMarker);

const vehicleMesh = new THREE.Mesh(
  new THREE.BoxGeometry(1, 1, 1),
  new THREE.MeshStandardMaterial({ color: COLORS.vehicle, emissive: 0x224466, emissiveIntensity: 0.4, roughness: 0.3 }),
);
const vehicleEdges = new THREE.LineSegments(
  new THREE.EdgesGeometry(new THREE.BoxGeometry(1, 1, 1)),
  new THREE.LineBasicMaterial({ color: 0x113355 }),
);
vehicleMesh.add(vehicleEdges);
vehicleMesh.visible = false;
scene.add(vehicleMesh);

function resize() {
  const w = canvas.clientWidth, h = canvas.clientHeight;
  renderer.setSize(w, h, false);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
window.addEventListener('resize', resize);

// ---------- scenario loading ----------
function boundsCenter(b) {
  return [(b.min[0] + b.max[0]) / 2, (b.min[1] + b.max[1]) / 2, (b.min[2] + b.max[2]) / 2];
}
function boundsSize(b) {
  return [b.max[0] - b.min[0], b.max[1] - b.min[1], b.max[2] - b.min[2]];
}

function clearGroup(g) {
  while (g.children.length) {
    const c = g.children.pop();
    c.geometry?.dispose?.();
  }
}

function renderObstacles(scn) {
  clearGroup(obstacleGroup);
  for (const o of scn.obstacles) {
    let geo;
    if (o.type === 'box') geo = new THREE.BoxGeometry(o.half[0] * 2, o.half[1] * 2, o.half[2] * 2);
    else if (o.type === 'sphere') geo = new THREE.SphereGeometry(o.radius, 26, 20);
    else geo = new THREE.CylinderGeometry(o.radius, o.radius, 34, 28);
    const mesh = new THREE.Mesh(geo, obstacleMaterial);
    mesh.position.set(...o.center);
    if (o.type === 'box' && o.yaw) mesh.rotation.z = o.yaw;
    obstacleGroup.add(mesh);
    if (o.type === 'box') {
      obstacleGroup.add(new THREE.LineSegments(new THREE.EdgesGeometry(geo), edgeMaterial));
      obstacleGroup.children[obstacleGroup.children.length - 1].position.copy(mesh.position);
      obstacleGroup.children[obstacleGroup.children.length - 1].rotation.copy(mesh.rotation);
    }
  }
}

function frameWater(b) {
  const c = boundsCenter(b), s = boundsSize(b);
  waterVolume.position.set(...c);
  waterVolume.scale.set(...s);
  waterFrame.position.set(...c);
  waterFrame.scale.set(...s);
  camera.position.set(c[0] + s[0] * 0.9, c[1] - s[1] * 1.1, c[2] + s[2] * 0.85);
  controls.target.set(...c);
  controls.update();
}

async function loadScenarios() {
  const res = await fetch('/api/scenarios');
  const data = await res.json();
  state.scenarios = data.scenarios;
  const wrap = $('scenarioButtons');
  wrap.innerHTML = '';
  for (const scn of state.scenarios) {
    const b = document.createElement('button');
    b.textContent = scn.name;
    b.dataset.id = scn.id;
    b.onclick = () => selectScenario(scn.id);
    wrap.appendChild(b);
  }
  selectScenario(state.scenarios[0].id);
}

function selectScenario(id) {
  state.scenario = state.scenarios.find((s) => s.id === id);
  state.vehicle = { ...state.scenario.defaultVehicle };
  state.result = null;
  $('vLen').value = state.vehicle.length;
  $('vWid').value = state.vehicle.width;
  $('vHgt').value = state.vehicle.height;
  $('scenarioDesc').textContent = state.scenario.description;
  document.querySelectorAll('#scenarioButtons button').forEach((b) =>
    b.classList.toggle('active', b.dataset.id === id));
  renderObstacles(state.scenario);
  frameWater(state.scenario.bounds);
  startMarker.position.set(...state.scenario.start);
  setGoal(state.scenario.goals[0]);
  clearGroup(voxelGroup);
  clearGroup(pathGroup);
  vehicleMesh.visible = false;
  setStatus('idle', '已加载场景，等待运行');
  $('metrics').innerHTML = '';
  $('failureBox').classList.add('hidden');
  $('play').disabled = true;
  buildGoalList();
  updateClipRange();
}

function buildGoalList() {
  const wrap = $('goalList');
  wrap.innerHTML = '';
  state.scenario.goals.forEach((g, i) => {
    const b = document.createElement('button');
    b.textContent = `预设目标 ${i + 1}：[${g.map((v) => v.toFixed(1)).join(', ')}]`;
    b.onclick = () => setGoal(g);
    wrap.appendChild(b);
  });
}

function setGoal(p) {
  state.goal = p.slice();
  goalMarker.position.set(...p);
  $('goalText').textContent = `[${p.map((v) => v.toFixed(2)).join(', ')}]`;
}

// ---------- voxel instancing ----------
function voxelMesh(payload, color, opacity) {
  const n = payload.indices.length;
  const geo = new THREE.BoxGeometry(payload.cell * 0.92, payload.cell * 0.92, payload.cell * 0.92);
  const mat = new THREE.MeshStandardMaterial({
    color, transparent: true, opacity, roughness: 0.7,
    clippingPlanes: [clipPlane],
  });
  const mesh = new THREE.InstancedMesh(geo, mat, n);
  const m = new THREE.Matrix4();
  payload.indices.forEach(([i, j, k], idx) => {
    const p = [
      payload.origin[0] + (i + 0.5) * payload.cell,
      payload.origin[1] + (j + 0.5) * payload.cell,
      payload.origin[2] + (k + 0.5) * payload.cell,
    ];
    m.makeTranslation(p[0], p[1], p[2]);
    mesh.setMatrixAt(idx, m);
  });
  mesh.instanceMatrix.needsUpdate = true;
  return mesh;
}

function showVoxels(payload, color, opacity, tag) {
  if (!payload || !payload.indices.length) return;
  const mesh = voxelMesh(payload, color, opacity);
  mesh.userData.tag = tag;
  voxelGroup.add(mesh);
}

function removeVoxelTag(tag) {
  for (let i = voxelGroup.children.length - 1; i >= 0; i--) {
    if (voxelGroup.children[i].userData.tag === tag) {
      voxelGroup.children[i].geometry.dispose();
      voxelGroup.children[i].material.dispose();
      voxelGroup.splice(i, 1);
    }
  }
}

async function refreshGridVoxels() {
  removeVoxelTag('grid');
  if (!$('showGrid').checked) return;
  const res = await fetch('/api/grid', {
    method: 'POST', headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ scenario: state.scenario.id, vehicle: readVehicle() }),
  });
  const data = await res.json();
  showVoxels(data.grid, COLORS.grid, 0.35, 'grid');
}

// ---------- planning ----------
function readVehicle() {
  return {
    length: Math.max(0.1, parseFloat($('vLen').value) || 1),
    width: Math.max(0.1, parseFloat($('vWid').value) || 1),
    height: Math.max(0.1, parseFloat($('vHgt').value) || 1),
  };
}

function setStatus(kind, text) {
  const el = $('status');
  el.className = `status ${kind}`;
  el.textContent = text;
}

function reasonText(r) {
  if (r.reason === 'unreachable') return '目标不可达：搜索分量无法到达目标体素（参考网格洪水填充一致判定）';
  if (r.reason === 'start_in_collision') return `起点无法容纳载具（净空 ${r.startClearance} m）`;
  if (r.reason === 'goal_in_collision') return `目标点无法容纳载具（净空 ${r.goalClearance} m）`;
  return r.reason;
}

async function runPlan() {
  if (state.playing) stopPlayback();
  setStatus('busy', '载具正在三维配置空间中搜索…');
  removeVoxelTag('block');
  removeVoxelTag('reached');
  clearGroup(pathGroup);
  vehicleMesh.visible = false;
  try {
    const res = await fetch('/api/plan', {
      method: 'POST', headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        scenario: state.scenario.id,
        vehicle: readVehicle(),
        start: state.scenario.start,
        goal: state.goal,
      }),
    });
    const r = await res.json();
    state.result = r;
    window.__lastResult = r;
    if (r.error) { setStatus('fail', `服务错误：${r.error}`); return; }

    if (r.ok) {
      setStatus('ok', '路径已找到并通过连续扫掠核验');
      drawPath(r);
      placeVehicle(r);
      $('play').disabled = false;
      fillMetricsOk(r);
      $('failureBox').classList.add('hidden');
    } else {
      setStatus('fail', '导航失败：' + reasonText(r));
      $('play').disabled = true;
      fillMetricsFail(r);
      showBlocking(r);
    }
  } catch (e) {
    setStatus('fail', '请求失败：' + e.message);
  }
}

function drawPath(r) {
  const pts = r.samples.map((p) => new THREE.Vector3(...p));
  const lineGeo = new THREE.BufferGeometry().setFromPoints(pts);
  const line = new THREE.Line(lineGeo, new THREE.LineBasicMaterial({ color: COLORS.path, linewidth: 2 }));
  pathGroup.add(line);
  // Smoothed waypoints as small markers.
  const wpGeo = new THREE.SphereGeometry(0.16, 10, 10);
  const wpMat = new THREE.MeshBasicMaterial({ color: 0x9ff2ff });
  for (const w of r.waypoints) {
    const s = new THREE.Mesh(wpGeo, wpMat);
    s.position.set(...w);
    pathGroup.add(s);
  }
}

function placeVehicle(r) {
  const v = readVehicle();
  vehicleMesh.scale.set(v.length, v.width, v.height);
  vehicleMesh.visible = true;
  state.playT = 0;
  updateVehiclePose(0);
}

function updateVehiclePose(t) {
  const r = state.result;
  if (!r?.ok) return;
  const n = r.samples.length - 1;
  const f = Math.min(0.99999, Math.max(0, t)) * n;
  const i = Math.floor(f), k = f - i;
  const a = r.samples[i], b = r.samples[Math.min(n, i + 1)];
  const p = a.map((v, d) => v + (b[d] - v[d]) * k);
  vehicleMesh.position.set(...p);
  vehicleMesh.rotation.z = Math.atan2(b[1] - a[1], b[0] - a[0]);
  // Live closest-obstacle distance from the dense verification trace.
  const traceIdx = Math.min(r.verification.trace.length - 1, Math.round(t * (r.verification.trace.length - 1)));
  const cl = r.verification.trace[traceIdx]?.clearance;
  $('hudClearance').innerHTML = cl !== undefined ? `最近障碍距离 <b>${cl.toFixed(2)} m</b>` : '';
}

function showBlocking(r) {
  showVoxels(r.blocking, COLORS.block, 0.85, 'block');
  if ($('showReached').checked) showVoxels(r.reached, COLORS.reached, 0.28, 'reached');
  const fb = $('failureBox');
  fb.classList.remove('hidden');
  fb.innerHTML = `<h3>失败原因与阻断体积</h3>
    <div>${reasonText(r)}</div>
    <div>阻断壳面体素：<b>${r.blocking?.indices.length ?? 0}</b> 个（红色，仅取与可达分量相邻的壳面）</div>
    <div>搜索扩展：${r.expansions ?? '-'} · 扫掠否决边：${r.sweptRejected ?? '-'}</div>
    <div>高分辨率参考（${r.reference.cell} m 格）：可达体素 ${r.reference.reachedVoxels} / ${r.reference.totalVoxels}，
    目标${r.reference.enclosedGoal ? '处于封闭空腔内' : r.reference.goalBlocked ? '自身被实体占据' : '不可达'}</div>`;
}

function fillMetricsOk(r) {
  const v = r.verification;
  $('metrics').innerHTML = `
    <div>路径长度：<b>${r.pathLength} m</b>（密集采样 ${r.samples.length} 点）</div>
    <div>连续扫掠核验：<b style="color:#7ce2ad">${v.continuousSafe ? '通过' : '失败'}</b></div>
    <div>实测最小净空：<b>${v.measuredClearance} m</b> · 保守界：${v.conservativeClearance} m</div>
    <div>朴素中心线判定：${r.naiveCenterline.clear ? '空（忽视体积）' : `碰撞 ${r.naiveCenterline.clearance} m`}</div>
    <div>A* 扩展：${r.expansions} · 扫掠否决：${r.sweptRejected}</div>
    <div>搜索格 ${r.grid.cell} m（${r.grid.blocked}/${r.grid.total} 阻挡）· 参考格 ${r.reference.cell} m</div>
    <div>载具内切球半径：${r.vehicle.erosionRadius} m · 耗时：${r.timings.totalMs} ms</div>`;
}

function fillMetricsFail(r) {
  $('metrics').innerHTML = `
    <div>搜索格 ${r.grid.cell} m（阻挡 ${r.grid.blocked}/${r.grid.total}）</div>
    <div>参考格 ${r.reference.cell} m · 参考可达：<b style="color:#ff9dad">${r.reference.reachable ? '是' : '否'}</b></div>
    <div>A* 扩展：${r.expansions ?? '-'} · 扫掠否决：${r.sweptRejected ?? '-'} · 耗时：${r.timings?.totalMs ?? '-'} ms</div>`;
}

// ---------- playback ----------
function startPlayback() {
  if (!state.result?.ok) return;
  state.playing = true;
  state.playT = 0;
  $('play').textContent = '⏸ 暂停';
}
function stopPlayback() {
  state.playing = false;
  $('play').textContent = '▶ 沿路径行驶';
}

const clock = new THREE.Clock();
function animate() {
  requestAnimationFrame(animate);
  const dt = clock.getDelta();
  if (state.playing && state.result?.ok) {
    // Traverse in ~14 seconds at 1x regardless of path length.
    const dur = 14 / state.speed;
    state.playT += dt / dur;
    if (state.playT >= 1) { state.playT = 1; stopPlayback(); }
    updateVehiclePose(state.playT);
  }
  controls.update();
  renderer.render(scene, camera);
}

// ---------- picking ----------
const raycaster = new THREE.Raycaster();
const pointer = new THREE.Vector2();
canvas.addEventListener('pointerdown', (e) => {
  if (!state.picking) return;
  const rect = canvas.getBoundingClientRect();
  pointer.x = ((e.clientX - rect.left) / rect.width) * 2 - 1;
  pointer.y = -((e.clientY - rect.top) / rect.height) * 2 + 1;
  raycaster.setFromCamera(pointer, camera);
  const hit = raycaster.intersectObject(waterVolume, false)[0];
  if (hit) {
    const p = hit.point;
    const b = state.scenario.bounds;
    const clamped = [
      THREE.MathUtils.clamp(p.x, b.min[0] + 0.2, b.max[0] - 0.2),
      THREE.MathUtils.clamp(p.y, b.min[1] + 0.2, b.max[1] - 0.2),
      THREE.MathUtils.clamp(p.z, b.min[2] + 0.2, b.max[2] - 0.2),
    ];
    setGoal(clamped);
    state.picking = false;
    $('pickMode').classList.remove('active');
    $('pickMode').textContent = '点选目标';
  }
});

// ---------- clipping ----------
function updateClipRange() {
  const b = state.scenario.bounds;
  const size = boundsSize(b)[state.clipAxis];
  $('clipSlider').value = 1000;
  $('clipValue').textContent = '关闭';
  state.clipEnabled = false;
  applyClip(size);
}
function applyClip(size) {
  if (!state.scenario) return;
  const b = state.scenario.bounds;
  const axis = state.clipAxis;
  const normal = new THREE.Vector3();
  if (!state.clipEnabled) {
    clipPlane.constant = 1e6;
    return;
  }
  const frac = Number($('clipSlider').value) / 1000;
  const pos = b.min[axis] + frac * size;
  normal.setComponent(axis, -1);
  clipPlane.normal.copy(normal);
  // Keep everything with dot(normal, x) <= pos: plane constant = pos.
  clipPlane.constant = pos;
  $('clipValue').textContent = `${pos.toFixed(1)} m`;
}

// ---------- UI wiring ----------
$('run').onclick = runPlan;
$('play').onclick = () => (state.playing ? stopPlayback() : startPlayback());
$('stop').onclick = () => { stopPlayback(); state.playT = 0; updateVehiclePose(0); };
$('speed').oninput = (e) => { state.speed = parseFloat(e.target.value); $('speedValue').textContent = state.speed.toFixed(1) + '×'; };
$('pickMode').onclick = () => {
  state.picking = !state.picking;
  $('pickMode').classList.toggle('active', state.picking);
  $('pickMode').textContent = state.picking ? '点击水体选择…' : '点选目标';
};
$('resetGoal').onclick = () => setGoal(state.scenario.goals[0]);
$('showGrid').onchange = refreshGridVoxels;
$('showReached').onchange = () => {
  if (!$('showReached').checked) removeVoxelTag('reached');
  else if (state.result && !state.result.ok) showBlocking(state.result);
};
$('clipAxis').onchange = (e) => { state.clipAxis = parseInt(e.target.value, 10); updateClipRange(); };
$('clipSlider').oninput = (e) => {
  state.clipEnabled = Number(e.target.value) < 1000;
  applyClip(boundsSize(state.scenario.bounds)[state.clipAxis]);
};
document.querySelectorAll('[data-preset]').forEach((btn) => {
  btn.onclick = () => {
    const base = state.scenario.defaultVehicle;
    const v = btn.dataset.preset === 'fat'
      ? { length: 2.4, width: 3.2, height: 2.6 }
      : { ...base };
    $('vLen').value = v.length; $('vWid').value = v.width; $('vHgt').value = v.height;
    refreshGridVoxels();
  };
});
for (const id of ['vLen', 'vWid', 'vHgt']) $(id).onchange = refreshGridVoxels;
$('startText').textContent = '';
const origSetGoal = setGoal;
startMarker.name = 'start';

resize();
loadScenarios().then(() => {
  $('startText').textContent = `[${state.scenario.start.map((v) => v.toFixed(2)).join(', ')}]`;
  animate();
});

// Debug surface for automated UI checks: counts of actual rendered objects.
window.__navDebug = () => ({
  obstacleMeshes: obstacleGroup.children.filter((c) => c.isMesh).length,
  voxelInstancedMeshes: voxelGroup.children.map((c) => ({
    tag: c.userData.tag, count: c.count,
  })),
  pathObjects: pathGroup.children.length,
  vehicleVisible: vehicleMesh.visible,
  vehicleScale: [vehicleMesh.scale.x, vehicleMesh.scale.y, vehicleMesh.scale.z],
  clipConstant: clipPlane.constant,
  hasResult: !!state.result,
  sampleCount: state.result?.samples?.length ?? 0,
  playT: state.playT,
  playing: state.playing,
});
