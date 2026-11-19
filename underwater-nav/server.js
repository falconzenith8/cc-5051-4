// Zero-dependency Node HTTP server: static frontend + JSON navigation API.
// Every path and blocking volume returned by /api/plan is computed live by the
// vehicle navigation engine — no canned trajectories are ever served.
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { buildScenario, DEFAULT_DIMS, scenarioIds } from './src/scenarios.js';
import { runNavigation, gridPayload } from './src/engine.js';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const PUBLIC = path.join(__dirname, 'public');
const PORT = process.env.PORT ? Number(process.env.PORT) : 8080;

const MIME = {
  '.html': 'text/html; charset=utf-8',
  '.js': 'text/javascript; charset=utf-8',
  '.css': 'text/css; charset=utf-8',
  '.json': 'application/json; charset=utf-8',
  '.map': 'application/json; charset=utf-8',
};

function sendJson(res, code, obj) {
  const body = JSON.stringify(obj);
  res.writeHead(code, { 'Content-Type': 'application/json; charset=utf-8' });
  res.end(body);
}

function readBody(req) {
  return new Promise((resolve, reject) => {
    let data = '';
    req.on('data', (c) => {
      data += c;
      if (data.length > 5e6) reject(new Error('payload too large'));
    });
    req.on('end', () => {
      try { resolve(data ? JSON.parse(data) : {}); } catch (e) { reject(e); }
    });
    req.on('error', reject);
  });
}

function validPoint(p) {
  return Array.isArray(p) && p.length === 3 && p.every((v) => Number.isFinite(v));
}

const server = http.createServer(async (req, res) => {
  const url = new URL(req.url, `http://${req.headers.host}`);
  try {
    if (req.method === 'GET' && url.pathname === '/api/scenarios') {
      const list = scenarioIds().map((id) => {
        const w = buildScenario(id);
        return {
          id: w.id, name: w.name, description: w.description,
          bounds: w.bounds, start: w.start, goals: w.goals,
          obstacles: w.obstacles,
          defaultVehicle: DEFAULT_DIMS,
        };
      });
      return sendJson(res, 200, { scenarios: list });
    }

    if (req.method === 'POST' && url.pathname === '/api/grid') {
      const body = await readBody(req);
      if (!scenarioIds().includes(body.scenario)) return sendJson(res, 400, { error: 'unknown scenario' });
      const dims = { ...DEFAULT_DIMS, ...(body.vehicle || {}) };
      const world = buildScenario(body.scenario);
      return sendJson(res, 200, { grid: gridPayload(world, dims, body.margin ?? 0.05) });
    }

    if (req.method === 'POST' && url.pathname === '/api/plan') {
      const body = await readBody(req);
      if (!scenarioIds().includes(body.scenario)) return sendJson(res, 400, { error: 'unknown scenario' });
      if (!validPoint(body.start) || !validPoint(body.goal)) {
        return sendJson(res, 400, { error: 'start and goal must be [x,y,z] numbers' });
      }
      const dims = { ...DEFAULT_DIMS, ...(body.vehicle || {}) };
      for (const k of ['length', 'width', 'height']) {
        if (!(Number(dims[k]) > 0)) return sendJson(res, 400, { error: `bad vehicle dimension ${k}` });
      }
      const world = buildScenario(body.scenario);
      const result = runNavigation(world, dims, body.start, body.goal, { margin: body.margin ?? 0.05 });
      return sendJson(res, 200, result);
    }

    if (req.method === 'GET') {
      let rel = decodeURIComponent(url.pathname);
      if (rel === '/') rel = '/index.html';
      // Map /vendor/three to the locally installed npm package.
      let file;
      if (rel.startsWith('/vendor/three/')) {
        file = path.join(__dirname, 'node_modules/three', rel.slice('/vendor/three/'.length));
      } else {
        file = path.join(PUBLIC, rel);
      }
      const normalized = path.normalize(file);
      if (!normalized.startsWith(PUBLIC) && !normalized.startsWith(path.join(__dirname, 'node_modules'))) {
        res.writeHead(403); return res.end('forbidden');
      }
      const stat = fs.existsSync(normalized) && fs.statSync(normalized);
      if (!stat || !stat.isFile()) { res.writeHead(404); return res.end('not found'); }
      res.writeHead(200, { 'Content-Type': MIME[path.extname(normalized)] || 'application/octet-stream' });
      return fs.createReadStream(normalized).pipe(res);
    }

    res.writeHead(405); res.end('method not allowed');
  } catch (err) {
    sendJson(res, 500, { error: String(err && err.message || err) });
  }
});

server.listen(PORT, () => {
  console.log(`Underwater navigation demo: http://localhost:${PORT}`);
});
