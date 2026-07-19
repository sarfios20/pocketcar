'use strict';

// ---------------------------------------------------------------- constants

const SCALE = 30; // px per meter

const CAR = {
  wheelbase: 2.7 * SCALE,
  width: 1.8 * SCALE,
  track: 1.5 * SCALE,
  frontOverhang: 0.9 * SCALE,
  rearOverhang: 0.8 * SCALE,
  wheelLen: 0.65 * SCALE,
  wheelWid: 0.22 * SCALE,
  maxSteer: 35 * Math.PI / 180,
};
CAR.length = CAR.wheelbase + CAR.frontOverhang + CAR.rearOverhang;
// body center relative to the rear axle (the car's origin)
CAR.bodyCx = (CAR.wheelbase + CAR.frontOverhang - CAR.rearOverhang) / 2;

// drawn steering-wheel turns per radian of road wheel (full lock = 135° of wheel)
const WHEEL_VISUAL_K = (135 * Math.PI / 180) / CAR.maxSteer;

const COLORS = {
  asphalt: '#24282c',
  road: '#2c3136',
  sidewalk: '#3a4046',
  chalk: 'rgba(230, 225, 211, 0.45)',
  chalkSoft: 'rgba(230, 225, 211, 0.18)',
  amber: '#f2ae2e',
  amberDark: '#c98d1d',
  teal: '#45d6b8',
  coral: '#ff8a5c',
  danger: '#ff5c5c',
  parked: '#55606b',
  parkedDark: '#454f58',
};

// ---------------------------------------------------------------- state

const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
let W = 0, H = 0;

// car pose: (x, y) = center of the rear axle, h = heading, steer = road-wheel angle
const car = { x: 0, y: 0, h: 0, steer: 0 };

let obstacles = [];      // {x, y, h, len, wid, kind: 'curb' | 'car'}
let decor = [];          // {type: 'road' | 'dashH' | 'dashV' | 'lineH', ...}
let ghosts = [];         // stamped body poses
let traces = { rl: [], rr: [], fl: [], fr: [] };
let hits = 0;
let inContact = false;
let flash = 0;
let hitObstacle = null;
let showGeometry = true;

let sinceTrace = 0;
let sinceGhost = 0;

const keys = new Set();
let drag = null;         // {mode: 'car', grabLocal} | {mode: 'wheel', lastAngle}
const mouse = { x: 0, y: 0 };

const wheelWidget = { x: 0, y: 0, r: 44 };

// ---------------------------------------------------------------- geometry

function obbCorners(cx, cy, h, len, wid) {
  const c = Math.cos(h), s = Math.sin(h);
  const hl = len / 2, hw = wid / 2;
  return [
    [cx + c * hl - s * hw, cy + s * hl + c * hw],
    [cx + c * hl + s * hw, cy + s * hl - c * hw],
    [cx - c * hl + s * hw, cy - s * hl - c * hw],
    [cx - c * hl - s * hw, cy - s * hl + c * hw],
  ];
}

function bodyOBB(pose) {
  const c = Math.cos(pose.h), s = Math.sin(pose.h);
  return {
    x: pose.x + c * CAR.bodyCx,
    y: pose.y + s * CAR.bodyCx,
    h: pose.h,
    len: CAR.length,
    wid: CAR.width,
  };
}

function obbsSeparated(a, b) {
  const ca = obbCorners(a.x, a.y, a.h, a.len, a.wid);
  const cb = obbCorners(b.x, b.y, b.h, b.len, b.wid);
  const axes = [
    [Math.cos(a.h), Math.sin(a.h)], [-Math.sin(a.h), Math.cos(a.h)],
    [Math.cos(b.h), Math.sin(b.h)], [-Math.sin(b.h), Math.cos(b.h)],
  ];
  for (const [ax, ay] of axes) {
    let minA = Infinity, maxA = -Infinity, minB = Infinity, maxB = -Infinity;
    for (const [px, py] of ca) {
      const p = px * ax + py * ay;
      if (p < minA) minA = p;
      if (p > maxA) maxA = p;
    }
    for (const [px, py] of cb) {
      const p = px * ax + py * ay;
      if (p < minB) minB = p;
      if (p > maxB) maxB = p;
    }
    if (maxA < minB || maxB < minA) return true; // separating axis: no overlap
  }
  return false;
}

function collides(pose) {
  const body = bodyOBB(pose);
  for (const o of obstacles) {
    if (!obbsSeparated(body, o)) return o;
  }
  return null;
}

function carLocalToWorld(lx, ly) {
  const c = Math.cos(car.h), s = Math.sin(car.h);
  return [car.x + c * lx - s * ly, car.y + s * lx + c * ly];
}

function pointInCar(px, py) {
  const c = Math.cos(car.h), s = Math.sin(car.h);
  const dx = px - car.x, dy = py - car.y;
  const lx = c * dx + s * dy, ly = -s * dx + c * dy;
  return lx > -CAR.rearOverhang - 6 && lx < CAR.wheelbase + CAR.frontOverhang + 6 &&
         Math.abs(ly) < CAR.width / 2 + 6;
}

function wheelPoints() {
  const t = CAR.track / 2;
  return {
    rl: carLocalToWorld(0, -t),
    rr: carLocalToWorld(0, t),
    fl: carLocalToWorld(CAR.wheelbase, -t),
    fr: carLocalToWorld(CAR.wheelbase, t),
  };
}

// ---------------------------------------------------------------- motion

// kinematic bicycle model: the rear axle travels ds along its arc
function move(ds) {
  if (ds === 0) return;
  const steps = Math.max(1, Math.ceil(Math.abs(ds) / 2));
  const step = ds / steps;
  let collided = null;

  for (let i = 0; i < steps; i++) {
    const cand = {
      x: car.x + Math.cos(car.h) * step,
      y: car.y + Math.sin(car.h) * step,
      h: car.h + step * Math.tan(car.steer) / CAR.wheelbase,
      steer: car.steer,
    };
    const obs = collides(cand);
    if (obs) { collided = obs; break; }
    car.x = cand.x;
    car.y = cand.y;
    car.h = cand.h;
    sinceTrace += Math.abs(step);
    sinceGhost += Math.abs(step);

    if (sinceTrace >= 3) {
      sinceTrace = 0;
      const w = wheelPoints();
      for (const k of ['rl', 'rr', 'fl', 'fr']) {
        traces[k].push(w[k]);
        if (traces[k].length > 4000) traces[k].shift();
      }
    }
    if (sinceGhost >= 34) {
      sinceGhost = 0;
      ghosts.push({ x: car.x, y: car.y, h: car.h });
      if (ghosts.length > 300) ghosts.shift();
    }
  }

  if (collided) {
    if (!inContact) {
      hits++;
      flash = 1;
      hitObstacle = collided;
    }
    inContact = true;
  } else {
    inContact = false;
  }
}

function setSteer(v) {
  car.steer = Math.max(-CAR.maxSteer, Math.min(CAR.maxSteer, v));
}

// ---------------------------------------------------------------- scenarios

function block(x0, y0, x1, y1) {
  return { x: (x0 + x1) / 2, y: (y0 + y1) / 2, h: 0, len: x1 - x0, wid: y1 - y0, kind: 'curb' };
}

function parkedCar(x, y, h) {
  return { x, y, h, len: CAR.length, wid: CAR.width, kind: 'car' };
}

const SCENARIOS = [
  {
    name: 'Open',
    hint: 'Empty lot. Turn the wheel to full lock and push slowly: notice how the <b>green</b> track (rear wheels) always runs inside the <b>orange</b> one (front wheels).',
    build() {
      decor = [];
      obstacles = [];
      car.x = W * 0.45; car.y = H * 0.55; car.h = 0; car.steer = 0;
    },
  },
  {
    name: 'Corner with parked car',
    hint: 'Turn left to enter the street. A car is parked right past the corner: if you turn too early, <b>the tail cuts in</b> and clips it. Drive past the corner first, then turn.',
    build() {
      const sw = 6.2 * SCALE;                 // street width
      const xv = W * 0.62;                    // axis of the vertical street
      const yTop = H / 2 - sw / 2, yBot = H / 2 + sw / 2;
      decor = [
        { type: 'road', x: 0, y: yTop, w: W, h: sw },
        { type: 'road', x: xv - sw / 2, y: 0, w: sw, h: yTop },
        { type: 'dashH', x0: 0, x1: xv - sw / 2, y: H / 2 },
        { type: 'dashV', x: xv, y0: 0, y1: yTop },
      ];
      obstacles = [
        block(-40, -40, xv - sw / 2, yTop),   // top-left block
        block(xv + sw / 2, -40, W + 40, yTop),// top-right block
        block(-40, yBot, W + 40, H + 40),     // bottom block
        parkedCar(xv - sw / 2 + CAR.width / 2 + 5, yTop - CAR.length / 2 - 22, -Math.PI / 2),
      ];
      car.x = W * 0.26; car.y = H / 2 + sw / 4; car.h = 0; car.steer = 0;
    },
  },
  {
    name: 'Parallel park',
    hint: 'Back the car into the gap: line up parallel with the car ahead, turn to full lock toward the curb and reverse. Head-in, you will see it does not fit at all.',
    build() {
      const curbY = H * 0.62;
      const gap = CAR.length * 1.5;
      const cy = curbY - CAR.width / 2 - 5;
      const carA = W / 2 - gap / 2 - CAR.length / 2;
      const carB = W / 2 + gap / 2 + CAR.length / 2;
      decor = [
        { type: 'road', x: 0, y: 0, w: W, h: curbY },
        { type: 'lineH', x0: 0, x1: W, y: curbY + 1 },
        { type: 'dashH', x0: 0, x1: W, y: curbY - 5.4 * SCALE },
      ];
      obstacles = [
        block(-40, curbY, W + 40, H + 40),
        parkedCar(carA, cy, 0),
        parkedCar(carB, cy, 0),
      ];
      car.x = carB - CAR.length / 2 - 8; car.y = cy - CAR.width - 22; car.h = 0; car.steer = 0;
    },
  },
  {
    name: 'Tight alley',
    hint: 'A 3.5 m street with a right-angle turn. It will not clear in one pass: you have to <b>maneuver</b> (pull forward, counter-steer, reverse). Watch how much the nose sweeps out.',
    build() {
      const cw = 3.5 * SCALE;
      const xc = W * 0.6;
      const yTop = H / 2 - cw / 2, yBot = H / 2 + cw / 2;
      decor = [
        { type: 'road', x: 0, y: yTop, w: xc + cw / 2, h: cw },
        { type: 'road', x: xc - cw / 2, y: 0, w: cw, h: yTop },
      ];
      obstacles = [
        block(-40, -40, xc - cw / 2, yTop),
        block(xc + cw / 2, -40, W + 40, yBot),
        block(-40, yBot, W + 40, H + 40),
      ];
      car.x = Math.max(360, W * 0.24); car.y = H / 2; car.h = 0; car.steer = 0;
    },
  },
];

let currentScenario = 0;

function loadScenario(i) {
  currentScenario = i;
  SCENARIOS[i].build();
  clearMarks();
  hits = 0;
  inContact = false;
  flash = 0;
  hitObstacle = null;
  drag = null;
  document.getElementById('hint').innerHTML = SCENARIOS[i].hint;
  document.querySelectorAll('#scenarios button').forEach((b, j) =>
    b.classList.toggle('active', j === i));
  updateHud();
}

function clearMarks() {
  traces = { rl: [], rr: [], fl: [], fr: [] };
  ghosts = [];
  sinceTrace = 0;
  sinceGhost = 0;
}

// ---------------------------------------------------------------- drawing

function drawDecor() {
  for (const d of decor) {
    if (d.type === 'road') {
      ctx.fillStyle = COLORS.road;
      ctx.fillRect(d.x, d.y, d.w, d.h);
    }
  }
  ctx.strokeStyle = COLORS.chalk;
  ctx.lineWidth = 2;
  for (const d of decor) {
    if (d.type === 'dashH' || d.type === 'dashV') ctx.setLineDash([16, 22]);
    else ctx.setLineDash([]);
    if (d.type === 'dashH' || d.type === 'lineH') {
      ctx.beginPath(); ctx.moveTo(d.x0, d.y); ctx.lineTo(d.x1, d.y); ctx.stroke();
    } else if (d.type === 'dashV') {
      ctx.beginPath(); ctx.moveTo(d.x, d.y0); ctx.lineTo(d.x, d.y1); ctx.stroke();
    }
  }
  ctx.setLineDash([]);
}

function drawObstacle(o) {
  const isHit = flash > 0 && o === hitObstacle;
  if (o.kind === 'curb') {
    ctx.save();
    ctx.fillStyle = COLORS.sidewalk;
    ctx.fillRect(o.x - o.len / 2, o.y - o.wid / 2, o.len, o.wid);
    ctx.strokeStyle = isHit ? COLORS.danger : COLORS.chalkSoft;
    ctx.lineWidth = isHit ? 3 : 2;
    ctx.strokeRect(o.x - o.len / 2, o.y - o.wid / 2, o.len, o.wid);
    ctx.restore();
  } else {
    drawCarShape(o.x - Math.cos(o.h) * CAR.bodyCx, o.y - Math.sin(o.h) * CAR.bodyCx,
      o.h, 0, COLORS.parked, COLORS.parkedDark, isHit);
  }
}

// draw a car whose origin is the rear axle
function drawCarShape(rx, ry, h, steer, bodyColor, roofColor, hitOutline) {
  ctx.save();
  ctx.translate(rx, ry);
  ctx.rotate(h);

  // wheels sticking out a bit, toy style
  const t = CAR.width / 2 - 1;
  ctx.fillStyle = '#111417';
  for (const [wx, wy, ws] of [
    [0, -t, 0], [0, t, 0], [CAR.wheelbase, -t, steer], [CAR.wheelbase, t, steer],
  ]) {
    ctx.save();
    ctx.translate(wx, wy);
    ctx.rotate(ws);
    ctx.beginPath();
    ctx.roundRect(-CAR.wheelLen / 2, -CAR.wheelWid * 1.4, CAR.wheelLen, CAR.wheelWid * 2.8, 3);
    ctx.fill();
    ctx.restore();
  }

  ctx.beginPath();
  ctx.roundRect(-CAR.rearOverhang, -CAR.width / 2, CAR.length, CAR.width, 8);
  ctx.fillStyle = bodyColor;
  ctx.fill();
  if (hitOutline) {
    ctx.strokeStyle = COLORS.danger;
    ctx.lineWidth = 3;
    ctx.stroke();
  }

  // windshield and roof, so the facing direction is clear
  ctx.fillStyle = 'rgba(15, 18, 20, 0.55)';
  ctx.beginPath();
  ctx.roundRect(CAR.wheelbase * 0.62, -CAR.width / 2 + 5, CAR.wheelbase * 0.3, CAR.width - 10, 4);
  ctx.fill();
  ctx.fillStyle = roofColor;
  ctx.beginPath();
  ctx.roundRect(CAR.wheelbase * 0.02, -CAR.width / 2 + 4, CAR.wheelbase * 0.58, CAR.width - 8, 6);
  ctx.fill();

  ctx.fillStyle = 'rgba(255, 250, 220, 0.85)';
  const fx = CAR.wheelbase + CAR.frontOverhang - 4;
  ctx.beginPath(); ctx.arc(fx, -CAR.width / 2 + 7, 2.6, 0, 7); ctx.fill();
  ctx.beginPath(); ctx.arc(fx, CAR.width / 2 - 7, 2.6, 0, 7); ctx.fill();

  ctx.restore();
}

function drawTrace(points, color, width) {
  if (points.length < 2) return;
  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.lineJoin = 'round';
  ctx.beginPath();
  ctx.moveTo(points[0][0], points[0][1]);
  for (let i = 1; i < points.length; i++) ctx.lineTo(points[i][0], points[i][1]);
  ctx.stroke();
}

function drawGhosts() {
  ctx.strokeStyle = 'rgba(242, 174, 46, 0.09)';
  ctx.lineWidth = 1.5;
  for (const g of ghosts) {
    ctx.save();
    ctx.translate(g.x, g.y);
    ctx.rotate(g.h);
    ctx.beginPath();
    ctx.roundRect(-CAR.rearOverhang, -CAR.width / 2, CAR.length, CAR.width, 8);
    ctx.stroke();
    ctx.restore();
  }
}

function drawGeometry() {
  if (!showGeometry || Math.abs(car.steer) < 0.01) return;
  const R = CAR.wheelbase / Math.tan(car.steer); // signed
  const icx = car.x - Math.sin(car.h) * R;
  const icy = car.y + Math.cos(car.h) * R;

  const body = bodyOBB(car);
  const corners = obbCorners(body.x, body.y, body.h, body.len, body.wid);
  let rMax = 0;
  for (const [px, py] of corners) rMax = Math.max(rMax, Math.hypot(px - icx, py - icy));
  const w = wheelPoints();
  const rMin = Math.min(Math.hypot(w.rl[0] - icx, w.rl[1] - icy),
                        Math.hypot(w.rr[0] - icx, w.rr[1] - icy));

  ctx.save();
  ctx.setLineDash([5, 7]);
  ctx.lineWidth = 1.5;
  ctx.strokeStyle = 'rgba(69, 214, 184, 0.4)';
  ctx.beginPath(); ctx.arc(icx, icy, rMin, 0, 7); ctx.stroke();
  ctx.strokeStyle = 'rgba(255, 138, 92, 0.4)';
  ctx.beginPath(); ctx.arc(icx, icy, rMax, 0, 7); ctx.stroke();

  ctx.setLineDash([3, 5]);
  ctx.strokeStyle = COLORS.chalkSoft;
  ctx.beginPath(); ctx.moveTo(icx, icy); ctx.lineTo(car.x, car.y); ctx.stroke();
  ctx.setLineDash([]);

  ctx.strokeStyle = COLORS.chalk;
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  ctx.moveTo(icx - 7, icy); ctx.lineTo(icx + 7, icy);
  ctx.moveTo(icx, icy - 7); ctx.lineTo(icx, icy + 7);
  ctx.stroke();
  ctx.font = '11px ui-monospace, monospace';
  ctx.fillStyle = COLORS.chalk;
  ctx.textAlign = 'right';
  ctx.fillText('turning center', icx - 12, icy - 8);
  ctx.textAlign = 'left';
  ctx.restore();
}

function drawWheelWidget() {
  const { x, y, r } = wheelWidget;
  ctx.save();
  ctx.translate(x, y);

  ctx.fillStyle = 'rgba(22, 25, 28, 0.8)';
  ctx.beginPath(); ctx.arc(0, 0, r + 14, 0, 7); ctx.fill();

  ctx.rotate(car.steer * WHEEL_VISUAL_K);
  ctx.strokeStyle = COLORS.chalk;
  ctx.lineWidth = 6;
  ctx.beginPath(); ctx.arc(0, 0, r, 0, 7); ctx.stroke();
  ctx.lineWidth = 4;
  ctx.beginPath();
  ctx.moveTo(-r, 0); ctx.lineTo(r, 0);
  ctx.moveTo(0, 0); ctx.lineTo(0, r);
  ctx.stroke();
  ctx.fillStyle = COLORS.amber;
  ctx.beginPath(); ctx.arc(0, -r, 5, 0, 7); ctx.fill(); // 12 o'clock mark
  ctx.fillStyle = COLORS.chalk;
  ctx.beginPath(); ctx.arc(0, 0, 7, 0, 7); ctx.fill();
  ctx.restore();
}

function render() {
  ctx.fillStyle = COLORS.asphalt;
  ctx.fillRect(0, 0, W, H);

  drawDecor();
  drawGhosts();
  drawTrace(traces.fl, 'rgba(255, 138, 92, 0.55)', 1.6);
  drawTrace(traces.fr, 'rgba(255, 138, 92, 0.55)', 1.6);
  drawTrace(traces.rl, COLORS.teal, 2.2);
  drawTrace(traces.rr, COLORS.teal, 2.2);
  for (const o of obstacles) drawObstacle(o);
  drawGeometry();
  drawCarShape(car.x, car.y, car.h, car.steer, COLORS.amber, COLORS.amberDark, flash > 0);
  drawWheelWidget();
}

// ---------------------------------------------------------------- HUD

function updateHud() {
  const deg = Math.round(car.steer * 180 / Math.PI);
  const label = deg === 0 ? 'center' : `${Math.abs(deg)}° ${deg < 0 ? 'left' : 'right'}`;
  document.getElementById('steerText').textContent = label;
  document.getElementById('hitsText').textContent = hits;
}

// ---------------------------------------------------------------- input

canvas.addEventListener('pointerdown', (e) => {
  mouse.x = e.clientX; mouse.y = e.clientY;
  const dw = Math.hypot(e.clientX - wheelWidget.x, e.clientY - wheelWidget.y);
  if (dw < wheelWidget.r + 14) {
    drag = { mode: 'wheel', lastAngle: Math.atan2(e.clientY - wheelWidget.y, e.clientX - wheelWidget.x) };
  } else if (pointInCar(e.clientX, e.clientY)) {
    const c = Math.cos(car.h), s = Math.sin(car.h);
    const dx = e.clientX - car.x, dy = e.clientY - car.y;
    drag = { mode: 'car', grabLocal: [c * dx + s * dy, -s * dx + c * dy] };
  } else {
    return;
  }
  canvas.setPointerCapture(e.pointerId);
});

canvas.addEventListener('pointermove', (e) => {
  mouse.x = e.clientX; mouse.y = e.clientY;
  if (drag && drag.mode === 'wheel') {
    const a = Math.atan2(e.clientY - wheelWidget.y, e.clientX - wheelWidget.x);
    let delta = a - drag.lastAngle;
    if (delta > Math.PI) delta -= 2 * Math.PI;
    if (delta < -Math.PI) delta += 2 * Math.PI;
    drag.lastAngle = a;
    setSteer(car.steer + delta / WHEEL_VISUAL_K);
  }
  canvas.style.cursor = (drag && drag.mode === 'car') ? 'grabbing'
    : pointInCar(e.clientX, e.clientY) ? 'grab' : 'default';
});

canvas.addEventListener('pointerup', () => { drag = null; });
canvas.addEventListener('pointercancel', () => { drag = null; });

canvas.addEventListener('wheel', (e) => {
  e.preventDefault();
  setSteer(car.steer + Math.sign(e.deltaY) * 2 * Math.PI / 180);
}, { passive: false });

window.addEventListener('keydown', (e) => {
  if (e.repeat) return;
  const k = e.key.toLowerCase();
  if (k === 'r') loadScenario(currentScenario);
  else if (k === 'c') clearMarks();
  else if (k === 'g') showGeometry = !showGeometry;
  else keys.add(k);
});
window.addEventListener('keyup', (e) => keys.delete(e.key.toLowerCase()));
window.addEventListener('blur', () => keys.clear());

document.getElementById('btnReset').addEventListener('click', () => loadScenario(currentScenario));
document.getElementById('btnClear').addEventListener('click', clearMarks);
document.getElementById('btnGeo').addEventListener('click', () => { showGeometry = !showGeometry; });

const scenarioBox = document.getElementById('scenarios');
SCENARIOS.forEach((s, i) => {
  const b = document.createElement('button');
  b.textContent = s.name;
  b.addEventListener('click', () => loadScenario(i));
  scenarioBox.appendChild(b);
});

// ---------------------------------------------------------------- loop

let lastT = performance.now();

function frame(t) {
  const dt = Math.min(0.05, (t - lastT) / 1000);
  lastT = t;

  const steerRate = 55 * Math.PI / 180; // rad/s with keys
  if (keys.has('arrowleft') || keys.has('a')) setSteer(car.steer - steerRate * dt);
  if (keys.has('arrowright') || keys.has('d')) setSteer(car.steer + steerRate * dt);

  if (keys.has('arrowup') || keys.has('w')) move(140 * dt);
  if (keys.has('arrowdown') || keys.has('s')) move(-140 * dt);

  if (drag && drag.mode === 'car') {
    const [gx, gy] = carLocalToWorld(drag.grabLocal[0], drag.grabLocal[1]);
    const along = (mouse.x - gx) * Math.cos(car.h) + (mouse.y - gy) * Math.sin(car.h);
    const ds = Math.max(-7, Math.min(7, along * 0.22));
    if (Math.abs(ds) > 0.05) move(ds);
  }

  flash = Math.max(0, flash - dt * 1.6);
  render();
  updateHud();
  requestAnimationFrame(frame);
}

function resize() {
  W = canvas.width = window.innerWidth;
  H = canvas.height = window.innerHeight;
  wheelWidget.x = W / 2;
  wheelWidget.y = H - 84;
  loadScenario(currentScenario);
}

window.addEventListener('resize', resize);
resize();

// debug hook: ?sc=N loads a scenario, ?demo=steer:-35,fwd:400 drives it
{
  const params = new URLSearchParams(location.search);
  const sc = parseInt(params.get('sc'), 10);
  if (Number.isInteger(sc) && SCENARIOS[sc]) loadScenario(sc);
  const demo = params.get('demo');
  if (demo) {
    for (const cmd of demo.split(',')) {
      const [k, v] = cmd.split(':');
      if (k === 'steer') setSteer(parseFloat(v) * Math.PI / 180);
      else if (k === 'fwd') move(parseFloat(v));
    }
  }
}

requestAnimationFrame(frame);
