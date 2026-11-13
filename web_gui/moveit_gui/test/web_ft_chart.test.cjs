// Run with: node --test test/web_ft_chart.test.cjs
// OPERATE F/T chart (monitors.js): axis picking, saved choice, 100 Hz samples, two axes, redraw on a change.
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

// a minimal DOM: enough for el(), events, attributes and a canvas that records what is drawn
function fakeDom() {
  const drawn = [];
  const ctx = new Proxy({ drawn }, {
    get: (t, k) => (k in t ? t[k] : (...a) => { if (k === 'fillText') drawn.push(String(a[0])); }),
    set: () => true,
  });
  class Node {
    constructor(tag) {
      this.nodeType = 1; this.tagName = tag.toUpperCase(); this.children = []; this.attrs = {}; this.listeners = {};
      this.style = {}; this.className = ''; this.textContent = ''; this.open = false; this.value = '';
      this.clientWidth = 400; this.clientHeight = 170; this.offsetParent = {};
      this.classList = { remove: () => {}, add: () => {}, toggle: () => {} };
    }
    setAttribute(k, v) { this.attrs[k] = String(v); if (k === 'value') this.value = String(v); }
    getAttribute(k) { return this.attrs[k]; }
    addEventListener(k, fn) { (this.listeners[k] = this.listeners[k] || []).push(fn); }
    fire(k) { (this.listeners[k] || []).forEach((fn) => fn({})); }
    append(...c) { this.children.push(...c); }
    getContext() { return ctx; }
    get all() { return [this, ...this.children.flatMap((c) => (c.all ? c.all : []))]; }
  }
  const document = { createElement: (t) => new Node(t), createTextNode: (t) => ({ text: t }), hidden: false };
  return { document, drawn };
}

async function load() {
  const src = fs.readFileSync(path.join(__dirname, '../moveit_gui/web/static/monitors.js'), 'utf8');
  const url = 'data:text/javascript;base64,' + Buffer.from(src).toString('base64');
  return import(url);
}

function storage() {
  const m = new Map();
  return { getItem: (k) => (m.has(k) ? m.get(k) : null), setItem: (k, v) => m.set(k, v), m };
}

test('axisRange keeps zero in view and never zooms into noise', async () => {
  const { axisRange } = await load();
  const r = axisRange([10.1, 10.2, 10.15], 2);
  assert.ok(r.lo <= 0 && r.hi >= 10.2, JSON.stringify(r));
  const n = axisRange([0.01, -0.02], 2);
  assert.ok(n.hi - n.lo >= 2);
  const e = axisRange([], 0.2);
  assert.ok(e.hi - e.lo >= 0.2);
});

test('chips pick the axes, the choice is saved, samples arrive at 100 Hz, torques get the right axis', async () => {
  const { document, drawn } = fakeDom();
  globalThis.document = document;
  globalThis.devicePixelRatio = 1;
  let frames = 0;
  globalThis.requestAnimationFrame = (fn) => { frames++; setTimeout(() => fn(1e6 + frames * 100), 0); return frames; };
  const { ftChart } = await load();
  const st = storage();
  const c = ftChart(st);
  assert.deepEqual([...c.pick].sort(), ['Fx', 'Fy', 'Fz']);          // default: forces
  const chips = c.el.all.filter((n) => n.tagName === 'BUTTON' && n.className === 'ftc-chip');
  assert.equal(chips.length, 8);
  const tz = chips.find((b) => b.children.some((k) => k.text === 'Tz'));
  const fz = chips.find((b) => b.children.some((k) => k.text === 'Fz'));
  tz.fire('click'); fz.fire('click');
  assert.deepEqual([...c.pick].sort(), ['Fx', 'Fy', 'Tz']);
  assert.equal(tz.getAttribute('aria-pressed'), 'true');
  assert.deepEqual(JSON.parse(st.getItem('ftchart')).pick.sort(), ['Fx', 'Fy', 'Tz']);
  assert.deepEqual([...ftChart(st).pick].sort(), ['Fx', 'Fy', 'Tz']);   // a reload keeps it

  // one state carries ten 100 Hz samples: all are kept, a repeated one is dropped
  const samples = Array.from({ length: 10 }, (_, i) => [100 + i * 0.01, 1, 2, 3, 0.1, 0.2, 0.3]);
  c.el.open = true;
  c.push({ ft_samples: samples });
  c.push({ ft_samples: samples.slice(-1) });
  assert.equal(c.length, 10);
  await new Promise((r) => setTimeout(r, 5));
  assert.ok(frames >= 1, 'a change schedules a redraw');
  c.draw();
  assert.ok(drawn.includes('N') && drawn.includes('Nm'), 'forces and torques on their own axes: ' + drawn.join(' '));
});

test('the chart glides on the page clock, keeps its axis through noise and draws the same samples each frame', async () => {
  const { document } = fakeDom();
  globalThis.document = document;
  globalThis.devicePixelRatio = 1;
  globalThis.requestAnimationFrame = () => 0;                 // frames are drawn by hand here
  let ms = 1000;
  globalThis.performance = { now: () => ms };
  const lines = [];
  const { ftChart } = await load();
  const c = ftChart(storage());
  c.el.open = true;
  // grab what the force line goes through: x of the first point and the y-axis labels
  const canvas = c.el.all.find((n) => n.tagName === 'CANVAS');
  const moves = [], labels = [];
  const rec = { lineTo: (x) => moves.push(x), fillText: (s) => labels.push(String(s)) };
  const g = new Proxy(rec, { get: (o, k) => (k in o ? o[k] : () => {}), set: () => true });
  canvas.getContext = () => g;
  const send = (t0, n, fz) => c.push({ ft_samples: Array.from({ length: n }, (_, i) => [t0 + i * 0.01, 0, 0, fz(i), 0, 0, 0]) });
  // 30 s of 100 Hz samples arriving in batches of 4 (every 40 ms of page time)
  let ts = 500;
  for (let b = 0; b < 750; b++) { send(ts, 4, () => 10); ts += 0.04; ms += 40; }
  const draw = () => { moves.length = 0; labels.length = 0; c.draw(); return { x: moves.slice(), l: labels.slice() }; };
  // between two batches "now" moves with the page clock: the line scrolls instead of standing still
  const a = draw(); ms += 20; const b = draw();
  assert.ok(b.x[b.x.length - 1] < a.x[a.x.length - 1], 'scrolls between batches');
  // a spike grows the axis at once; when it leaves the window the axis holds, then shrinks after 2 s
  send(ts, 1, () => 40); ts += 0.01; ms += 10;
  assert.ok(draw().l.includes('40') || draw().l.some((s) => +s >= 40), 'grows for a spike');
  for (let b = 0; b < 503; b++) { send(ts, 4, () => 10); ts += 0.04; ms += 40; if (b % 5 === 0) c.draw(); }   // the spike just left the 20 s window
  const held = draw().l;
  assert.ok(held.some((s) => +s >= 40), 'holds right after: ' + held.join(' '));
  ms += 2500;
  const shrunk = draw().l;
  assert.ok(!shrunk.some((s) => +s >= 40), 'shrinks after 2 s: ' + shrunk.join(' '));
  delete globalThis.performance;
});

test('"− Payload" redraws the whole history with the payload each sample came with, so it matches the card', async () => {
  const { document } = fakeDom();
  globalThis.document = document;
  globalThis.devicePixelRatio = 1;
  globalThis.requestAnimationFrame = () => 0;
  let ms = 1000;
  globalThis.performance = { now: () => ms };
  const { ftChart } = await load();
  const c = ftChart(storage());
  c.el.open = true;
  const canvas = c.el.all.find((n) => n.tagName === 'CANVAS');
  const ys = [], labels = [];
  const g = new Proxy({ lineTo: (x, y) => ys.push(y), fillText: (s) => labels.push(String(s)) },
    { get: (o, k) => (k in o ? o[k] : () => {}), set: () => true });
  canvas.getContext = () => g;
  // 10 s at rest, then the wrist turns: the tool weight moves from Fz to Fx; the reading is only the weight
  let ts = 500;
  for (let b = 0; b < 250; b++) {
    const w = b < 125 ? [0, 0, 12, 0, 0, 0] : [9, 0, 8, 0, 0, 0];
    c.push({ ft_payload: { w, kg: 1.2 }, ft_samples: Array.from({ length: 4 }, (_, i) => [ts + i * 0.01, ...w]) });
    ts += 0.04; ms += 40;
  }
  const draw = () => { ys.length = 0; labels.length = 0; c.draw(); return { ys: ys.slice(), top: Math.max(...labels.map(Number).filter(Number.isFinite)) }; };
  const raw = draw();
  assert.ok(raw.top >= 12, 'as read: the weight is drawn');
  c.setPayloadRemoved(true);
  const comp = draw();
  assert.ok(comp.top < 5, 'payload removed: the axis falls to the residual, top label ' + comp.top);
  const n = {};                                                // series points: many lineTo at one y; grid lines: few
  for (const y of comp.ys) n[y.toFixed(1)] = (n[y.toFixed(1)] || 0) + 1;
  const lineYs = Object.keys(n).filter((y) => n[y] >= 20);
  assert.equal(lineYs.length, 1, 'every line flat at 0, before and after the turn: ' + JSON.stringify(n));
  c.setPayloadRemoved(false);
  assert.ok(draw().top >= 12, 'and back');
  delete globalThis.performance;
});
