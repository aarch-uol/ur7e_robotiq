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
