// Run with: node --test test/web_flask.test.cjs
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

async function flaskModule() {
  const src = fs.readFileSync(path.join(__dirname, '../moveit_gui/web/static/flask.js'), 'utf8');
  return import('data:text/javascript;base64,' + Buffer.from(src).toString('base64'));
}

const draw = { sphere: { c: [450, 100, 120], r: 42.5 }, neck: { a: [450, 100, 160], b: [450, 100, 210], r: 13.5 },
  pivot: [450, 100, 160] };

test('the 3D view gets the fitted flask, the touches, the next touch and the pivot', async () => {
  const { flaskShapes } = await flaskModule();
  const d = { result: { draw }, points: { rim: [[0.45, 0.1, 0.21]], bulb: [[0.45, 0.1, 0.075], [0.5, 0.1, 0.12]], neck: [] },
    hint: { xyz: [0.4, 0.1, 0.1] } };
  const s = flaskShapes(d, null);
  assert.equal(s.spheres.length, 1);
  assert.deepEqual(s.spheres[0].c, [450, 100, 120]);
  assert.equal(s.cylinders[0].r, 13.5);
  assert.equal(s.points.length, 1 + 3 + 1);                     // pivot, three touches, the hint
  assert.deepEqual(s.points[1].p, [450, 100, 210]);              // metres -> mm
  assert.equal(s.lines.length, 0);
});

test('without a fit the saved fixture is drawn, and reach poses are green or red', async () => {
  const { flaskShapes } = await flaskModule();
  const reach = { poses: [{ ok: true, tip: [1, 2, 3], tail: [1, 2, 63] }, { ok: false, tip: [4, 5, 6], tail: [4, 5, 66] }] };
  const s = flaskShapes({ result: null, fixture: { draw }, points: {} }, reach);
  assert.equal(s.spheres.length, 1);
  assert.deepEqual(s.lines.map((l) => l.color), [0x16a34a, 0xdc2626]);
  assert.deepEqual(flaskShapes(null, null), { spheres: [], cylinders: [], points: [], lines: [] });
});
