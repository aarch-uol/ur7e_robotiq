// Run with: node --test test/web_parts_draft.test.cjs
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

const ROOT = path.join(__dirname, '..');

async function poseHelpers() {
  const src = fs.readFileSync(path.join(ROOT, 'moveit_gui/web/static/view3d.js'), 'utf8');
  const a = src.indexOf('function originMatrix');
  const b = src.indexOf('// Wavefront OBJ');
  assert.ok(a >= 0 && b > a);
  const code = src.slice(a, b).replaceAll('export function', 'function');
  const threeSrc = fs.readFileSync(path.join(ROOT, 'moveit_gui/web/static/vendor/three/build/three.module.js'), 'utf8');
  const THREE = await import('data:text/javascript;base64,' + Buffer.from(threeSrc).toString('base64'));
  const context = { THREE, D2R: Math.PI / 180 };
  vm.runInNewContext(code + '\nthis.api = { partFrameWorld, partPoseFromWorld, partGeometryOffsetMm };', context);
  return { THREE, ...context.api };
}

async function workcellModule() {
  const src = fs.readFileSync(path.join(ROOT, 'moveit_gui/web/static/workcell.js'), 'utf8');
  return { src, mod: await import('data:text/javascript;base64,' + Buffer.from(src).toString('base64')) };
}

function near(a, b, eps = 1e-8) {
  assert.equal(a.length, b.length);
  a.forEach((v, i) => assert.ok(Math.abs(v - b[i]) <= eps, String(v) + ' != ' + String(b[i]) + ' at ' + String(i)));
}

test('part parent-frame pose round-trips through world without mesh/anchor offsets entering the frame', async () => {
  const { THREE, partFrameWorld, partPoseFromWorld, partGeometryOffsetMm } = await poseHelpers();
  const parent = new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(0.31, -0.47, 1.12, 'ZYX'));
  parent.setPosition(0.42, -0.18, 0.73);
  const original = { xyz_mm: [12.345, -67.89, 101.234], rpy_deg: [22.5, -13.25, 47.75] };
  let pose = structuredClone(original);
  for (let i = 0; i < 100; i++) pose = partPoseFromWorld(parent, partFrameWorld(parent, pose));
  near(pose.xyz_mm, original.xyz_mm, 1e-8);
  near(pose.rpy_deg, original.rpy_deg, 1e-8);

  near(partGeometryOffsetMm({ anchor: 'base', shape: { type: 'box', size_mm: [40, 30, 20] } }), [0, 0, 10]);
  near(partGeometryOffsetMm({ anchor: 'center', shape: { type: 'box', size_mm: [40, 30, 20] } }), [0, 0, 0]);
  near(partGeometryOffsetMm({ anchor: 'base', shape: { type: 'mesh', offset_mm: [3, -4, -36.169] } }), [3, -4, -36.169]);
});

test('Import mesh defaults are a unique lower_snake_case draft on the current parent', async () => {
  const { src, mod } = await workcellModule();
  const d = mod.meshPartDefaults('Robotiq_Camera_Mount_Y30_Z246_X0_neck.stl', 'robotiq_tcp', [], 0.001);
  assert.match(d.name, /^[a-z][a-z0-9_]{0,39}$/);
  assert.equal(d.parent, 'robotiq_tcp');
  assert.deepEqual(d.xyz_mm, [0, 0, 0]);
  assert.deepEqual(d.rpy_deg, [0, 0, 0]);
  assert.deepEqual(d.shape, { type: 'mesh', file: 'Robotiq_Camera_Mount_Y30_Z246_X0_neck.stl', scale: 0.001 });
  assert.equal(d.collision, 'auto');
  assert.deepEqual(d.collision_settings, { max_pieces: 8, max_triangles: 64, tolerance_mm: 0.5, concavity: 1.35 });
  const d2 = mod.meshPartDefaults('Robotiq_Camera_Mount_Y30_Z246_X0_neck.stl', '', [d.name], 0.001);
  assert.notEqual(d2.name, d.name);
  assert.equal(d2.parent, 'tool0');

  // The Import button uses those defaults, then applies "standing on the frame" before exposing the draft.
  assert.match(src, /if \(importing\)[\s\S]*meshPartDefaults\([\s\S]*startFrom\(d\);[\s\S]*meshPlace\("base"\)/);
  assert.match(src, /Mesh uploaded: not saved yet\. Place it and press Save part\./);
  assert.match(src, /auto \(recommended\)/);
  assert.match(src, /parts_collision_preview/);
});

test('draft collision preview follows the shared Collision view toggle', () => {
  const src = fs.readFileSync(path.join(ROOT, 'moveit_gui/web/static/view3d.js'), 'utf8');
  assert.match(src, /partDraftCollision\.visible = this\.show\.collision/);
  assert.match(src, /for \(const g of d\.collision_preview \|\| \[\]\)/);
  assert.match(src, /partDraftCollision\.add\(mesh\)/);
});
