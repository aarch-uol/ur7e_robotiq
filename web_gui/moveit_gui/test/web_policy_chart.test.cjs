const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

async function load() {
  const src = fs.readFileSync(path.join(__dirname, '../moveit_gui/web/static/monitors.js'), 'utf8');
  return import('data:text/javascript;base64,' + Buffer.from(src).toString('base64'));
}

test('policy chart buffer keeps only the last 30 seconds and accepts batched steps', async () => {
  const { policyStepBuffer, policyActionMeta, policyChartStatus } = await load();
  const b = policyStepBuffer(30);
  b.push([{ t: 0, action: [0, 0, 0] }, { t: 10, action: [1, 2, 3] }, { t: 40, action: [2, 3, 4] }]);
  assert.equal(b.length, 2);
  b.push([{ t: 70, action: [3, 4, 5] }, { t: 'bad' }]);
  assert.equal(b.length, 2);
  assert.deepEqual(b.rows.map((x) => x.t), [40, 70]);
  b.push([{ t: 80, action: [null, 0.2], command: [null, 0.04], obs_sigma: { x: null } }]);
  assert.equal(b.length, 2);
  assert.deepEqual(policyActionMeta({ action_names: ['travel', 'turn'], action_units: ['m', 'rad'] }, b.rows.at(-1)), {
    names: ['travel', 'turn'], units: ['m', 'rad']
  });
  assert.equal(policyActionMeta({}, { action: [null, 0.2] }).names.join(','), 'a0,a1');
  assert.equal(policyChartStatus({ policy: { busy: true }, policy_steps: [{ t: 100 }] }, 101).stale, false);
  assert.equal(policyChartStatus({ policy: { busy: true }, policy_steps: [{ t: 100 }] }, 103).stale, true);
  assert.equal(policyChartStatus({ policy: { busy: false }, policy_steps: [{ t: 100 }] }, 100.1).stale, true);
});

test('the policy Stop button is never disabled', () => {
  const src = fs.readFileSync(path.join(__dirname, '../moveit_gui/web/static/panels.js'), 'utf8');
  assert.doesNotMatch(src, /polStop\.disabled\s*=/);
});
