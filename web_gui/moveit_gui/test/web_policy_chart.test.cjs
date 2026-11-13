const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

async function load() {
  const src = fs.readFileSync(path.join(__dirname, '../moveit_gui/web/static/monitors.js'), 'utf8');
  return import('data:text/javascript;base64,' + Buffer.from(src).toString('base64'));
}

test('policy chart buffer keeps only the last 30 seconds and accepts batched steps', async () => {
  const { policyStepBuffer } = await load();
  const b = policyStepBuffer(30);
  b.push([{ t: 0, action: [0, 0, 0] }, { t: 10, action: [1, 2, 3] }, { t: 40, action: [2, 3, 4] }]);
  assert.equal(b.length, 2);
  b.push([{ t: 70, action: [3, 4, 5] }, { t: 'bad' }]);
  assert.equal(b.length, 2);
  assert.deepEqual(b.rows.map((x) => x.t), [40, 70]);
});
