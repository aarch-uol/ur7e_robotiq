const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

const source = fs.readFileSync(path.join(__dirname, '../moveit_gui/web/static/app.js'), 'utf8');

test('policy setup imports wait for the job and upload folders as an admin feature', () => {
  const start = source.indexOf('const importPolicy = async');
  const end = source.indexOf('ptcp.replaceChildren', start);
  const section = source.slice(start, end);
  assert.match(section, /waitCommand\(cmd\("policy_import"/);
  assert.match(section, /overwrite: true/);
  assert.doesNotMatch(section, /location\.reload/);
  assert.match(source, /webkitdirectory/);
  assert.match(source, /\/api\/policy-upload/);
  assert.match(source, /no normaliser \(model has none\)/);
});
