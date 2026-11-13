const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(require('node:path').join(__dirname, '../moveit_gui/web/static/app.js'), 'utf8');

test('saved policies render vision warnings and startup refusals after refresh', async () => {
  const start = source.indexOf('const loadPolicies = async () => {');
  const end = source.indexOf('const loadFolder =', start);
  let policies = [{ name: 'bands', vision_issues: [{ detail: 'No band publisher', blocking: false }] }];
  let rows;
  const context = {
    cmd: async () => ({ ok: true, policies }),
    plist: { replaceChildren: (...children) => { rows = children; } },
    h: (tag, props, ...children) => ({ tag, props, children }),
    lease: x => x, btn: () => ({}), ask: async () => false,
  };
  vm.createContext(context);
  vm.runInContext(source.slice(start, end) + '\nglobalThis.load = loadPolicies;', context);
  await context.load();
  assert.match(JSON.stringify(rows), /No band publisher/);
  policies = [{ name: 'bands', vision_issues: [{ detail: 'Runner refuses startup', blocking: true }] }];
  await context.load();
  assert.match(JSON.stringify(rows), /Runner refuses startup/);
  policies = [{ name: 'bands', vision_issues: [] }];
  await context.load();
  assert.doesNotMatch(JSON.stringify(rows), /No band publisher|Runner refuses startup/);
});
