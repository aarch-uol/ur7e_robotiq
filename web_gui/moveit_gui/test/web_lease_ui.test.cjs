// Browser lease/layout checks without a DOM: execute the small app.js functions in a VM.
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');

const source = fs.readFileSync(require('node:path').join(__dirname, '../moveit_gui/web/static/app.js'), 'utf8');

function context(extra = {}) {
  return vm.createContext({ console, setTimeout, JSON, ...extra });
}

test('busy release asks twice with danger styling and sends confirm', async () => {
  const calls = [], asks = [], ctx = context({
    holds: true, TOKEN: 't', MYID: 'i',
    post: async (path, body, silent) => { calls.push([path, body, silent]); return calls.length === 1
      ? { ok: false, msg: 'the robot is moving / a program, policy or force mode is running: release anyway?' }
      : { ok: true }; },
    ask: async (msg, options) => { asks.push([msg, options]); return true; },
    toast: () => { throw new Error('busy release should not toast'); },
    render: () => {},
  });
  const start = source.indexOf('async function releaseLease()');
  const end = source.indexOf('\n// ---- whose settings', start);
  vm.runInContext(`${source.slice(start, end)}; globalThis.releaseLease = releaseLease;`, ctx);
  await ctx.releaseLease();
  assert.equal(JSON.stringify(calls), JSON.stringify([
    ['/api/release', {}, true],
    ['/api/release', { confirm: true }, true],
  ]));
  assert.equal(asks.length, 2);
  assert.equal(asks[0][1].danger, true);
  assert.equal(asks[1][1].danger, true);
  assert.equal(ctx.holds, false);
  assert.equal(ctx.TOKEN, null);
});

test('idle release sends once without a prompt', async () => {
  const calls = [], ctx = context({
    holds: true, TOKEN: 't', MYID: 'i',
    post: async (...args) => { calls.push(args); return { ok: true }; },
    ask: async () => { throw new Error('idle release should not prompt'); },
    toast: () => {}, render: () => {},
  });
  const start = source.indexOf('async function releaseLease()');
  const end = source.indexOf('\n// ---- whose settings', start);
  vm.runInContext(`${source.slice(start, end)}; globalThis.releaseLease = releaseLease;`, ctx);
  await ctx.releaseLease();
  assert.equal(calls.length, 1);
  assert.equal(ctx.holds, false);
});

test('layout autosave stays bound to the loaded tab profile', async () => {
  const saved = [], values = new Map([['x', 'new'], ['operator', 'Other']]);
  const ctx = context({
    holds: true, prefsProfile: 'Loaded', prefsAutosaveStopped: false, prefsSynced: JSON.stringify({ x: 'old' }),
    prefsSaving: false, PREF_KEYS: ['x'],
    localStorage: { getItem: (k) => values.get(k) ?? null },
    localPrefs: () => ({ x: values.get('x') }),
    scmd: async (name, args) => { saved.push([name, args]); return { ok: true }; },
    setInterval: (fn) => { ctx.autosave = fn; },
  });
  const start = source.indexOf('setInterval(async () => {                    // a changed layout goes to the profile');
  const end = source.indexOf('\nwindow.addEventListener("storage"', start);
  vm.runInContext(`var prefsProfile = 'Loaded', prefsAutosaveStopped = false, prefsSynced = ${JSON.stringify(JSON.stringify({ x: 'old' }))}, prefsSaving = false; var holds = true; ${source.slice(start, end)};`, ctx);
  await ctx.autosave();
  assert.equal(saved[0][1].name, 'Loaded');
  values.set('operator', 'Changed in another tab');
  values.set('x', 'newer');
  await ctx.autosave();
  assert.equal(saved[1][1].name, 'Loaded');
});

test('initial profile layout sync waits for control', async () => {
  const saved = [], ctx = context({
    holds: false, prefsProfile: null, prefsSynced: null,
    localPrefs: () => ({ 'v3-show': '{}' }),
    scmd: async (name, args) => {
      saved.push([name, args]);
      return name === 'profile_prefs_get' ? { ok: true, prefs: {} } : { ok: true };
    },
  });
  const start = source.indexOf('async function profilePrefs(');
  const end = source.indexOf('\nlet prefsSaving', start);
  vm.runInContext(`${source.slice(start, end)}; globalThis.profilePrefs = profilePrefs;`, ctx);

  await ctx.profilePrefs('Viewer', false);
  assert.equal(JSON.stringify(saved), JSON.stringify([['profile_prefs_get', { name: 'Viewer' }]]));
  assert.equal(ctx.prefsSynced, null);

  vm.runInContext('holds = true;', ctx);
  await ctx.profilePrefs('Viewer', false);
  assert.equal(saved[2][0], 'profile_prefs_save');
  assert.equal(saved[2][1].name, 'Viewer');
});

