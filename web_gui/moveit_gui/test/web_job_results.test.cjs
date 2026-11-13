// Run with: node --test test/web_job_results.test.cjs
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');

function helpers() {
  const source = fs.readFileSync(path.join(__dirname, '../moveit_gui/web/static/app.js'), 'utf8');
  const a = source.indexOf('const myJobs = new Set();');
  const b = source.indexOf('function toast(', a);
  assert.ok(a >= 0 && b > a);
  const context = {};
  vm.runInNewContext(source.slice(a, b) +
    '\nthis.api = { ownJob, takeJobResult, clearJobResults, myJobs, earlyResults };', context);
  return context.api;
}

test('a result that beats its HTTP acknowledgement is claimed when the job id arrives', () => {
  const h = helpers();
  const result = { id: 7, cmd: 'plan', ok: true, msg: 'done' };
  assert.equal(h.takeJobResult(result), false);
  assert.equal(h.earlyResults.size, 1);
  assert.deepEqual(h.ownJob(7), result);
  assert.equal(h.earlyResults.size, 0);
  assert.equal(h.myJobs.size, 0);
});

test('normal results are owned once and unmatched replay is bounded', () => {
  const h = helpers();
  assert.equal(h.ownJob(3), null);
  assert.equal(h.takeJobResult({ id: 3 }), true);
  assert.equal(h.takeJobResult({ id: 3 }), false);
  for (let id = 10; id < 80; id++) h.takeJobResult({ id });
  assert.equal(h.earlyResults.size, 50);
  h.clearJobResults();
  assert.equal(h.earlyResults.size, 0);
  assert.equal(h.myJobs.size, 0);
});

function actionHelpers() {
  const source = fs.readFileSync(path.join(__dirname, "../moveit_gui/web/static/app.js"), "utf8");
  const a = source.indexOf("function actionState(");
  const b = source.indexOf("const action =", a);
  assert.ok(a >= 0 && b > a);
  const context = {
    setTimeout: () => 0,
    toast: () => {},
    waitResolve: null,
  };
  context.waitJob = () => new Promise((resolve) => { context.waitResolve = resolve; });
  vm.runInNewContext(source.slice(a, b) + "\nthis.api = { runAction };", context);
  return { api: context.api, context };
}

function fakeButton() {
  return {
    dataset: {},
    setAttribute(name, value) { this[name] = value; },
  };
}

test("action stays pending/running and suppresses a repeat click until the final job result", async () => {
  const { api, context } = actionHelpers();
  const b = fakeButton();
  let calls = 0, ackResolve;
  const ack = new Promise((resolve) => { ackResolve = resolve; });
  const p = api.runAction(b, () => { calls += 1; return ack; }, {});

  assert.equal(b.dataset.actionState, "pending");
  await api.runAction(b, () => { calls += 1; return { ok: true }; }, {});
  assert.equal(calls, 1);

  ackResolve({ ok: true, job: 9 });
  await Promise.resolve();
  await Promise.resolve();
  assert.equal(b.dataset.actionState, "running");

  context.waitResolve({ id: 9, ok: true, msg: "done" });
  const result = await p;
  assert.equal(result.ok, true);
  assert.equal(b.dataset.actionState, "done");
});

function profilePrefsHelper(saveOk) {
  const source = fs.readFileSync(path.join(__dirname, "../moveit_gui/web/static/app.js"), "utf8");
  const a = source.indexOf("async function profilePrefs(");
  const b = source.indexOf("let prefsSaving =", a);
  assert.ok(a >= 0 && b > a);
  const context = {
    prefsSynced: null,
    localPrefs: () => ({ "v3-show": "camera" }),
    scmd: async (name) => name === "profile_prefs_get"
      ? { ok: true, prefs: {} }
      : { ok: saveOk },
    location: { reload() {} },
    PREF_KEYS: [],
    localStorage: { removeItem() {}, setItem() {} },
  };
  vm.runInNewContext(source.slice(a, b) + "\nthis.api = { profilePrefs, synced: () => prefsSynced };", context);
  return context.api;
}

test("new profile preferences are not marked synced when the first save fails", async () => {
  const failed = profilePrefsHelper(false);
  await failed.profilePrefs("Cenk", false);
  assert.equal(failed.synced(), null);

  const saved = profilePrefsHelper(true);
  await saved.profilePrefs("Cenk", false);
  assert.equal(saved.synced(), JSON.stringify({ "v3-show": "camera" }));
});

test("server boot tracking advances to the new boot id after a restart", () => {
  const source = fs.readFileSync(path.join(__dirname, "../moveit_gui/web/static/app.js"), "utf8");
  assert.match(source, /if \(S\.boot\) boot = S\.boot;/);
  assert.doesNotMatch(source, /boot = boot \|\| S\.boot/);
});

test("a successful takeover updates the local lease before the SSE state catches up", async () => {
  const source = fs.readFileSync(path.join(__dirname, "../moveit_gui/web/static/app.js"), "utf8");
  const a = source.indexOf("function lostControl(");
  const b = source.indexOf("// ---- users:", a);
  assert.ok(a >= 0 && b > a);
  const context = {
    post: async () => ({ ok: true, lease: { token: "lease-token", client: "new-client", profile: "alpha",
      needs_switch: true, settings_owner: "other" } }),
    operatorName: async () => "alpha",
    render: () => {},
    document: { querySelectorAll: () => [] },
  };
  vm.runInNewContext(`let S = { lease: { client: "old-client", needs_switch: false } };
    let holds = false, TOKEN = null, MYID = null, BID = "browser-id", hbFailsSince = 0, leaseAt = 0, gaveAt = 0;
    ${source.slice(a, b)}
    this.api = { takeLease, state: () => S, isHolding: () => holds };`, context);
  await context.api.takeLease();
  assert.equal(context.api.isHolding(), true);
  assert.equal(context.api.state().lease.client, "new-client");
  assert.equal(context.api.state().lease.needs_switch, true);
});

test("the settings prompt is not auto-cancelled by a stale queued state", () => {
  const source = fs.readFileSync(path.join(__dirname, "../moveit_gui/web/static/app.js"), "utf8");
  const a = source.indexOf("async function switchSettings(");
  const b = source.indexOf("// Heartbeat:", a);
  assert.ok(a >= 0 && b > a);
  const body = source.slice(a, b);
  // the prompt may close when control is lost, but never because the (possibly lagging) state says needs_switch is off
  assert.doesNotMatch(body, /valid:[^\n]*needs_switch/);
  assert.doesNotMatch(body, /if \(!v\) \{ if \(holds && S\.lease && S\.lease\.needs_switch\)/);
});
