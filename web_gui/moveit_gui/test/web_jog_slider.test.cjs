// Run with: node --test test/web_jog_slider.test.cjs
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

test('six spring sliders stream normalized values, renew the deadman, and zero on release', async () => {
  const source = fs.readFileSync(path.join(__dirname, '../moveit_gui/web/static/app.js'), 'utf8');
  const jog = source.slice(source.indexOf('const JOG = '), source.indexOf('// ------------------------------------------------------------------- force'));
  const calls = [];
  const elements = new Map();
  const sliders = Array.from({ length: 6 }, (_, i) => ({
    id: `op-js${i}`, value: '0', disabled: false,
    parentElement: { classList: { contains: () => false } },
    closest: () => ({ classList: { contains: () => false } }),   // the slider's row (hidden rows are skipped)
    setPointerCapture() {},
  }));
  for (let i = 0; i < 6; i++) elements.set(`#op-jsv${i}`, { textContent: '0' });
  elements.set('#op-jog-type', { value: 'tool' });
  const handlers = {};
  const context = {
    document: {
      querySelectorAll: () => sliders,
      addEventListener: (name, fn) => { handlers[name] = fn; },
      hidden: false,
    },
    window: { addEventListener: (name, fn) => { handlers[name] = fn; } },
    $: (selector) => elements.get(selector),
    INFO: { joint_names: Array(6).fill('joint') }, holds: true,
    scaling: () => ({ vel: 0.3, acc: 0.3 }),
    cmd: async (name, args) => {
      calls.push({ name, args });
      return name === 'jog_hold' ? { ok: true, jogging: true } : { ok: true };
    },
    post: async (endpoint, body) => { calls.push({ name: body.cmd }); return { ok: true }; },
    setInterval: (fn) => { context.tick = fn; return 1; },
    clearInterval: () => { context.tick = null; },
  };
  vm.runInNewContext(jog + '\nthis.api = { jogSliderBegin, jogSliderInput, jogRelease };', context);
  const slider = sliders[2];
  context.api.jogSliderBegin(slider, { pointerId: 1 });
  slider.value = '-75';
  await context.api.jogSliderInput(slider);
  assert.equal(calls[0].name, 'jog_start');
  assert.equal(calls[0].args.kind, 'cartesian');
  assert.equal(calls[0].args.frame, 'tool');
  assert.equal(calls[0].args.values[2], -0.75);
  assert.equal(calls[0].args.values.filter(Boolean).length, 1);
  assert.equal(calls[1].name, 'jog_hold');       // renew immediately after jog_start returns
  assert.equal(calls[1].args.values[2], -0.75);
  slider.value = '40';
  await context.tick();
  assert.equal(calls[2].name, 'jog_hold');
  assert.equal(calls[2].args.values[2], 0.4);
  context.api.jogRelease();
  assert.equal(calls.at(-1).name, 'jog_stop');
  assert.equal(slider.value, '0');
  assert.equal(elements.get('#op-jsv2').textContent, '0');
  assert.equal(context.tick, null);
});

test('continuous Servo is the Jog default rather than a planned step', () => {
  const source = fs.readFileSync(path.join(__dirname, '../moveit_gui/web/static/app.js'), 'utf8');
  assert.match(source, /value: "hold", selected: true[^\n]*Continuous \(Servo\)/);
  assert.doesNotMatch(source, /value: i, selected:/);
});
