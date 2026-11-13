// Static contracts for the program run viewer and controls. The browser page has no build step,
// so these checks catch a missing hook before the fake-mode walkthrough.
const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

const ROOT = path.join(__dirname, '..');
const app = fs.readFileSync(path.join(ROOT, 'moveit_gui/web/static/app.js'), 'utf8');
const service = fs.readFileSync(path.join(ROOT, 'moveit_gui/web/service.py'), 'utf8');
const style = fs.readFileSync(path.join(ROOT, 'moveit_gui/web/static/style.css'), 'utf8');

test('PROGRAMS reuses the live F/T chart and OPERATE reuses the program preview layers', () => {
  assert.match(app, /this\.pgFtc\s*=\s*ftChart\(\)/);
  assert.match(app, /pg-ft-chart/);
  assert.match(app, /program_preview_path/);
  assert.match(app, /setPathMm\(/);
  assert.match(app, /setMarkers\(/);
  assert.match(app, /setHighlight\(/);
});

test('OPERATE exposes the floating program run control hooks and confirmed restart', () => {
  for (const id of ['op-program-run-view', 'op-program-run-step', 'op-program-run-pause',
    'op-program-run-stop', 'op-program-run-restart']) assert.match(app, new RegExp(id));
  assert.match(app, /program_restart/);
  assert.match(app, /Restart.*first step|first step.*Restart/s);
  assert.match(app, /ask\([^\n]*restart/i);
  assert.match(style, /op-program-run/);
});

test('OPERATE tracks the runner path and clears stale layers after the run', () => {
  assert.match(app, /programRunUpdate\(s\)/);
  assert.match(app, /programRunPaint\(s\)/);
  assert.match(app, /programRunClear\(\)/);
  assert.match(app, /setHighlightMm\(hits\.length/);
  assert.match(app, /e\.pause\.textContent\s*=\s*r\.state === "PAUSED" \? "Resume" : "Pause"/);
  assert.match(app, /pcmd\("program_pause"\)/);
  assert.match(app, /pcmd\("program_stop"\)/);
  assert.match(app, /pcmd\("program_restart"\)/);
  assert.match(app, /programRunDragStart/);
});

test('restart is a server command, not a client-side stop plus unguarded start', () => {
  assert.match(service, /def cmd_program_restart\(self\)/);
  assert.match(service, /self\.core\.runner\.preflight\(self\.program, 0\)/);
  assert.match(service, /self\.core\.runner\.stop\(\)/);
  assert.match(service, /self\.core\.runner\.start\(self\.program, 0, False, False\)/);
});
