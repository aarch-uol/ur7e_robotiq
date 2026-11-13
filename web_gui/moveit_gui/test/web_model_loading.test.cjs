const { test } = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');

const ROOT = path.join(__dirname, '..');

async function retryModule() {
  const src = fs.readFileSync(path.join(ROOT, 'moveit_gui/web/static/load_retry.js'), 'utf8');
  return import('data:text/javascript;base64,' + Buffer.from(src).toString('base64'));
}

test('retryAsync retries transient model or mesh failures with bounded backoff', async () => {
  const { retryAsync } = await retryModule();
  let attempts = 0;
  const waits = [];
  const result = await retryAsync(async () => {
    attempts++;
    if (attempts < 3) throw new Error('temporary');
    return 'loaded';
  }, { attempts: 4, baseMs: 25, maxMs: 40, sleep: async (ms) => waits.push(ms) });

  assert.equal(result, 'loaded');
  assert.equal(attempts, 3);
  assert.deepEqual(waits, [25, 40]);
});

test('retryAsync reports the final failure after the retry budget', async () => {
  const { retryAsync } = await retryModule();
  let attempts = 0;
  await assert.rejects(() => retryAsync(async () => {
    attempts++;
    throw new Error('offline');
  }, { attempts: 3, baseMs: 1, sleep: async () => {} }), /offline/);
  assert.equal(attempts, 3);
});
