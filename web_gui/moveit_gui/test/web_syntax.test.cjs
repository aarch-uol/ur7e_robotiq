// Every front-end file must parse as an ES module: the page loads them that way, and one syntax
// error leaves a blank page while the other tests still pass.
const test = require("node:test");
const assert = require("node:assert");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const { execFileSync } = require("node:child_process");

const dir = path.join(__dirname, "..", "moveit_gui", "web", "static");
const tmp = fs.mkdtempSync(path.join(os.tmpdir(), "gui-syntax-"));
for (const f of fs.readdirSync(dir).filter((x) => x.endsWith(".js"))) {
  test(`${f} parses as a module`, () => {
    const copy = path.join(tmp, f.replace(/\.js$/, ".mjs"));
    fs.copyFileSync(path.join(dir, f), copy);
    assert.doesNotThrow(() => execFileSync(process.execPath, ["--check", copy], { stdio: "pipe" }));
  });
}
