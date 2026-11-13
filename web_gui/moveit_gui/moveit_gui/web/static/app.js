// MoveIt GUI web front end. No framework, no build step, no internet: the robot PC serves everything.
// State arrives as Server-Sent Events (10 Hz); commands are POSTs. The server owns every safety rule;
// this file only shows state and asks.

import { jointMonitor, ftChart } from "/static/monitors.js";
import { workcell } from "/static/workcell.js";
import { flaskPage } from "/static/flask.js";
import { drivers, driversBadge } from "/static/drivers.js";
import { workspace } from "/static/views.js";
import { extraViews } from "/static/panels.js";
import { icon } from "/static/icons.js";
import { ask, askText, tell, dismiss } from "/static/dialog.js";
// short joint names for compact readouts: "shoulder_pan_joint" -> "Shoulder pan"
const jshort = (n) => { const x = n.replace(/^.*?(shoulder|elbow|wrist)/, "$1").replace(/_joint$/, "").replace(/_/g, " ");
  return x.charAt(0).toUpperCase() + x.slice(1); };

const $ = (s, r = document) => r.querySelector(s);
function h(tag, props = {}, ...kids) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(props || {})) {
    if (k === "class") el.className = v;
    else if (k === "style") el.style.cssText = v;
    else if (k.startsWith("on")) el.addEventListener(k.slice(2), v);
    else if (k === "html") el.innerHTML = v;
    else if (v === true) el.setAttribute(k, "");
    else if (v !== false && v != null) el.setAttribute(k, v);
  }
  for (const c of kids.flat()) if (c != null && c !== false) el.append(c.nodeType ? c : document.createTextNode(c));
  return el;
}
const btn = (text, role = "", onclick = null, title = "") =>
  h("button", { class: `btn ${role}`, onclick, title: title || null }, text);
// one short line of help; the details sit behind ⓘ (hover / long-press) instead of a paragraph on the page
const note = (short, details = "") => h("div", { class: "muted small" }, short,
  details ? h("span", { class: "hint", title: details }, " ⓘ") : null);
const fmt = (v, d = 1) => (v == null || Number.isNaN(v) ? "—" : Number(v).toFixed(d));
const num = (value, step = "0.1") => h("input", { type: "number", step, value });
const AXES = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"];
const PROBLEM = /FAILED|refused|error|cannot|STOP|lost|fault/i;
const mmss = (s) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, "0")}`;

// ------------------------------------------------------------------ session
// The lease token is a secret the server hands out on "Take control". A reload keeps control (pagehide stores it,
// the next load takes it out again); a duplicated tab copies sessionStorage but finds nothing, because we remove
// the token as soon as we have read it.
let TOKEN = null, MYID = null;
try {
  const saved = JSON.parse(sessionStorage.getItem("lease") || "null");
  if (saved && saved.token) ({ token: TOKEN, id: MYID } = saved);
  sessionStorage.removeItem("lease");
} catch (e) { /* no sessionStorage: no reload-keeps-control */ }
let INFO = null, S = null, holds = false, connected = false, netBad = false, srvError = "";
// This browser's own id (kept in localStorage; every tab of it is the same user), so browsers on one PC or behind one
// IP are told apart. Never shown to others: they see a public id the server derives from it (MYUID, from "hello").
const BID = (() => {
  let b = "";
  try { b = localStorage.getItem("browser-id") || ""; } catch (e) { /* no storage */ }
  if (!/^[A-Za-z0-9_-]{16,64}$/.test(b)) {
    const r = crypto.getRandomValues(new Uint8Array(18));
    b = btoa(String.fromCharCode(...r)).replace(/\+/g, "-").replace(/\//g, "_").replace(/=+$/, "");
    try { localStorage.setItem("browser-id", b); } catch (e) { /* this tab only */ }
  }
  return b;
})();
let MYUID = null;
const logs = [];

async function post(path, body = {}, silent = false) {
  let r, data;
  try {
    r = await fetch(path, { method: "POST", headers: { "Content-Type": "application/json" },
                            body: JSON.stringify({ token: TOKEN, bid: BID, ...body }) });
    data = await r.json();
  } catch (e) {
    data = { ok: false, msg: `no answer from the robot PC (${e.message})` };
  }
  if (silent) return data;
  if (data.job) myJobs.add(data.job);
  if (!data.ok) toast(data.msg || "failed", false);
  else if (data.msg && data.msg !== "ok" && data.msg !== "started") say(data.msg, true);   // the page shows the result
  return data;
}
const cmd = (name, args = {}) => post("/api/cmd", { cmd: name, args });
// program commands carry the revision the page shows; the robot PC refuses them if the program changed meanwhile
const pcmd = (name, args = {}) => {
  const P = pages.PROGRAMS;
  return cmd(name, P && P.prog && P.prog.rev != null ? { ...args, rev: P.prog.rev } : args).then((r) => {
    if (!r.ok && /program was changed/.test(r.msg || "")) P.reload();
    return r;
  });
};

// the bottom line: the latest log line (errors stay for 5 s before a normal line may replace them)
let errUntil = 0;
function say(msg, ok) {
  if (ok && Date.now() < errUntil) return;
  if (!ok) errUntil = Date.now() + 5000;
  $("#lastlog").textContent = msg;
  $("#logbar").classList.toggle("err", !ok);
}
// toasts: answers to what THIS browser did. Errors stay until dismissed, the rest fade after 3 s.
const myJobs = new Set();
function toast(msg, ok, sticky) {
  say(msg, ok);
  const box = $("#toasts");
  const dup = [...box.children].find((x) => x.dataset.msg === msg);
  if (dup) dup.remove();
  const el = h("div", { class: "toast " + (ok ? "ok" : "err") + (sticky ? " sticky" : "") },
    h("span", {}, msg), h("a", { class: "x", title: "Dismiss", onclick: () => el.remove() }, "×"));
  el.dataset.msg = msg;
  box.append(el);
  while (box.children.length > 5) box.firstChild.remove();
  if (ok && !sticky) setTimeout(() => el.remove(), 3000);
}

const modeLabel = (n) => INFO.modes.find((m) => m.name === n)?.label || n;
// gate(el, cond): enabled only with the control lease AND cond
function gate(el, cond = true) { el.disabled = !(holds && cond); }

// ------------------------------------------------------------------- lease
// The profile: who you are, asked once per browser. The others see the name, and who has control.
// Your settings, setups, programs and screen layout are kept under it on the robot PC (user_profiles.py, worksets.py).
const PREF_KEYS = ["ftchart", "op-joints", "op-layouts1", "pg-jog-open", "pg-path", "v3-frames", "v3-pose", "v3-show", "v3-zones"];   // sorted
const scmd = (name, args = {}) => post("/api/cmd", { cmd: name, args }, true);
const localPrefs = () => Object.fromEntries(PREF_KEYS.map((k) => [k, localStorage.getItem(k)]).filter(([, v]) => v != null));
let prefsSynced = null;                     // JSON of the layout as last read from / saved to the profile
async function profilePrefs(name, reload) {
  // the profile's layout replaces this browser's; a profile with none yet gets this browser's
  const r = await scmd("profile_prefs_get", { name });
  if (!r.ok) { prefsSynced = null; return; }
  const local = localPrefs();
  if (!Object.keys(r.prefs).length) {
    prefsSynced = JSON.stringify(local);
    if (Object.keys(local).length) scmd("profile_prefs_save", { name, prefs: local });
    return;
  }
  const sorted = (o) => JSON.stringify(Object.fromEntries(Object.entries(o).sort()));
  prefsSynced = sorted(r.prefs);
  if (sorted(local) === prefsSynced) return;
  for (const k of PREF_KEYS) if (!(k in r.prefs)) localStorage.removeItem(k);
  for (const [k, v] of Object.entries(r.prefs)) localStorage.setItem(k, v);
  if (!reload) return false;
  location.reload();                        // the navigation is not at once: the caller must stop here
  return true;
}
setInterval(() => {                         // a changed layout goes to the profile (5 s)
  const n = localStorage.getItem("operator");
  if (!n || prefsSynced == null) return;
  const now = JSON.stringify(localPrefs());
  if (now !== prefsSynced) { prefsSynced = now; scmd("profile_prefs_save", { name: n, prefs: localPrefs() }); }
}, 5000);
async function operatorName(force) {
  let n = localStorage.getItem("operator") || "";
  if (!n || force) {
    const r = await scmd("profile_list");
    const names = r.ok ? r.profiles.map((p) => p.name) : [];
    const sel = h("select", { style: "min-width:160px" }, ...names.map((x) => h("option", { value: x }, x)),
      h("option", { value: "+" }, "New profile…"));
    sel.value = names.find((x) => x.toLowerCase() === n.toLowerCase()) || names[0] || "+";
    const nw = h("input", { type: "text", maxLength: 24, placeholder: "your name", style: "width:180px" });
    const nwRow = h("div", { class: "row" }, "Name", nw);
    const paint = () => { nwRow.hidden = sel.value !== "+"; };
    sel.onchange = paint;
    paint();
    const form = h("div", { style: "display:flex;flex-direction:column;gap:8px" }, h("div", { class: "row" }, "Profile", sel), nwRow);
    const v = await ask("Who are you?\nYour profile keeps your own settings (TCPs, keep-out zones, gains, ...), setups, programs " +
      "and screen layout. The robot runs the settings of the one in control. The others see the name.", { content: form, result: () => ({ name: sel.value === "+" ? nw.value.trim() : sel.value }) });
    if (v && v.name) {
      let name = v.name;
      if (sel.value === "+") {
        const c = await cmd("profile_create", { name });
        if (!c.ok) return n;
        name = c.name;
      }
      const changed = name !== n;
      n = name;
      localStorage.setItem("operator", n);
      post("/api/hello", { bid: BID, name: n }, true);
      if (changed) {
        pages.SETUP?.wc?.W?.setups?.showOwner?.(n);
        await profilePrefs(n, true);            // the profile's layout: the page reloads if it differs
      }
    }
  }
  return n;
}
let leaseAt = 0;
let gaveAt = 0;               // this browser just gave control away: losing it is no news
function lostControl(why) {
  TOKEN = MYID = null;
  if (!holds) return;
  if (Date.now() - gaveAt < 5000) { holds = false; render(); return; }
  holds = false;
  toast(`You lost control of the robot: ${why}. Take control again to operate.`, false, true);
  render();
}
async function takeLease() {
  const r = await post("/api/lease", { name: await operatorName(false), bid: BID });
  holds = !!r.ok;
  if (holds) { TOKEN = r.lease.token; MYID = r.lease.client; hbFailsSince = 0; }
  if (holds) document.querySelectorAll(".toast").forEach((x) => /lost control/.test(x.dataset.msg) && x.remove());
  leaseAt = Date.now();
  render();
}
// ---- users: everyone with this GUI open, one per browser (the server's "users" list); asking for control
let usersPop = null, askKey = "", offerTaken = "", keptSeen = "";
function usersFill() {
  const us = (S && S.users) || [];
  usersPop.replaceChildren(
    h("div", { class: "up-head" }, `Open in ${us.length} browser${us.length === 1 ? "" : "s"}`),
    ...us.map((u) => h("div", { class: "up-row" + (u.id === MYUID ? " me" : "") },
      h("span", { class: "up-dot" + (u.control ? " on" : "") }),
      h("span", { class: "up-name" }, u.name + (u.id === MYUID ? " (you)" : "")),
      u.control ? h("span", { class: "chip ok" }, S && S.lease && S.lease.away ? "control · away" : "control") : "",
      u.role ? h("span", { class: "chip", title: "Role (SETTINGS › Roles)" }, u.role) : "",
      h("span", { class: "up-where muted small", title: "Where it connects from (Tailscale user or IP)" },
        u.where + (u.tabs > 1 ? ` · ${u.tabs} tabs` : "")))),
    h("div", { class: "up-foot" },
      h("button", { class: "btn quiet", type: "button", onclick: async () => {
        await operatorName(true);
        if (holds) takeLease();
      } }, "Change my name")));
}
function usersToggle(e) {
  e.stopPropagation();
  if (usersPop) { usersPop.remove(); usersPop = null; return; }
  usersPop = h("div", { class: "users-pop" });
  const r = $("#h-users").getBoundingClientRect();
  usersPop.style.top = r.bottom + 6 + "px";
  usersPop.style.right = Math.max(8, innerWidth - r.right) + "px";
  document.body.append(usersPop);
  usersFill();
  const off = (ev) => {
    if (usersPop && (ev.type === "keydown" ? ev.key === "Escape" : !usersPop.contains(ev.target))) {
      usersPop.remove(); usersPop = null;
      document.removeEventListener("pointerdown", off, true); document.removeEventListener("keydown", off, true);
    }
  };
  document.addEventListener("pointerdown", off, true); document.addEventListener("keydown", off, true);
}
let robotAsked = false;
async function askControl() {
  const r = await post("/api/ask", { bid: BID }, true);
  toast(r.msg || (r.ok ? "asked" : "failed"), !!r.ok);
}
function usersRender(s) {
  const us = s.users || [], L = s.lease, b = $("#h-users");
  b.innerHTML = icon("users") + `<span>${us.length || "—"}</span>`;
  b.title = `Open in ${us.length} browser${us.length === 1 ? "" : "s"}: ` + us.map((u) => u.name + (u.control ? " (control)" : "")).join(", ");
  if (usersPop) usersFill();
  // someone else has control: ask them for it
  const mine = L.ask && L.ask.id === MYUID;
  $("#lease-ask").hidden = holds || !L.client || !MYUID || L.uid === MYUID;
  $("#lease-ask").disabled = !!(mine && !L.ask.kept);
  $("#lease-ask").textContent = mine && !L.ask.kept ? "Asked…" : "Ask";
  if (mine && L.ask.kept && keptSeen !== L.ask.id + L.name) {
    keptSeen = L.ask.id + L.name;
    toast(`${L.name.split(" @ ")[0]} keeps control`, false);
  }
  // control was given to this browser: take it
  if (L.offer && L.offer.id === MYUID && !holds && offerTaken !== L.offer.name + L.offer.id) {
    offerTaken = L.offer.name + L.offer.id;
    takeLease().then(() => holds && toast("You have control now (it was given to you)", true));
  }
  // this browser has control and someone asks for it: Give / Keep
  const key = holds && L.ask && !L.ask.kept && L.ask.id !== MYUID ? "ask:" + L.ask.id : "";
  if (key !== askKey) {
    dismiss("ask");
    askKey = key;
    if (key) {
      const who = L.ask.name, id = L.ask.id;
      ask(`${who} asks for control\n\nGive: ${who} gets control now (only while the robot is idle). Keep: you keep it.`,
        { ok: "Give", cancel: "Keep", timeout: 45, key: "ask", valid: () => holds && S.lease.ask && S.lease.ask.id === id })
        .then(async (give) => {
          if (give == null || !holds || !(S.lease.ask && S.lease.ask.id === id)) return;    // no answer: it expires
          if (give) gaveAt = Date.now();
          const r = await cmd("control_answer", { give });
          if (r.ok && give) { holds = false; TOKEN = MYID = null; render(); } else gaveAt = 0;
        });
    }
  }
}
// a batch waiting between runs asks the person in control (any page); the answer starts the next run or ends the batch
let batchAskId = 0;
function batchAsk(s) {
  const b = s.batch, id = b && b.active && b.phase === "waiting" && holds ? b.ask_id : 0;
  if (id === batchAskId) return;
  if (batchAskId) dismiss("batch");
  batchAskId = id;
  if (!id) return;
  const still = () => holds && S.batch && S.batch.phase === "waiting" && S.batch.ask_id === id;
  ask(b.question, { ok: "Run", cancel: "Stop batch", key: "batch", valid: still })
    .then((go) => { if (go != null && still()) cmd("batch_answer", { go: !!go }); });
}
async function releaseLease() { await post("/api/release"); holds = false; TOKEN = MYID = null; render(); }
// ---- whose settings are in use (moveit_gui/worksets.py). The robot runs the settings of the one in control: taking
// control while someone else's are in use asks to switch to your own (theirs stay in their profile).
let switchAsked = "";
async function switchSettings() {
  const L = S && S.lease;
  if (!holds || !L || !L.needs_switch) return;
  const key = `${L.client}|${L.settings_owner}`;
  if (switchAsked === key) return;
  switchAsked = key;
  const me = L.profile, act = L.settings_owner || "someone else";
  if (!me) {
    const v = await ask(`Who are you?\n\nThe robot has ${act}'s settings in use. To control it, pick your profile: ` +
      "you then switch to your own settings.", { ok: "Pick my profile", result: () => true });
    if (v) { await operatorName(true); takeLease(); } else releaseLease();
    return;
  }
  const list = await scmd("profile_list");
  const have = list.ok && list.settings.have.some((n) => n.toLowerCase() === me.toLowerCase());
  let sel = null;
  if (!have) {
    const [mine, shared] = await Promise.all([scmd("setup_list", { owner: me }), scmd("setup_list", { owner: "" })]);
    const opts = (r, who, label) => (r.ok ? r.setups.filter((x) => !x.auto)
      .map((x) => h("option", { value: `${who}/${x.name}` }, `${label}: ${x.name}`)) : []);
    sel = h("select", { style: "min-width:260px" }, h("option", { value: "" }, `A copy of ${act}'s settings`),
      ...opts(mine, me, "My setup"), ...opts(shared, "", "Shared setup"));
  }
  const v = await ask(`${act}'s settings are in use\n\nTo control the robot, switch to your own settings (TCPs, ` +
    `keep-out zones, fixtures, gains, F/T bias and payload, parts, ...). ${act}'s stay in their profile and come back ` +
    "when they take control. The GUI restarts (about a minute; the robot stack too when your parts or tool differ)." +
    (sel ? "\n\nYou have no settings of your own yet." : ""),
  { ok: "Use my settings", content: sel ? h("div", { class: "row" }, "Start from", sel) : undefined,
    result: () => ({ v: sel ? sel.value : "" }), valid: () => holds && S.lease && S.lease.needs_switch });
  if (!v) { if (holds && S.lease && S.lease.needs_switch) releaseLease(); return; }
  const args = {};
  if (v.v) { const i = v.v.indexOf("/"); Object.assign(args, { start: "setup", owner: v.v.slice(0, i), name: v.v.slice(i + 1) }); }
  const r = await cmd("profile_activate", args);
  if (r.ok) sessionStorage.setItem("retake", "1");          // after the restart: reload and take control again
  else switchAsked = "";
}
// Heartbeat: every 500 ms, never two at once, with a timeout. Control is lost ONLY when the server answers
// holds:false. A failed or slow request is a connection problem (banner, keeps trying), not a lost lease: the server
// decides after LEASE_TIMEOUT_S, and any answer that arrives in time keeps it.
let hbBusy = false, hbFailsSince = 0;
async function heartbeat() {
  if (hbBusy || !TOKEN) return;
  hbBusy = true;
  const ac = new AbortController(), timer = setTimeout(() => ac.abort(), 2000);
  try {
    const r = await fetch("/api/heartbeat", { method: "POST", headers: { "Content-Type": "application/json" },
                                              body: JSON.stringify({ token: TOKEN }), signal: ac.signal });
    const d = await r.json();
    hbFailsSince = 0;
    if (netBad) { netBad = false; render(); }
    if (d.holds) { if (!holds) { holds = true; render(); } }
    else lostControl(S && S.lease && S.lease.client && S.lease.client !== MYID ? `${S.lease.name} took over`
      : "this browser went quiet while the robot moved, or was away for over 2 min");
  } catch (e) {
    hbFailsSince = hbFailsSince || Date.now();
    if (!netBad && Date.now() - hbFailsSince > 800) { netBad = true; render(); }
  } finally { clearTimeout(timer); hbBusy = false; }
}
setInterval(heartbeat, 500);
window.addEventListener("pagehide", () => {
  if (TOKEN) try { sessionStorage.setItem("lease", JSON.stringify({ token: TOKEN, id: MYID })); } catch (e) { /* ignore */ }
});
window.addEventListener("pageshow", (e) => { if (e.persisted) try { sessionStorage.removeItem("lease"); } catch (x) { /* ignore */ } });

// ------------------------------------------------------------------- pages
const pages = {};
let current = "OPERATE";

async function confirmMove(text) {
  return !$("#op-confirm") || !$("#op-confirm").checked || (await ask(text + "\n\nExecute?"));
}
// OPERATE's optional mm/s of a linear move: null = none (empty or 0); the server checks the range
function mmS() {
  const v = +$("#op-mms").value;
  return v > 0 ? v : null;
}
function scaling() {
  return { vel: Math.min(1, Math.max(0.01, (+$("#op-vel").value || 10) / 100)),
           acc: Math.min(1, Math.max(0.01, (+$("#op-acc").value || 10) / 100)) };
}
// Hold-to-run: the motion runs only while the button is held. The page renews every 150 ms and the robot PC
// stops the motion 0.4 s after the last renewal, so a quick tap, a dropped connection or a closed tab cannot
// run the whole move. Releasing also sends STOP at once.
const HOLD = { btn: null, job: null, pressed: false, timer: null };
const holdCancelled = new Set();
function holdButton(text, start, title) {
  const b = btn(text, "hold", null, title);
  b.addEventListener("pointerdown", async (e) => {
    if (b.disabled || HOLD.btn) return;
    if (b.setPointerCapture) b.setPointerCapture(e.pointerId);
    HOLD.btn = b; HOLD.pressed = true; b.classList.add("held");
    HOLD.timer = setInterval(() => post("/api/cmd", { cmd: "hold_renew", args: {} }, true), 150);
    const r = await start();
    if (!r.ok) { holdEnd(b, false); return; }
    HOLD.job = r.job;
    if (!HOLD.pressed) holdStop(r.job);            // released before the reply arrived
  });
  for (const ev of ["pointerup", "pointercancel", "lostpointercapture"]) b.addEventListener(ev, () => holdEnd(b, true));
  return b;
}
function holdStop(job) {
  holdCancelled.add(job);
  post("/api/stop");
}
function holdEnd(b, stopIfRunning) {
  if (HOLD.btn !== b || !HOLD.pressed) return;
  HOLD.pressed = false;
  clearInterval(HOLD.timer);
  b.classList.remove("held");
  if (stopIfRunning && HOLD.job != null) holdStop(HOLD.job);
  HOLD.btn = null;
  if (HOLD.job == null) return;                   // reply still pending: the pointerdown handler stops it
  HOLD.job = null;
}
window.addEventListener("pointerup", () => HOLD.btn && holdEnd(HOLD.btn, true));
window.addEventListener("blur", () => HOLD.btn && holdEnd(HOLD.btn, true));
function holdJobDone(id) { if (HOLD.job === id) HOLD.job = null; }
// set a numeric field; follow=false only writes untouched fields that do not have the focus
function setField(sel, v, mark, clear) {
  const el = $(sel);
  if (!mark && (el.classList.contains("edited") || document.activeElement === el)) return;
  el.value = (+v).toFixed(1);
  if (mark && !clear) el.classList.add("edited");
  if (clear) el.classList.remove("edited");
}
function jointDeltas(q) {
  const cur = (S && S.joints_deg) || [], moved = [];
  let big = "";
  q.forEach((v, i) => {
    const d = v - (cur[i] ?? v), n = INFO.joint_names[i].replace("_joint", "");
    if (Math.abs(d) >= 0.05) moved.push(`  ${n}: ${(cur[i] ?? 0).toFixed(1)}° → ${v.toFixed(1)}°  (${d > 0 ? "+" : ""}${d.toFixed(1)}°)`);
    if (Math.abs(d) > 30 && !big) big = n;
  });
  return { moved, big };
}
function freedriveButton() {
  const b = btn("Freedrive", "", () => cmd("freedrive", { on: !(S && S.freedrive) }),
    "Hand-guide the arm (UR freedrive). Ending it returns to the previous control mode.");
  b.dataset.freedrive = "1";
  return b;
}
function frameSelect(id) {
  const sel = h("select", { id });
  sel.dataset.frames = "1";
  return sel;
}

// ================================================================== OPERATE
pages.OPERATE = {
  build(el) {
    // Manual: one card, four tabs, one footer (Speed · Acceleration · Plan · Move) shared by Joints and Pose
    const jrows = INFO.joint_names.map((n, i) =>
      h("div", { class: "jrow" }, h("span", { class: "jname" }, jshort(n)),
        h("input", { type: "number", step: "0.5", id: `op-j${i}`, value: "0",
          oninput: (e) => e.target.classList.add("edited") }),
        h("span", { class: "cur", id: `op-jc${i}` }, "—")));
    const wp = h("select", { id: "op-wp", title: "Fill in the joints from a saved pose", onchange: (e) => {
      const w = INFO.waypoints.find((x) => x.name === e.target.value);
      if (w) w.joints_deg.forEach((v, i) => setField(`#op-j${i}`, v, true));
      this.loadedWp = w ? w.name : "";
      $("#op-wp-name").textContent = w ? w.name : "";
      e.target.value = "";
      if (S) this.wpGhost(S);
    } }, h("option", { value: "" }, "Go to pose…"), INFO.waypoints.map((w) => h("option", { value: w.name }, w.name)));
    const cfield = (i, n) => h("label", { class: "cfield" }, h("span", { class: "k" }, n),
      h("input", { type: "number", step: "1", id: `op-c${i}`, value: "0",
        oninput: (e) => { e.target.classList.add("edited"); this.fieldsToGoal(); } }));
    const tabs = h("div", { class: "tabs" },
      h("a", { class: "on", id: "op-tab-j", onclick: () => this.tab("j") }, "Joints"),
      h("a", { id: "op-tab-c", onclick: () => this.tab("c") }, "Pose"),
      h("a", { id: "op-tab-g", onclick: () => this.tab("g") }, "Jog"),
      h("a", { id: "op-tab-f", onclick: () => this.tab("f") }, "Force"));
    const useCur = (f, tip) => h("button", { class: "btn quiet ibtn", type: "button", title: tip, html: icon("reset") + "<span>Use current</span>", onclick: f });
    const manual = h("div", { class: "card manual" },
      h("div", { class: "row chead" }, h("span", { class: "ctitle grow" }, "Manual")), tabs,
      h("div", { id: "op-j" },
        h("div", { class: "jtable" },
          h("div", { class: "jrow jhead" }, h("span", {}, "Joint"), h("span", {}, "Target (°)"), h("span", {}, "Current")), jrows),
        h("div", { class: "row" }, wp, h("span", { class: "muted small grow", id: "op-wp-name" }),
          useCur(() => this.resetJoints(), "Discard your edits: every field follows the arm again. Edited fields are blue."))),
      h("div", { id: "op-c", class: "hidden" },
        h("div", { class: "row" }, h("span", { class: "muted" }, "Frame"),
          (() => { const f = frameSelect("op-frame"); f.onchange = () => this.resetCart(); return f; })(),
          h("div", { class: "grow" }),
          h("label", { class: "tchip", title: "Move the TCP along a straight line (Cartesian path) instead of a free joint move" },
            h("input", { type: "checkbox", id: "op-linear", onchange: () => this.mmsShow() }), "Straight line")),
        h("div", { class: "cgrid" },
          h("div", { class: "cgroup" }, h("div", { class: "cap" }, "Position (mm)"), ["X", "Y", "Z"].map((n, i) => cfield(i, n))),
          h("div", { class: "cgroup" }, h("div", { class: "cap" }, "Rotation (°)"), ["Roll", "Pitch", "Yaw"].map((n, i) => cfield(i + 3, n)))),
        h("div", { class: "row" }, h("div", { class: "grow" }),
          useCur(() => this.resetCart(), "Fill in where the TCP is now, in the selected frame"))),
      jogPanel(),
      forcePanel(),
      h("div", { class: "mfoot" },
        h("div", { class: "row", id: "op-motion" },
          h("label", { class: "nfield", title: "Velocity scaling, % of the joint limits" }, h("span", { class: "muted" }, "Speed"),
            h("input", { type: "number", id: "op-vel", value: "10", min: "1", max: "100" }), h("span", { class: "muted" }, "%")),
          h("label", { class: "nfield hidden", id: "op-mms-f", title: "Straight line only: the TCP moves at this constant speed " +
              `(mm/s, up to ${INFO.max_cart_speed_mm_s ?? 250}). Empty or 0: Speed % only` },
            h("input", { type: "number", id: "op-mms", min: "0", max: String(INFO.max_cart_speed_mm_s ?? 250), step: "1",
                         placeholder: "–" }), h("span", { class: "muted" }, "mm/s")),
          h("label", { class: "nfield", title: "Acceleration scaling, %" }, h("span", { class: "muted" }, "Accel."),
            h("input", { type: "number", id: "op-acc", value: "10", min: "1", max: "100" }), h("span", { class: "muted" }, "%")),
          h("div", { class: "grow" }),
          h("label", { class: "chk", title: "Ask before every move (large joint moves always ask)" },
            h("input", { type: "checkbox", id: "op-confirm", checked: !INFO.fake }), "Confirm")),
        h("div", { class: "banner-warn hidden", id: "op-refusal" }),
        h("div", { class: "row", id: "op-gorow" },
          btn("Preview", "", () => this.plan(), "Plan only and show it in the 3D view; nothing moves until Execute"),
          btn("Move", "primary grow", () => this.go(), "Plan and move")),
        // right under Preview: the plan just made (shown in the 3D view) and its Execute
        h("div", { class: "row hidden planrow", id: "op-planrow" },
          h("span", { class: "badge", id: "op-plan-info" }), h("div", { class: "grow" }),
          btn("Execute plan", "primary", () => this.execute(), "Run the previewed plan, if the arm is still at its start"),
          btn("Clear", "quiet", () => cmd("plan_clear")))));
    $("#op-gorow", manual).firstChild.id = "op-plan";
    $("#op-gorow", manual).lastChild.id = "op-go";
    $("#op-planrow", manual).querySelectorAll(".btn").forEach((b) => (b.dataset.lease = "1"));

    // 3D view (RViz-like): robot, goal gizmo, plan preview, force arrows. Its controls float on top of it; the TCP
    // pose is a small box in its corner. It is the first tile of OPERATE > Views (views.js).
    const v3box = h("div", { class: "v3box" },
      h("div", { class: "view3d", id: "v3" }, h("div", { class: "muted v3msg", id: "v3-msg" }, "Loading the robot model…")),
      this.viewBar(),
      h("div", { class: "v3ovl v3ik hidden", id: "v3-goal-ik" }, ""),
      this.poseCard());

    const modeSel = h("select", { id: "op-mode" });
    for (const m of INFO.modes.filter((m) => !m.hidden))
      modeSel.append(h("option", { value: m.name, disabled: !m.allowed },
        m.label + (m.direct_torque ? " (direct torque)" : "") + (m.allowed ? "" : " — disabled")));
    const robot = this.robotCard();
    // the modes as one segmented control: the active one is filled; picking another shows "Switch to …" (nothing
    // changes until that is pressed). The hidden select keeps the pick for switchMode().
    modeSel.classList.add("hidden");
    const modeSeg = h("div", { class: "mseg", id: "op-mseg" }, INFO.modes.filter((m) => !m.hidden).map((m) =>
      h("button", { type: "button", "data-mode": m.name, disabled: !m.allowed,
        title: m.allowed ? `${m.label}: ${m.interface} command interface` + (m.moveit ? " · MoveIt moves" : " · no MoveIt moves")
          + (m.direct_torque ? " · direct torque" : "") : m.disabled_reason || "Not available",
        onclick: () => { modeSel.value = m.name; this.modePick(); } }, m.label)),
      h("button", { type: "button", id: "op-fdseg", class: "fd", html: icon("hand") + "<span>Freedrive</span>",
        title: "Hand-guide the arm (UR freedrive). Ending it returns to the mode before.",
        onclick: () => cmd("freedrive", { on: true }) }));
    const stack = h("div", { class: "card armcard" },
      h("div", { class: "row chead" }, h("span", { class: "ctitle grow" }, "Arm control"),
        h("span", { class: "badge warn hidden", id: "op-exp" }, "Experimental"),
        h("span", { class: "chip hidden", id: "op-iface" })),
      h("span", { class: "hidden", id: "op-mode-title" }),
      modeSeg, modeSel,
      h("div", { class: "fdband hidden", id: "op-fdband" }, h("span", { html: icon("hand") }),
        h("span", { class: "grow", title: "Move the arm by hand. Ending it returns to the mode before." }, h("b", {}, "Freedrive on")),
        freedriveButton()),
      h("div", { class: "banner-bad hidden", id: "op-unknown" }),
      h("div", { class: "op-src", id: "op-source" }),
      h("div", { class: "swrow hidden", id: "op-switchrow" },
        h("span", { class: "grow", id: "op-switch-q" }),
        btn("Cancel", "quiet", () => { modeSel.value = S.mode; this.modePick(); }),
        btn("Switch", "primary", () => this.switchMode(), "Switch the arm to this control mode")),
      h("div", { class: "row hidden", id: "op-recoverrow" },
        btn("Re-verify", "", () => cmd("recover"), "After recovering the controllers by hand, check which mode is really active")));
    stack.querySelector("#op-switchrow").lastChild.id = "op-switch";
    stack.querySelector("#op-recoverrow").firstChild.id = "op-recover";

    const g = INFO.gripper;
    const grip = h("div", { class: "card" },
      h("div", { class: "row chead" }, h("span", { class: "ctitle grow", title: g.model || "" }, "Gripper"),
        g.enabled ? h("span", { class: "chip", id: "op-grip-chip" }, "—") : ""),
      g.enabled ? [
        h("div", { class: "row" }, h("span", { class: "muted", style: "width:40px" }, "Open"), h("div", { class: "bar grow" },
          h("div", { id: "op-grip-bar" })), h("span", { class: "muted" }, "Closed"),
          h("span", { class: "mono", id: "op-grip-pct", style: "width:38px;text-align:right" }, "—")),
        h("div", { class: "muted small", id: "op-grip-status" }),
        h("div", { class: "row" },
          btn("Open", "grow", () => cmd("gripper", { percent: 0, effort: gripForce() })),
          btn("Close", "grow", () => cmd("gripper", { percent: 100, effort: gripForce() }))),
        h("div", { class: "row gcustom" }, h("span", { class: "muted small" }, "To"),
          h("input", { type: "number", id: "op-grip-to", min: "0", max: "100", step: "5", value: "50", style: "width:52px",
            title: "% closed" }),
          h("span", { class: "muted" }, "%"),
          h("input", { type: "number", id: "op-grip-force", min: "1", max: String(g.effort_limit || g.max_effort),
            step: "5", value: String(g.max_effort), style: "width:52px", title: "Grip force, N" }),
          h("span", { class: "muted" }, "N"),
          btn("Go", "", () => cmd("gripper", { percent: +$("#op-grip-to").value, effort: gripForce() }), "Move to this % closed with this force")),
        g.can_reactivate ? h("div", { class: "row hidden", id: "op-grip-fix" },
          h("span", { class: "error grow", id: "op-grip-err" }),
          btn("Reactivate", "", () => cmd("gripper_reactivate"),
            "Clears a gripper fault (re-runs the Robotiq activation; the fingers open and close once)")) : "",
        g.can_reconnect ? h("div", { class: "row hidden", id: "op-grip-reconn" },
          h("span", { class: "error grow", id: "op-grip-reconn-why" }),
          btn("⟳ Reconnect gripper", "primary", () => cmd("gripper_reconnect"),
            "Brings the gripper hardware and its controllers up again. Needed when the robot stack started while the "
            + "robot was powered off: the gripper had no tool power and never connected")) : ""]
        : h("div", { class: "muted" }, "No gripper in this robot profile."));
    grip.querySelectorAll(".btn").forEach((b) => (b.dataset.lease = "1", b.dataset.grip = "1", b.dataset.tip = b.title || ""));

    const ftCell = (i) => h("div", { class: "ftc" }, h("span", { class: "n" }, AXES[i]),
      h("span", { class: "v", id: `op-ft${i}` }, "—"), h("div", { class: "cbar" }, h("div", { id: `op-ftb${i}` })));
    this.ftc = ftChart();
    const ft = h("div", { class: "card" },
      h("div", { class: "row chead" }, h("span", { class: "ctitle grow", title: `Frame: ${INFO.ft_frame || "sensor frame"}` }, "F/T sensor"),
        h("span", { class: "chip hidden", id: "op-ft-bias", title: "A software bias is set (SETUP › F/T + payload)" }, "Biased"),
        h("span", { class: "chip hidden", id: "op-ft-lim", title: "F/T limit (SETUP › F/T + payload): amber over the soft limit, red over the hard one" }, ""),
        h("button", { class: "btn quiet ibtn", type: "button", id: "op-ft-show", title: "Show the F/T chart under this card",
          html: icon("chart") + "<span>Chart</span>", onclick: () => this.vg.toggleChart() })),
      h("div", { class: "ft3" }, h("span", { class: "u" }, "N"), [0, 1, 2].map(ftCell),
        h("span", { class: "u" }, "Nm"), [3, 4, 5].map(ftCell)));
    this.ftc.el.classList.remove("card");

    this.jm = jointMonitor(INFO.joint_names, INFO.joint_limits_deg);
    // Joints: bars with limits, a list, or one line of angles (pick in the card's corner; kept per browser)
    const jv = ["bars", "list", "line"].includes(localStorage.getItem("op-joints")) ? localStorage.getItem("op-joints") : "bars";
    const jmCard = h("div", { class: "card jmcard", "data-v": jv },
      h("div", { class: "row chead" }, h("span", { class: "ctitle grow" }, "Joints"),
        h("span", { class: "tseg jvseg" }, [["bars", "Bars with limits"], ["list", "List"], ["line", "One line"]].map(([k, t]) =>
          h("button", { type: "button", class: k === jv ? "on" : "", title: t, html: icon(k), onclick: (e) => {
            jmCard.dataset.v = k; localStorage.setItem("op-joints", k);
            for (const b of e.currentTarget.parentNode.children) b.classList.toggle("on", b === e.currentTarget);
          } })))),
      h("div", { class: "jbars" }, this.jm.el), h("div", { class: "jsum", id: "op-jsum" }), h("div", { class: "jline", id: "op-jline" }));
    if (this.jm.unknownCount()) this.retryLimits(6);
    // the workspace and its layouts (views.js): left what you command and what the tool feels, middle the 3D view
    // and cameras, right the robot, arm mode, gripper and joints. The cards themselves stay as they are.
    // more windows for your own layouts: Motion planner, Joint chart, Program run, Log (panels.js)
    this.xv = extraViews({ h, btn, cmd, pcmd, INFO, getS: () => S, op: this, navTo, logs });
    this.vg = workspace({ h, cmd, toast, view3d: v3box, ftEl: this.ftc.el, ftBtn: $("#op-ft-show", ft), extra: this.xv,
      cards: { manual, fts: ft, robot, arm: stack, grip, joints: jmCard }, header: $("#h-layouts"), navSlot: $("#nav-lay"),
      onEdit: () => navTo("OPERATE") });
    el.append(this.vg.el);
    this.mode = "j";
    this.planId = -1;
    import("/static/view3d.js").then((m) => this.initView(m)).catch((e) => {
      const m = $("#v3-msg");
      if (m) m.textContent = "3D view unavailable in this browser: " + e.message;
      else console.warn("3D view unavailable:", e);
    });
  },
  // ---- camera presets, trail, follow, TF frames overlay (drawing is in view3d.js)
  viewBar() {
    const v = (f) => () => this.view && f(this.view);
    this.frameSel = new Set(JSON.parse(localStorage.getItem("v3-frames") || "null") || ["@base", "@tcp", "@fixtures"]);
    this.frameList = [];
    const pop = h("div", { class: "popover hidden", id: "v3-fpop" });
    document.addEventListener("click", (e) => { if (!e.composedPath().some((n) => n.classList && n.classList.contains("popwrap"))) pop.classList.add("hidden"); });
    const tog = (id, label, tip, checked, onchange) => h("label", { class: "tchip", title: tip },
      h("input", { type: "checkbox", id, checked, onchange }), label);
    return [
      h("div", { class: "v3ovl v3tl" },
        h("span", { class: "popwrap" },
          h("button", { class: "tchip", type: "button", title: "Show TF frames in the 3D view",
            onclick: () => { pop.classList.toggle("hidden"); this.framesRender(); this.framesPoll(true); } }, "Frames ▾"), pop),
        tog("v3-trail", "Trail", "Fading line of the last 10 s of TCP motion", true, (e) => this.view && this.view.setTrail(e.target.checked)),
        tog("v3-zones", "Zones", "Keep-out zones (MoveIt collision boxes, SETUP › Keep-out zones). Disabled ones dashed grey",
          localStorage.getItem("v3-zones") !== "0",
          (e) => { localStorage.setItem("v3-zones", e.target.checked ? "1" : "0"); if (S && this.view) this.view.setZones(S.keepout.boxes, e.target.checked); }),
        tog("v3-follow", "Follow TCP", "Keep the orbit centre on the TCP", false, (e) => this.view && this.view.setFollow(e.target.checked))),
      h("div", { class: "v3ovl v3tr" },
        h("span", { class: "tseg v3goalopts hidden", id: "v3-goalopts" },
          h("button", { type: "button", class: "on", id: "v3-gm-translate", title: "Drag the goal to move it",
            onclick: () => this.gizmoMode("translate") }, "Move"),
          h("button", { type: "button", id: "v3-gm-rotate", title: "Drag the goal to rotate it",
            onclick: () => this.gizmoMode("rotate") }, "Rotate"),
          h("button", { type: "button", title: "Put the goal on the TCP", onclick: () => this.goalToTcp() }, "To TCP")),
        tog("v3-goal", "Goal", "A goal for the TCP: drag it in the view, then Plan (Cartesian tab)", false,
          (e) => this.showGoal(e.target.checked)),
        h("span", { class: "tseg v3cams", title: "Camera" },
          ["Top", "Front", "Side", "Iso"].map((n) => h("button", { type: "button", title: `${n} view`,
            onclick: v((w) => w.setCameraPreset(n.toLowerCase())) }, n))))];
  },
  gizmoMode(m) {
    if (this.view) this.view.setGizmoMode(m);
    for (const k of ["translate", "rotate"]) $("#v3-gm-" + k).classList.toggle("on", k === m);
  },
  // the small card in the 3D view's corner: where the TCP is and the joint angles; each part folds away
  poseCard() {
    let st = { tcp: true, joints: false };
    try { st = { ...st, ...JSON.parse(localStorage.getItem("v3-pose") || "{}") }; } catch (e) { /* */ }
    const part = (k, title, body) => {
      const d = h("details", { class: "v3pp", open: st[k] }, h("summary", {}, title), body);
      d.addEventListener("toggle", () => { st[k] = d.open; localStorage.setItem("v3-pose", JSON.stringify(st)); });
      return d;
    };
    return h("div", { class: "v3pose" },
      part("tcp", h("span", { id: "op-tcpname" }, "TCP"), h("div", { class: "grid pose2", id: "op-pose" })),
      part("joints", "Joints", h("div", { class: "grid pose2 pj", id: "op-posej" })));
  },
  frameChecked(f) {
    const sel = this.frameSel;
    return sel.has(f.name) || (f.kind === "base" && sel.has("@base")) || (f.kind === "tcp" && f.active && sel.has("@tcp"))
      || ((f.kind === "fixture" || f.kind === "vision") && sel.has("@fixtures"));
  },
  frameToggle(name, on) {
    // first touch turns the group defaults into explicit names, so single frames can be unchecked
    if ([...this.frameSel].some((k) => k[0] === "@")) {
      this.frameSel = new Set(this.frameList.filter((f) => !f.error && this.frameChecked(f)).map((f) => f.name));
    }
    if (on) this.frameSel.add(name); else this.frameSel.delete(name);
    localStorage.setItem("v3-frames", JSON.stringify([...this.frameSel]));
    this.framesRender(); this.framesApply();
  },
  frameAll(on) {
    this.frameSel = new Set(on ? this.frameList.filter((f) => !f.error).map((f) => f.name) : []);
    localStorage.setItem("v3-frames", JSON.stringify([...this.frameSel]));
    this.framesRender(); this.framesApply();
  },
  framesRender() {
    const pop = $("#v3-fpop");
    pop.replaceChildren(
      h("div", { class: "row" }, btn("All", "quiet", () => this.frameAll(true)), btn("None", "quiet", () => this.frameAll(false))),
      ...this.frameList.map((f) => h("label", { class: "chk", title: f.error || `${f.kind}: ${f.xyz_mm} mm` },
        h("input", { type: "checkbox", checked: this.frameChecked(f) && !f.error, disabled: !!f.error,
          onchange: (e) => this.frameToggle(f.name, e.target.checked) }),
        f.name + (f.kind === "tcp" ? " (TCP)" : "") + (f.error ? " — unavailable" : ""))));
  },
  framesApply() {
    if (!this.view) return;
    const live = S && S.tcp, act = S && S.calibration.active_tcp;
    this.view.setFrames(this.frameList.filter((f) => !f.error && this.frameChecked(f)).map((f) =>
      f.kind === "tcp" && f.name === act && live ? { ...f, xyz_mm: live.xyz_mm, rpy_deg: live.rpy_deg } : f));
  },
  async framesPoll(force) {                        // ~2 Hz, only while something is checked and the page is shown
    if (document.hidden || !this.view) return;
    if (!force && !this.frameList.some((f) => !f.error && this.frameChecked(f))) return;
    const r = await cmd("tf_frames").catch(() => null);
    if (!r || !r.ok || !$("#v3-fpop")) return;
    const names = JSON.stringify(r.frames.map((f) => [f.name, !!f.error, f.active]));
    const changed = names !== this.frameNames;
    this.frameNames = names;
    this.frameList = r.frames;
    if (changed && !$("#v3-fpop").classList.contains("hidden")) this.framesRender();
    this.framesApply();
  },
  async initView(m) {
    this.v3mod = m;
    this.view = new m.View3D($("#v3"), { baseLink: INFO.base_link, onTargetChange: (p) => this.goalToFields(p) });
    this.framesPoll(true);
    clearInterval(this.framesTimer);
    this.framesTimer = setInterval(() => (document.body.contains($("#v3-fpop")) ? this.framesPoll() : clearInterval(this.framesTimer)), 500);
    for (let i = 0; i < 20; i++) {                           // the URDF arrives once robot_state_publisher runs
      const r = await fetch("/api/robot_model").then((x) => x.json()).catch(() => ({ ok: false, msg: "no answer" }));
      if (r.ok) { await this.view.load(r); $("#v3-msg").remove(); if (S) this.update(S); return; }
      $("#v3-msg").textContent = "Robot model: " + r.msg;
      await new Promise((res) => setTimeout(res, 3000));
    }
  },
  // ---- goal <-> Cartesian fields (both in the base frame)
  showGoal(on) {
    $("#v3-goalopts").classList.toggle("hidden", !on);      // Move / Rotate / To TCP only while the goal is shown
    if (!this.view) return;
    if (on && !this.goalSet) this.goalToTcp();
    this.view.showTarget(on);
    this.goalState();
  },
  goalToTcp() {
    if (!this.view || !S || !S.tcp) return;
    this.goalSet = true;
    this.view.goalQ = null;                    // re-seed the goal robot from the arm
    this.view.setTarget(S.tcp);
    this.goalToFields(S.tcp);
  },
  // RViz MotionPlanning's goal state: an orange arm at the IK solution for the goal, updated as it moves
  goalState() {
    if (this.goalPending) return;
    this.goalPending = true;
    requestAnimationFrame(() => {
      this.goalPending = false;
      const v = this.view, on = !!(v && $("#v3-goal").checked && S && S.joints_deg);
      const msg = $("#v3-goal-ik");
      if (!v) return;
      if (!on) { v.showGoalState(false); msg.classList.add("hidden"); return; }
      const seed = { ...this.extraJoints(S) }, lim = {};
      S.joints_deg.forEach((x, i) => (seed[INFO.joint_names[i]] = x * Math.PI / 180));
      (INFO.joint_limits_deg || []).forEach((l, i) => l && (lim[INFO.joint_names[i]] = [l[0] * Math.PI / 180, l[1] * Math.PI / 180]));
      const r = v.showGoalState(true, INFO.joint_names, seed, lim, S.tcp);
      msg.textContent = r && !r.ok ? "Goal out of reach (or past a joint limit): the orange arm shows the last pose that worked"
        : "Orange = goal state (IK in the page, no collision check: Plan checks)";
      msg.classList.toggle("error", !!(r && !r.ok));
      msg.classList.remove("hidden");
    });
  },
  goalToFields(p) {
    this.goalState();
    if (this.mode !== "c") this.tab("c");
    const f = $("#op-frame");
    if ([...f.options].some((o) => o.value === INFO.base_link)) f.value = INFO.base_link;
    [...p.xyz_mm, ...p.rpy_deg].forEach((v, i) => setField(`#op-c${i}`, v, true));
  },
  fieldsToGoal() {
    if (!this.view || !$("#v3-goal").checked || $("#op-frame").value !== INFO.base_link) return;
    const v = [0, 1, 2, 3, 4, 5].map((i) => +$(`#op-c${i}`).value);
    this.view.setTarget({ xyz_mm: v.slice(0, 3), rpy_deg: v.slice(3) });
    this.goalState();
  },
  plan() {
    const { vel, acc } = scaling();
    if (this.mode === "j")
      cmd("plan_joints", { q_deg: INFO.joint_names.map((_, i) => +$(`#op-j${i}`).value), vel, acc });
    else {
      const v = [0, 1, 2, 3, 4, 5].map((i) => +$(`#op-c${i}`).value);
      const linear = $("#op-linear").checked;
      cmd("plan_pose", { frame: $("#op-frame").value, xyz_mm: v.slice(0, 3), rpy_deg: v.slice(3),
                         linear, vel, acc, speed_mm_s: linear ? mmS() : null });
    }
  },
  async execute() {
    if (await confirmMove("Execute the previewed plan (" + (S.plan.what || "") + ")")) cmd("plan_execute");
  },
  async refreshPlan(s) {
    if (!s.plan.what && this.planId !== -2) { this.planId = -2; this.view && this.view.clearPreview(); }   // used / gone
    if (s.plan.id === this.planId || !s.plan.what) return;
    this.planId = s.plan.id;
    const id = s.plan.id;
    const r = await post("/api/cmd", { cmd: "plan_get", args: {} }, true);
    // the plan may have been executed or cleared while this was on its way: never start a stale preview
    if (r.ok && r.plan && this.view && S.plan.id === id && S.plan.what && this.planId === id)
      this.view.playPreview(r.plan, INFO.joint_names, this.extraJoints(S));
  },
  async replayPreview() {
    if (!S.plan.what || !this.view) return;
    const id = S.plan.id, r = await post("/api/cmd", { cmd: "plan_get", args: {} }, true);
    if (r.ok && r.plan && S.plan.id === id && S.plan.what) this.view.playPreview(r.plan, INFO.joint_names, this.extraJoints(S));
  },
  // The saved pose picked in the Joints tab, as a ghost arm: while the fields still hold it and the arm is not there
  // yet. A plan preview uses the same ghost, so it gives way to one; Use current or a finished MoveJ clears it.
  wpGhost(s) {
    if (!this.view || !this.view.ghost) return;             // the robot model is not loaded yet
    const w = this.loadedWp && INFO.waypoints.find((x) => x.name === this.loadedWp);
    const held = !!w && w.joints_deg.every((v, i) => Math.abs(+$(`#op-j${i}`).value - v) < 0.051);
    const there = held && !!s.joints_deg && w.joints_deg.every((v, i) => Math.abs(s.joints_deg[i] - v) < 0.2);
    const key = held && !there && !s.plan.what ? w.name : "";
    if (key === this.ghostWp && this.view.ghost === this.ghostOf) return;   // (a model reload makes a new ghost)
    this.ghostWp = key; this.ghostOf = this.view.ghost;
    const q = {};
    if (key) w.joints_deg.forEach((v, i) => (q[INFO.joint_names[i]] = v * Math.PI / 180));
    this.view.showGhost(key ? { ...this.extraJoints(s), ...q } : null);
  },
  extraJoints(s) {
    return s.gripper_joint && s.gripper_position != null ? { [s.gripper_joint]: s.gripper_position } : {};
  },
  resetJoints() {
    INFO.joint_names.forEach((_, i) => $(`#op-j${i}`).classList.remove("edited"));
    this.loadedWp = ""; $("#op-wp-name").textContent = "";
    if (S && S.joints_deg) S.joints_deg.forEach((v, i) => setField(`#op-j${i}`, v, false));
  },
  resetCart() {
    [0, 1, 2, 3, 4, 5].forEach((i) => $(`#op-c${i}`).classList.remove("edited"));
    this.cartAt = 0;
    if (S) this.followCart(S);
  },
  // untouched Cartesian fields follow the TCP in the selected frame (state has it in the base frame;
  // other frames are asked for at most once a second)
  async followCart(s) {
    const frame = $("#op-frame").value;
    let p = frame === INFO.base_link && s.tcp && !s.tcp.stale ? s.tcp : null;
    if (!p && frame && Date.now() - (this.cartAt || 0) > 1000) {
      this.cartAt = Date.now();
      const r = await post("/api/cmd", { cmd: "current_pose", args: { frame } }, true);
      if (r.ok && $("#op-frame").value === frame) p = r;
    }
    if (p) [...p.xyz_mm, ...p.rpy_deg].forEach((v, i) => setField(`#op-c${i}`, v, false));
  },
  // the jog panel is one set of controls shared with PROGRAMS (moved there while PROGRAMS is open)
  show() { dockJog(null); this.vg.enter(); },
  hide() { this.vg.leave(); },
  tab(t) {
    this.mode = t;
    $("#op-j").classList.toggle("hidden", t !== "j");
    $("#op-c").classList.toggle("hidden", t !== "c");
    $("#op-g").classList.toggle("hidden", t !== "g");
    $("#op-f").classList.toggle("hidden", t !== "f");
    for (const k of ["j", "c", "g", "f"]) $(`#op-tab-${k}`).classList.toggle("on", t === k);
    $("#op-go").textContent = "Move";
    $("#op-go").title = t === "j" ? "Plan and move: a joint move (MoveJ)" : "Plan and move the TCP to this pose";
    $("#op-gorow").classList.toggle("hidden", t === "g" || t === "f");
    $("#op-motion").classList.toggle("hidden", t === "f" || t === "g");   // Jog has its own speed; a force has none
    this.mmsShow();
    if (t !== "g") jogRelease();
  },
  mmsShow() {             // the mm/s field: Pose tab, straight line only
    $("#op-mms-f").classList.toggle("hidden", this.mode !== "c" || !$("#op-linear").checked);
  },
  async go() {
    const { vel, acc } = scaling();
    if (this.mode === "j") {
      const q = INFO.joint_names.map((_, i) => +$(`#op-j${i}`).value);
      const d = jointDeltas(q);
      if (!d.moved.length) { toast("MoveJ: the target is where the arm already is (edit a joint first)", true); return; }
      const text = "MoveJ" + (this.loadedWp ? ` to ${this.loadedWp}` : "") + " — joints that move:\n" + d.moved.join("\n")
        + (d.big ? `\n\nWARNING: ${d.big} moves more than 30°.` : "") + `\n\nVelocity ${Math.round(vel * 100)} %`;
      // a large move always asks, even with "Confirm moves" off
      if (d.big ? (await ask(text + "\n\nExecute?")) : await confirmMove(text))
        cmd("move_joints", { q_deg: q, vel, acc });
    } else {
      const v = [0, 1, 2, 3, 4, 5].map((i) => +$(`#op-c${i}`).value);
      const frame = $("#op-frame").value, linear = $("#op-linear").checked, mms = linear ? mmS() : null;
      if (await confirmMove(`${linear ? "Linear move" : "Move"}: TCP ${S.calibration.active_tcp} to ${v.slice(0, 3)} mm in ${frame}`
                            + (mms ? ` at ≤ ${mms} mm/s` : "")))
        cmd("move_pose", { frame, xyz_mm: v.slice(0, 3), rpy_deg: v.slice(3), linear, vel, acc, speed_mm_s: mms });
    }
  },
  modePick() {
    const pick = $("#op-mode").value, cur = S && S.mode;
    for (const b of $("#op-mseg").children) {
      b.classList.toggle("on", b.dataset.mode === cur);
      b.classList.toggle("pick", b.dataset.mode === pick && pick !== cur);
    }
    const m = INFO.modes.find((x) => x.name === pick);
    const fd = !!(S && S.freedrive);
    $("#op-switchrow").classList.toggle("hidden", !m || pick === cur || fd);
    if (m) $("#op-switch-q").innerHTML = `Switch to <b>${m.label}</b>?` + (m.direct_torque ? ' <span class="badge warn">Direct torque</span>' : "");
  },
  async switchMode() {
    const name = $("#op-mode").value, m = INFO.modes.find((x) => x.name === name);
    if (!m) return;
    if (m.direct_torque && !(await ask(`Enter ${m.label}?\n\nThis commands DIRECT TORQUE on the robot.`, { ok: "Enter", danger: true }))) return;
    cmd("switch_mode", { mode: name });
  },
  // Robot power through the UR Dashboard (pendant in Remote Control), as three steps: Power on → Release brakes →
  // ▶ Play program. Power on and brake release from any browser with control and "I can see the robot", unless
  // the robot PC limits them to its own browser (MOVEIT_GUI_LOCAL_POWER_ONLY): the server checks that too.
  robotCard() {
    const seen = "Check that you can see the robot, nobody is in its reach and the workspace is clear.";
    this.pOn = btn("Power on", "", async () => (await ask(`Power on the robot?\n\n${seen}`)) && cmd("power_on", { seen: true }),
      "Power on the arm (UR Dashboard). The brakes stay engaged.");
    this.pBrake = btn("Release brakes", "", async () => (await ask(`Release the brakes?\n\nThe joints move slightly and click.\n${seen}`)) &&
      cmd("brake_release", { seen: true }), "Release the brakes: the robot goes to RUNNING");
    this.pOff = btn("Power off…", "quiet", async () => (await ask("Power off the robot?\n\nGUI motion is stopped first; the brakes engage. " +
      (INFO.remote_power ? "Power on again here when you can see the robot." :
      "Powering on again needs a browser on the robot PC or the pendant."))) && cmd("power_off"), "Power off the arm (UR Dashboard)");
    const play = async () => {
      const paused = S.program_state === "PAUSED";
      const r = await cmd("robot_loaded_program");      // play starts whatever is loaded: show it by name first
      const gui = S.runner && S.runner.state === "PAUSED" && S.runner.pause_reason ? "\nThe paused GUI program carries on with its step." : "";
      if (r && r.ok && (await ask(paused ? `Resume ${r.program} on the robot?${gui}` :
        `Start ${r.program} on the robot?\n\nIt must be your External Control program. ` +
        "Pending GUI motions are cancelled first; the GUI can command the arm once it runs." + gui))) cmd("robot_play", { program: r.program });
    };
    this.pPlay = btn("▶ Play program", "", play,
      "Start the program loaded on the pendant (External Control). In Remote Control the pendant's Play button is disabled.");
    this.pResume = btn("▶ Resume program", "primary", play, "Continue the paused robot program");
    this.pPause = btn("⏸ Pause program", "", () => cmd("robot_pause"),
      "Pause the robot program: the arm halts, power and brakes stay on. A GUI program run pauses with it.");
    this.pStopProg = btn("■ Stop program", "quiet", async () => (await ask("Stop the robot program?\n\nThe arm halts; power and brakes stay on. " +
      "▶ Play starts it again from the beginning. A GUI program run pauses and can resume after Play.")) && cmd("robot_stop_program"),
      "Stop the program on the pendant (External Control): power and brakes stay on");
    const step = (n, b, label) => {
      const el = h("div", { class: "pstep", id: `op-ps${n}` }, h("span", { class: "pnum" }, String(n)),
        INFO.local || INFO.remote_power || n === 3 ? b : h("span", { class: "muted" }, label + " (robot PC / pendant)"));
      return el;
    };
    [this.pOn, this.pBrake, this.pPlay, this.pOff, this.pResume, this.pPause, this.pStopProg].forEach((b) => (b.dataset.lease = "1"));
    // icons + short words; the full explanation is in each button's hover text
    const ib = (b, ic, t) => { b.innerHTML = icon(ic) + (t ? `<span>${t}</span>` : ""); b.classList.add("ibtn"); return b; };
    ib(this.pOff, "power", ""); this.pOff.title = "Power off… (UR Dashboard)";
    ib(this.pPause, "pause", "Pause pendant"); ib(this.pStopProg, "stop", "Stop pendant"); ib(this.pResume, "play", "Resume");
    ib(this.pPlay, "play", "Play program");
    return h("div", { class: "card", id: "op-robot" },
      h("div", { class: "row chead" }, h("span", { class: "ctitle grow" }, "Robot"),
        h("span", { class: "chip", id: "op-remote" }, "—"), this.pOff),
      h("div", { class: "rhead" }, h("span", { class: "dot", id: "op-rdot" }),
        h("div", { class: "grow" }, h("div", { class: "rstate", id: "op-rstate" }, "—"), h("div", { class: "muted small", id: "op-rsub" })),
        h("div", { class: "row", id: "op-progctl" }, this.pResume, this.pPause, this.pStopProg)),
      h("div", { class: "psteps" }, step(1, this.pOn, "Power on"), step(2, this.pBrake, "Release brakes"), step(3, this.pPlay, "Play program")));
  },

  robotUpdate(s) {
    const rm = s.robot_mode || "?", prog = s.program_running, paused = prog !== true && s.program_state === "PAUSED";
    const powered = ["IDLE", "RUNNING"].includes(rm), running = rm === "RUNNING";
    const st = $("#op-rstate");
    const label = { POWER_OFF: "Powered off", IDLE: "Powered on · brakes engaged", RUNNING: "Running",
      BOOTING: "Booting…", POWER_ON: "Powering on…", DISCONNECTED: "Not connected", CONFIRM_SAFETY: "Confirm safety on the pendant",
      BACKDRIVE: "Backdrive", "?": "Robot mode unknown" }[rm] || rm;
    const safety = s.safety_mode && s.safety_mode !== "NORMAL" ? nice(s.safety_mode) : "";
    st.textContent = safety || label;
    $("#op-rsub").textContent = safety ? label : running ? (prog === true ? "Pendant program running" : s.program_state === "PLAYING"
      ? "A pendant program plays, but not External Control for this PC (set its Host IP to this PC)" : paused
      ? "Pendant program paused" : "Pendant program stopped") : "";
    const cls = safety ? "bad" : running && prog === true ? "ok" : "warn";
    st.className = "rstate " + cls;
    $("#op-rdot").className = "dot " + cls;
    // the next step is the primary button; done steps show ✓
    const done = [powered, running, running && prog === true];
    const next = done.indexOf(false);
    [1, 2, 3].forEach((n) => {
      const el = $(`#op-ps${n}`);
      el.classList.toggle("done", done[n - 1]);
      el.classList.toggle("next", next === n - 1);
      el.querySelector(".pnum").textContent = done[n - 1] ? "✓" : String(n);
    });
    [this.pOn, this.pBrake, this.pPlay].forEach((b, i) => b.classList.toggle("primary", next === i));
    // all done, or paused: the state line says it, and the program buttons below take over
    $("#op-robot .psteps").classList.toggle("hidden", next === -1 || (running && paused));
    const playing = running && prog === true;
    $("#op-progctl").classList.toggle("hidden", !(playing || (running && paused)));
    this.pResume.classList.toggle("hidden", !paused);
    this.pPause.classList.toggle("hidden", paused);
    gate(this.pPause, playing);
    gate(this.pStopProg, playing || paused);
    gate(this.pResume, running && paused);
    gate(this.pOn, s.idle && !["POWER_ON", "IDLE", "RUNNING", "BOOTING"].includes(rm));
    gate(this.pBrake, s.idle && (rm === "IDLE" || rm === "?"));
    gate(this.pOff, rm !== "POWER_OFF");
    gate(this.pPlay, (s.idle || (s.runner && s.runner.state === "PAUSED")) && running && prog !== true);   // a paused GUI run may wait
    // what the robot says about Local / Remote Control (asked every few seconds), not a fixed reminder
    const rc = $("#op-remote");
    rc.textContent = s.remote_control === true ? "Remote control" : s.remote_control === false ? "Local control" : "Pendant ?";
    rc.title = s.remote_control === true ? "The pendant is in Remote Control: this GUI can power on, release brakes and play" :
      s.remote_control === false ? "The pendant is in Local Control: power, brakes and Play only work in Remote Control" :
      "Pendant control mode unknown (Dashboard not answering)";
    rc.className = "chip " + (s.remote_control === false ? "warn" : "");
  },
  retryLimits(n) {   // the URDF can arrive after the page loaded; ask /api/info again a few times
    setTimeout(async () => {
      try {
        const r = await (await fetch("/api/info")).json();
        INFO.joint_limits_deg = r.joint_limits_deg;
        this.jm.setLimits(r.joint_limits_deg);
      } catch (e) { /* offline: try again */ }
      if (this.jm.unknownCount() && n > 1) this.retryLimits(n - 1);
    }, 5000);
  },
  update(s) {
    if (this.jm) this.jm.update(s);
    if (this.ftc) this.ftc.push(s);
    if (this.xv) for (const x of Object.values(this.xv)) x.update(s);
    if (this.vg) this.vg.update(s);
    // Untouched fields follow the arm, so a MoveJ never sends a joint back to a stale value.
    if (s.joints_deg) s.joints_deg.forEach((v, i) => setField(`#op-j${i}`, v, false));
    if (this.mode === "c") this.followCart(s);
    const refusal = s.motion_refusal;
    gate($("#op-go"), !refusal);
    gate($("#op-plan"), !s.busy);
    jogUpdate(s);
    forceUpdate(s);
    const why = refusal && s.idle && !"gf".includes(this.mode) ? refusal : "";   // Jog, Force: own note
    $("#op-refusal").textContent = why ? "Cannot move: " + why : "";
    $("#op-refusal").classList.toggle("hidden", !why);
    $("#op-planrow").classList.toggle("hidden", !s.plan.what);
    $("#op-go").classList.toggle("primary", !s.plan.what);       // with a plan, Execute is THE button
    if (s.plan.what) $("#op-plan-info").textContent = `PLAN · ${s.plan.what} · ${s.plan.duration_s.toFixed(1)} s`;
    $("#op-planrow").querySelectorAll(".btn")[0].disabled = !(holds && !refusal);
    if (this.view) {
      const q = {};
      (s.joints_deg || []).forEach((v, i) => (q[INFO.joint_names[i]] = v * Math.PI / 180));
      this.view.setJoints({ ...q, ...this.extraJoints(s) }, s.tcp_off);
      this.view.setMeasuredForce(s.wrench, INFO.ft_frame);
      this.view.pushTcp(s.tcp);
      if (this.frameList.length) this.framesApply();
      this.view.setZones(s.keepout.boxes, $("#v3-zones").checked);
      this.view.setCommandedForce(s.force.active ? s.force.current : null, s.force.frame, s.tcp);
      this.refreshPlan(s);
      this.wpGhost(s);
    }
    (s.joints_deg || []).forEach((v, i) => ($(`#op-jc${i}`).textContent = v.toFixed(1) + "°"));
    $("#op-tcpname").textContent = `TCP ${s.calibration.active_tcp} in ${INFO.base_link}`;
    const jd = s.joints_deg || [];
    $("#op-posej").innerHTML = INFO.joint_names.map((n, i) =>
      `<span class="k">${jshort(n)}</span><span class="v">${jd[i] == null ? "—" : jd[i].toFixed(1)} °</span>`).join("");
    $("#op-jline").innerHTML = INFO.joint_names.map((n, i) =>
      `<span title="${jshort(n)}"><span class="k">${["B", "S", "E", "W1", "W2", "W3"][i] || i + 1}</span>${jd[i] == null ? "—" : jd[i].toFixed(1)}</span>`).join("") + "<span class='u'>°</span>";
    $("#op-jsum").innerHTML = INFO.joint_names.map((n, i) =>
      `<span><span class="k">${jshort(n)}</span><span class="v">${jd[i] == null ? "—" : jd[i].toFixed(1)}°</span></span>`).join("");
    const p = s.tcp;
    // X Roll / Y Pitch / Z Yaw side by side: one short block instead of six lines
    $("#op-pose").classList.toggle("muted", !!(p && p.stale));
    $("#op-pose").title = p && p.stale ? "TF is late: the last pose, held for up to 2 s" : "";
    $("#op-pose").innerHTML = p ? [0, 1, 2].map((i) =>
      `<span class="k">${"XYZ"[i]}</span><span class="v">${fmt(p.xyz_mm[i])} mm</span>` +
      `<span class="k">${["Roll", "Pitch", "Yaw"][i]}</span><span class="v">${fmt(p.rpy_deg[i])} °</span>`).join("")
      : [0, 1, 2].map((i) => `<span class="k">${"XYZ"[i]}</span><span class="v muted">— mm</span>` +
        `<span class="k">${["Roll", "Pitch", "Yaw"][i]}</span><span class="v muted">— °</span>`).join("");   // same size, no jump
    const m = INFO.modes.find((x) => x.name === s.mode);
    $("#op-mode-title").textContent = m ? m.label : (!s.mode_label || /^(none|unknown)$/i.test(s.mode_label) ?
      "No arm controller active" : s.mode_label);
    $("#op-exp").classList.toggle("hidden", !(m && m.experimental));
    // only what matters: the command interface is in the hover text
    $("#op-iface").textContent = m && m.direct_torque ? "Direct torque" : "";
    $("#op-iface").classList.toggle("hidden", !(m && m.direct_torque));
    $("#op-mseg").classList.toggle("hidden", !!s.freedrive);
    $("#op-fdband").classList.toggle("hidden", !s.freedrive);
    gate($("#op-fdseg"), s.idle && !s.unknown);
    $("#op-mode-title").title = m ? `${m.interface} command interface` + (m.moveit ? " · MoveIt moves available" : "") : "";
    $("#op-unknown").classList.toggle("hidden", !s.unknown);
    $("#op-unknown").textContent = "CONTROL STATE UNKNOWN. " + (s.unknown_reason || "") + " Recover the controllers by hand, then Re-verify.";
    $("#op-source").textContent = s.command_source !== "idle" ? "Commanding: " + s.command_source : "";
    $("#op-source").classList.toggle("hidden", s.command_source === "idle" || !!s.freedrive);
    // Follow the robot only when its mode changes: the user's pick must survive until Switch is pressed
    // (clicking Switch takes the focus off the select, so a focus check is not enough).
    if (s.mode !== this.shownMode) {
      this.shownMode = s.mode;
      if (m && !m.hidden) $("#op-mode").value = s.mode;
    }
    gate($("#op-switch"), s.idle && !s.unknown && !s.freedrive);
    this.modePick();
    $("#op-switch").className = "btn primary " + (INFO.modes.find((x) => x.name === $("#op-mode").value)?.direct_torque ? "experimental" : "");
    gate($("#op-recover"), true);
    this.robotUpdate(s);
    $("#op-recoverrow").classList.toggle("hidden", !s.unknown);
    if (INFO.gripper.enabled) {
      const g = INFO.gripper, pos = s.gripper_position;
      const pct = pos == null ? null : 100 * (pos - g.open_position) / ((g.closed_position - g.open_position) || 1);
      $("#op-grip-bar").style.width = (pct == null ? 0 : Math.max(0, Math.min(100, pct))) + "%";
      $("#op-grip-pct").textContent = pct == null ? "—" : pct.toFixed(0) + "%";
      const gs = s.gripper_status;
      const ctrlBad = s.gripper_controller && s.gripper_controller !== "active";
      const fault = gs && gs.fault_code && gs.fault_code !== 255;
      const chip = $("#op-grip-chip");
      const [txt, cls] = !s.connected.Gripper ? ["Not connected", "bad"] : ctrlBad ? ["Controller " + s.gripper_controller, "bad"]
        : fault ? ["Fault", "bad"] : gs && gs.holding ? ["Holding object", "good"] : ["Ready", "good"];
      chip.textContent = txt; chip.className = "chip " + cls;
      $("#op-grip-status").textContent = gs ? `${gs.object.charAt(0).toUpperCase()}${gs.object.slice(1)} · ${gs.current_a.toFixed(2)} A` : "";
      const rc = $("#op-grip-reconn");
      if (rc) {
        const down = !s.connected.Gripper || ctrlBad;
        rc.classList.toggle("hidden", !down);
        const off = !["IDLE", "RUNNING", "?"].includes(s.robot_mode);
        $("#op-grip-reconn-why").textContent = !down ? "" : off ? "Gripper not connected: power the robot on first"
          : "Gripper not connected (the stack started while the robot was off?)";
      }
      const fix = $("#op-grip-fix");
      if (fix) {
        fix.classList.toggle("hidden", !(fault || (s.connected.Gripper && ctrlBad && !$("#op-grip-reconn"))));
        $("#op-grip-err").textContent = fault ? `Fault: ${gs.fault} (${gs.severity})` : ctrlBad ? `controller ${s.gripper_controller}` : "";
      }
    }
    const w = s.wrench;
    $("#op-ft-bias").classList.toggle("hidden", !(s.ft_bias && s.ft_bias.bias && !s.ft_bias.raw));
    const G = s.ft_guard, lim = $("#op-ft-lim");
    lim.classList.toggle("hidden", !(G && G.enabled));
    if (G && G.enabled) {
      lim.className = "chip" + (G.level === 2 ? " err" : G.level === 1 ? " warn" : "");
      lim.textContent = G.hard_f ? `|F| ${G.f.toFixed(0)} / ${G.hard_f} N` : `|T| ${G.t.toFixed(1)} / ${G.hard_t} Nm`;
      const t = G.last_trip;
      if (t && t.at !== this.ftTripSeen) {
        if (this.ftTripSeen !== undefined && t.stopped) toast(`F/T hard limit: ${t.why}. The GUI's motion was stopped.`, false, true);
        this.ftTripSeen = t.at;
      }
    }
    AXES.forEach((_, i) => {
      $(`#op-ft${i}`).textContent = w ? (w[i] >= 0 ? "+" : "") + w[i].toFixed(i < 3 ? 2 : 3) : "—";
      const lim = i < 3 ? 50 : 5, x = w ? Math.max(-1, Math.min(1, w[i] / lim)) : 0;
      const b = $(`#op-ftb${i}`);
      b.style.left = (x < 0 ? 50 + 50 * x : 50) + "%";
      b.style.width = Math.abs(50 * x) + "%";
    });
  },
};

function gripForce() {
  const g = INFO.gripper, f = +$("#op-grip-force").value;
  return f > 0 ? Math.min(f, g.effort_limit || g.max_effort) : g.max_effort;
}

// --------------------------------------------------------------------- jog
// Step: one click = one short MoveIt move. Hold: the robot PC streams MoveIt Servo while this page renews the
// hold every 100 ms; the robot PC stops 0.5 s after the last renewal, so a lost network stops the arm too.
const JOG = { down: false, timer: null, labelled: false, slider: null, pending: false, sequence: 0 };
const CART = ["X", "Y", "Z", "Rx", "Ry", "Rz"];

function jogPanel() {
  const type = h("select", { id: "op-jog-type", onchange: () => { jogRelease(); jogLabels(); } },
    h("option", { value: "base" }, "Cartesian · base"), h("option", { value: "tool" }, "Cartesian · tool"),
    h("option", { value: "joint" }, "Joint"));
  const step = h("select", { id: "op-jog-step", onchange: () => { jogRelease(); jogLabels(); } },
    h("option", { value: "hold", selected: true }, "Continuous (Servo)"),
    INFO.jog.step_mm.map((v, i) => h("option", { value: i }, `${v} mm · ${INFO.jog.step_deg[i]} °`)));
  const n = Math.max(6, INFO.joint_names.length);
  const rows = [...Array(n).keys()].map((i) => h("div", { class: "jogrow" },
    h("span", { class: "muted", id: `op-jl${i}` }), jogButton(i, -1), jogButton(i, +1)));
  const sliders = [...Array(n).keys()].map((i) => {
    const input = h("input", { type: "range", min: "-100", max: "100", step: "1", value: "0",
      id: `op-js${i}`, "aria-label": `Jog axis ${i + 1}`, "data-jog-slider": "1" });
    input.addEventListener("pointerdown", (e) => jogSliderBegin(input, e));
    input.addEventListener("input", () => jogSliderInput(input));
    input.addEventListener("pointerup", () => jogRelease());
    input.addEventListener("pointercancel", () => jogRelease());
    input.addEventListener("lostpointercapture", () => { if (JOG.slider === input) jogRelease(); });
    input.addEventListener("keydown", (e) => {
      if (["ArrowLeft", "ArrowRight", "ArrowUp", "ArrowDown", "Home", "End", "PageUp", "PageDown"].includes(e.key)) {
        if (!JOG.slider) JOG.slider = input;
      }
    });
    input.addEventListener("keyup", () => { if (JOG.slider === input) jogRelease(); });
    input.addEventListener("blur", () => { if (JOG.slider === input) jogRelease(); });
    return h("div", { class: "jogsliderrow" },
      h("span", { class: "muted", id: `op-jsl${i}` }),
      h("div", { class: "jogtrack" }, h("span", { class: "end" }, "−"), input, h("span", { class: "end" }, "+")),
      h("output", { class: "mono", id: `op-jsv${i}`, for: `op-js${i}` }, "0"));
  });
  return h("div", { id: "op-g", class: "hidden" },
    h("div", { class: "row" }, type, step),
    h("div", { class: "jog", id: "op-jog-steps" }, rows),
    h("div", { class: "jog hidden", id: "op-jog-sliders" }, sliders),
    h("div", { class: "muted", id: "op-jog-note" }));
}

function jogButton(axis, sign) {
  const b = h("button", { class: "btn jogbtn" }, sign < 0 ? "−" : "+");
  b.dataset.jog = "1";
  b.addEventListener("pointerdown", (e) => { e.preventDefault(); jogPress(axis, sign); });
  for (const ev of ["pointerup", "pointerleave", "pointercancel"]) b.addEventListener(ev, () => JOG.down && jogRelease());
  b.addEventListener("contextmenu", (e) => e.preventDefault());       // long press on a touch screen
  return b;
}

function jogLabels() {
  const joint = $("#op-jog-type").value === "joint";
  const continuous = $("#op-jog-step").value === "hold";
  $("#op-jog-steps").classList.toggle("hidden", continuous);
  $("#op-jog-sliders").classList.toggle("hidden", !continuous);
  const n = Math.max(6, INFO.joint_names.length);
  for (let i = 0; i < n; i++) {
    const row = $(`#op-jl${i}`).parentElement;
    const sliderRow = $(`#op-jsl${i}`).parentElement;
    const name = joint ? INFO.joint_names[i] : CART[i];
    row.classList.toggle("hidden", !name);
    sliderRow.classList.toggle("hidden", !name);
    // joints: just J1…J6 (the full name in the tooltip), so the sliders get the width
    const full = !name ? "" : joint ? `J${i + 1} ${name.replace("_joint", "")}` : name;
    for (const id of [`#op-jl${i}`, `#op-jsl${i}`]) { $(id).textContent = !name ? "" : joint ? `J${i + 1}` : name; $(id).title = full; }
    $(`#op-js${i}`).setAttribute("aria-label", `${full} velocity (-100 to 100)`);
  }
}

function jogSliderBegin(input, e) {
  if (input.disabled || !holds) return;
  if (JOG.slider && JOG.slider !== input) jogRelease();
  JOG.slider = input;
  if (e.pointerId != null && input.setPointerCapture) input.setPointerCapture(e.pointerId);
}

function jogSliderValues() {
  return [...document.querySelectorAll("[data-jog-slider]")]
    .filter((x) => !x.closest(".jogsliderrow").classList.contains("hidden"))
    .map((x) => Number(x.value) / 100);
}

async function jogSliderInput(input) {
  if (JOG.slider !== input || input.disabled || !holds) { input.value = "0"; return; }
  $(`#op-jsv${input.id.slice(5)}`).textContent = input.value;
  if (Number(input.value) === 0 || JOG.down || JOG.pending) return;
  const sequence = ++JOG.sequence;
  JOG.pending = true;
  const t = $("#op-jog-type").value;
  const r = await cmd("jog_start", { kind: t === "joint" ? "joint" : "cartesian",
    frame: t === "tool" ? "tool" : "base", values: jogSliderValues(), speed: scaling().vel });
  JOG.pending = false;
  if (!r.ok || sequence !== JOG.sequence || JOG.slider !== input) {
    if (r.ok) post("/api/cmd", { cmd: "jog_stop", args: {} });
    if (!r.ok) jogRelease();
    return;
  }
  JOG.down = true;
  const first = await cmd("jog_hold", { values: jogSliderValues() });
  if (!first.ok || !first.jogging || sequence !== JOG.sequence || JOG.slider !== input) {
    jogRelease();
    return;
  }
  JOG.timer = setInterval(async () => {
    if (!JOG.down) return;
    const reply = await cmd("jog_hold", { values: jogSliderValues() });
    if (!reply.ok || !reply.jogging) jogRelease();
  }, 50);
}

function jogArgs(axis, sign) {
  const t = $("#op-jog-type").value;
  return { kind: t === "joint" ? "joint" : "cartesian", axis, sign, frame: t === "tool" ? "tool" : "base" };
}

async function jogPress(axis, sign) {
  const { vel, acc } = scaling();
  const step = $("#op-jog-step").value;
  if (step !== "hold") {
    const args = jogArgs(axis, sign);
    const size = args.kind === "cartesian" && axis < 3 ? INFO.jog.step_mm[+step] : INFO.jog.step_deg[+step];
    cmd("jog_step", { ...args, step: size, vel, acc });
    return;
  }
  jogRelease();
  JOG.down = true;
  const r = await cmd("jog_start", { ...jogArgs(axis, sign), speed: vel });
  if (!r.ok || !JOG.down) { if (r.ok) jogRelease(); else JOG.down = false; return; }
  const first = await cmd("jog_hold");
  if (!first.ok || !first.jogging || !JOG.down) { jogRelease(); return; }
  JOG.timer = setInterval(async () => {
    const h = await cmd("jog_hold");
    if (!h.ok || !h.jogging) jogRelease();
  }, 50);
}

function jogRelease() {
  const was = JOG.down || JOG.timer || JOG.pending;
  ++JOG.sequence;
  JOG.down = false;
  JOG.slider = null;
  if (JOG.timer) { clearInterval(JOG.timer); JOG.timer = null; }
  document.querySelectorAll("[data-jog-slider]").forEach((input) => {
    input.value = "0";
    const output = $(`#op-jsv${input.id.slice(5)}`);
    if (output) output.textContent = "0";
  });
  if (was) post("/api/cmd", { cmd: "jog_stop", args: {} });
}
window.addEventListener("pointerup", () => (JOG.down || JOG.slider) && jogRelease());
window.addEventListener("blur", () => (JOG.down || JOG.slider) && jogRelease());
document.addEventListener("visibilitychange", () => document.hidden && (JOG.down || JOG.slider) && jogRelease());

// Move the jog panel into `host` (PROGRAMS) or back into OPERATE (host null). One panel, so ids stay unique.
function dockJog(host) {
  const g = $("#op-g");
  if (!g) return;
  if (!dockJog.home) dockJog.home = { parent: g.parentElement, next: g.nextSibling };
  jogRelease();
  if (host) { host.append(g); g.classList.remove("hidden"); }
  else if (g.parentElement !== dockJog.home.parent) {
    dockJog.home.parent.insertBefore(g, dockJog.home.next);
    g.classList.toggle("hidden", pages.OPERATE.mode !== "g");
  }
}

function jogUpdate(s) {
  if (!$("#op-jog-type")) return;
  if (!JOG.labelled) { jogLabels(); JOG.labelled = true; }
  const hold = $("#op-jog-step").value === "hold";
  const why = hold ? s.jog.hold_refusal || (s.jog.active ? null : s.motion_refusal) : s.motion_refusal;
  // Do not disable the native range control while its jog_start request is in flight: browsers may cancel
  // pointer capture when a range input becomes disabled, leaving the server with no renewal loop.
  if (!JOG.down && !JOG.pending && !JOG.slider)
    document.querySelectorAll("[data-jog], [data-jog-slider]").forEach((b) => gate(b, !why));
  $("#op-jog-note").textContent = s.jog.servo_warning ? "Servo: " + s.jog.servo_warning
    : why && s.idle ? why
    : hold ? "Move a slider to jog. Release to stop; centre is zero. Slider is linear; Velocity % caps max speed."
    : "Each click moves one step (Cartesian: straight line, rotations about the TCP).";
}

// ------------------------------------------------------------------- force
// Admittance / Impedance: the tool exerts this wrench (at the active TCP). The robot PC streams it, ramped,
// while this page renews every 200 ms; Release ramps it down, STOP / a closed page / lost control zero it at once.
const FORCE = { timer: null };

function forcePanel() {
  const f = INFO.force;
  const cell = (i) => h("label", { class: "fcell" }, h("span", { class: "muted" }, AXES[i]),
    h("input", { type: "number", id: `op-f${i}`, value: "0", step: i < 3 ? "1" : "0.1", style: "width:68px" }));
  const el = h("div", { id: "op-f", class: "hidden" },
    h("div", { class: "row" }, h("span", { class: "muted" }, "Frame"),
      h("select", { id: "op-f-frame", title: "The axes of the wrench" }, h("option", { value: "tool" }, "Tool (active TCP)"), h("option", { value: "base" }, "Base")),
      h("div", { class: "grow" }),
      h("button", { class: "btn quiet ibtn", type: "button", title: "Set every field to 0", html: icon("reset") + "<span>Clear</span>",
        onclick: () => [0, 1, 2, 3, 4, 5].forEach((i) => ($(`#op-f${i}`).value = "0")) })),
    h("div", { class: "fgrid" }, h("span", { class: "u" }, "N"), [0, 1, 2].map(cell), h("span", { class: "u" }, "Nm"), [3, 4, 5].map(cell)),
    h("div", { class: "row" },
      btn("Apply", "primary", () => forceApply(), "Apply (or change) this wrench; it ramps in"),
      btn("Ramp to zero", "", () => forceRelease(), "Ramp the force down to zero, then end"),
      btn("Zero now", "danger", () => forceStop(), "Zero at once")),
    h("div", { class: "secondary", id: "op-f-status" }),
    h("div", { class: "muted", id: "op-f-note" }),
    note(`Max ${f.max_force_n} N · ${f.max_torque_nm} Nm`,
      `The tool pushes with this wrench at the active TCP. In free air the arm moves until the controller's ` +
      `stiffness balances it (≈ force / stiffness, see SETUP › Gains); on a surface it presses. ` +
      `Ramps at ${f.ramp_force_nps} N/s. ` +
      `Zeroed at once on STOP, losing control, or if this page stops renewing ` +
      `(keep it in front: hidden tabs are throttled). Ends after ${f.max_duration_s} s.`));
  el.querySelectorAll(".btn").forEach((b) => { if (b.textContent === "Apply") b.dataset.lease = "1"; });
  el.querySelector(".btn.primary").id = "op-f-apply";
  return el;
}

async function forceApply() {
  const wrench = [0, 1, 2, 3, 4, 5].map((i) => +$(`#op-f${i}`).value || 0);
  const frame = $("#op-f-frame").value;
  const verb = S.force.active ? "Change the force to" : "Apply";
  if (!S.force.active && !(await ask(`${verb} F = [${wrench.slice(0, 3).join(", ")}] N, ` +
      `T = [${wrench.slice(3).join(", ")}] Nm (${frame} axes) in ${S.mode_label}?`))) return;
  const r = await cmd("force_apply", { wrench, frame });
  if (!r.ok) return;
  if (!FORCE.timer) FORCE.timer = setInterval(async () => {
    const x = await cmd("force_hold");
    if (!x.ok || !x.active) { clearInterval(FORCE.timer); FORCE.timer = null; }
  }, 200);
}
function forceRelease() { post("/api/cmd", { cmd: "force_release", args: {} }); }
function forceStop() {
  if (FORCE.timer) { clearInterval(FORCE.timer); FORCE.timer = null; }
  post("/api/cmd", { cmd: "force_stop", args: {} });
}

function forceUpdate(s) {
  if (!$("#op-f")) return;
  const f = s.force;
  gate($("#op-f-apply"), !f.refusal && (!s.busy || f.active));
  $("#op-f-apply").textContent = f.active && !f.releasing ? "Change" : "Apply";
  const fmtW = (w) => `F [${w.slice(0, 3).map((v) => v.toFixed(1)).join(", ")}] N  T [${w.slice(3).map((v) => v.toFixed(2)).join(", ")}] Nm`;
  $("#op-f-status").textContent = f.active ? `${f.releasing ? "Releasing" : "Applying"} (${f.frame} axes): ${fmtW(f.current)}`
    : f.last_end ? "Last: " + f.last_end : "";
  const fn = f.refusal || f.note || "";
  $("#op-f-note").textContent = fn.charAt(0).toUpperCase() + fn.slice(1);
  if (!f.active && FORCE.timer && !s.busy) { clearInterval(FORCE.timer); FORCE.timer = null; }
}

// ================================================================= PROGRAMS
// UR-style program tree: the server owns the program (rows = flattened tree); this page shows it and asks for edits.
// A step is addressed by its path (list of ints); the selected path is compared as JSON.
const PG_ICON = { movej: "MoveJ", move_pose: "Move", gripper: "Grip", wait: "Wait", set_io: "I/O", speed: "Spd",
  controller: "Mode", policy: "Pol", loop: "Loop", folder: "Dir", comment: "//", move_relative: "Rel",
  move_until_contact: "Touch", force: "Force", force_off: "F off", zero_ft: "Zero",
  photo: "Photo", record_start: "Rec", record_stop: "Rec off", replay: "Play" };
// direction pickers for force work: the six axis directions, plus whatever the step already has
const AXIS_DIRS = [["+x", [1, 0, 0]], ["-x", [-1, 0, 0]], ["+y", [0, 1, 0]], ["-y", [0, -1, 0]], ["+z", [0, 0, 1]], ["-z", [0, 0, -1]]];
const dirKey = (v) => (AXIS_DIRS.find(([, d]) => d.every((x, i) => Math.abs(x - (v || [])[i]) < 1e-9)) || [null])[0];
const forceFrames = (withFixtures) => [["tool", "tool (active TCP axes)"], ["base", "base"],
  ...(withFixtures && S ? S.calibration.frames.filter((f) => f !== INFO.base_link).map((f) => [f, f]) : [])];
const pathEq = (a, b) => !!a && !!b && JSON.stringify(a) === JSON.stringify(b);
const pct = (f) => Math.round((f ?? 0.1) * 100);
pages.PROGRAMS = {
  path: null, where: "after", tab: "insert", prog: null, lastExec: null,
  build(el) {
    const files = h("select", { id: "pg-files" });
    // whose programs the list shows: a person's profile, or "" = Shared (the programs that were here before profiles)
    this.owner = null; this.profiles = [];
    const ownerSel = h("select", { id: "pg-owner", title: "Whose programs to show. Save and New put the program here." });
    ownerSel.onchange = () => { this.owner = ownerSel.value; this.files(); };
    this.ownerSel = ownerSel;
    const policySel = h("select", { id: "pg-sc-policy" }, INFO.policies.map((p) => h("option", { value: p.name }, p.name)));
    const top = h("div", { class: "row" },
      h("span", { class: "title", id: "pg-title" }, "untitled"), h("span", { class: "muted", id: "pg-file" }),
      h("div", { class: "grow" }), ownerSel, files,
      btn("Open", "", async () => (await this.guard("open " + files.value)) && files.value &&
        this.apply(pcmd("program_open", { name: files.value, owner: this.owner || "" }))),
      btn("Copy from…", "quiet", () => this.copyFrom(), "Copy a program of another profile (or Shared) into the profile shown"),
      btn("New", "quiet", async () => (await this.guard("start a new program")) && this.apply(pcmd("program_new"))),
      h("input", { type: "text", id: "pg-name", placeholder: "file name", style: "width:140px" }),
      btn("Save", "", async () => { await this.apply(pcmd("program_save", { name: $("#pg-name").value || null, owner: this.owner || "" })); this.files(); }),
      btn("Check", "quiet", async () => {
        const r = await pcmd("program_check");
        if (r.ok) toast(r.problems.length ? "Program check found problems:\n• " + r.problems.join("\n• ") : "Program OK: preflight passed.",
          !r.problems.length, !!r.problems.length);
      }));
    top.querySelectorAll(".btn").forEach((b) => b.textContent !== "Check" && (b.dataset.lease = "1"));
    this.undoBtn = btn("↶ Undo", "quiet", () => this.undo(false), "Undo the last program edit (Ctrl+Z)");
    this.redoBtn = btn("↷ Redo", "quiet", () => this.undo(true), "Redo (Ctrl+Y)");
    // run controls beside the program name: state chip, Run, Pause, Stop; Step / From selected / Loop in the menu
    this.stateChip = h("span", { class: "chip", id: "pg-state" }, "READY");
    this.runBtn = btn("▶ Run", "primary", () => this.run(false), "Run the program");
    this.pauseBtn = btn("Pause", "", () => (S.runner.state === "PAUSED" ? pcmd("program_resume") : pcmd("program_pause")));
    this.stopBtn = btn("■ Stop", "stopo", () => pcmd("program_stop"), "Stop this program run (the red STOP in the top bar stops everything)");
    this.stepBtn = btn("Step", "", () => this.run(true), "Run one command at a time");
    const runMenu = h("details", { class: "pg-runmenu" }, h("summary", { class: "btn quiet", title: "Run options" }, "⋯"),
      h("div", { class: "pg-runpop" }, this.stepBtn,
        h("label", { class: "chk" }, h("input", { type: "checkbox", id: "pg-from" }), "From selected"),
        h("label", { class: "chk" }, h("input", { type: "checkbox", id: "pg-loop" }), "Loop")));
    const runGroup = h("div", { class: "row pg-run" }, this.stateChip, this.runBtn, this.pauseBtn, this.stopBtn, runMenu,
      h("span", { class: "badge hidden", id: "pg-loops" }), h("span", { class: "muted small", id: "pg-pause-why" }));
    top.insertBefore(runGroup, top.children[2]);
    top.insertBefore(this.redoBtn, runGroup);
    top.insertBefore(this.undoBtn, this.redoBtn);
    document.addEventListener("keydown", (e) => {
      if (current !== "PROGRAMS" || !(e.ctrlKey || e.metaKey) || /INPUT|TEXTAREA|SELECT/.test(document.activeElement?.tagName)) return;
      const k = e.key.toLowerCase();
      if (k === "z" || k === "y") { e.preventDefault(); this.undo(k === "y" || e.shiftKey); }
    });
    window.addEventListener("beforeunload", (e) => {
      if (S && S.runner.dirty && holds) { e.preventDefault(); e.returnValue = ""; }
    });
    const left = h("div", { class: "card pg-left" },
      h("div", { class: "scroll pg-tree", id: "pg-tree" }),
      h("div", { class: "row" },
        btn("↑", "", () => this.move(-1), "Move up"), btn("↓", "", () => this.move(1), "Move down"),
        btn("Duplicate", "", () => this.dup()), btn("Delete", "quiet", () => this.del()),
        h("div", { class: "grow" }),
        btn("Fold all", "quiet", () => this.foldAll(true), "Fold every folder and loop"),
        btn("Unfold all", "quiet", () => this.foldAll(false))));
    [...left.lastChild.querySelectorAll(".btn")].slice(0, 4).forEach((b) => (b.dataset.lease = "1"));
    // 3D preview beside the tree, always there: the TCP path of the whole program (relative moves and loops too),
    // the waypoints (selected = orange) and a ghost arm for a selected joint target
    const mid = h("div", { class: "card pg-mid" },
      h("div", { class: "row" }, h("span", { class: "section", style: "margin:0" }, "3D preview"), h("div", { class: "grow" }),
        h("label", { class: "chk", title: "The TCP path of the program from where the robot is now (kinematics only: no planning, " +
          "no contact). Blue line; it follows every edit." },
          h("input", { type: "checkbox", id: "pg-v3-path", checked: localStorage.getItem("pg-path") !== "0",
            onchange: (e) => { localStorage.setItem("pg-path", e.target.checked ? "1" : "0"); this.pathKey = null; this.paint3d(); } }), "TCP path"),
        btn("↻ From here", "quiet", () => { this.pathKey = null; this.paint3d(); }, "Recompute the path from the robot's current position"),
        (() => { const b = btn("▶ Play", "", () => this.pgPlay(), "A ghost arm plays the whole program from where the robot " +
          "is now (loops included). Nothing moves: kinematics only, no planning, no contact"); b.id = "pg-play"; return b; })()),
      h("div", { class: "view3d pg-v3", id: "pg-v3" }, h("div", { class: "muted v3msg", id: "pg-v3-msg" }, "Loading…")),
      // step through the preview one move at a time (loops unrolled): the ghost repeats that move, orange on the path
      h("div", { class: "row pg-steprow" },
        btn("|◀", "quiet", () => this.pgStep(1), "First move"),
        btn("◀", "quiet", () => this.pgStep((this.stepK || 1) - 1), "Previous move"),
        h("input", { type: "range", id: "pg-step", min: 1, max: 1, value: 1, class: "grow",
          title: "Drag to any move of the program (loops unrolled)", oninput: (e) => this.pgStep(+e.target.value) }),
        btn("▶", "quiet", () => this.pgStep((this.stepK || 0) + 1), "Next move"),
        btn("▶|", "quiet", () => this.pgStep(1e9), "Last move")),
      h("div", { class: "mono small", id: "pg-step-lab" }, ""),
      h("div", { class: "muted small", id: "pg-v3-note" }, ""));
    // --- Insert tab
    const whereSel = h("select", { id: "pg-where", onchange: (e) => (this.where = e.target.value) },
      h("option", { value: "after" }, "After selection"), h("option", { value: "before" }, "Before selection"),
      h("option", { value: "into" }, "Into (container)"));
    const ins = (label, type, params = {}, title = "") => btn(label, "", () => this.insert(type, params), title);
    const poseParams = (cart) => () => ({ cartesian: cart, frame: $("#pg-iframe").value });
    const grip = INFO.gripper.enabled ? h("div", { class: "row" }, h("span", { class: "muted" }, "Gripper"),
      btn("Open", "", () => cmd("gripper", { percent: 0, effort: gripForce() })),
      btn("Close", "", () => cmd("gripper", { percent: 100, effort: gripForce() })),
      h("span", { class: "mono muted", id: "pg-grip-pct" })) : null;
    if (grip) grip.querySelectorAll(".btn").forEach((b) => (b.dataset.lease = "1", b.dataset.grip = "1", b.dataset.tip = ""));
    // closed by default so the commands start at the top; it stays as the user left it
    const drawer = h("details", { class: "pg-drawer", id: "pg-drawer", open: localStorage.getItem("pg-jog-open") === "1",
      ontoggle: (e) => { localStorage.setItem("pg-jog-open", e.target.open ? "1" : "0");
        if (current === "PROGRAMS") dockJog(e.target.open ? $("#pg-jog") : null); } },
      h("summary", { class: "section" }, "Jog & gripper (move the arm, then add a MoveJ / MoveL here)"),
      h("div", { id: "pg-jog" }), grip);
    const insertTab = h("div", { class: "col", id: "pg-t-insert" },
      drawer,
      h("div", { class: "row" }, h("span", { class: "muted" }, "Place:"), whereSel),
      h("div", { class: "section" }, "Move (records the current robot position)"),
      h("div", { class: "row" }, ins("MoveJ (here)", "movej"),
        btn("MoveL (here)", "", () => this.insert("move_pose", poseParams(true)())),
        btn("Move to pose (here)", "", () => this.insert("move_pose", poseParams(false)()))),
      h("div", { class: "row" }, h("span", { class: "muted" }, "Pose frame"), frameSelect("pg-iframe")),
      h("div", { class: "section" }, "Basic"),
      h("div", { class: "row" }, ins("Wait", "wait"), ins("Gripper", "gripper"), ins("Set output", "set_io"),
        ins("Speed", "speed"), ins("Comment", "comment")),
      h("div", { class: "section" }, "Structure"),
      h("div", { class: "row" }, ins("Loop", "loop"), ins("Folder", "folder")),
      h("div", { class: "section" }, "Force & contact (Admittance / Impedance)"),
      h("div", { class: "row" },
        ins("Zero F/T", "zero_ft", {}, "Software F/T bias at the current load (in free air, before Admittance)"),
        ins("Move until contact", "move_until_contact", {}, "Linear move that stops when the measured force changes"),
        ins("Force", "force", {}, "Apply a force; it stays on across the next steps until Force off"),
        ins("Force off", "force_off", {}, "Ramp the force out"),
        ins("Move relative", "move_relative", {}, "Move the TCP by an offset in tool, base or fixture axes")),
      h("div", { class: "section" }, "Record & photo (nothing moves)"),
      h("div", { class: "row" },
        ins("Record start", "record_start", {}, "Start a recording (stops by itself when the program ends)"),
        ins("Record stop", "record_stop", {}, "Finish the recording"),
        ins("Photo", "photo", {}, "Save one still from a camera into the run folder")),
      h("div", { class: "section" }, "Advanced"),
      h("div", { class: "row" }, ins("Control mode", "controller"), ins("Policy", "policy")));
    insertTab.querySelectorAll(".btn").forEach((b) => (b.dataset.lease = "1"));
    // --- Teach & more tab
    const teach = h("div", { class: "card" }, h("div", { class: "section" }, "Teach"),
      h("div", { class: "muted" }, "Freedrive, drag the arm, record. Recording in a fixture frame (e.g. vial) makes the pose follow it."),
      h("div", { class: "row" }, freedriveButton(), h("span", { class: "muted" }, "in"), frameSelect("pg-frame")),
      h("div", { class: "row" },
        btn("Record pose", "", () => this.apply(pcmd("program_record", { kind: "pose", frame: $("#pg-frame").value }))),
        btn("Record joints", "quiet", () => this.apply(pcmd("program_record", { kind: "joints" })))));
    teach.lastChild.querySelectorAll(".btn").forEach((b) => (b.dataset.lease = "1"));
    const teachHand = this.buildTeachHand();
    const scrape = h("details", { class: "card" }, h("summary", { class: "section" }, "New scraping program"),
      h("div", { class: "muted" }, "Put the tool on the vial rim where scraping starts (freedrive), then create. Every other angle is that pose turned about the vial axis."),
      h("div", { class: "grid", style: "grid-template-columns:auto 1fr" },
        h("span", { class: "k" }, "Vial frame"), frameSelect("pg-sc-frame"),
        h("span", { class: "k" }, "Policy"), policySel,
        h("span", { class: "k" }, "Per angle s"), num("8", "0.5"),
        h("span", { class: "k" }, "Angle step °"), num("10", "1"),
        h("span", { class: "k" }, "Angles"), num("37", "1"),
        h("span", { class: "k" }, "Approach mm"), num("50", "5"),
        h("span", { class: "k" }, ""), h("label", { class: "chk" }, h("input", { type: "checkbox", id: "pg-sc-back" }), "Then sweep back")),
      h("div", { class: "row" }, h("span", { class: "muted" }, "At each angle:"),
        h("label", { class: "chk" }, h("input", { type: "radio", name: "pg-sc-kind", value: "policy", checked: true,
          onchange: () => this.scrapeKind() }), "run the policy"),
        h("label", { class: "chk" }, h("input", { type: "radio", name: "pg-sc-kind", value: "baseline",
          onchange: () => this.scrapeKind() }), "force baseline (scripted)")),
      h("div", { class: "grid hidden", id: "pg-sc-base", style: "grid-template-columns:auto 1fr" },
        h("span", { class: "k" }, "Press along tool"), h("select", { id: "pg-sc-axis" }, AXIS_DIRS.map(([n]) =>
          h("option", { value: n, selected: n === "+z" }, n.toUpperCase()))),
        h("span", { class: "k" }, "Force N"), h("input", { type: "number", id: "pg-sc-f", value: "5", step: "0.5" }),
        h("span", { class: "k" }, "Contact |ΔF| N"), h("input", { type: "number", id: "pg-sc-c", value: "2", step: "0.5" }),
        h("span", { class: "k" }, "Stroke mm"), h("input", { type: "number", id: "pg-sc-s", value: "15", step: "1" }),
        h("span", { class: "k" }, "Strokes"), h("input", { type: "number", id: "pg-sc-n", value: "2", step: "1", min: "1" }),
        h("span", { class: "k" }, "Standoff / search mm"), h("div", { class: "row v3in" },
          h("input", { type: "number", id: "pg-sc-so", value: "3", step: "0.5" }),
          h("input", { type: "number", id: "pg-sc-se", value: "10", step: "1" }))),
      h("div", { class: "muted small hidden", id: "pg-sc-base-note" }, "Per angle: approach, search the wall, press with the " +
        "force, stroke down and up along the vial axis, release, lift. F/T is zeroed in free air first. Every number stays " +
        "editable as program steps."),
      btn("Create from current pose", "", async () => {
        if (!(await this.guard("create a new scraping program"))) return;
        const v = [...scrape.querySelector(".grid").querySelectorAll("input[type=number]")].map((i) => +i.value);
        const common = { frame: $("#pg-sc-frame").value, step_deg: v[1], count: v[2], approach_mm: v[3],
          back_and_forth: $("#pg-sc-back").checked };
        const fold = (r) => r.ok && this.foldAll(true);      // one folder per angle: start folded
        if (scrape.querySelector("input[name=pg-sc-kind]:checked").value === "baseline")
          this.apply(pcmd("program_scrape_baseline", { ...common, press_axis: $("#pg-sc-axis").value, force_n: +$("#pg-sc-f").value,
            contact_n: +$("#pg-sc-c").value, stroke_mm: +$("#pg-sc-s").value, strokes: +$("#pg-sc-n").value,
            standoff_mm: +$("#pg-sc-so").value, search_mm: +$("#pg-sc-se").value })).then(fold);
        else this.apply(pcmd("program_scrape", { ...common, policy: policySel.value, duration: v[0] }));
      }));
    scrape.lastChild.dataset.lease = "1";
    const yamlBox = h("details", { class: "card" }, h("summary", { class: "section" }, "Edit as YAML"),
      h("textarea", { id: "pg-yaml", rows: "16", spellcheck: "false" }),
      btn("Apply YAML", "", () => this.apply(pcmd("program_set_yaml", { text: $("#pg-yaml").value }))));
    yamlBox.lastChild.dataset.lease = "1";
    const teachTab = h("div", { class: "col", id: "pg-t-teach" }, teach, teachHand, scrape, yamlBox);
    const batchTab = this.buildBatch();
    const tabs = h("div", { class: "tabs" }, ["cmd:Command", "insert:Insert", "teach:Teach & more", "batch:Batch"].map((t) => {
      const [k, label] = t.split(":");
      return h("a", { id: `pg-tab-${k}`, onclick: () => this.setTab(k) }, label);
    }));
    const right = h("div", { class: "card pg-right" }, tabs,
      h("div", { id: "pg-t-cmd", class: "col" }), insertTab, teachTab, batchTab);
    el.append(h("div", { class: "col" }, top, h("div", { class: "cols pg-cols" }, left, mid, right)));
    this.setTab("insert");
    this.files();
    cmd("program_get").then((r) => r.ok && this.display(r.program));
  },
  async reload() { const r = await cmd("program_get"); if (r.ok) this.display(r.program); },
  scrapeKind() {
    const base = document.querySelector("input[name=pg-sc-kind]:checked").value === "baseline";
    $("#pg-sc-base").classList.toggle("hidden", !base);
    $("#pg-sc-base-note").classList.toggle("hidden", !base);
    // the policy and its time per angle only matter for the policy variant
    const g = $("#pg-sc-frame").closest(".grid");
    [...g.children].slice(2, 6).forEach((e) => e.classList.toggle("hidden", base));
  },
  async files() {
    if (this.owner == null) {             // first time: this browser's profile (top right), else Shared
      const p = await cmd("profile_list");
      const me = (localStorage.getItem("operator") || "").toLowerCase();
      this.profiles = p.ok ? p.profiles : [];
      this.owner = this.profiles.find((x) => x.name.toLowerCase() === me)?.name || "";
    }
    const r = await cmd("program_list", { owner: this.owner });
    if (!r.ok && this.owner) { this.owner = ""; return this.files(); }       // the profile was deleted
    if (!r.ok) return;
    this.profiles = r.profiles.map((name) => ({ name }));
    this.ownerSel.replaceChildren(...["", ...r.profiles].map((o) => h("option", { value: o }, o || "Shared")));
    this.ownerSel.value = this.owner = r.owner;
    $("#pg-files").replaceChildren(...(r.files.length ? r.files.map((f) => h("option", { value: f }, f)) : [h("option", { value: "" }, "no programs")]));
    if (this.prog?.file && (this.prog.owner || "") === this.owner) $("#pg-files").value = this.prog.file;
  },
  // copy a program of another profile (or Shared) into the profile shown
  async copyFrom() {
    const mine = this.owner || "", name = (o) => o || "Shared";
    const others = ["", ...this.profiles.map((p) => p.name)].filter((o) => o !== mine);
    const from = h("select", {}, ...others.map((o) => h("option", { value: o }, name(o))));
    const pick = h("select", { style: "min-width:200px" });
    const as = h("input", { type: "text", placeholder: "same name", maxlength: 60, style: "width:200px" });
    const fillPick = async () => {
      const r = await cmd("program_list", { owner: from.value });
      const l = r.ok ? r.files : [];
      pick.replaceChildren(...(l.length ? l.map((f) => h("option", { value: f }, f)) : [h("option", { value: "" }, "no programs")]));
    };
    from.onchange = fillPick;
    await fillPick();
    const form = h("div", { style: "display:flex;flex-direction:column;gap:8px" }, h("div", { class: "row" }, "From", from),
      h("div", { class: "row" }, "Program", pick), h("div", { class: "row" }, "Save as", as));
    const c = await ask(`Copy a program into ${name(mine)}\nA copy: later changes in either one don't touch the other.`,
      { content: form, ok: "Copy", result: () => ({ from: from.value, name: pick.value, as: as.value.trim() }) });
    if (!c || !c.name) return;
    const go = (replace) => cmd("program_copy", { source_owner: c.from, name: c.name, owner: mine, new_name: c.as, replace });
    let r = await go(false);
    if (!r.ok && /already exists/.test(r.msg || "") && (await ask(`Replace "${c.as || c.name}" in ${name(mine)} with the copy?`))) r = await go(true);
    if (r.ok) { toast(r.msg, true); this.files(); }
  },
  // unsaved changes: ask before replacing the program
  async guard(what) {
    return !(this.prog && this.prog.dirty) ||
      (await ask(`"${this.prog.name}" has unsaved changes. Discard them and ${what}?`));
  },
  undo(redo) {
    if (holds && S && S.runner.state === "IDLE") this.apply(cmd(redo ? "program_redo" : "program_undo"));
  },
  async apply(promise) { const r = await promise; if (r.ok && r.program) this.display(r.program, r.selected); return r; },
  show() { if (this.tab === "insert" && $("#pg-drawer").open) dockJog($("#pg-jog")); },
  hide() { dockJog(null); },
  setTab(k) {
    if (k !== "insert") dockJog(null); else if (current === "PROGRAMS" && $("#pg-drawer").open) dockJog($("#pg-jog"));
    this.tab = k;
    for (const t of ["cmd", "insert", "teach", "batch"]) {
      $(`#pg-t-${t}`).classList.toggle("hidden", t !== k);
      $(`#pg-tab-${t}`).classList.toggle("on", t === k);
    }
    if (k === "batch") { this.batchVials(); this.batchSteps(); }
  },
  row(path = this.path) { return path && this.prog ? this.prog.rows.find((r) => pathEq(r.path, path)) : null; },
  select(path) {
    this.path = path;
    if (this.tab === "teach" && !(S && S.teach)) this.setTab("cmd");    // Insert stays (jog, insert, jog, insert)
    this.paintTree(); this.editor(); this.paint3d();
  },
  // folders and loops fold in the tree (a generated scraping program has hundreds of rows)
  collapsed: new Set(),
  toggleFold(path, e) {
    e.stopPropagation();
    const k = JSON.stringify(path);
    this.collapsed.has(k) ? this.collapsed.delete(k) : this.collapsed.add(k);
    this.paintTree();
  },
  foldAll(fold) {
    this.collapsed = new Set(fold ? this.prog.rows.filter((r) => r.container).map((r) => JSON.stringify(r.path)) : []);
    this.paintTree();
  },
  hiddenRow(r) {
    for (let n = 1; n < r.path.length; n++) if (this.collapsed.has(JSON.stringify(r.path.slice(0, n)))) return true;
    return false;
  },
  async init3d() {
    if (this.view) return;
    try {
      const m = await import("/static/view3d.js");
      this.view = new m.View3D($("#pg-v3"), { baseLink: INFO.base_link });
      const r = await fetch("/api/robot_model").then((x) => x.json());
      if (!r.ok) { $("#pg-v3-msg").textContent = "Robot model: " + r.msg; return; }
      await this.view.load(r);
      $("#pg-v3-msg").remove();
      this.paint3d();
    } catch (e) { $("#pg-v3-msg").textContent = "3D view unavailable: " + e.message; }
  },
  // the ghost's joint trajectory for the previewed path, worked out once per path (IK in the page)
  pgTraj() {
    const d = this.prevData, v = this.view;
    if (!v || !d || !d.poses || d.poses.length < 2 || !S || !S.joints_deg) {
      toast("Nothing to play: turn on TCP path (the preview needs the program's path first)", false); return null;
    }
    if (this.trFor === d) return this.tr;
    const seed = { ...pages.OPERATE.extraJoints(S) }, lim = {};
    S.joints_deg.forEach((x, i) => (seed[INFO.joint_names[i]] = x * Math.PI / 180));
    (INFO.joint_limits_deg || []).forEach((l, i) => l && (lim[INFO.joint_names[i]] = [l[0] * Math.PI / 180, l[1] * Math.PI / 180]));
    this.tr = v.programTrajectory(d.poses, d.joints, d.linear, INFO.joint_names, seed, lim);
    this.trFor = d;
    if (!this.tr) toast("Nothing to play", false);
    return this.tr;
  },
  // show move k (1..n) alone: the ghost repeats it, the move is orange on the path, the label says which step it is
  pgStep(k) {
    const d = this.prevData, tr = this.pgTraj(), v = this.view;
    if (!tr) return;
    const n = d.poses.length - 1;
    k = Math.max(1, Math.min(n, Math.round(k) || 1));
    this.pgStop();
    this.stepK = k;
    const a = tr.ends[k - 1], b = tr.ends[k], t0 = tr.points[a].t;
    let seg = tr.points.slice(a, b + 1).map((p) => ({ t: p.t - t0, q: p.q }));
    if (seg.length < 2 || seg[seg.length - 1].t <= 0) seg = [seg[0], { t: 0.3, q: seg[seg.length - 1].q }];
    const extra = pages.OPERATE.extraJoints(S);
    v.setHighlight(v.planPath(seg, INFO.joint_names, extra));
    v.playPreview({ points: seg }, INFO.joint_names, extra, true);
    this.playing = "step";
    const sl = $("#pg-step");
    sl.max = n; sl.value = k;
    const loops = (d.loops && d.loops[k] || []).map(([lab, i, m]) => `Loop ${lab} · ${i}/${m}`).join(" › ");
    const p0 = d.points[k - 1], p1 = d.points[k];
    const mm = Math.hypot(p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]);
    $("#pg-step-lab").textContent = `Move ${k} / ${n} · step ${d.steps[k]}` + (loops ? ` · ${loops}` : "") +
      ` · ${(d.what && d.what[k]) || ""} · ${mm.toFixed(1)} mm` + (d.linear[k] ? " straight" : " (joint move)");
  },
  pgPlay() {
    if (this.playing === true) { this.pgStop(); return; }
    const d = this.prevData, v = this.view, tr = this.pgTraj();
    if (!tr) return;
    this.pgStop();
    v.playPreview({ points: tr.points }, INFO.joint_names, pages.OPERATE.extraJoints(S));
    this.playing = true;
    $("#pg-play").textContent = "■ Stop";
    $("#pg-v3-note").textContent = `Playing ${d.poses.length - 1} moves (${tr.duration.toFixed(0)} s, loops) as a ghost; nothing moves.` +
      (tr.fails ? ` ${tr.fails} point(s) had no IK solution (out of reach?): the ghost holds there.` : "") +
      (d.note ? ` · ${d.note}` : "");
  },
  pgStop() {
    if (!this.playing) return;
    if (this.playing === "step") $("#pg-step-lab").textContent = "";
    this.playing = false;
    if (this.view) { this.view.clearPreview(); this.view.setHighlight(null); if (this.prevData) this.view.setPathMm(this.prevData.points); }
    const b = $("#pg-play");
    if (b) b.textContent = "▶ Play";
  },
  paint3d() {
    if (!this.view || !this.prog) return;
    if (this.teachPrev) {                        // reviewing a taught path: the raw motion (blue) and its MoveL points
      const r = this.teachPrev;
      this.pgStop();
      this.pathKey = null;                       // the program path comes back when the review ends
      this.view.setPathMm(r.raw_mm);
      this.view.setMarkers(r.kept.map((k) => ({ ...k, selected: false })), true);
      this.view.showGhost(null);
      $("#pg-v3-note").style.color = "";
      $("#pg-v3-note").textContent = `Taught path (blue) and the ${r.points} points it becomes (grey: the MoveL steps). Nothing moves.`;
      return;
    }
    // the program's TCP path, recomputed when the program changes (or on ↻); never while it runs
    const want = $("#pg-v3-path").checked;
    const key = want ? `${this.prog.rev}|${S && S.tcp_sig}` : "off";   // a TCP change or edit redraws it too
    if (key !== this.pathKey && !(S && S.runner.state !== "IDLE" && want)) {
      this.pathKey = key;
      if (!want) { this.view.setPathMm(null); $("#pg-v3-note").textContent = ""; }
      else cmd("program_preview_path").then((r) => {
        this.pgStop();
        this.prevData = r.ok ? r : null;
        if (!r.ok) { this.view.setPathMm(null); $("#pg-v3-note").textContent = r.msg; return; }
        this.view.setPathMm(r.points);
        this.stepK = 0;
        const sl = $("#pg-step"); sl.max = Math.max(1, r.points.length - 1); sl.value = 1;
        $("#pg-v3-note").style.color = (r.note || "").startsWith("WARNING") ? "var(--danger)" : "";
        $("#pg-v3-note").textContent = `TCP path: ${r.points.length - 1} moves from the current position` + (r.note ? ` · ${r.note}` : "") +
          ". Kinematics only (no planning, no contact).";
      });
    }
    if (S && S.keepout) this.view.setZones(S.keepout.boxes, localStorage.getItem("v3-zones") !== "0");
    const seen = new Set(), marks = [];
    for (const r of this.prog.rows) {
      const w = r.waypoint;
      if (!w || !w.base_xyz_mm || seen.has(w.name)) continue;
      seen.add(w.name);
      marks.push({ xyz_mm: w.base_xyz_mm, rpy_deg: w.base_rpy_deg, selected: pathEq(r.path, this.path) });
    }
    this.view.setMarkers(marks, !$("#pg-v3-path").checked);
    const sel = this.row(), w = sel && sel.waypoint;
    if (w && w.joints_deg) {
      const q = {};
      w.joints_deg.forEach((v, i) => (q[INFO.joint_names[i]] = v * Math.PI / 180));
      this.view.showGhost(q);
    } else this.view.showGhost(null);
  },
  display(p, selected) {
    this.prog = p;
    if (selected != null) this.path = selected;          // null: keep the current selection if it still exists
    if (this.path && !this.row()) this.path = null;
    $("#pg-title").textContent = p.name + (p.dirty ? " *" : "");
    $("#pg-title").title = p.dirty ? "Unsaved changes" : "";
    this.undoBtn.dataset.can = p.can_undo ? "1" : "";
    this.redoBtn.dataset.can = p.can_redo ? "1" : "";
    $("#pg-file").textContent = p.file ? (p.owner ? p.owner + " / " : "Shared / ") + p.file : "unsaved";
    if (p.file) $("#pg-name").value = p.file;
    if (document.activeElement !== $("#pg-yaml")) $("#pg-yaml").value = p.yaml;
    const tree = $("#pg-tree");
    if (!p.rows.length) tree.replaceChildren(h("div", { class: "muted pg-empty" }, "Empty program: add commands with Insert →"));
    else tree.replaceChildren(...p.rows.map((r) => {
      const bad = r.problems && r.problems.length;
      return h("div", { class: "pg-row", style: `padding-left:${8 + 18 * r.depth}px`, onclick: () => this.select(r.path) },
        r.container ? h("span", { class: "pg-fold", title: "Fold / unfold", onclick: (e) => this.toggleFold(r.path, e) }, "▾")
          : h("span", { class: "pg-fold" }),
        h("span", { class: "muted pg-n" }, r.label),
        h("span", { class: "badge pg-ic" + (r.container ? " cont" : "") }, PG_ICON[r.type] || r.type),
        h("span", { class: "pg-title" }, r.title), h("span", { class: "secondary pg-detail" }, r.detail),
        bad ? h("span", { class: "pg-warn", title: r.problems.join("\n") }, "⚠") : null);
    }));
    this.paintTree();
    this.editor();
    this.paint3d();
    if (this.tab === "batch") this.batchSteps();
  },
  paintTree() {
    const exec = S && S.runner.state !== "IDLE" ? S.runner.path : null;
    this.lastExec = exec;
    (this.prog ? this.prog.rows : []).forEach((r, i) => {
      const el = $("#pg-tree").children[i];
      if (!el) return;
      el.classList.toggle("sel", pathEq(r.path, this.path));
      el.classList.toggle("exec", pathEq(r.path, exec) || (!!exec && this.hiddenRow({ path: exec }) &&
        exec.length > r.path.length && pathEq(exec.slice(0, r.path.length), r.path) && !this.hiddenRow(r)));
      el.classList.toggle("hidden", this.hiddenRow(r));
      const fold = el.querySelector(".pg-fold");
      if (r.container && fold) fold.textContent = this.collapsed.has(JSON.stringify(r.path)) ? "▸" : "▾";
    });
    const row = this.row(), w = $("#pg-where");
    if (w) {
      w.querySelector("option[value=into]").disabled = !(row && row.container);
      if (this.where === "into" && !(row && row.container)) w.value = this.where = "after";
    }
  },
  async insert(type, params = {}) {
    const r = await this.apply(pcmd("program_insert", { type, path: this.path, where: this.where, params }));
    // moves: stay on Insert to keep teaching (jog, insert, jog, insert); other commands need their values set
    if (r.ok && r.selected && type !== "movej" && type !== "move_pose") this.setTab("cmd");
    else if (r.ok) toast(`Added ${this.row()?.label || ""} ${this.row()?.title || type} at the current position`, true);
  },
  // --- Command tab: editor for the selected row
  editor() {
    const box = $("#pg-t-cmd"), row = this.row();
    const key = JSON.stringify(row && [row.path, row.type, row.params, row.waypoint]);
    if (key === this.edKey) return;            // nothing changed: keep what the user is typing
    this.edKey = key;
    if (!row) return box.replaceChildren(h("div", { class: "muted" }, "Select a command in the program"));
    const p = row.params || {}, f = {}, lines = [];
    const field = (label, el) => lines.push(h("span", { class: "k" }, label), el);
    const pctIn = (v) => h("input", { type: "number", step: "5", min: "1", max: "100", value: pct(v) });
    const sel = (opts, v) => h("select", {}, opts.map(([val, lab]) => h("option", { value: val, selected: val === v }, lab)));
    const opt = (el) => (el.value === "" ? null : +el.value);
    // optional TCP speed limit of a linear step (empty = Velocity % only)
    const mmsIn = (v) => h("input", { type: "number", step: "1", min: "0", max: String(INFO.max_cart_speed_mm_s ?? 250),
      value: v ?? "", placeholder: "off", title: `Linear moves: the TCP moves at this constant speed (mm/s, up to ${INFO.max_cart_speed_mm_s ?? 250}). Empty: Velocity % only` });
    const T = row.type;
    if (T === "movej" || T === "move_pose") {
      f.v = pctIn(p.velocity); f.a = pctIn(p.acceleration);
      field("Velocity %", f.v); field("Acceleration %", f.a);
      const lin = T === "move_pose" && p.cartesian;
      if (lin) { f.mm = mmsIn(p.speed_mm_s); field("Max mm/s", f.mm); }
      this.get = () => ({ velocity: Math.min(1, Math.max(0.01, +f.v.value / 100)), acceleration: Math.min(1, Math.max(0.01, +f.a.value / 100)),
        ...(lin ? { speed_mm_s: opt(f.mm) } : {}) });
    } else if (T === "gripper") {
      f.c = sel([["open", "Open"], ["close", "Close"], ["", "Position"]], p.command || "");
      f.p = num(p.position ?? 0, "0.05"); f.e = num(p.effort ?? "", "1"); f.e.placeholder = INFO.gripper?.effort_n ? String(INFO.gripper.effort_n) : "default";
      field("Command", f.c); field("Position (rad)", f.p); field("Effort (N)", f.e);
      this.get = () => ({ command: f.c.value || null, position: f.c.value ? null : +f.p.value, effort: opt(f.e) });
    } else if (T === "wait") {
      f.s = num(p.seconds ?? 1, "0.5"); field("Seconds", f.s);
      this.get = () => ({ seconds: +f.s.value });
    } else if (T === "set_io") {
      f.p = sel([...Array(8).keys()].map((i) => [String(i), `Output ${i}`]), String(p.pin ?? 0));
      f.s = sel([["1", "On"], ["0", "Off"]], p.state === false ? "0" : "1");
      field("Pin", f.p); field("State", f.s);
      this.get = () => ({ pin: +f.p.value, state: f.s.value === "1" });
    } else if (T === "speed") {
      f.s = pctIn(p.fraction ?? 0.5); field("Speed %", f.s);
      this.get = () => ({ fraction: Math.min(1, Math.max(0.01, +f.s.value / 100)) });
    } else if (T === "controller") {
      f.m = sel(INFO.modes.filter((m) => !m.hidden).map((m) => [m.name, m.label || m.name]), p.mode);
      field("Mode", f.m); this.get = () => ({ mode: f.m.value });
    } else if (T === "policy") {
      f.p = sel(INFO.policies.map((x) => [x.name, x.name]), p.policy); f.d = num(p.duration ?? "", "0.5"); f.d.placeholder = "policy default";
      field("Policy", f.p); field("Duration (s)", f.d);
      this.get = () => ({ policy: f.p.value, duration: opt(f.d) });
    } else if (T === "loop") {
      f.t = num(p.times ?? 2, "1"); f.t.min = "1"; field("Repeat", f.t);
      this.get = () => ({ times: Math.max(1, Math.round(+f.t.value)) });
    } else if (T === "folder") {
      f.n = h("input", { type: "text", value: p.name ?? "" }); field("Name", f.n); this.get = () => ({ name: f.n.value });
    } else if (T === "comment") {
      f.t = h("textarea", { rows: "3" }); f.t.value = p.text ?? ""; field("Text", f.t); this.get = () => ({ text: f.t.value });
    } else if (T === "move_relative") {
      const v3 = (vals, step) => vals.map((x) => num(x ?? 0, step));
      f.o = v3(p.offset_mm || [0, 0, 0], "1"); f.r = v3(p.rotate_deg || [0, 0, 0], "1");
      f.fr = sel(forceFrames(true), p.frame || "tool"); f.v = pctIn(p.velocity ?? 0.05);
      f.l = h("input", { type: "checkbox", checked: p.cartesian !== false });
      field("Offset X Y Z mm", h("div", { class: "row v3in" }, f.o)); field("Rotate X Y Z °", h("div", { class: "row v3in" }, f.r));
      f.pv = sel([["tcp", "the TCP (turn the tool in place)"], ["frame", "the frame's origin (orbit, e.g. around the vial axis)"]], p.pivot || "tcp");
      field("Axes of", f.fr); field("Rotate about", f.pv);
      field("Linear", h("label", { class: "chk" }, f.l, "straight line (MoveL)")); field("Velocity %", f.v);
      f.mm = mmsIn(p.speed_mm_s); field("Max mm/s", f.mm);
      this.get = () => ({ offset_mm: f.o.map((x) => +x.value), rotate_deg: f.r.map((x) => +x.value), frame: f.fr.value,
        pivot: f.pv.value === "tcp" ? null : "frame", cartesian: f.l.checked, velocity: Math.min(1, Math.max(0.01, +f.v.value / 100)),
        speed_mm_s: opt(f.mm) });
    } else if (T === "move_until_contact") {
      const k = dirKey(p.direction);
      f.d = sel([...AXIS_DIRS.map(([n]) => [n, n.toUpperCase()]), ...(k ? [] : [["custom", `[${(p.direction || []).join(", ")}]`]])], k || "custom");
      f.fr = sel(forceFrames(true), p.frame || "tool"); f.m = num(p.distance_mm ?? 20, "1"); f.n = num(p.force_n ?? 3, "0.5");
      f.v = pctIn(p.velocity ?? 0.02); f.v.max = "10";
      f.c = sel([["fail", "fail the program"], ["continue", "continue"]], p.on_no_contact || "fail");
      field("Direction", f.d); field("Axes of", f.fr); field("At most mm", f.m); field("Contact at |ΔF| N", f.n);
      f.mm = mmsIn(p.speed_mm_s);
      field("Velocity %", f.v); field("Max mm/s", f.mm); field("No contact", f.c);
      this.get = () => ({ direction: f.d.value === "custom" ? p.direction : AXIS_DIRS.find(([n]) => n === f.d.value)[1],
        frame: f.fr.value, distance_mm: +f.m.value, force_n: +f.n.value,
        velocity: Math.min(0.1, Math.max(0.005, +f.v.value / 100)), speed_mm_s: opt(f.mm), on_no_contact: f.c.value });
    } else if (T === "force") {
      const w = p.wrench || [0, 0, 0, 0, 0, 0];
      f.w = w.map((x, i) => num(x, i < 3 ? "0.5" : "0.05"));
      f.fr = sel(forceFrames(false), p.frame || "tool");
      field("Force X Y Z N", h("div", { class: "row v3in" }, f.w.slice(0, 3)));
      field("Torque X Y Z Nm", h("div", { class: "row v3in" }, f.w.slice(3)));
      field("Axes of", f.fr);
      f.wt = h("input", { type: "checkbox", checked: p.wait !== false });
      field("Wait", h("label", { class: "chk" }, f.wt, "until the force is reached (off: go on while it ramps, e.g. a preload during the next move)"));
      lines.push(h("span"), h("span", { class: "muted small" }, `What the tool exerts, ramped in. Stays on through the next ` +
        `steps until Force off (or the end of the program, STOP, a failed step). Limits ${INFO.force?.max_force_n ?? "?"} N / ` +
        `${INFO.force?.max_torque_nm ?? "?"} Nm. Needs Admittance or Impedance (a Control mode step before it).`));
      this.get = () => ({ wrench: f.w.map((x) => +x.value), frame: f.fr.value, wait: f.wt.checked ? null : false });
    } else if (T === "force_off") {
      f.t = num(p.timeout_s ?? 10, "1"); field("Ramp-out timeout s", f.t);
      this.get = () => ({ timeout_s: +f.t.value });
    } else if (T === "photo") {
      const cams = [...new Map([...(INFO.cameras || []).map((c) => [c.topic, c.name || c.topic]),
        ...(p.topic ? [[p.topic, p.topic]] : [])]).entries()];
      f.t = sel(cams, p.topic); f.l = h("input", { type: "text", value: p.label ?? "", placeholder: "step number", maxlength: "60" });
      field("Camera", f.t); field("Label", f.l);
      lines.push(h("span"), h("span", { class: "muted small" }, "One JPEG named date_time_label.jpg, in the photos folder of the " +
        "recording running now, else in the photos folder next to the recordings."));
      this.get = () => ({ topic: f.t.value, label: f.l.value.trim() || null });
    } else if (T === "record_start") {
      f.p = sel([["all", "Everything for a run"], ["", "Topics ticked in DIAGNOSTICS"]], p.preset || "");
      f.n = h("input", { type: "text", value: p.note ?? "", maxlength: "200", placeholder: "note (optional)" });
      field("Record", f.p); field("Note", f.n);
      lines.push(h("span"), h("span", { class: "muted small" }, "A recording this program starts also ends when it ends or is stopped. " +
        "Already recording: carries on with that one."));
      this.get = () => ({ preset: f.p.value || null, note: f.n.value.trim() || null });
    } else if (T === "record_stop") {
      lines.push(h("span"), h("span", { class: "muted small" }, "Finishes the recording that is running."));
      this.get = () => ({});
    } else if (T === "replay") {
      f.n = h("input", { type: "text", value: p.name ?? "" }); f.s = num(Math.round(100 * (p.speed ?? 1)), "5");
      f.s.min = "5"; f.s.max = "200"; f.v = pctIn(p.velocity ?? 0.1);
      field("Name", f.n); field("Speed % of taught", f.s); field("MoveJ to start %", f.v);
      lines.push(h("span"), h("span", { class: "muted small" }, "Runs the motion taught by hand back with its timing, as one " +
        "trajectory. First a planned MoveJ to its start. Refused if it enters a keep-out zone or leaves the joint limits; " +
        "slowed down (never sped up) if it would exceed a joint speed or acceleration limit. The GUI speed scales it too."));
      this.get = () => ({ name: f.n.value.trim() || null, speed: Math.min(2, Math.max(0.05, +f.s.value / 100)),
        velocity: Math.min(1, Math.max(0.01, +f.v.value / 100)) });
    } else if (T === "zero_ft") {
      lines.push(h("span"), h("span", { class: "muted small" }, "Averages the F/T reading for 1 s with the arm still and makes it " +
        "read zero (software bias, SETUP › F/T + payload). Refused in Admittance or with a force applied: put it before them."));
      this.get = () => ({});
    } else this.get = () => ({});
    if (["movej", "move_pose", "move_relative", "move_until_contact", "replay"].includes(T)) {
      f.of = sel([["stop", "stop the program"], ["continue", "skip it and continue"]], p.on_fail || "stop");
      field("If it fails", f.of);
      const get = this.get;
      this.get = () => ({ ...get(), on_fail: f.of.value === "stop" ? null : "continue" });   // default: not saved
    }
    const kids = [h("div", { class: "title" }, `${row.label} ${row.title}`), h("div", { class: "secondary" }, row.detail)];
    if (row.problems && row.problems.length) kids.push(h("div", { class: "banner-warn" }, row.problems.join("\n")));
    const wp = row.waypoint;
    if (wp) {
      const name = h("input", { type: "text", value: wp.name, style: "width:140px",
        title: "Rename this waypoint" + (wp.users > 1 ? ` (used by ${wp.users} steps: all follow)` : "") });
      name.addEventListener("change", () => this.apply(pcmd("program_rename_waypoint", { old: wp.name, new: name.value })));
      // the waypoint's numbers can be TYPED (e.g. a pose in the vial frame), not only taught: changes apply on leave
      const typed = h("div", { class: "grid", style: "grid-template-columns:auto 1fr" });
      const typeWp = (args) => holds ? this.apply(pcmd("program_type_waypoint", { name: wp.name, ...args }))
        : toast("Take control to edit the program", false);
      let numbers = h("span", { class: "v mono small" }, wp.kind);
      if (wp.joints_deg && !wp.profile) {
        const js = wp.joints_deg.map((v) => num(v, "0.5"));
        numbers = h("div", { class: "row v3in wrap" }, js);
        numbers.addEventListener("change", () => typeWp({ joints_deg: js.map((x) => +x.value) }));
      } else if (wp.xyz_mm) {
        const xyz = wp.xyz_mm.map((v) => num(v, "1")), rpy = wp.rpy_deg.map((v) => num(v, "1"));
        const fr = h("select", {}, [...new Set([wp.frame, INFO.base_link, ...((S && S.calibration.frames) || [])])]
          .map((x) => h("option", { value: x, selected: x === wp.frame }, x)));
        // which TCP goes to this pose: a named one, or "active" (whichever TCP is selected when it runs, UR style)
        const tcps = ((S && S.calibration && S.calibration.tcps) || []).map((t) => t.name);
        const tc = h("select", { title: "Which TCP goes to this pose. Active TCP: whichever is selected (top bar) when it " +
          "runs, so switching to e.g. a spatula tip makes the tip go here. A named TCP: always that one; another active " +
          "TCP then ends up offset (the preview shows where)" },
          [["active", "active TCP (" + ((S && S.calibration && S.calibration.active_tcp) || "?") + ")"],
           ...[...new Set([...(wp.tcp !== "active" ? [wp.tcp] : []), ...tcps])].map((n) => [n, n])]
            .map(([v, l]) => h("option", { value: v, selected: v === wp.tcp }, l)));
        const send = () => typeWp({ frame: fr.value, tcp: tc.value, xyz_mm: xyz.map((x) => +x.value), rpy_deg: rpy.map((x) => +x.value) });
        numbers = h("div", { class: "col", style: "gap:4px" }, h("div", { class: "row v3in" }, xyz), h("div", { class: "row v3in" }, rpy));
        numbers.addEventListener("change", send);
        fr.addEventListener("change", send);
        tc.addEventListener("change", send);
        typed.append(h("span", { class: "k" }, "Frame"), h("div", { class: "row" }, fr,
          h("span", { class: "muted small" }, "numbers are in this frame (typing them keeps the frame)")),
          h("span", { class: "k" }, "TCP"), tc);
      }
      // point the step at another waypoint (e.g. back to "up"): the program's waypoints of the same kind
      const kind = wp.kind, mine = new Map();
      (this.prog?.rows || []).forEach((x) => x.waypoint && !x.waypoint.profile && x.waypoint.kind === kind && mine.set(x.waypoint.name, 1));
      const choices = [...mine.keys(), ...(kind === "joints" ? INFO.waypoints.map((x) => x.name).filter((n) => !mine.has(n)) : [])];
      const pick = h("select", { title: "Move to another waypoint instead (this one stays in the program while used)" },
        choices.map((n) => h("option", { value: n, selected: n === wp.name }, n)));
      pick.addEventListener("change", () => holds ? this.apply(pcmd("program_update", { path: row.path, params: { target: pick.value } }))
        : toast("Take control to edit the program", false));
      typed.prepend(h("span", { class: "k" }, "Go to"), pick);
      typed.prepend(h("span", { class: "k" }, "Waypoint"), wp.profile ? h("span", { class: "v" }, wp.name + " (robot profile)") : name,
        h("span", { class: "k" }, wp.kind === "joints" ? "Joints °" : "XYZ mm / RPY °"), numbers);
      if (wp.users > 1) typed.append(h("span", { class: "k" }, "Used by"), h("span", { class: "v" }, `${wp.users} steps`));
      kids.push(typed);
    }
    kids.push(h("div", { class: "grid", style: "grid-template-columns:auto 1fr" }, lines));
    // edits apply as soon as a field is changed (Enter, leaving the field, or picking from a list), like PolyScope
    const grid = kids[kids.length - 1];
    grid.addEventListener("change", () => {
      if (holds) this.apply(pcmd("program_update", { path: row.path, params: this.get() }));
      else toast("Take control to edit the program", false);
    });
    if (lines.length) kids.push(h("div", { class: "muted small" }, "Changes apply when you leave a field."));
    if (wp) {
      const set = btn("Set waypoint here", "", async () => {
        if (!(await ask(`Re-teach waypoint ${wp.name} from the current robot position? Other steps using it follow.`))) return;
        this.apply(pcmd("program_set_waypoint", { path: row.path }));
      }, "Re-teach this waypoint from the robot's current position");
      const here = holdButton("Hold: Move here", () => {
        const { vel, acc } = scaling();
        return pcmd("program_move_here", { path: row.path, vel, acc, hold: true });
      }, "Hold to move the robot to this waypoint; releasing stops it (like the pendant). " +
         "Speed: OPERATE velocity × GUI speed.");
      set.dataset.lease = here.dataset.lease = "1";
      kids.push(h("div", { class: "row" }, set, here));
    }
    box.replaceChildren(...kids);
  },
  move(d) { if (this.path) this.apply(pcmd("program_move_step", { path: this.path, delta: d })).then((r) => r.ok && r.selected === undefined && this.moved(d)); },
  moved(d) { const p = [...this.path]; p[p.length - 1] += d; this.path = p; this.display(this.prog); },
  dup() { if (this.path) this.apply(pcmd("program_duplicate", { path: this.path })); },
  async del() {
    const r = this.row();
    if (!r) return;
    const kids = this.prog.rows.filter((x) => x.path.length > r.path.length && pathEq(x.path.slice(0, r.path.length), r.path)).length;
    if (kids && !(await ask(`Delete ${r.label} ${r.title} and the ${kids} command(s) inside it? (Undo brings it back.)`))) return;
    this.apply(pcmd("program_delete_step", { path: this.path }));
  },
  async run(step) {
    const start = $("#pg-from").checked && this.path ? this.path : null;
    if (S.runner.state === "PAUSED") return pcmd("program_resume", { step_mode: step });
    if (!INFO.fake && !step && !(await ask(`Run ${this.prog?.name || "program"} on the robot${start ? " from step " + this.row()?.label : ""}?`))) return;
    pcmd("program_run", { start, step_mode: step, loop: $("#pg-loop").checked });
  },
  update(s) {
    if (!this.v3init && $("#pg-v3").offsetParent) { this.v3init = true; this.init3d(); }   // first time PROGRAMS shows
    const st = s.runner.state, idle = st === "IDLE";
    if (this.lastSt !== st) { if (st === "IDLE") this.pathKey = null; this.lastSt = st; if (st === "IDLE") this.paint3d(); }
    if (s.tcp_sig !== this.lastTcp) { const was = this.lastTcp; this.lastTcp = s.tcp_sig; if (was !== undefined) this.paint3d(); }
    $("#pg-state").textContent = { IDLE: "READY", RUNNING: "RUNNING", PAUSED: "PAUSED", STOPPING: "STOPPING" }[st] || st;
    $("#pg-state").className = "chip " + ({ RUNNING: "ok", PAUSED: "warn", STOPPING: "err" }[st] || "");
    gate(this.runBtn, (idle && s.idle) || st === "PAUSED");
    gate(this.stepBtn, (idle && s.idle) || st === "PAUSED");
    gate(this.pauseBtn, st === "RUNNING" || st === "PAUSED");
    this.pauseBtn.textContent = st === "PAUSED" ? "Resume" : "Pause";
    // which iteration each loop around the running step is on: "Loop 9 · 3 / 37"
    const loops = (s.runner.loops || []).map(([lab, k, n]) => `Loop ${lab} · ${k} / ${n}`).join("  ›  ");
    $("#pg-loops").textContent = loops + (s.runner.skipped ? `  · ${s.runner.skipped} skipped` : "");
    $("#pg-loops").classList.toggle("hidden", !loops && !s.runner.skipped);
    // paused by the robot (pendant Pause / Stop, protective stop): say why, and what Resume needs
    const why = st === "PAUSED" ? s.runner.pause_reason : "";
    $("#pg-pause-why").textContent = !why ? "" : s.runner.resume_with_robot ? "resumes when the robot program plays" :
      `paused: ${why}. Resume repeats the interrupted step once the robot runs`;
    this.stopBtn.disabled = idle;           // stopping needs no lease
    gate(this.undoBtn, idle && !!this.undoBtn.dataset.can);
    gate(this.redoBtn, idle && !!this.redoBtn.dataset.can);
    if ($("#pg-jog").contains($("#op-g"))) jogUpdate(s);
    if (this.prog && s.runner.rev != null && s.runner.rev !== this.prog.rev && !this.reloading) {
      this.reloading = true;                                        // changed elsewhere: show the real program
      this.reload().finally(() => (this.reloading = false));
    }
    if (this.view) {
      const q = {};
      (s.joints_deg || []).forEach((v, i) => (q[INFO.joint_names[i]] = v * Math.PI / 180));
      this.view.setJoints({ ...q, ...pages.OPERATE.extraJoints(s) }, s.tcp_off);
    }
    if ($("#pg-grip-pct")) {
      const g = INFO.gripper, pos = s.gripper_position;
      $("#pg-grip-pct").textContent = pos == null ? "" :
        (100 * (pos - g.open_position) / ((g.closed_position - g.open_position) || 1)).toFixed(0) + " % closed";
    }
    const exec = st !== "IDLE" ? s.runner.path : null;
    if (JSON.stringify(exec) !== JSON.stringify(this.lastExec)) this.paintTree();
    this.teachUpdate(s);
    this.batchUpdate(s);
  },

  // --- Teach by hand: Record, move the arm by hand, Mark (waypoints) or Finish (path). Nothing moves by itself.
  buildTeachHand() {
    const sel = (id, opts, onchange) => h("select", { id, onchange }, opts.map(([v, l]) => h("option", { value: v }, l)));
    const tol = (id, value, onchange) => h("input", { type: "number", id, value, step: "0.5", min: "0.1", style: "width:64px", onchange });
    this.tc = {
      rec: btn("● Record", "primary", () => this.teachStart(), "Start teaching. Then move the arm by hand (freedrive)"),
      mark: btn("Mark", "", () => this.apply(cmd("teach_mark")), "Add the current pose as a waypoint and a move step"),
      fin: btn("Finish", "", () => cmd("teach_finish"), "End the recording"),
      cancel: btn("Cancel", "quiet", () => cmd("teach_cancel"), "End teaching; marks already made stay (Undo removes them)"),
      ins: btn("Insert", "primary", () => this.teachInsert(), "Add the path to the program (as a Replay step, or a folder of MoveL steps)"),
      discard: btn("Discard", "quiet", () => cmd("teach_cancel"), "Throw the taught path away"),
    };
    return h("div", { class: "card", id: "pg-tc" }, h("div", { class: "section" }, "Teach by hand"),
      h("div", { class: "muted" }, "Press Record, then move the arm by hand (freedrive). Nothing moves by itself. " +
        "Waypoints: each Mark adds a waypoint and a move. Path: the motion is recorded; Insert it as a Replay (the arm " +
        "moves exactly as you moved it, with your timing) or as a few MoveL points. " +
        "Waypoints are named taught_001, taught_002, … in the frame above. Placed at the selection (Insert tab: Place)."),
      h("div", { class: "row" }, h("span", { class: "muted" }, "Teach"),
        sel("pg-tc-mode", [["waypoints", "Waypoints"], ["path", "Path"]], () => this.teachForm()),
        h("span", { class: "muted", id: "pg-tc-as" }, "as"),
        sel("pg-tc-move", [["linear", "MoveL"], ["joints", "MoveJ"], ["pose", "MoveJ to pose"]]),
        h("label", { class: "chk", id: "pg-tc-autol", title: "Also mark by itself when the arm has stood still for 1 s after moving" },
          h("input", { type: "checkbox", id: "pg-tc-auto" }), "Mark when still 1 s")),
      h("div", { class: "row hidden", id: "pg-tc-tol", title: "How far the MoveL steps may be from the taught path" },
        h("span", { class: "muted" }, "Tolerance"), tol("pg-tc-mm", "1", () => this.teachSimplify()), h("span", { class: "muted" }, "mm"),
        tol("pg-tc-deg", "2", () => this.teachSimplify()), h("span", { class: "muted" }, "°")),
      h("div", { class: "row" }, this.tc.rec, this.tc.mark, this.tc.fin, this.tc.cancel),
      h("div", { class: "mono small", id: "pg-tc-status" }),
      h("div", { class: "col hidden", id: "pg-tc-review" },
        h("div", { class: "secondary", id: "pg-tc-res" }),
        h("div", { class: "row", title: "Replay: one step that runs the recorded joint motion back with its timing " +
            "(a MoveJ to its start first; slowed down if it would exceed a joint limit). MoveL points: a few straight " +
            "moves within the tolerance (needed for pressing with a force)" },
          h("span", { class: "muted" }, "Insert as"),
          sel("pg-tc-kind", [["replay", "Replay (as taught)"], ["movel", "MoveL points"]], () => this.teachKind()),
          h("span", { class: "muted", id: "pg-tc-rsl" }, "Speed %"),
          h("input", { type: "number", id: "pg-tc-rspeed", step: "5", min: "5", max: "200", value: "100", style: "width:64px",
            title: "Of the taught timing: 100 = as taught, 50 = half as fast" }),
          h("div", { class: "grow" }), this.tc.ins, this.tc.discard),
        h("div", { class: "row", id: "pg-tc-mms" }, h("span", { class: "muted" }, "Max mm/s"),
          h("input", { type: "number", id: "pg-tc-speed", step: "1", min: "0", style: "width:72px",
            title: "TCP speed limit for the MoveL steps (empty: no limit). Suggested: your average speed while teaching" })),
        h("div", { class: "row", id: "pg-tc-pressrow", title: "Replay the path pressing with this force (Admittance): it starts 10 mm off, " +
            "moves until contact, applies the force, follows the path, lets go and backs off. Empty: no force" },
          h("span", { class: "muted" }, "Press N"),
          h("input", { type: "number", id: "pg-tc-press", step: "0.5", min: "0", placeholder: "off", style: "width:72px" }),
          sel("pg-tc-pdir", [["tool", "along tool Z"], ["down", "down"]]))));
  },
  teachForm() {
    const path = $("#pg-tc-mode").value === "path";
    for (const id of ["#pg-tc-as", "#pg-tc-move", "#pg-tc-autol"]) $(id).classList.toggle("hidden", path);
    this.teachKind();
  },
  teachKind() {                                  // Replay: Speed %; MoveL points: tolerance, Max mm/s, Press
    const path = $("#pg-tc-mode").value === "path", rep = $("#pg-tc-kind").value === "replay";
    const review = !!(S && S.teach && S.teach.phase === "review");
    $("#pg-tc-tol").classList.toggle("hidden", !path || (review && rep));
    for (const id of ["#pg-tc-mms", "#pg-tc-pressrow"]) $(id).classList.toggle("hidden", rep);
    for (const id of ["#pg-tc-rsl", "#pg-tc-rspeed"]) $(id).classList.toggle("hidden", !rep);
  },
  teachTol() { return { pos_tol_mm: +$("#pg-tc-mm").value, rot_tol_deg: +$("#pg-tc-deg").value }; },
  teachStart() {
    const path = $("#pg-tc-mode").value === "path";
    cmd("teach_start", { mode: path ? "path" : "waypoints", move: $("#pg-tc-move").value, frame: $("#pg-frame").value,
      auto: !path && $("#pg-tc-auto").checked, path: this.path, where: this.where, ...this.teachTol() });
  },
  teachSimplify() { if (S && S.teach && S.teach.phase === "review") cmd("teach_simplify", this.teachTol()); },
  teachInsert() {
    if ($("#pg-tc-kind").value === "replay") {
      const sp = $("#pg-tc-rspeed").value.trim();
      return this.apply(cmd("teach_insert", { kind: "replay", speed_pct: sp === "" ? null : +sp, path: this.path, where: this.where }));
    }
    const v = $("#pg-tc-speed").value.trim();
    const f = $("#pg-tc-press").value.trim();
    this.apply(cmd("teach_insert", { kind: "movel", ...this.teachTol(), speed_mm_s: v === "" ? null : +v, path: this.path, where: this.where,
      press_n: f === "" || +f === 0 ? null : +f, press_dir: $("#pg-tc-pdir").value }));
  },
  teachUpdate(s) {
    const t = s.teach, review = !!(t && t.phase === "review" && t.result), r = review ? t.result : null;
    gate(this.tc.rec, !t && s.idle && s.runner.state === "IDLE" && !(s.batch && s.batch.active));
    gate(this.tc.mark, !!t && t.mode === "waypoints");
    gate(this.tc.fin, !!t && t.phase === "recording");
    gate(this.tc.cancel, !!t && !review);
    gate(this.tc.ins, review && s.runner.state === "IDLE");
    gate(this.tc.discard, review);
    $("#pg-tc-mode").disabled = $("#pg-tc-move").disabled = $("#pg-tc-auto").disabled = !!t;
    if (t && $("#pg-tc-mode").value !== t.mode) { $("#pg-tc-mode").value = t.mode; this.teachForm(); }
    if (t && document.activeElement !== $("#pg-tc-mm") && document.activeElement !== $("#pg-tc-deg")) {
      $("#pg-tc-mm").value = t.pos_tol_mm; $("#pg-tc-deg").value = t.rot_tol_deg;
    }
    let line = "";
    if (t && t.mode === "waypoints")
      line = `Teaching waypoints (${t.move_label}) in ${t.frame} · ${t.marks} marked` + (t.last ? ` · last ${t.last}` : "") +
        (t.auto ? " · marks when still 1 s" : "");
    else if (t && !review)
      line = `● Recording the path in ${t.frame} · ${Math.round(t.elapsed_s)} s · ${t.samples} samples` +
        (t.full ? " · full: press Finish" : "") + (t.missed ? ` · ${t.missed} missed` : "");
    else if (review) line = "Check the path in the 3D view, then Insert or Discard.";
    if (t && t.by && !holds) line += ` · by ${t.by}`;
    if (t && t.error) line += ` · ${t.error}`;
    $("#pg-tc-status").textContent = line;
    $("#pg-tc-review").classList.toggle("hidden", !review);
    if (review !== !!this.tcWasReview) { this.tcWasReview = review; this.teachKind(); }
    if (review) {
      const rp = r.replay || {};
      $("#pg-tc-res").textContent = (rp.points ? `Replay: ${rp.points} points, ${rp.duration_s.toFixed(1)} s of motion ` +
        `(${rp.samples} joint samples). ` : `Replay not possible: ${rp.error || "no joint states"}. `) +
        `MoveL: ${r.points} points from ${r.samples} samples · max ${r.max_mm.toFixed(1)} mm / ` +
        `${r.max_deg.toFixed(1)}° off the taught path · ${r.length_mm.toFixed(0)} mm in ${r.duration_s.toFixed(1)} s · ` +
        `average ${r.avg_mm_s.toFixed(0)} mm/s while moving`;
      if (!this.tcFilled) { $("#pg-tc-kind").value = rp.points ? "replay" : "movel"; this.teachKind(); }
      if (!this.tcFilled) $("#pg-tc-speed").value = r.speed_mm_s ?? "";    // suggest the taught speed once per review
      this.tcFilled = true;
    } else this.tcFilled = false;
    const key = review ? `${r.samples}|${r.points}|${r.max_mm}|${r.max_deg}` : "";
    if (key !== (this.teachKey || "")) {
      this.teachKey = key;
      this.teachPrev = review ? r : null;
      this.paint3d();
    }
  },

  // --- Batch: run the program several times, one thing changed per run (vial profile, one step parameter, nothing)
  buildBatch() {
    const sel = (id, opts, onchange) => h("select", { id, onchange }, opts.map(([v, l]) => h("option", { value: v }, l)));
    const lab = (t) => h("span", { class: "muted" }, t);
    this.bt = {
      save: btn("Save", "", () => this.batchSave(), "Save the batch next to the program (<name>.batch.yaml)"),
      load: btn("Load saved", "quiet", () => this.batchLoad(), "Load the batch saved next to the program"),
      check: btn("Check", "quiet", () => this.batchCheck()),
      run: btn("▶ Run batch", "primary", () => this.batchRun()),
      stop: btn("■ Stop batch", "danger", () => cmd("batch_stop"), "End the run and the batch now"),
    };
    this.bt.save.dataset.lease = "1";
    return h("div", { class: "col hidden", id: "pg-t-batch" },
      h("div", { class: "muted" }, "Run the program several times with one thing changed per run. Each run is a normal program " +
        "run with the usual checks. STOP ends the whole batch. The vial and the program are put back afterwards."),
      h("div", { class: "row" }, lab("Change per run"),
        sel("pg-b-kind", [["repeat", "Nothing (repeat)"], ["vials", "Vial profile"], ["sweep", "One step parameter"]], () => this.batchForm())),
      h("div", { class: "row", id: "pg-b-repeat" }, lab("Runs"),
        h("input", { type: "number", id: "pg-b-n", value: "3", min: "1", step: "1", style: "width:72px" })),
      h("div", { class: "col hidden", id: "pg-b-vials" },
        h("div", { class: "row", id: "pg-b-vlist" }),
        h("div", { class: "row" }, lab("Use as fixture"), h("input", { type: "text", id: "pg-b-fx", value: "vial", style: "width:110px",
          title: "The fixture frame the program uses; each run makes the next vial profile this fixture" }),
          btn("↻", "quiet", () => this.batchVials(), "Reload the vial profiles"))),
      h("div", { class: "col hidden", id: "pg-b-sweep" },
        h("div", { class: "row" }, lab("Step"), h("select", { id: "pg-b-step", class: "grow", onchange: () => this.batchParams() })),
        h("div", { class: "row" }, lab("Parameter"), h("select", { id: "pg-b-param", onchange: () => this.batchNow() }),
          h("span", { class: "muted small", id: "pg-b-now" })),
        h("div", { class: "row" }, lab("Values"), h("input", { type: "text", id: "pg-b-vals", class: "grow", placeholder: "e.g. 2, 3, 4" }))),
      h("div", { class: "row" },
        h("label", { class: "chk" }, h("input", { type: "checkbox", id: "pg-b-pause" }), "Ask before each next run"),
        lab("On failure"), sel("pg-b-fail", [["stop", "stop"], ["continue", "go on"]])),
      h("div", { class: "row" }, this.bt.save, this.bt.load, this.bt.check, h("div", { class: "grow" }), this.bt.run, this.bt.stop),
      h("div", { class: "secondary", id: "pg-b-status" }),
      h("div", { class: "scroll", style: "max-height:300px" },
        h("table", { class: "hist" }, h("thead", {}, h("tr", {}, ["#", "Changed", "Result", "Time", "Recording"].map((x) => h("th", {}, x)))),
          h("tbody", { id: "pg-b-rows" }))));
  },
  batchForm() {
    const k = $("#pg-b-kind").value;
    $("#pg-b-repeat").classList.toggle("hidden", k !== "repeat");
    $("#pg-b-vials").classList.toggle("hidden", k !== "vials");
    $("#pg-b-sweep").classList.toggle("hidden", k !== "sweep");
  },
  async batchVials() {
    const r = await cmd("vial_profiles");
    if (!r.ok) return;
    const was = new Set([...document.querySelectorAll("#pg-b-vlist input:checked")].map((x) => x.value).concat(this.bWant || []));
    this.bWant = null;
    $("#pg-b-vlist").replaceChildren(...(r.vials.length ? r.vials.map((v) => h("label", { class: "chk" },
      h("input", { type: "checkbox", value: v.name, checked: was.has(v.name) }), v.name))
      : [h("span", { class: "muted" }, "No vial profiles yet (SETUP › Vial).")]));
  },
  batchSteps() {
    if (!this.prog) return;
    const sel = $("#pg-b-step"), was = this.bStepWant || sel.value;
    this.bStepWant = null;
    const rows = this.prog.rows.filter((r) => r.sweep && r.sweep.length);
    sel.replaceChildren(...(rows.length ? rows.map((r) => h("option", { value: JSON.stringify(r.path), selected: JSON.stringify(r.path) === was },
      `${r.label} ${r.title}`)) : [h("option", { value: "" }, "no step with a number to change")]));
    this.batchParams(this.bParamWant);
    this.bParamWant = null;
  },
  bRow() { const v = $("#pg-b-step").value; return v ? this.row(JSON.parse(v)) : null; },
  batchParams(want) {
    const r = this.bRow(), sel = $("#pg-b-param"), was = want || sel.value;
    sel.replaceChildren(...(r ? r.sweep : []).map((o) => h("option", { value: o.key, selected: o.key === was }, o.label)));
    this.batchNow();
  },
  batchNow() {
    const r = this.bRow(), o = r && r.sweep.find((x) => x.key === $("#pg-b-param").value);
    $("#pg-b-now").textContent = o ? `now ${o.value ?? "not set"}` : "";
  },
  batchDef() {
    const r = this.bRow();
    return { kind: $("#pg-b-kind").value, repeats: +$("#pg-b-n").value, fixture: $("#pg-b-fx").value.trim(),
      vials: [...document.querySelectorAll("#pg-b-vlist input:checked")].map((x) => x.value),
      step: r ? r.path : null, step_type: r ? r.type : "", param: $("#pg-b-param").value, values: $("#pg-b-vals").value,
      pause: $("#pg-b-pause").checked, on_fail: $("#pg-b-fail").value };
  },
  batchFill(d) {
    $("#pg-b-kind").value = d.kind; this.batchForm();
    if (d.repeats) $("#pg-b-n").value = d.repeats;
    $("#pg-b-fx").value = d.fixture || "vial";
    $("#pg-b-pause").checked = !!d.pause;
    $("#pg-b-fail").value = d.on_fail || "stop";
    if (d.kind === "vials") {
      document.querySelectorAll("#pg-b-vlist input").forEach((x) => (x.checked = false));
      this.bWant = d.vials || []; this.batchVials();
    }
    if (d.kind === "sweep") {
      this.bStepWant = JSON.stringify(d.step); this.bParamWant = d.param; this.batchSteps();
      $("#pg-b-vals").value = (d.values || []).join(", ");
    }
  },
  async batchSave() { await cmd("batch_save", { batch: this.batchDef() }); },
  async batchLoad() {
    const r = await cmd("batch_get");
    if (r.ok && r.batch) this.batchFill(r.batch);
    else if (r.ok) toast(r.msg, false);
  },
  async batchCheck() {
    const r = await cmd("batch_check", { batch: this.batchDef() });
    if (r.ok) toast(r.problems.length ? "Batch check found problems:\n• " + r.problems.join("\n• ") : `Batch OK: ${r.runs} runs.`,
      !r.problems.length, !!r.problems.length);
    return r;
  },
  async batchRun() {
    const batch = this.batchDef();
    const c = await cmd("batch_check", { batch });
    if (!c.ok) return;
    if (c.problems.length) { toast("Batch check found problems:\n• " + c.problems.join("\n• "), false, true); return; }
    if (!INFO.fake && !(await ask(`Run ${this.prog?.name || "the program"} ${c.runs} times on the robot?\n\n` +
      "Each run is a normal program run. STOP ends the whole batch.", { ok: "Run" }))) return;
    cmd("batch_run", { batch });
  },
  batchUpdate(s) {
    const b = s.batch;
    if (!b) return;
    gate(this.bt.run, !b.active && s.idle && s.runner.state === "IDLE" && !s.teach);
    this.bt.stop.disabled = !b.active;                 // stopping needs no lease
    const k = Math.min(b.done + 1, b.total);
    $("#pg-b-status").textContent = b.active
      ? `Batch of ${b.name}: run ${k} of ${b.total}` + (b.phase === "waiting" ? " · waiting for the person in control" : "")
      : b.result ? (b.result[0] ? "✓ " : "✗ ") + b.result[1] + (b.restored ? ` · ${b.restored}` : "") : "";
    const key = JSON.stringify(b.rows);
    if (key === this.bRowsKey) return;
    this.bRowsKey = key;
    $("#pg-b-rows").replaceChildren(...b.rows.map((r) => {
      const rec = r.recording || "", base = rec.split("/").filter(Boolean).pop() || "";
      return h("tr", {}, h("td", {}, String(r.n)), h("td", {}, r.change),
        h("td", { style: r.ok === false ? "color:var(--danger)" : "" }, r.ok == null ? "running…" : (r.ok ? "✓ " : "✗ ") + (r.result || "")),
        h("td", { class: "mono" }, r.duration_s != null ? r.duration_s.toFixed(1) + " s" : ""),
        h("td", { class: "mono", title: rec }, base));
    }));
  },
};

// =============================================================== AUTOMATION
pages.AUTOMATION = {
  sel: null, reqT: 0,
  build(el) {
    const list = h("div", { class: "list", id: "au-list" }, INFO.policies.length ? INFO.policies.map((p) =>
      h("div", { class: "item", onclick: () => this.select(p.name) }, h("span", {}, p.name)))
      : h("div", { class: "muted" }, "No policy manifests in the policy folder."));
    const tri = (id, label) => h("label", { class: "chk" }, label,
      h("select", { id }, h("option", { value: "" }, "config default"), h("option", { value: "1" }, "on"), h("option", { value: "0" }, "off")));
    el.append(h("div", { class: "cols" },
      h("div", { class: "card", style: "width:200px" }, h("div", { class: "section" }, "Policies"), list),
      h("div", { class: "card", style: "flex:1;min-width:300px" },
        h("div", { class: "title", id: "au-title" }, "Select a policy"),
        h("div", { class: "secondary", id: "au-desc" }), h("div", { class: "muted", id: "au-facts" }),
        h("div", { class: "secondary", id: "au-flow" }),
        h("details", {}, h("summary", { class: "muted" }, "Recording"),
          h("div", { class: "row" }, tri("au-obs", "Observations "), tri("au-vid", "Video ")))),
      h("div", { class: "card", style: "width:340px" },
        h("div", { class: "row" }, h("span", { class: "dot", id: "au-dot" }), h("span", { class: "title", id: "au-state" }, "—")),
        h("div", { class: "col", id: "au-req", style: "gap:4px" }),
        h("div", { id: "au-live", class: "col hidden" },
          h("div", { class: "muted", id: "au-live-title" }),
          h("div", { class: "bar" }, h("div", { id: "au-bar" })),
          h("div", { class: "grid", id: "au-fb" })),
        h("div", { class: "secondary", id: "au-result" }),
        h("div", { class: "muted", id: "au-hint" }),
        h("div", { style: "flex:1" }),
        h("div", { class: "row" },
          btn("Prepare", "", () => cmd("policy_prepare", { name: this.sel, recording: this.rec() })),
          btn("▶ Run", "primary", () => this.run()),
          btn("■ Stop policy", "stopo", () => cmd("policy_stop"), "Stop the policy (the red STOP in the top bar stops everything)")))));
    [this.prep, this.runB, this.stopB] = el.querySelectorAll(".card:last-child .row:last-child .btn");
    if (INFO.policies.length) this.select(INFO.policies[0].name);
  },
  rec() {
    const v = (id) => ($(id).value === "" ? null : $(id).value === "1");
    return { record_obs: v("#au-obs"), record_video: v("#au-vid") };
  },
  select(name) {
    this.sel = name;
    const p = INFO.policies.find((x) => x.name === name);
    [...$("#au-list").children].forEach((it) => it.classList.toggle("on", it.textContent === name));
    $("#au-title").textContent = name;
    $("#au-desc").textContent = p.description || "";
    $("#au-facts").textContent = [p.algorithm, p.hz && p.hz + " Hz", p.duration && p.duration + " s"].filter(Boolean).join(" · ");
    const m = INFO.modes.find((x) => x.name === p.mode);
    $("#au-flow").textContent = `Backend ${p.backend} · needs ${m ? m.label : p.mode}${m && m.direct_torque ? " (DIRECT TORQUE)" : ""}` +
      (p.tcp ? ` · TCP ${p.tcp}` : "");
    this.reqT = 0;
  },
  async run() {
    const p = INFO.policies.find((x) => x.name === this.sel), m = INFO.modes.find((x) => x.name === p.mode);
    if (!INFO.fake && !(await ask(`Run ${p.name} on the robot?` + (m && m.direct_torque ? "\n\nThis commands DIRECT TORQUE." : "")))) return;
    cmd("policy_run", { name: this.sel, recording: this.rec() });
  },
  async requirements() {
    if (!this.sel || Date.now() - this.reqT < 1500) return;
    this.reqT = Date.now();
    const r = await cmd("policy_requirements", { name: this.sel });
    if (!r.ok) return;
    this.reqOk = r.requirements.every((q) => q.state !== "fail");
    const key = JSON.stringify(r.requirements);
    if (key === this.reqKey) return;                    // keep buttons stable (a click must not hit a rebuilt row)
    this.reqKey = key;
    $("#au-req").replaceChildren(...r.requirements.map((q) => {
      let fix = null;
      if (q.fix) {
        fix = btn(q.fix.label, "fix", async () => {
          if (q.fix.page) { if (q.fix.hint) toast(q.fix.hint, true); return navTo(q.fix.page); }
          if (q.fix.confirm && !(await ask(q.fix.confirm))) return;
          await cmd(q.fix.cmd, q.fix.args);
          this.reqT = 0;
        });
        if (q.fix.cmd) fix.dataset.lease = "1";
      }
      return h("div", { class: "row", style: "gap:6px", title: q.state === "runner" ? q.detail : "" },
        h("span", { class: "dot " + (q.state === "ok" ? "ok" : q.state === "fail" ? "bad" : "") }),
        h("span", {}, q.label), q.state === "fail" && q.detail ? h("span", { class: "muted error" }, q.detail) : null,
        q.state === "runner" ? h("span", { class: "muted" }, "(checked when the run starts)") : null, fix);
    }));
  },
  update(s) {
    this.requirements();
    const pol = s.policy, fb = pol.feedback, busy = pol.busy, running = busy && !!pol.current, preparing = busy && !pol.current;
    const prepared = !!this.sel && pol.prepared === this.sel;      // (no policy picked: nothing is ready to run)
    const state = running ? "RUNNING" : preparing ? "PREPARING…" : !this.sel ? "NO POLICY PICKED" : prepared ? "READY TO RUN"
      : this.reqOk ? "NOT PREPARED" : "NOT READY";
    $("#au-state").textContent = state;
    $("#au-dot").className = "dot " + (running || prepared ? "ok" : !this.sel ? "off" : preparing || this.reqOk ? "warn" : "bad");
    this.prep.title = this.reqOk ? "Moves to the start, measures the force baseline, checks the policy files" :
      "Fix the red checks first";
    this.prep.textContent = pol.result && !prepared ? "Prepare again" : "Prepare";
    $("#au-hint").textContent = !this.sel ? "" : running || preparing ? "" : prepared ? "Press ▶ Run."
      : this.reqOk ? "Step 1: Prepare. Step 2: Run." : "Fix the red checks (buttons on the right of each), then Prepare.";
    gate(this.prep, !!this.sel && this.reqOk && s.idle);
    gate(this.runB, prepared && this.reqOk && s.idle);
    this.stopB.classList.toggle("hidden", !busy);
    this.stopB.textContent = preparing ? "■ Cancel prepare" : "■ Stop policy";
    this.stopB.disabled = false;
    $("#au-live").classList.toggle("hidden", !(fb && (busy || fb.state === "baseline")));
    if (fb) {
      const p = INFO.policies.find((x) => x.name === (pol.current || this.sel));
      if (fb.state === "baseline") {
        $("#au-live-title").textContent = `Force startup baseline ${fb.baseline_count} / ${fb.baseline_total}`;
        $("#au-bar").style.width = (100 * fb.baseline_count / (fb.baseline_total || 1)) + "%";
      } else {
        const d = p && p.duration;
        $("#au-live-title").textContent = `Elapsed ${fmt(fb.elapsed)}${d ? " / " + d : ""} s`;
        $("#au-bar").style.width = d ? Math.min(100, 100 * fb.elapsed / d) + "%" : "0";
      }
      const rows = [];
      if (fb.action) rows.push(["Policy output", fb.action.map((v, i) => `<span title="action[${i}]">a${i} ${fmt(v, 2)}</span>`).join("  ")]);
      if (fb.wrench) AXES.slice(0, 3).forEach((a, i) => rows.push([a, fmt(fb.wrench[i], 2) + " N"]));
      $("#au-fb").innerHTML = rows.map(([k, v]) => `<span class="k">${k}</span><span class="v">${v}</span>`).join("");
    }
    if (busy && !pol.supports_feedback) $("#au-live-title").textContent = "No live feedback from the subprocess runner.";
    const r = pol.result;
    $("#au-result").textContent = r ? (r.success ? "Finished: " : "Stopped: ") + r.reason + (r.log_dir ? ` (log ${r.log_dir})` : "") : "";
  },
};

// ==================================================================== SETUP
pages.SETUP = {
  sub: "Setups",
  // the menu in the order a new user needs it: what is on the robot, then calibrate it, then the cell around it
  GROUPS: [["Robot", ["Setups", "Parts", "TCPs"]], ["Calibration", ["Camera", "F/T + payload", "Vial", "Flask"]],
    ["Cell", ["Cameras", "Keep-out zones", "Gains"]]],
  build(el) {
    this.nav = h("div", { class: "list subnav" }, this.GROUPS.flatMap(([g, ns]) => [h("div", { class: "navgrp" }, g),
      ...ns.map((n) => h("div", { class: "item", "data-name": n, onclick: () => this.show(n) },
        h("span", { class: "grow" }, n), h("span", { class: "nb" })))]));
    this.body = h("div", { class: "grow col" });
    // one bar for every SETUP page: saved changes that the robot model (URDF) only gets at a robot stack restart
    this.mbar = h("div", { class: "banner-warn row hidden", id: "su-modelbar" }, h("span", { class: "grow", id: "su-modelbar-msg" }),
      (() => { const b = btn("Restart robot stack…", "", () => this.wc.restart("stack"),
        "Driver, controllers, MoveIt and the GUI restart (about a minute) and the robot model is rebuilt"); b.dataset.lease = "1"; return b; })());
    el.append(h("div", { class: "cols" }, h("div", { class: "card subnav" }, this.nav), h("div", { class: "grow col" }, this.mbar, this.body)));
    this.views = {};
    // TCPs: one list, one editor. Name and parent once, then type the offset or measure the tip (pivot)
    const lb = (b) => (b.dataset.lease = "1", b);
    const n3 = (i, step) => h("input", { type: "number", id: `su-to${i}`, value: "0", step, style: "width:80px" });
    this.tcpHow = "type";
    const howBtn = (k, label) => h("button", { class: "btn" + (k === "type" ? " on" : ""), "data-how": k, onclick: () => this.tcpSetHow(k) }, label);
    this.views["TCPs"] = h("div", { class: "col su-main" },
      h("div", { class: "card" },
        h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "TCPs"),
          btn("+ New TCP", "", () => this.tcpNew(), "Clear the editor below for a new TCP")),
        note("A TCP is the tool point the robot moves: tool0 (the flange), robotiq_tcp, or your own, e.g. a spatula tip.",
          "Your own TCP is a fixed offset from a parent TCP or link. Recorded poses store their TCP. A TCP changes poses only, " +
          "not MoveIt's collision model (for that, add the tool as a part)."),
        h("div", { class: "list", id: "su-tcps" })),
      h("div", { class: "card" },
        h("div", { class: "section", id: "su-t-title" }, "New TCP"),
        h("div", { class: "grid", style: "grid-template-columns:auto 1fr" },
          h("span", { class: "k" }, "Name"), h("input", { type: "text", id: "su-tn", placeholder: "spatula_tip" }),
          h("span", { class: "k" }, "From"), h("select", { id: "su-tp", title: "The parent: the offset is measured from this frame",
            onchange: () => this.tcpHow === "pivot" && this.pivot?.samples.length && this.pivotReset() }),
          h("span", { class: "k" }, "How"), h("div", {}, h("span", { class: "seg", id: "su-t-how" },
            howBtn("type", "Type the offset"), howBtn("pivot", "Measure the tip (pivot)")))),
        h("div", { id: "su-t-type" },
          h("div", { class: "grid", style: "grid-template-columns:auto 1fr" },
            h("span", { class: "k" }, "X Y Z mm"), h("div", { class: "row" }, [0, 1, 2].map((i) => n3(i, "0.5"))))),
        h("div", { id: "su-t-pivot", class: "hidden" },
          h("ol", { class: "small steps" },
            h("li", {}, "Freedrive on. Hold the tool tip on one fixed point (a cone, a screw head)."),
            h("li", {}, "Keep the tip there, tilt the tool to a new angle, press Capture. 6–12 times, about two different axes."),
            h("li", {}, "Solve and check the error (under 1 mm is good), then Save TCP.")),
          h("div", { class: "row" }, freedriveButton(),
            lb(btn("Capture", "", () => this.pivotCmd("pivot_capture", { parent: $("#su-tp").value }), "Reads where the tool is now. Nothing moves")),
            lb(btn("Solve", "", () => this.pivotCmd("pivot_solve"))),
            h("div", { class: "grow" }),
            lb(btn("Start over", "quiet", () => this.pivotReset(), "Forget every sample"))),
          h("div", { class: "row wrap", id: "su-pivot-samples" }),
          h("div", { class: "muted small", id: "su-pivot-quality" }, "No samples"),
          h("div", { class: "mono small", id: "su-pivot-result" }, "")),
        h("div", { class: "grid", style: "grid-template-columns:auto 1fr" },
          h("span", { class: "k" }, "R P Y °"), h("div", { class: "row" }, [3, 4, 5].map((i) => n3(i, "1")))),
        h("div", { class: "muted small", id: "su-t-rpyhint" }, "Rotation of the TCP from its parent (0 0 0 = the same axes)."),
        h("div", { class: "row" },
          (() => { const b = lb(btn("Save TCP", "primary", () => this.tcpHow === "pivot" ? this.pivotSave() : this.saveTcp())); b.id = "su-t-save"; return b; })(),
          h("span", { class: "muted small" }, "Saving a name that exists changes that TCP."))));
    // Vial
    const vrows = INFO.vial_points.map((n, k) => h("div", { class: "row" }, h("span", { class: "dot", id: `su-vd-${k}` }),
      h("span", { style: "width:130px" }, n + (n === "Bottom" ? " (optional)" : "")),
      h("span", { class: "mono muted grow", id: `su-vv-${k}` }, "—"),
      btn("Capture", "", () => cmd("vial_capture", { point: n }).then((r) => r.ok && this.vial(r.vial)))));
    vrows.forEach((r) => (r.lastChild.dataset.lease = "1"));
    this.views["Vial"] = h("div", { class: "col su-main" },
      h("div", { class: "card" }, h("div", { class: "row" }, h("span", { class: "section grow" }, "Touch points"), freedriveButton()),
        h("div", { class: "muted" }, "In freedrive, touch the TCP to the rim at three points spread around it and capture each. Optionally touch the bottom."),
        vrows, h("div", {}, btn("Reset", "quiet", () => cmd("vial_reset").then((r) => r.ok && this.vial(r.vial))))),
      h("div", { class: "card" }, h("div", { class: "section" }, "Vial"),
        h("div", { class: "grid", id: "su-vres" }), h("div", { class: "error", id: "su-verr" }),
        note("XY circle in the base frame.", "XY circle in the base frame, assuming a level rim and a vertical vial. Without a bottom touch, Z is the lowest rim point."),
        h("div", { class: "row" }, h("input", { type: "text", id: "su-vname", value: "vial", style: "flex:1" }),
          btn("Save as fixture", "primary", () => cmd("vial_save", { name: $("#su-vname").value }))),
        h("div", { class: "row" }, btn("Save as vial profile…", "", async () => {
          const n = (await askText("Name of this vial (e.g. 'vial A', 'rack 2'). It is kept in the vial profiles list:", ""));
          if (n) cmd("vial_profile_save", { name: n, source: "touch", fixture: $("#su-vname").value || "vial",
            use: (await ask(`Also use it as "${$("#su-vname").value || "vial"}" now (the frame programs use)?`)) })
            .then((r) => r.ok && this.vialProfiles(r.vials));
        }, "Keep the fitted vial as a named profile, to switch back to it later"))),
      // several vials: named profiles (built-in baseline = scraping.py's), one of them is the fixture programs use
      h("div", { class: "card" },
        h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "Vial profiles"),
          h("span", { class: "muted small" }, "Use as"),
          h("input", { type: "text", id: "su-vp-as", value: "vial", style: "width:90px",
            title: "The fixture a profile becomes when you press Use. Programs use \"vial\"; a second vial in the " +
              "same program could be vial_2" })),
        h("table", { class: "su-ptable" }, h("thead", {}, h("tr", {}, ["Name", "Centre (bottom) mm", "R mm", "H mm", "Source", "In use as", ""]
          .map((x) => h("th", {}, x)))), h("tbody", { id: "su-vp-list" })),
        h("div", { class: "row wrap", style: "margin-top:6px" },
          h("input", { type: "text", id: "su-vp-name", placeholder: "name", style: "width:110px" }),
          ...["x", "y", "z"].map((a) => h("input", { type: "number", id: `su-vp-${a}`, placeholder: a + " mm", step: "0.5", style: "width:80px" })),
          h("input", { type: "number", id: "su-vp-r", placeholder: "radius", step: "0.5", style: "width:70px" }),
          h("input", { type: "number", id: "su-vp-h", placeholder: "height", step: "0.5", style: "width:70px" }),
          btn("Add typed", "", () => cmd("vial_profile_save", { name: $("#su-vp-name").value, source: "typed",
            xyz_mm: ["x", "y", "z"].map((a) => +$(`#su-vp-${a}`).value), radius_mm: $("#su-vp-r").value || null,
            height_mm: $("#su-vp-h").value || null }).then((r) => r.ok && this.vialProfiles(r.vials)),
            "A vial whose position you know: centre of its bottom in the robot base frame"),
          btn("Save current fixture…", "quiet", async () => {
            const f = $("#su-vp-as").value || "vial";
            const n = (await askText(`Save the fixture "${f}" as a vial profile named:`, ""));
            if (n) cmd("vial_profile_save", { name: n, source: "fixture", fixture: f }).then((r) => r.ok && this.vialProfiles(r.vials));
          }, "Keep the vial that is taught now (e.g. before teaching another one)")),
        note("Use makes a profile the fixture (\"vial\") that programs, keep-out boxes and the preview use.",
          "Centre of the vial bottom in the robot base frame, base axes. baseline is built in: scraping.py's target_xyz " +
          "(only right if your vial is really there). Saved profiles are kept per robot; Use is refused while a program runs.")));
    this.vsave = this.views["Vial"].querySelector(".btn.primary");
    this.views["Vial"].querySelectorAll(".btn").forEach((b) => (b.dataset.lease = "1"));
    // Keep-out zones: MoveIt collision boxes (planning scene objects gui_keepout/<name>)
    const v3 = (id, step) => h("div", { class: "row" }, [0, 1, 2].map((i) =>
      h("input", { type: "number", id: `${id}${i}`, value: "0", step, style: "width:80px" })));
    this.views["Keep-out zones"] = h("div", { class: "col su-main" },
      h("div", { class: "card" },
        h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "Keep-out zones"),
          (() => { const b = btn("Re-apply", "quiet", () => cmd("keepout_apply"),
            "Send every enabled box to MoveIt again (e.g. after restarting move_group)"); b.id = "su-ko-apply"; return b; })()),
        note("Boxes MoveIt plans around (table, holders, camera mounts). They never move the robot.",
          "Each box hangs on the base frame or a fixture (it follows the fixture when that is re-taught). A box " +
          "around the current TCP or the robot base is refused: MoveIt would start in collision and every plan would fail."),
        h("div", { class: "ko-status", id: "su-ko-status" }, "—"),
        h("div", { class: "list", id: "su-ko-list" })),
      h("div", { class: "card" },
        h("div", { class: "section", id: "su-ko-ftitle" }, "New box"),
        h("div", { class: "grid", style: "grid-template-columns:auto 1fr" },
          h("span", { class: "k" }, "Name"), h("input", { type: "text", id: "su-ko-name", placeholder: "table" }),
          h("span", { class: "k" }, "Frame"), h("select", { id: "su-ko-frame" }),
          h("span", { class: "k" }, "Size X Y Z mm"), v3("su-ko-s", "5"),
          h("span", { class: "k" }, "Centre X Y Z mm"), v3("su-ko-p", "1"),
          h("span", { class: "k" }, "R P Y °"), v3("su-ko-r", "1"),
          h("span", { class: "k" }, ""), h("label", { class: "chk" }, h("input", { type: "checkbox", id: "su-ko-en", checked: true }), "Enabled")),
        note("Position is the box centre.", "Under TCP: touch the surface with the TCP, then the box is centred " +
          "under it with its top face 5 mm below the TCP (keeps the size)."),
        h("div", { class: "row" },
          (() => { const b = btn("Under TCP", "", () => this.koFromTcp(), "Fill the centre so the box top is 5 mm below the current TCP"); b.id = "su-ko-tcp"; return b; })(),
          h("div", { class: "grow" }),
          (() => { const b = btn("New", "quiet", () => this.koForm(null), "Clear the form"); b.id = "su-ko-new"; return b; })(),
          (() => { const b = btn("Save box", "primary", () => this.koSave()); b.id = "su-ko-save"; return b; })())));
    for (const i of [0, 1, 2]) $(`#su-ko-s${i}`, this.views["Keep-out zones"]).value = ["1200", "800", "20"][i];
    this.views["Keep-out zones"].querySelector("#su-ko-apply").dataset.lease = "1";
    // Payload
    this.views["F/T + payload"] = h("div", { class: "cols" },
      h("div", { class: "card", style: "flex:0 0 330px" }, h("div", { class: "section" }, "Calibration poses"),
        h("table", { class: "su-ptable" }, h("thead", {}, h("tr", {}, h("th", {}, "Pose"), h("th", {}, "Fx N"), h("th", {}, "Fy N"), h("th", {}, "Fz N"))),
          h("tbody", { id: "su-ptab" })),
        h("div", { class: "bar" }, h("div", { id: "su-pbar" })),
        h("div", { class: "row" },
          btn("◀", "", () => this.pStep(-1), "Previous pose (ghost only, nothing moves)"),
          btn("▶", "", () => this.pStep(1), "Next pose (ghost only, nothing moves)"),
          (() => { const b = btn("Move & collect", "primary", () => this.pCollect(),
            "Move to the selected pose (the blue ghost) and sample the F/T there. Only this pose"); b.id = "su-pcollect"; return b; })(),
          h("span", { class: "muted", id: "su-pstat" })),
        h("div", { class: "row" },
          (() => { const b = btn("Collect all poses", "", async () => {
            const p = S && S.payload, n = p ? p.poses.length : 0;
            if (INFO.fake || (await ask(`Move through all ${n} calibration poses in order and evaluate?\n\n` +
                "This starts over (collected poses are cleared). Check the poses with ◀ ▶ first and that the " +
                "workspace is clear. Keep a hand near STOP.")))
              cmd("payload_collect");
          }, "Start over and collect every pose in order, then evaluate"); b.id = "su-pall"; return b; })(),
          (() => { const b = btn("Evaluate", "", () => cmd("payload_evaluate"),
            "Fit mass and CoG from the poses collected so far (3 or more; more and more different is better)"); b.id = "su-peval"; return b; })(),
          (() => { const b = btn("Start over", "quiet", async () => (await ask("Clear the collected poses?")) && cmd("payload_reset"),
            "Clear every collected pose"); b.id = "su-preset"; return b; })()),
        note("One pose at a time (select, check the ghost, Move & collect), or Collect all poses.", "Click a pose or use ◀ ▶: the blue " +
          "ghost shows it and nothing moves. Move & collect goes there with MoveIt at 10% (GUI SPEED applies), " +
          "waits 2 s and samples. The ghost is the target, not the path to it. Skip any pose that is not safe; " +
          "Evaluate needs 3 or more, with the tool pointing in clearly different directions.")),
      h("div", { class: "card", style: "width:360px" },
        h("div", { class: "section" }, "Saved (kept across restarts)"),
        h("div", { class: "grid", id: "su-psaved" }, "—"),
        h("div", { class: "muted small", id: "su-psaved-note" }),
        h("div", { class: "section" }, "Result (this session)"),
        h("div", { class: "title", id: "su-pverdict" }, "Not evaluated"), h("div", { class: "grid", id: "su-pres" }),
        h("div", { class: "muted", id: "su-pissues" }),
        h("div", { class: "section" }, "Admittance gravity compensation"),
        h("pre", { class: "log", id: "su-pyaml", style: "max-height:none" }, "—"),
        note("Apply (outside Admittance) sets it, reads it back and saves it.",
          "The GUI puts saved values back by itself after a controller restart and before switching to Admittance."),
        h("div", { class: "row" }, btn("Save result", "", () => cmd("payload_save")),
          h("div", { class: "grow" }), btn("Apply to admittance", "primary", () => this.applyPayload()))));
    this.pcollect = $("#su-pcollect", this.views["F/T + payload"]);
    const pb = this.views["F/T + payload"].querySelectorAll(".card:last-child .btn");
    [this.psave, this.papply] = pb;
    this.pcols = this.views["F/T + payload"];
    this.views["F/T + payload"] = h("div", { class: "col" }, this.buildBias(), this.pcols);
    this.views["Gains"] = this.buildGains();
    // Drivers & cameras: drivers.js
    this.dr = drivers({ h, btn, note, cmd, toast, ask, lease: (b) => { b.dataset.lease = "1"; return b; } });
    this.views["Cameras"] = this.dr.view;
    // one 3D view for the pages that place things in space: the live robot and what the page edits
    this.sv = h("div", { class: "card su-v3card" },
      h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "3D view"),
        h("span", { class: "muted small", id: "su-v3-what" })),
      h("div", { class: "view3d su-v3", id: "su-v3" }, h("div", { class: "muted v3msg", id: "su-v3-msg" }, "Loading the robot model…")));
    // Parts, Camera calibration, Setups: workcell.js
    const lease = (b) => { b.dataset.lease = "1"; return b; };
    this.wc = workcell({ h, btn, note, cmd, post, $, toast, ask, askText, lease, freedriveButton, navTo, fake: () => INFO.fake });
    const pt = this.wc.buildParts(() => this.svPaint(true)), hc = this.wc.buildCalib(), st = this.wc.buildSetups();
    this.views["Parts"] = h("div", { class: "col su-main" }, pt.card, pt.editor);
    this.views["Camera"] = h("div", { class: "cols" }, hc.left, hc.right);
    this.views["Setups"] = h("div", { class: "cols" }, st.card, st.side);
    // Flask Geometry + flask scrape: flask.js
    this.fl = flaskPage({ h, btn, note, cmd, pcmd, toast, ask, askText, lease, freedriveButton, gate, fake: () => INFO.fake,
      repaint: () => this.sub === "Flask" && this.svPaint(true),
      guardProgram: (what) => pages.PROGRAMS.guard(what),
      openProgram: (r) => { pages.PROGRAMS.display(r.program, r.selected); navTo("PROGRAMS"); } });
    this.views["Flask"] = this.fl.view;
    this.show("TCPs");
  },
  SV_PAGES: { "TCPs": "TCP frames (the active one is larger)", "Parts": "the robot with its parts (saved ones)", "Vial": "touched points and the fitted rim",
    "Flask": "the flask, touches (blue rim, violet bulb, teal neck), next touch and pivot (orange), poses (green/red)",
    "Keep-out zones": "keep-out boxes (red = enabled)",
    "F/T + payload": "calibration poses: the blue ghost is the selected pose" },
  async svInit() {
    if (this.svv) return;
    this.svv = true;
    try {
      const m = await import("/static/view3d.js");
      const v = new m.View3D($("#su-v3", this.sv), { baseLink: INFO.base_link });
      this.v3m = m;
      for (let i = 0; i < 20; i++) {
        const r = await fetch("/api/robot_model").then((x) => x.json()).catch(() => ({ ok: false, msg: "no answer" }));
        if (r.ok) { await v.load(r); $("#su-v3-msg", this.sv)?.remove(); this.view = v; this.svPaint(true); return; }
        $("#su-v3-msg", this.sv).textContent = "Robot model: " + r.msg;
        await new Promise((res) => setTimeout(res, 3000));
      }
    } catch (e) { $("#su-v3-msg", this.sv).textContent = "3D view unavailable: " + e.message; }
  },
  async svPaint(full) {
    const v = this.view, n = this.sub;
    if (!v || !this.SV_PAGES[n]) return;
    if (S && S.joints_deg) {
      const q = {};
      S.joints_deg.forEach((x, i) => (q[INFO.joint_names[i]] = x * Math.PI / 180));
      v.setJoints({ ...q, ...pages.OPERATE.extraJoints(S) }, S.tcp_off);
    }
    this.zKey = null;
    if (!full) return;
    v.setMarkers([]); v.setPathMm(null); v.setFrames([]); v.setShapes(n === "Flask" ? this.fl.shapes() : null);
    if (n === "Parts") {
      const p = this.wc.W.parts.draft();
      v.setPartDraft(p, (pose) => this.wc.W.parts.poseFromGizmo(pose));
    } else v.clearPartDraft();
    if (n !== "F/T + payload") { this.pStop(); v.showGhost(null); } else this.pGhost();
    if (n === "TCPs" || n === "Vial") {
      const r = await cmd("tf_frames").catch(() => null);
      if (r && r.ok && this.sub === n)
        v.setFrames(r.frames.filter((f) => !f.error && (n === "TCPs" ? f.kind === "tcp" : f.kind === "fixture")));
    }
    if (n === "Vial" && this.vialData) {
      const vd = this.vialData;
      v.setMarkers(Object.values(vd.points || {}).filter(Boolean).map((p) => ({ xyz_mm: p.map((x) => x * 1000), rpy_deg: [0, 0, 0] })), false);
      const r = vd.result;
      if (r) {                       // the fitted rim circle at the highest touched point
        const zt = Math.max(...Object.values(vd.points || {}).filter(Boolean).map((p) => p[2]));
        v.setPathMm([...Array(49).keys()].map((k) => [1000 * (r.cx + r.r * Math.cos(k * Math.PI / 24)),
          1000 * (r.cy + r.r * Math.sin(k * Math.PI / 24)), 1000 * zt]));
      }
    }
  },
  pGhost() {
    const p = S && S.payload, v = this.view;
    if (!v || this.sub !== "F/T + payload") return;
    const q = p && p.joints && this.pPick != null ? p.joints[this.pPick] : null;
    v.showGhost(q ? Object.fromEntries(q.map((x, i) => [INFO.joint_names[i], x])) : null);
  },
  pStep(d) {
    const p = S && S.payload;
    if (!p || !p.poses.length) return;
    this.pStop();
    const n = p.poses.length;
    this.pPick = this.pPick == null ? (d > 0 ? 0 : n - 1) : (this.pPick + d + n) % n;
    this.pGhost(); this.update(S);
  },
  pStop() {},
  async pCollect() {
    const p = S && S.payload;
    if (!p || this.pPick == null) return;
    const name = p.poses[this.pPick];
    if (INFO.fake || (await ask(`Move to "${name}" (the blue ghost) and collect it?\n\nCheck that the path ` +
        "from where the arm is now is clear. Keep a hand near STOP.")))
      cmd("payload_collect", { index: this.pPick });
  },
  // F/T limit card (ft_guard.py): soft warns, hard stops the GUI's motion
  ftLimitCard() {
    if (this.ftl) return this.ftl;
    const num = (id, ph) => h("input", { type: "number", id, min: "0", step: "1", placeholder: ph, style: "width:72px" });
    const on = h("input", { type: "checkbox", id: "su-ftl-on" });
    const save = btn("Save", "primary", () => cmd("ft_guard_save", { enabled: on.checked,
      soft_f: +$("#su-ftl-sf").value || 0, hard_f: +$("#su-ftl-hf").value || 0,
      soft_t: +$("#su-ftl-st").value || 0, hard_t: +$("#su-ftl-ht").value || 0 }).then((r) => r.ok && this.ftlFill(r)),
      "Admin only. Saved on the robot PC");
    save.dataset.lease = "1";
    this.ftl = h("div", { class: "card" },
      h("div", { class: "row" }, h("div", { class: "section grow" }, "Force limit"),
        h("label", { class: "row" }, on, "On")),
      h("div", { class: "muted small" }, "On the F/T reading as shown above. Soft: a warning. Hard: the GUI's motion " +
        "stops at once (program, move, policy, jog, applied force). In freedrive it only warns. 0 or empty: that limit off."),
      h("div", { class: "row" }, h("span", { style: "width:40px" }, "|F|"), "soft", num("su-ftl-sf", "N"), "hard", num("su-ftl-hf", "N"), "N"),
      h("div", { class: "row" }, h("span", { style: "width:40px" }, "|T|"), "soft", num("su-ftl-st", "Nm"), "hard", num("su-ftl-ht", "Nm"), "Nm"),
      h("div", { class: "row" }, h("span", { class: "grow mono small", id: "su-ftl-now" }, ""),
        btn("Reset peak", "quiet", () => cmd("ft_guard_reset_peak")), save),
      h("div", { class: "small", id: "su-ftl-trip" }, ""));
    cmd("ft_guard_get").then((r) => r.ok && this.ftlFill(r));
    return this.ftl;
  },
  ftlFill(r) {
    $("#su-ftl-on").checked = !!r.enabled;
    for (const [k, id] of [["soft_f", "sf"], ["hard_f", "hf"], ["soft_t", "st"], ["hard_t", "ht"]]) $(`#su-ftl-${id}`).value = r[k] || "";
  },
  ftlLive(G) {
    if (!this.ftl || !this.ftl.isConnected || !G) return;
    $("#su-ftl-now").textContent = `now |F| ${G.f.toFixed(1)} N  |T| ${G.t.toFixed(2)} Nm   peak ${G.peak_f.toFixed(1)} N  ${G.peak_t.toFixed(2)} Nm`;
    const t = G.last_trip;
    $("#su-ftl-trip").textContent = t ? `Last hard limit: ${new Date(t.at * 1000).toLocaleTimeString()} — ${t.why}` +
      (t.stopped ? " (stopped the GUI's motion)" : t.stopped === false ? " (nothing to stop)" : "") : "";
  },
  show(n) {
    if (this.sub === "Parts" && n !== "Parts" && this.view) this.view.clearPartDraft();
    this.sub = n;
    this.nav.querySelectorAll(".item").forEach((it) => it.classList.toggle("on", it.dataset.name === n));
    if (this.SV_PAGES[n]) {
      if (n === "F/T + payload") {            // the poses, their ghost, the result side by side under the bias
        this.pcols.insertBefore(this.sv, this.pcols.lastChild);
        this.body.replaceChildren(this.views[n]);
        this.views[n].insertBefore(this.ftLimitCard(), this.pcols);
      } else this.body.replaceChildren(h("div", { class: "cols" }, this.views[n], this.sv));
      $("#su-v3-what").textContent = this.SV_PAGES[n];
      this.svInit(); this.svPaint(true);
    } else this.body.replaceChildren(this.views[n]);
    if (n === "Cameras") this.dr.load(); else this.dr.leave();
    if (n === "Vial") cmd("vial_get").then((r) => r.ok && this.vial(r.vial));
    if (n === "Flask") this.fl.load();
    if (n === "Gains") this.gLoad(true);
    if (n === "Parts") this.wc.W.parts.load();
    if (n === "TCPs") this.pivotCmd("pivot_get");
    if (n === "Camera") this.wc.W.he.load();
    if (n === "Setups") this.wc.W.setups.load();
  },
  // ---- roles (roles.py): Viewer watches, Operator runs the robot, Admin also changes the setup
  rolesView() {
    const box = h("div", { class: "col su-main" });
    const draw = (r) => {
      const people = { ...r.people };
      const pick = (v, on) => h("select", { onchange: (e) => on(e.target.value) },
        ...r.roles.map((x) => h("option", { value: x, ...(x === v ? { selected: true } : {}) }, x[0].toUpperCase() + x.slice(1))));
      let dflt = r.default;
      const rows = h("div", { class: "col" });
      const fill = () => rows.replaceChildren(...Object.keys(people).sort().map((k) => h("div", { class: "row" },
        h("span", { class: "grow mono" }, k), pick(people[k], (v) => { people[k] = v; }),
        btn("Remove", "quiet", () => { delete people[k]; fill(); }))),
        ...(Object.keys(people).length ? [] : [h("div", { class: "muted" }, "Nobody listed: everyone gets the default role.")]));
      fill();
      const online = ((S && S.users) || []).filter((u) => u.key && !(u.key in people) && !/^(127\.|::1$|localhost)/.test(u.key));
      const add = h("input", { type: "text", placeholder: "Tailscale login or IP", style: "min-width:220px" });
      const mine = INFO.role === "admin";
      const save = btn("Save", "primary", async () => {
        const res = await cmd("roles_save", { default: dflt, people });
        if (res.ok) draw(res);
      });
      save.dataset.lease = "1";
      box.replaceChildren(
        h("div", { class: "section" }, "Roles"),
        h("div", { class: "muted" }, "Viewer: watches. Operator: runs the robot. Admin: also changes SETUP and roles. " +
          "This PC is always Admin." + (r.saved ? "" : " Not saved yet: everyone is Admin.")),
        h("div", { class: "muted" }, `You are ${INFO.role[0].toUpperCase() + INFO.role.slice(1)}` +
          (INFO.key ? ` (${INFO.key})` : "") + "."),
        h("div", { class: "row" }, h("span", { class: "grow" }, "Anyone not listed"), pick(dflt, (v) => { dflt = v; })),
        rows,
        h("div", { class: "row" }, add, btn("Add", "", () => {
          const k = add.value.trim(); if (k) { people[k] = people[k] || "operator"; add.value = ""; fill(); }
        })),
        online.length ? h("div", { class: "col" }, h("div", { class: "muted small" }, "Open now:"),
          ...online.map((u) => h("div", { class: "row" }, h("span", { class: "grow" }, `${u.name} · ${u.key}`),
            btn("Add", "quiet", () => { people[u.key] = u.role || "operator"; fill(); })))) : "",
        mine ? h("div", { class: "row" }, save) : h("div", { class: "muted" }, "Only an Admin can change roles."));
    };
    cmd("roles_get").then((r) => r.ok && draw(r));
    return box;
  },
  // ---- controller gains: one table per mode, edited fields highlighted, Apply shows the change first
  buildGains() {
    this.gMode = null;
    this.gTabs = h("div", { class: "tabs", id: "su-g-tabs" });
    this.gTable = h("table", { class: "gains", id: "su-g-table" });
    this.gStatus = h("div", { class: "muted small", id: "su-g-status" });
    this.gApply = btn("Apply…", "primary", () => this.gApplyAsk());
    this.gApply.id = "su-g-apply";
    this.gPreset = h("select", { id: "su-g-preset", style: "flex:1;min-width:0", onchange: () => this.gLoadPreset() });
    this.gYaml = h("pre", { class: "log", id: "su-g-yaml", style: "max-height:240px" }, "—");
    const left = h("div", { class: "card grow" },
      h("div", { class: "row" }, this.gTabs, h("div", { class: "grow" }),
        btn("↻ Read", "quiet", () => this.gLoad(true), "Read the values from the running controller")),
      this.gStatus, this.gTable,
      h("div", { class: "row" },
        btn("Undo edits", "quiet", () => this.gFill(this.gLive()), "Back to the controller's current values"),
        btn("Values at GUI start", "quiet", () => this.gFill(this.gData.modes[this.gMode].baseline),
            "The values the controller had when this GUI first read it (not applied until you press Apply)"),
        h("div", { class: "grow" }), this.gApply));
    const right = h("div", { class: "card", style: "width:340px" },
      h("div", { class: "section" }, "Presets"),
      h("div", { class: "row" }, this.gPreset,
        btn("Save…", "quiet", () => this.gSavePreset(), "Save the table as a named preset"),
        btn("✕", "quiet", () => this.gDelPreset(), "Delete the selected preset")),
      note("Loading a preset only fills the table."),
      h("div", { class: "section" }, "Saved"),
      this.gSaved = h("div", { id: "su-g-saved" }),
      h("div", { class: "row" },
        this.gRestore = btn("Restore saved", "", () => cmd("gains_restore", { mode: this.gMode }).then((r) => r.ok && setTimeout(() => this.gLoad(true), 700)),
          "Put the saved values on the controller now (ramped if it is active)"),
        this.gForget = btn("Forget saved", "quiet", async () => (await ask("Forget the saved values? The controller keeps its current values until it restarts, then its YAML values come back.")) &&
          cmd("gains_forget", { mode: this.gMode }).then(() => this.gLoad(true)))),
      h("details", {}, h("summary", { class: "muted small" }, "Show as controller YAML"), this.gYaml));
    return h("div", { class: "cols" }, left, right);
  },
  async gLoad(refresh) {
    const r = await post("/api/cmd", { cmd: "gains_get", args: { refresh } }, true);
    if (!r.ok) return;
    this.gData = r;
    const modes = Object.keys(r.modes);
    if (!this.gMode || !modes.includes(this.gMode)) this.gMode = modes.includes(r.current_mode) ? r.current_mode : modes[0];
    this.gTabs.replaceChildren(...modes.map((m) => h("a", { class: m === this.gMode ? "on" : "",
      onclick: () => { this.gMode = m; this.gLoad(false); } }, (modeLabel(m)) + (r.modes[m].active ? " ●" : ""))));
    this.gRender();
  },
  gLive() { return (this.gData && this.gData.modes[this.gMode].live) || {}; },
  gRender() {
    const m = this.gData.modes[this.gMode], live = m.live || {};
    const ax = [...new Set(m.rows.flatMap((r) => r.axes))];
    const cols = ["X", "Y", "Z", "Rx", "Ry", "Rz"].filter((a) => ax.includes(a));
    const head = h("tr", {}, h("th", {}, ""), ...cols.map((a) => h("th", {}, a)), h("th", {}, ""));
    this.gInputs = {};
    const rows = m.rows.map((r) => {
      const v = live[r.key];
      const cells = cols.map(() => h("td", {}));
      const single = r.axes.length === 1 && r.axes[0] === "";
      const put = (i, el) => cells[single ? 0 : cols.indexOf(r.axes[i])].append(el);
      this.gInputs[r.key] = [];
      r.axes.forEach((a, i) => {
        let el;
        if (r.readonly || v == null) el = h("span", { class: "mono muted", title: r.readonly || "not reported by the controller" }, v == null ? "—" : String(v[i]));
        else if (r.kind === "bool") {
          el = h("input", { type: "checkbox", checked: !!v[i], title: r.live ? "" : `changes only outside ${modeLabel(this.gMode)}`,
                            onchange: () => this.gMark() });
          this.gInputs[r.key].push(el);
        } else {
          el = h("input", { type: "number", value: String(+v[i]), step: "any", min: r.min[i] ?? "", max: r.max[i] ?? "",
                            title: `${r.min[i] ?? "−∞"} … ${r.max[i] ?? "∞"}${r.unit ? " " + r.unit : ""} · now ${v[i]}`,
                            oninput: () => this.gMark() });
          this.gInputs[r.key].push(el);
        }
        put(i, el);
      });
      return h("tr", {}, h("th", { title: [r.help, r.readonly, r.params.join(", ")].filter(Boolean).join("\n") },
        r.label, r.help || r.readonly ? h("span", { class: "hint" }, " ⓘ") : null),
        ...cells, h("td", { class: "muted small" }, r.unit));
    });
    this.gTable.replaceChildren(h("thead", {}, head), h("tbody", {}, ...rows));
    const when = m.read_at ? new Date(m.read_at * 1000).toLocaleTimeString() : "never";
    this.gStatus.replaceChildren(
      h("span", { class: "badge" + (m.active ? " warn" : "") }, m.active ? "ACTIVE" : "NOT ACTIVE"), " ",
      m.error ? h("span", { class: "error" }, m.error) :
        `${m.node} · read ${when} · ` + (m.active ? `changes ramp in over ${m.ramp_s} s, at most ${this.gData.max_factor}× per apply`
                                                   : "changes take effect when the mode is next switched on"),
      m.note ? h("div", {}, m.note) : "");
    this.gYaml.textContent = this.gData.yaml?.[this.gMode] || "—";
    const savedKeys = Object.keys(m.saved || {}), n = savedKeys.length;
    const savedRows = m.rows.filter((r) => r.params.some((p) => savedKeys.includes(p))).map((r) => r.label);
    const other = savedKeys.filter((k) => !m.rows.some((r) => r.params.includes(k)) && !/\.update_|update_stiffness/.test(k));
    if (other.some((k) => k.startsWith("gravity_compensation"))) savedRows.push("Payload compensation");
    this.gSaved.replaceChildren(
      n ? h("div", { class: "good" }, `✓ ${savedRows.join(", ") || "values"} · saved ${m.saved_at || ""}`)
        : h("div", { class: "muted" }, "Nothing saved: the controller uses its YAML values."),
      note("Apply saves them; the GUI puts them back by itself.", "Saved values are restored after a controller " +
        `restart and just before switching to ${modeLabel(this.gMode)}. An active controller is never changed without you: ` +
        "a difference there is shown here with Restore saved."),
      m.unsynced ? h("div", { class: "error small" }, "⚠ " + m.unsynced) : "");
    this.gSavedN = n;
    const names = Object.keys(this.gData.presets).filter((n) => this.gData.presets[n].mode === this.gMode);
    this.gPreset.replaceChildren(h("option", { value: "" }, names.length ? "Load preset…" : "No presets yet"),
      ...names.map((n) => h("option", { value: n }, `${n} (${this.gData.presets[n].saved})`)));
    this.gMark();
  },
  gFill(vals) {
    if (!vals) return;
    for (const [k, els] of Object.entries(this.gInputs)) {
      if (!vals[k]) continue;
      els.forEach((el, i) => { if (el.type === "checkbox") el.checked = !!vals[k][i]; else el.value = String(+vals[k][i]); });
    }
    this.gMark();
  },
  gValues(changedOnly = true) {
    const live = this.gLive(), out = {};
    for (const [k, els] of Object.entries(this.gInputs)) {
      if (!els.length) continue;
      const v = els.map((el) => (el.type === "checkbox" ? el.checked : el.value === "" ? NaN : +el.value));
      if (!live[k]) continue;
      if (!changedOnly || v.some((x, i) => (typeof x === "boolean" ? x !== live[k][i] : Math.abs(x - live[k][i]) > 1e-9 || Number.isNaN(x)))) out[k] = v;
    }
    return out;
  },
  gMark() {
    const live = this.gLive();
    for (const [k, els] of Object.entries(this.gInputs)) els.forEach((el, i) => {
      if (!live[k]) return;                  // the table of another mode, about to be replaced
      const x = el.type === "checkbox" ? el.checked : +el.value;
      const changed = el.type === "checkbox" ? x !== live[k][i] : el.value === "" || Math.abs(x - live[k][i]) > 1e-9;
      el.classList.toggle("edited", changed);
      if (el.type === "checkbox") el.parentElement.classList.toggle("edited", changed);
    });
    const n = Object.keys(this.gValues()).length;
    this.gApply.textContent = n ? `Apply ${n} change${n > 1 ? "s" : ""}…` : "Apply…";
    gate(this.gApply, n > 0 && !(S && S.gains_applying));
    if (this.gRestore) { gate(this.gRestore, !!this.gSavedN && !(S && S.gains_applying)); gate(this.gForget, !!this.gSavedN); }
  },
  async gApplyAsk() {
    const values = this.gValues();
    const r = await post("/api/cmd", { cmd: "gains_check", args: { mode: this.gMode, values } }, true);
    if (!r.ok) return toast(r.msg, false);
    const rows = this.gData.modes[this.gMode].rows;
    const lines = Object.keys(r.new).map((k) => {
      const row = rows.find((x) => x.key === k);
      return `${row.label}: ${r.old[k].map(String).join(", ")}  →  ${r.new[k].map(String).join(", ")}`;
    });
    if (!lines.length) return toast("nothing changed", true);
    if (!(await ask(`Set on ${this.gData.modes[this.gMode].node}:\n\n${lines.join("\n")}\n\n${r.warnings.join("\n")}`))) return;
    const a = await cmd("gains_apply", { mode: this.gMode, values });
    if (a.ok) { this.gPending = true; setTimeout(() => this.gLoad(true), 700); }
  },
  gLoadPreset() {
    const p = this.gData.presets[this.gPreset.value];
    if (p) { this.gFill(p.values); toast(`preset ${this.gPreset.value} loaded into the table: Apply to use it`, true); }
  },
  async gSavePreset() {
    const name = (await askText("Preset name (saves the table as shown):", this.gPreset.value || ""));
    if (!name) return;
    const r = await cmd("gains_preset_save", { name, mode: this.gMode, values: this.gValues(false) });
    if (r.ok) { this.gData.presets = r.presets; this.gRender(); }
  },
  async gDelPreset() {
    const n = this.gPreset.value;
    if (!n || !(await ask(`Delete preset ${n}?`))) return;
    const r = await cmd("gains_preset_delete", { name: n });
    if (r.ok) { this.gData.presets = r.presets; this.gRender(); }
  },
  hide() {
    this.dr.leave();
    if (this.view) this.view.clearPartDraft();
  },
  vial(v) {
    this.vialData = v;
    if (this.sub === "Vial") this.svPaint(true);
    for (const [k, n] of INFO.vial_points.entries()) {
      const p = v.points[n];
      $(`#su-vd-${k}`).className = "dot" + (p ? " ok" : "");
      $(`#su-vv-${k}`).textContent = p ? p.map((x) => (x * 1000).toFixed(1)).join("  ") + " mm" : "—";
    }
    const r = v.result;
    $("#su-vres").innerHTML = [["Centre X", "cx"], ["Centre Y", "cy"], ["Radius", "r"], ["Bottom Z", "z"], ["Rim height spread", "spread"]]
      .map(([k, key]) => `<span class="k">${k}</span><span class="v">${r ? (r[key] * 1000).toFixed(2) + " mm" : "—"}</span>`).join("");
    $("#su-verr").textContent = v.error || "";
    this.vialOk = !!r;
    if (v.profiles) this.vialProfiles(v.profiles);
  },
  vialProfiles(list) {
    if (!list) return;
    const as = () => $("#su-vp-as").value || "vial";
    $("#su-vp-list").replaceChildren(...list.map((e) => h("tr", { class: e.in_use_as.length ? "on" : "" },
      h("td", {}, e.name + (e.builtin ? " ★" : "")),
      h("td", { class: "num mono" }, e.xyz_mm.map((x) => x.toFixed(1)).join(", ")),
      h("td", { class: "num" }, e.radius_mm != null ? (+e.radius_mm).toFixed(1) : "—"),
      h("td", { class: "num" }, e.height_mm != null ? (+e.height_mm).toFixed(1) : "—"),
      h("td", { class: "muted small" }, (e.builtin ? "built in · " : "") + (e.source || "") + (e.saved ? " · " + e.saved : "")),
      h("td", { class: e.in_use_as.length ? "good" : "muted" }, e.in_use_as.join(", ") || "—"),
      h("td", {}, (() => { const b = btn("Use", "primary", async () => {
          if ((await ask(`Use vial profile "${e.name}" as "${as()}"?\n\nCentre ${e.xyz_mm.join(", ")} mm in ${INFO.base_link}` +
              (e.builtin ? `\n(${e.source}: only right if your vial is really there)` : "") + "\n\nPrograms move relative to it."))) 
            cmd("vial_profile_use", { name: e.name, fixture: as() }).then((r) => { if (r.ok) { this.vialProfiles(r.vials); this.svPaint(true); } });
        }, `Make "${e.name}" the fixture programs use`); b.dataset.lease = "1"; return b; })(),
        e.builtin ? "" : (() => { const b = btn("Delete", "quiet", async () => (await ask(`Delete vial profile "${e.name}"? Fixtures made from it stay.`)) &&
          cmd("vial_profile_delete", { name: e.name }).then((r) => r.ok && this.vialProfiles(r.vials))); b.dataset.lease = "1"; return b; })()))));
  },
  // ---- keep-out zones
  koForm(b) {
    this.koEditing = b ? b.name : null;
    $("#su-ko-ftitle").textContent = b ? `Edit · ${b.name}` : "New box";
    $("#su-ko-name").value = b ? b.name : "";
    if (b) {
      $("#su-ko-frame").value = b.frame;
      [0, 1, 2].forEach((i) => {
        $(`#su-ko-s${i}`).value = b.size_mm[i]; $(`#su-ko-p${i}`).value = b.xyz_mm[i]; $(`#su-ko-r${i}`).value = b.rpy_deg[i];
      });
      $("#su-ko-en").checked = b.enabled;
    }
    this.koKey = null;
  },
  koVals(id) { return [0, 1, 2].map((i) => +$(`#${id}${i}`).value); },
  // the box in the form, drawn dashed in the 3D view before it is saved (its frame resolved like the saved ones)
  koDraft() {
    if (!this.v3m || !$("#su-ko-frame")) return null;
    const size = this.koVals("su-ko-s");
    if (size.some((v) => !(v > 0))) return null;
    const fr = $("#su-ko-frame").value, saved = S.keepout.boxes.find((b) => b.base && b.frame === fr);
    let F = null;
    if (fr === INFO.base_link) F = this.v3m.poseToMatrix([0, 0, 0], [0, 0, 0]);
    else if (saved) F = this.v3m.poseToMatrix(saved.base.xyz_mm, saved.base.rpy_deg)
      .multiply(this.v3m.poseToMatrix(saved.xyz_mm, saved.rpy_deg).invert());
    else return null;                            // a fixture no saved box uses yet: shown once saved
    const base = this.v3m.matrixToPose(F.multiply(this.v3m.poseToMatrix(this.koVals("su-ko-p"), this.koVals("su-ko-r"))));
    return { name: ($("#su-ko-name").value || "new box") + " · draft", size_mm: size, enabled: false, base };
  },
  async koFromTcp() {
    const r = await cmd("keepout_from_tcp", { frame: $("#su-ko-frame").value, size_mm: this.koVals("su-ko-s") });
    if (!r.ok) return;
    r.xyz_mm.forEach((v, i) => ($(`#su-ko-p${i}`).value = v));
    r.rpy_deg.forEach((v, i) => ($(`#su-ko-r${i}`).value = v));
  },
  koSave() {
    const name = $("#su-ko-name").value.trim();
    const args = { name, frame: $("#su-ko-frame").value, size_mm: this.koVals("su-ko-s"), xyz_mm: this.koVals("su-ko-p"),
                   rpy_deg: this.koVals("su-ko-r"), enabled: $("#su-ko-en").checked };
    if (this.koEditing) args.replace = this.koEditing;
    else if (S.keepout.boxes.some((b) => b.name === name)) { toast(`a box named ${name} exists: Edit it instead`, false); return; }
    cmd("keepout_save", args).then((r) => { if (r.ok) this.koForm({ ...args }); });
  },
  koUpdate(s) {
    const k = s.keepout, st = k.status;
    const el = $("#su-ko-status");
    el.className = "ko-status " + (st.state === "ok" ? "good" : st.state === "pending" ? "" : st.ok ? "good" : "bad");
    el.textContent = (st.state === "ok" ? "✓ MoveIt: " : st.state === "pending" ? "" : "⚠ MoveIt: ") + st.msg +
      (st.t ? `  (${new Date(st.t * 1000).toLocaleTimeString()})` : "");
    gate($("#su-ko-save"), s.idle);
    gate($("#su-ko-apply"), s.idle);
    const fsel = $("#su-ko-frame"), fkey = JSON.stringify(k.frames);
    if (fkey !== this.koFrames) {
      this.koFrames = fkey;
      const cur = fsel.value;
      fsel.replaceChildren(...k.frames.map((f) => h("option", { value: f }, f)));
      fsel.value = k.frames.includes(cur) ? cur : k.frames[0];
    }
    const key = JSON.stringify([k.boxes, holds, s.idle, this.koEditing, st.applied]);
    if (key === this.koKey) return;
    this.koKey = key;
    const applied = new Set(st.applied || []);
    $("#su-ko-list").replaceChildren(...(k.boxes.length ? k.boxes.map((b) => {
      const en = h("input", { type: "checkbox", checked: b.enabled, title: "Enabled: MoveIt plans around it",
        onchange: (e) => cmd("keepout_enable", { name: b.name, enabled: e.target.checked }).then((r) => { if (!r.ok) e.target.checked = !e.target.checked; }) });
      gate(en, s.idle);
      const ed = btn("Edit", "quiet", () => this.koForm(b)), del = btn("Delete", "quiet", async () =>
        (await ask(`Delete keep-out box ${b.name}? MoveIt will plan through that space again.`)) && cmd("keepout_delete", { name: b.name }));
      gate(del, s.idle);
      const where = b.error ? h("span", { class: "error" }, b.error)
        : `${b.size_mm.join(" × ")} mm · centre ${b.xyz_mm.join(", ")} mm` + (b.rpy_deg.some((v) => v) ? ` · ${b.rpy_deg.join(", ")} °` : "") + ` in ${b.frame}`;
      return h("div", { class: "item" + (b.name === this.koEditing ? " on" : "") + (b.enabled ? "" : " ko-off") },
        en, h("div", { class: "grow", onclick: () => this.koForm(b) }, h("div", {}, b.name),
          h("div", { class: "muted small" }, where)),
        b.enabled ? h("span", { class: "badge" + (applied.has(b.id) ? "" : " warn"), title: applied.has(b.id)
          ? "In MoveIt's planning scene" : "Not (yet) confirmed in MoveIt's planning scene" }, applied.has(b.id) ? "IN SCENE" : "NOT APPLIED")
          : h("span", { class: "badge" }, "OFF"),
        ed, del);
    }) : [h("div", { class: "muted" }, "No keep-out zones. Add the table first (see the form).")]));
  },
  editTcp(t) {
    this.tcpSetHow("type");
    $("#su-t-title").textContent = `Edit TCP ${t.name}`;
    $("#su-tn").value = t.name; $("#su-tp").value = t.parent;
    [...t.xyz_mm, ...t.rpy_deg].forEach((v, i) => ($(`#su-to${i}`).value = v));
  },
  tcpNew() {
    $("#su-t-title").textContent = "New TCP";
    $("#su-tn").value = "";
    [0, 1, 2, 3, 4, 5].forEach((i) => ($(`#su-to${i}`).value = "0"));
    $("#su-tn").focus();
  },
  tcpSetHow(k) {
    this.tcpHow = k;
    $("#su-t-how").querySelectorAll("button").forEach((b) => b.classList.toggle("on", b.dataset.how === k));
    $("#su-t-type").classList.toggle("hidden", k !== "type");
    $("#su-t-pivot").classList.toggle("hidden", k !== "pivot");
    $("#su-t-rpyhint").textContent = k === "pivot" ? "The tip measurement gives the position only. Rotation from the parent: usually 0 0 0."
      : "Rotation of the TCP from its parent (0 0 0 = the same axes).";
    if (k === "pivot") this.pivotCmd("pivot_get"); else gate($("#su-t-save"), true);
  },
  async saveTcp() {
    const name = $("#su-tn").value.trim(), v = [0, 1, 2, 3, 4, 5].map((i) => +$(`#su-to${i}`).value);
    const exists = S.calibration.tcps.some((t) => t.name === name && !t.builtin);
    if (exists && !(await ask(`Change TCP ${name}? Poses taught with it will now mean a different tool point.`))) return;
    cmd("tcp_save", { name, parent: $("#su-tp").value, xyz_mm: v.slice(0, 3), rpy_deg: v.slice(3), replace: exists });
  },
  async pivotCmd(command, args = {}) {
    const r = await cmd(command, args);
    if (r.ok && r.pivot) this.pivotUpdate(r.pivot);
  },
  pivotReset() { this.pivotCmd("pivot_reset", { parent: $("#su-tp").value }); },
  pivotUpdate(p) {
    this.pivot = p;
    $("#su-pivot-samples").replaceChildren(...p.samples.map((x, i) => h("span", { class: "chip", title: `${x.xyz_mm.join(", ")} mm · ${x.rpy_deg.join(", ")}°` },
      `#${i + 1}`, h("a", { class: "x", title: "Remove this sample", onclick: () => this.pivotCmd("pivot_remove", { index: i }) }, " ×"))));
    $("#su-pivot-quality").textContent = p.samples.length ? `${p.samples.length} samples · ${p.quality}: ${p.quality_reason}` : "No samples yet.";
    const r = p.result;
    $("#su-pivot-result").textContent = r
      ? `Tip from ${p.parent}: ${r.xyz_mm.join(", ")} mm · error ${r.rms_mm} mm (worst ${r.max_mm} mm, sample #${r.worst_sample})`
      : "";
    if (this.tcpHow === "pivot") gate($("#su-t-save"), !!r);
  },
  async pivotSave() {
    if (!this.pivot?.result) return;
    const name = $("#su-tn").value.trim();
    const existing = S.calibration.tcps.some(t => t.name === name);
    if (existing && !(await ask(`Replace TCP ${name}? Previously taught poses will use the new offset.`))) return;
    cmd("pivot_save", { name, replace: existing, rpy_deg: [3, 4, 5].map(i => +$(`#su-to${i}`).value) });
  },
  // F/T bias: software offsets on the F/T broadcaster (the ATI box itself is never biased)
  buildBias() {
    const cell = (id) => h("td", { class: "num mono", id });
    const B = this.fb = {};
    B.inputs = AXES.map((a, i) => h("input", { type: "number", step: i < 3 ? "0.01" : "0.001", id: `su-fb-in${i}`,
      style: "width:84px", title: `${a} bias to set (${i < 3 ? "N" : "Nm"})` }));
    const lb = (b) => (b.dataset.lease = "1", b);
    B.zero = lb(btn("Zero now", "primary", () => this.biasZero(),
      "Removes only the sensor drift. With a payload loaded on the admittance controller, the reading keeps that "
      + "tool weight (as the payload fit, the controller and the MuJoCo training expect); without one, it reads 0"));
    B.zeroAll = lb(btn("Zero all", "", () => this.biasZero(true),
      "Make the current load read 0, tool weight included. Admittance with a loaded payload would then subtract "
      + "the weight twice"));
    B.clear = lb(btn("Clear (raw)", "", () => this.biasClear(), "Bias 0: the reading is the sensor's raw value"));
    B.set = lb(btn("Set these values", "", () => this.biasSet(), "Set the bias to the values typed in the last column"));
    B.fill = btn("Copy current", "quiet", () => this.biasFill(this.S_fb && this.S_fb.bias), "Copy the current bias into the fields");
    B.reapply = lb(btn("Re-apply", "", () => this.biasSet(this.S_fb.last.bias, "the last bias")));
    B.reapply.id = "su-fb-reapply";
    B.presetSel = h("select", { id: "su-fb-sets", style: "min-width:140px" });
    B.use = lb(btn("Use", "", () => this.biasUse(), "Apply the selected set"));
    B.load = btn("Edit", "quiet", () => { const p = this.S_fb.presets[B.presetSel.value]; if (p) this.biasFill(p.bias); },
      "Copy the selected set into the fields (not applied)");
    B.del = btn("Delete", "quiet", async () => B.presetSel.value && (await ask(`Delete the F/T bias set '${B.presetSel.value}'?`)) &&
      cmd("ft_bias_preset_delete", { name: B.presetSel.value }));
    B.save = btn("Save fields as set…", "quiet", async () => {
      const name = (await askText("Name of this F/T bias set (e.g. 'gripper, no part'):", ""));
      if (name) cmd("ft_bias_preset_save", { name, bias: this.biasFields() });
    });
    B.del.dataset.lease = B.save.dataset.lease = "1";
    const rows = AXES.map((a, i) => h("tr", {}, h("td", {}, a + (i < 3 ? " N" : " Nm")),
      cell(`su-fb-b${i}`), cell(`su-fb-w${i}`), cell(`su-fb-r${i}`), h("td", {}, B.inputs[i])));
    // a live chart beside the table: what Zero / Clear / a set does to the reading, as it happens
    this.fbChart = ftChart();
    this.fbChart.el.open = true;
    const chartBox = h("div", { class: "fb-chart" }, this.fbChart.el);
    return h("div", { class: "card", id: "su-fb" },
      h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "F/T bias (software)"),
        h("span", { class: "muted small", id: "su-fb-node" }, "")),
      h("div", { class: "ko-status", id: "su-fb-state" }, "—"),
      h("div", { class: "row", style: "align-items:flex-start;gap:var(--l)" },
        h("div", { class: "col", style: "gap:var(--s)" },
        h("table", { class: "fb-table" }, h("thead", {}, h("tr", {}, h("th", {}, ""), h("th", {}, "Bias now"), h("th", {}, "Reading"),
          h("th", {}, "Raw sensor"), h("th", {}, "New bias"))), h("tbody", {}, rows)),
        h("div", { class: "col", style: "gap:var(--s)" },
          h("div", { class: "row" }, B.zero, B.zeroAll, B.clear),
          h("div", { class: "row" }, B.set, B.fill),
          h("div", { class: "row" }, B.presetSel, B.use, B.load, B.del),
          h("div", { class: "row" }, B.save),
          h("div", { class: "muted small", id: "su-fb-why" }, ""))),
        chartBox),
      note("Reading = raw sensor − bias. The ATI box itself is never biased.",
        "The bias is the offset of the F/T broadcaster, so the GUI, payload calibration, policies and the admittance " +
        "controller all see the same values. It changes at once, so it is refused while anything uses the reading " +
        "(Admittance, an applied force, a program, policy or motion). The broadcaster starts with no bias each " +
        "time the robot stack starts; the GUI puts the last one back by itself as soon as nothing uses the reading (logged). " +
        "Zero now keeps the payload loaded on the admittance controller in the reading (it removes only the drift), " +
        "so the gravity compensation stays right at every pose and a sensor that drifted is fixed by one Zero, " +
        "no re-calibration. Zero all also removes the tool weight: Admittance would then subtract it twice."));
  },
  biasFields() { return this.fb.inputs.map((x) => +x.value || 0); },
  biasFill(b) { if (b) this.fb.inputs.forEach((x, i) => (x.value = String(b[i]))); },
  biasWarn() {
    const p = S.payload && S.payload.result;
    return p && p.applied ? "\n\nThe admittance gravity compensation was set from a payload calibration with the " +
      "current bias: after this change it no longer matches. Re-run the calibration, or switch back." : "";
  },
  async biasZero(all) {
    const what = all ? "Zero ALL of the F/T reading at the current load, tool weight included?" + this.biasWarn()
      : "Zero the F/T drift?\n\nIf a payload is loaded on the admittance controller, the reading keeps that tool " +
        "weight and only the drift is removed; the gravity compensation stays valid.";
    if ((await ask(what + "\n\nDo not touch the tool; the arm must be still for 1 s. The bias is software only " +
        "(the ATI box is not biased) and Clear (raw) undoes it.")))
      cmd("ft_bias_zero", all ? { plain: true } : {});
  },
  async biasClear() {
    if ((await ask("Clear the F/T bias? The reading becomes the raw sensor values." + this.biasWarn())))
      cmd("ft_bias_clear");
  },
  async biasSet(b, what) {
    const v = b || this.biasFields();
    const txt = `F [${v.slice(0, 3).join(", ")}] N, T [${v.slice(3).join(", ")}] Nm`;
    if ((await ask(`Set the F/T bias to ${what ? what + ": " : ""}${txt}?\n\nThe reading becomes raw − bias.` + this.biasWarn())))
      cmd("ft_bias_set", { bias: v });
  },
  async biasUse() {
    const n = this.fb.presetSel.value, p = n && this.S_fb.presets[n];
    if (!p) return;
    const v = p.bias, txt = `F [${v.slice(0, 3).join(", ")}] N, T [${v.slice(3).join(", ")}] Nm`;
    if ((await ask(`Use the F/T bias set '${n}': ${txt}?` + this.biasWarn()))) cmd("ft_bias_use", { name: n });
  },
  biasUpdate(s) {
    const f = s.ft_bias, B = this.fb;
    this.S_fb = f;
    if (!f || !f.available) {
      $("#su-fb-state").textContent = "This robot profile has no F/T broadcaster.";
      [B.zero, B.zeroAll, B.clear, B.set, B.use, B.reapply].forEach((b) => gate(b, false));
      return;
    }
    $("#su-fb-node").textContent = f.node;
    const st = $("#su-fb-state");
    st.className = "ko-status";
    B.reapply.remove();
    if (f.error) { st.textContent = f.error; st.classList.add("bad"); }
    else if (!f.bias) st.textContent = "Reading the broadcaster…";
    else if (f.lost) {
      st.textContent = f.raw ? `The bias is gone (the robot stack restarted): the reading is raw. Last bias, ${f.last.how} ${f.last.at}: ` +
        `F [${f.last.bias.slice(0, 3).map((v) => v.toFixed(2)).join(", ")}] N.  `
        : `The broadcaster has a bias this GUI did not set (last set here: ${f.last.how} ${f.last.at}).  `;
      if (f.auto_restore) st.textContent += f.restore_waits ? `Auto re-apply waits: ${f.restore_waits}.  `
        : "Re-applying it automatically…  ";
      st.classList.add("bad");
      st.append(B.reapply);
    } else if (f.raw) { st.textContent = "RAW: no bias, the reading is the sensor's own value."; st.classList.add("good"); }
    else st.textContent = `BIASED${f.last ? ` (${f.last.how} ${f.last.at}${f.last.restored ? `, re-applied ${f.last.restored} after a restart` : ""})` : ""}: ` +
      "the reading is raw − bias." + (f.last && f.last.how === "zeroed, payload kept"
        ? " Payload kept: the tool weight stays in the reading (Fz ≈ the tool weight at rest); Admittance removes it." : "");
    if (f.busy) st.textContent = "ZEROING: hold still…";
    const w = s.wrench;
    for (let i = 0; i < 6; i++) {
      const d = i < 3 ? 2 : 3, b = f.bias && f.bias[i];
      $(`#su-fb-b${i}`).textContent = fmt(b, d);
      $(`#su-fb-w${i}`).textContent = w ? fmt(w[i], d) : "—";
      $(`#su-fb-r${i}`).textContent = w && f.bias ? fmt(w[i] + b, d) : "—";
    }
    if (f.bias && !this.fbFilled) { this.fbFilled = true; this.biasFill(f.bias); }
    const names = Object.keys(f.presets || {});
    if (B.presetSel.dataset.k !== names.join("\n")) {
      const keep = B.presetSel.value;
      B.presetSel.dataset.k = names.join("\n");
      B.presetSel.replaceChildren(...(names.length ? names.map((n) => h("option", { value: n }, n))
        : [h("option", { value: "" }, "no saved sets")]));
      if (names.includes(keep)) B.presetSel.value = keep;
    }
    const free = !f.why_not && !!f.bias;
    [B.zero, B.clear, B.set, B.reapply].forEach((b) => gate(b, free));
    gate(B.zero, free && !!w);
    gate(B.zeroAll, free && !!w);
    gate(B.clear, free && !f.raw);
    gate(B.use, free && names.length > 0);
    gate(B.del, names.length > 0);
    gate(B.save, true);
    $("#su-fb-why").textContent = f.why_not && f.bias ? `Bias changes refused: ${f.why_not}` : "";
  },
  // what survives a restart: the payload the GUI restores on the controller, the last calibration file, the bias
  pSaved(sv, s) {
    if (!sv) return;
    const g = (k, v, cls = "") => `<span class="k">${k}</span><span class="v ${cls}">${v}</span>`;
    const esc = (x) => String(x ?? "").replace(/[&<>"]/g, (c) => ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;" }[c]));
    const cog = (c) => c ? c.map((v) => (v * 1000).toFixed(1)).join(", ") + " mm" : "—";
    const c = sv.controller, f = sv.file, b = sv.bias;
    let html = "";
    html += c ? g("Payload", `${c.mass.toFixed(3)} kg (${c.force.toFixed(2)} N)`) + g("CoG", `${cog(c.cog)} in ${esc(c.frame)}`) +
      g("Saved", `${esc(c.at)} · ${c.differs ? "controller differs (restored before Admittance)" : "on " + esc(c.node)}`, c.differs ? "warn" : "good")
      : g("Payload", "none saved: Admittance compensates no tool weight", "warn");
    html += f ? (f.error ? g("Last file", esc(f.error), "warn") :
      g("Last file", `${f.passed ? "PASS" : "FAIL"} ${f.mass != null ? f.mass.toFixed(3) + " kg" : ""}, ${f.poses} poses, ${esc(f.at)}`)) : "";
    html += b ? g("F/T bias", `${b.in_use ? "in use" : "NOT on the sensor now"} · ${esc(b.how)} ${esc(b.at)}` +
      (b.restored ? ` · restored ${esc(b.restored)}` : ""), b.in_use ? "good" : "warn") +
      g("", `F [${b.bias.slice(0, 3).map((v) => v.toFixed(2)).join(", ")}] N`) : g("F/T bias", "none set by this GUI");
    if (this.pSavedHtml !== html) { this.pSavedHtml = html; $("#su-psaved").innerHTML = html; }
    $("#su-psaved-note").textContent = f && f.path ? `Calibration files: ${f.count} in ${f.path.replace(/\/[^/]*$/, "")}. ` +
      "The payload is restored on the controller before every switch to Admittance; the bias after a stack restart." : "";
  },
  async applyPayload() {
    const r = S.payload && S.payload.result;
    if (!r) return;
    const off = r.offset_n != null ? `\n\nThe fit found a sensor offset of ${r.offset_n.toFixed(2)} N, which the controller cannot remove. Keep ATI hardware biasing off, and keep the F/T bias (above) as it is now.` : "";
    if ((await ask(`Set the admittance gravity compensation to ${r.mass.toFixed(3)} kg at ${r.cog.map((c) => (c * 1000).toFixed(1)).join(", ")} mm?` + off)))
      cmd("payload_apply");
  },
  // the small status after each menu item, and the robot-model bar
  navStatus(s) {
    const w = s.workcell || {}, st = w.setup || {}, c = s.calibration || {};
    const own = (c.tcps || []).filter((t) => !t.builtin).length, ko = (s.keepout?.boxes || []).filter((b) => b.enabled).length;
    const vial = (c.fixtures || []).some((f) => (f.name || f) === "vial");
    const flask = (c.fixtures || []).some((f) => (f.name || f) === "flask");
    const nb = {
      "Setups": st.name ? (st.changed?.length || st.base_changed ? ["changed", "warn", `"${st.name}" is loaded; changed since: ` +
        [...(st.changed || []), ...(st.base_changed ? ["base"] : [])].join(", ")] : ["✓ " + st.name, "ok", `"${st.name}" is loaded, unchanged`])
        : ["none", "", "No setup loaded or saved yet"],
      "Parts": w.restart_for?.length ? [String(w.parts_n), "warn", "Some parts are not in the robot model yet (restart the robot stack)"]
        : w.parts_n ? [String(w.parts_n), "", "parts"] : null,
      "TCPs": own ? [String(own), "", "your own TCPs"] : null,
      "Camera": w.cams_n ? ["✓ " + w.cams_n, "ok", "saved camera calibrations"] : ["—", "", "No camera calibrated"],
      "F/T + payload": s.payload?.saved?.file ? ["✓", "ok", "a payload calibration is saved"] : ["—", "", "No payload calibration saved"],
      "Vial": vial ? ["✓", "ok", "the vial is taught"] : null,
      "Flask": flask ? ["✓", "ok", "the flask is saved"] : null,
      "Keep-out zones": ko ? [String(ko), "", "enabled keep-out boxes"] : null,
      "Cameras": driversBadge(w.drivers),
    };
    this.nav.querySelectorAll(".item").forEach((it) => {
      const v = nb[it.dataset.name], b = it.querySelector(".nb");
      b.textContent = v ? v[0] : ""; b.className = "nb" + (v && v[1] ? " " + v[1] : ""); b.title = v ? v[2] : "";
    });
    const rf = w.restart_for || [], bn = w.base_next;
    const msg = w.restarting ? "Restarting…" : [rf.length ? `Not in the robot model yet: ${rf.join(", ")}. They already work (TF, MoveIt, 3D view).` : "",
      bn ? `The next robot stack start uses another base (${bn}).` : ""].filter(Boolean).join(" ");
    this.mbar.classList.toggle("hidden", !msg);
    if (msg) $("#su-modelbar-msg").textContent = msg + (w.restarting ? "" : " Restart the robot stack to write them into the robot model (URDF).");
    this.mbar.querySelector("button").classList.toggle("hidden", !!w.restarting);
  },
  update(s) {
    this.navStatus(s);
    if (this.sub === "F/T + payload" && this.fbChart) this.fbChart.push(s);
    if (this.sub === "F/T + payload") this.ftlLive(s.ft_guard);
    if (this.view && this.SV_PAGES[this.sub]) {          // the shared 3D view follows the arm; zones when they change
      const q = {};
      (s.joints_deg || []).forEach((x, i) => (q[INFO.joint_names[i]] = x * Math.PI / 180));
      this.view.setJoints({ ...q, ...pages.OPERATE.extraJoints(s) }, s.tcp_off);
      const ko = this.sub === "Keep-out zones", draft = ko ? this.koDraft() : null;
      const boxes = ko ? [...s.keepout.boxes.filter((b) => !draft || b.name !== this.koEditing), ...(draft ? [draft] : [])] : [];
      const zk = this.sub + JSON.stringify(boxes);
      if (zk !== this.zKey) { this.zKey = zk; this.view.setZones(boxes, ko); }
    }
    if (this.sub === "TCPs") {
      const c = s.calibration, key = JSON.stringify([c.tcps, c.active_tcp, holds]);
      if (key !== this.tcpKey) {
        if (this.tcpKey) this.svPaint(true);              // a TCP was added / changed: redraw the frames
        this.tcpKey = key;
        $("#su-tcps").replaceChildren(...c.tcps.map((t) => h("div", { class: "item" + (t.name === c.active_tcp ? " on" : "") },
          h("div", { class: "grow" }, h("div", {}, t.name + (t.builtin ? "" : " · virtual")),
            t.parent ? h("div", { class: "muted small" }, `${t.xyz_mm.join(", ")} mm · ${t.rpy_deg.join(", ")} ° from ${t.parent}`) : null),
          t.builtin ? null : (() => { const b = btn("Edit", "quiet", () => this.editTcp(t)); gate(b); return b; })(),
          t.builtin ? null : (() => { const b = btn("Delete", "quiet", async () =>
            (await ask(`Delete TCP ${t.name}? Program poses that use it will not run.`)) && cmd("tcp_delete", { name: t.name })); gate(b, s.idle); return b; })(),
          t.name === c.active_tcp ? h("span", { class: "badge" }, "ACTIVE")
            : (() => { const b = btn("Set active", "quiet", () => cmd("set_tcp", { name: t.name })); gate(b); return b; })())));
        const sel = $("#su-tp"), cur = sel.value;
        sel.replaceChildren(...c.tcp_parents.map((n) => h("option", { value: n }, n)));
        sel.value = c.tcp_parents.includes(cur) ? cur : c.active_tcp;
        if (this.tcpHow === "pivot" && this.pivot?.samples.length && c.tcp_parents.includes(this.pivot.parent)) sel.value = this.pivot.parent;
      }
    }
    if (this.sub === "Vial") gate(this.vsave, this.vialOk);
    if (this.sub === "Flask") this.fl.update(s);
    if (this.sub === "Gains" && this.gData) {
      if (this.gWasApplying && !s.gains_applying) this.gLoad(true);
      this.gWasApplying = !!s.gains_applying;
      if (s.mode !== this.gSeenMode) { this.gSeenMode = s.mode; this.gLoad(false); }
      if (s.gains_applying) this.gStatus.firstChild.textContent = `APPLYING ${s.gains_applying.step}/${s.gains_applying.steps}`;
      this.gMark();
    }
    if (this.sub === "Keep-out zones") this.koUpdate(s);
    if (this.sub === "Parts") this.wc.W.parts.update(s);
    if (this.sub === "Camera") this.wc.W.he.update(s);
    if (this.sub === "Setups") this.wc.W.setups.update(s);
    if (this.sub === "F/T + payload") {
      this.biasUpdate(s);
      const p = s.payload;
      if (!p) { $("#su-pstat").textContent = "This robot profile has no F/T calibration poses."; gate(this.pcollect, false); return; }
      const have = p.samples.filter(Boolean).length, nextOpen = (from) => {
        for (let k = 0; k < p.poses.length; k++) { const i = (from + k) % p.poses.length; if (!p.samples[i]) return i; }
        return from;
      };
      if (p.running && p.pose != null && this.pPick !== p.pose) { this.pPick = p.pose; this.pGhost(); }
      if (this.pWas && !p.running && !p.error && p.pose != null && p.samples[p.pose]) {   // collected: show the next one
        this.pPick = nextOpen(p.pose); this.pGhost();
      }
      this.pWas = p.running;
      if (this.pPick == null) { this.pPick = nextOpen(0); this.pGhost(); }
      $("#su-ptab").replaceChildren(...p.poses.map((n, i) => h("tr", { class: "pick" + (i === this.pPick ? " on" : ""),
        title: "Select: the blue ghost shows this pose, nothing moves", onclick: () => { this.pPick = i; this.pGhost(); this.update(S); } },
        h("td", {}, (p.samples[i] ? "✓ " : "") + n),
        ...[0, 1, 2].map((k) => h("td", { class: "num" }, p.samples[i] ? (p.samples[i][k] >= 0 ? "+" : "") + p.samples[i][k].toFixed(2) : "")))));
      $("#su-pbar").style.width = (100 * have / (p.poses.length || 1)) + "%";
      const sel = p.poses[this.pPick] || "";
      this.pcollect.textContent = p.running ? "Collecting…" : p.samples[this.pPick] ? `Collect ${sel} again` : `Move & collect ${sel}`;
      $("#su-pstat").textContent = p.running ? (p.total > 1 ? `pose ${p.done + 1} of ${p.total}: ` : "") +
        `moving to ${p.poses[p.pose]} and sampling…` : p.error || `${have} of ${p.poses.length} collected`;
      gate(this.pcollect, !p.running && s.idle && !s.motion_refusal);
      gate($("#su-peval"), !p.running && have >= 3);
      gate($("#su-pall"), !p.running && s.idle && !s.motion_refusal);
      $("#su-peval").textContent = `Evaluate (${have})`;
      gate($("#su-preset"), !p.running && have > 0);
      this.pSaved(p.saved, s);
      const r = p.result;
      $("#su-pverdict").textContent = r ? (r.passed ? "PASS" : "FAIL") +
        (r.applied ? " · applied and saved" : "") : "Not evaluated";
      $("#su-pres").innerHTML = r ? `<span class="k">Mass</span><span class="v">${r.mass.toFixed(3)} kg</span>` +
        ["X", "Y", "Z"].map((a, i) => `<span class="k">CoG ${a}</span><span class="v">${(r.cog[i] * 1000).toFixed(1)} mm</span>`).join("") : "";
      $("#su-pissues").textContent = r ? r.issues.join(" ") + (r.bias_changed ? " The F/T bias changed since " +
        "these poses were collected: collect again before applying." : "") : "";
      $("#su-pyaml").textContent = r ? r.yaml : "—";
      gate(this.psave, !!r);
      gate(this.papply, !!(r && r.can_apply) && s.mode !== "admittance");
      this.papply.textContent = s.mode === "admittance" ? "Apply (leave Admittance first)" : "Apply to admittance";
    }
    if (this.sub === "Cameras") this.dr.update(s);
  },
};

// ============================================================== DIAGNOSTICS
function outcomeText(e) {
  const o = e.outcome || (e.ok ? "finished" : "failed");
  return o === "finished" ? "✓ finished" : o === "stopped" ? `✗ stopped by ${e.stopped_by}` : `✗ failed: ${e.result}`;
}
pages.DIAGNOSTICS = {
  build(el) {
    el.append(h("div", { class: "cols" },
      h("div", { class: "card grow" }, h("div", { class: "section" }, "Arm controllers"), h("table", {}, h("tbody", { id: "dg-ctrl" }))),
      h("div", { class: "card grow" }, h("div", { class: "section" }, "State"), h("div", { class: "grid", id: "dg-state" }))),
      this.buildRec(),
      h("div", { class: "card", style: "margin-top:8px" },
        h("div", { class: "row" }, h("span", { class: "section" }, "Run history"), h("div", { class: "grow" }),
          h("select", { id: "dg-hname", onchange: () => this.history(), title: "Show one program or policy only" },
            h("option", { value: "" }, "All programs")),
          btn("Refresh", "quiet", () => this.history())),
        h("div", { class: "scroll", style: "max-height:240px" }, h("table", { class: "hist" },
          h("thead", {}, h("tr", {}, ["When", "What", "Name", "Started by", "Length", "Steps", "Result", "Recording"].map((x) => h("th", {}, x)))),
          h("tbody", { id: "dg-hist" }))),
        h("div", { class: "muted small", id: "dg-hfile", style: "margin-top:4px" }),
        h("div", { id: "dg-hdetail" })),
      h("div", { class: "card", style: "margin-top:8px" },
        h("div", { class: "row" }, h("span", { class: "section" }, "Activity"), h("div", { class: "grow" }),
          h("select", { id: "dg-awho", onchange: () => this.activity(), title: "Show one person only" },
            h("option", { value: "" }, "Everyone")),
          h("select", { id: "dg-akind", onchange: () => this.activity(), title: "Show one kind of event only" },
            h("option", { value: "" }, "All kinds"))),
        h("div", { class: "scroll", style: "max-height:260px" }, h("table", { class: "hist" },
          h("thead", {}, h("tr", {}, ["When", "Who", "What", "Details"].map((x) => h("th", {}, x)))),
          h("tbody", { id: "dg-act" }))),
        h("div", { class: "muted small", id: "dg-afile", style: "margin-top:4px" })),
      h("div", { class: "card", style: "margin-top:8px" },
        h("div", { class: "row" }, h("span", { class: "section" }, "Log"), h("div", { class: "grow" }),
          h("input", { type: "search", id: "dg-q", placeholder: "filter…", style: "width:160px", oninput: () => this.show() }),
          h("label", { class: "chk" }, h("input", { type: "checkbox", id: "dg-err", onchange: () => this.show() }), "Problems only"),
          btn("Download", "quiet", () => {
            const a = h("a", { href: URL.createObjectURL(new Blob([logs.join("\n")], { type: "text/plain" })),
              download: `moveit_gui_log_${new Date().toISOString().slice(0, 19).replace(/:/g, "")}.txt` });
            a.click();
          }, "Save the whole log (this session) as a text file")),
        h("pre", { class: "log", id: "dg-log" })));
  },
  show() {
    const q = ($("#dg-q").value || "").toLowerCase(), bad = $("#dg-err").checked;
    const lines = logs.filter((l) => (!q || l.toLowerCase().includes(q)) && (!bad || PROBLEM.test(l)));
    $("#dg-log").textContent = lines.join("\n");
    $("#dg-log").scrollTop = 1e9;
    if (Date.now() - (this.recT || 0) > 5000) this.recCatalog();
  },

  // ---------------------------------------------------------- recording (rosbag2 / MCAP)
  buildRec() {
    const lim = (id, label, unit, tip) => h("label", { class: "chk", title: tip }, label,
      h("input", { type: "number", id, min: "1", step: "1", style: "width:72px", "data-needs-lease": true,
        onchange: () => this.recSave() }), unit);
    return h("div", { class: "card", id: "dg-rec", style: "margin-top:8px" },
      h("div", { class: "row" }, h("span", { class: "section" }, "Recording"),
        h("span", { id: "dg-rec-status", class: "small" }), h("div", { class: "grow" }),
        h("input", { type: "text", id: "dg-rec-name", placeholder: "name (optional)", maxlength: "60", style: "width:150px",
          "data-needs-lease": true }),
        btn("● Record", "danger", () => cmd("record_start", { name: $("#dg-rec-name").value }).then(() => this.recCatalog()),
          "Start a rosbag (MCAP) of the topics ticked below"),
        btn("■ Stop", "", () => cmd("record_stop").then(() => setTimeout(() => this.recCatalog(), 800)),
          "Finish the recording (anyone may stop it)")),
      h("div", { class: "row wrap", style: "margin-top:6px;gap:14px" },
        h("label", { class: "chk", title: "Start a recording when a program or policy starts, stop it when the run ends" },
          h("input", { type: "checkbox", id: "dg-rec-auto", "data-needs-lease": true,
            onchange: (e) => cmd("record_configure", { auto: e.target.checked }) }), "Auto-record every program / policy run"),
        lim("dg-rec-maxmin", "Stop after", "min", "A recording ends on its own after this long"),
        lim("dg-rec-maxmb", "or", "MB", "…or when the bag reaches this size (it also stops below 1 GB free disk)")),
      h("div", { class: "row wrap", id: "dg-rec-presets", style: "margin-top:6px;gap:10px" }),
      h("details", { id: "dg-rec-topics", style: "margin-top:6px" },
        h("summary", { class: "small", id: "dg-rec-sum" }, "Topics"),
        h("div", { class: "row", style: "margin:4px 0" },
          h("input", { type: "search", id: "dg-rec-q", placeholder: "filter topics…", style: "width:200px",
            oninput: () => this.recTopics() }),
          h("span", { class: "muted small", id: "dg-rec-missing" })),
        h("div", { class: "scroll", style: "max-height:240px" }, h("table", { class: "hist" }, h("tbody", { id: "dg-rec-list" })))),
      h("div", { class: "muted small", style: "margin-top:6px" }, "Saved in ", h("span", { class: "mono", id: "dg-rec-dir" }),
        ". Open the .mcap file in PlotJuggler 4 or Lichtblick/Foxglove; ros2 bag play replays it."),
      h("div", { class: "scroll", style: "max-height:180px;margin-top:4px" }, h("table", { class: "hist" },
        h("thead", {}, h("tr", {}, ["When", "Name", "Started by", "Length", "Size", "Topics", "Result"].map((x) => h("th", {}, x)))),
        h("tbody", { id: "dg-rec-recent" }))));
  },
  async recCatalog() {
    this.recT = Date.now();
    const r = await cmd("record_catalog", { n: 30 });
    if (!r.ok) return;
    this.rec = r;
    $("#dg-rec-dir").textContent = r.dir;
    $("#dg-rec-missing").textContent = r.missing.length ? `not published now: ${r.missing.join(", ")}` : "";
    this.recTopics();
    const mb = (v) => (v >= 1000 ? (v / 1000).toFixed(2) + " GB" : (v || 0).toFixed(1) + " MB");
    $("#dg-rec-recent").replaceChildren(...(r.recent.length ? r.recent.map((m) => h("tr", { title: m.dir },
      h("td", { class: "mono" }, new Date(m.started * 1000).toLocaleString()), h("td", { class: "mono small" }, m.name),
      h("td", {}, m.run && m.run.name ? `${m.source}: ${m.run.kind} ${m.run.name}` : m.source),
      h("td", { class: "num" }, (m.duration_s || 0).toFixed(0) + " s"), h("td", { class: "num" }, mb(m.size_mb)),
      h("td", { class: "num" }, m.topics.length),
      h("td", { style: `color:var(--${m.ok ? "ok" : m.ok === false ? "danger" : "muted"})` },
        m.ok == null ? "recording…" : (m.ok ? "✓ " : "✗ ") + m.reason))) :
      [h("tr", {}, h("td", { class: "muted" }, "No recordings yet."))]));
  },
  recTopics() {
    const r = this.rec, S0 = S && S.record;
    if (!r || !S0) return;
    const st = S0.settings, q = ($("#dg-rec-q").value || "").toLowerCase();
    const inPreset = (t) => t.presets.some((p) => st.presets.includes(p));
    const chosen = (t) => (inPreset(t) || st.topics.includes(t.topic)) && !st.exclude.includes(t.topic);
    $("#dg-rec-sum").textContent = `Topics: ${r.topics.filter(chosen).length} of ${r.topics.length} published topics selected`;
    $("#dg-rec-list").replaceChildren(...r.topics.filter((t) => !q || t.topic.toLowerCase().includes(q) || t.type.toLowerCase().includes(q))
      .map((t) => h("tr", {},
        h("td", {}, h("input", { type: "checkbox", checked: chosen(t), "data-needs-lease": true, disabled: !holds,
          onchange: (e) => {
            const on = e.target.checked, topics = new Set(st.topics), exclude = new Set(st.exclude);
            if (on) { exclude.delete(t.topic); if (!inPreset(t)) topics.add(t.topic); }
            else { topics.delete(t.topic); if (inPreset(t)) exclude.add(t.topic); }
            cmd("record_configure", { topics: [...topics], exclude: [...exclude] }).then(() => setTimeout(() => this.recTopics(), 300));
          } })),
        h("td", { class: "mono small" }, t.topic), h("td", { class: "muted small" }, t.type),
        h("td", { class: "muted small" }, t.presets.map((p) => (r.presets.find((x) => x.id === p) || {}).label).join(", ")))));
  },
  recSave() {
    cmd("record_configure", { max_s: 60 * +$("#dg-rec-maxmin").value, max_mb: +$("#dg-rec-maxmb").value });
  },
  recUpdate(s) {
    const R = s.record, st = R.settings, live = R.recording, el = $("#dg-rec-status");
    el.textContent = live ? `● ${live.stopping ? "finishing" : "recording"} ${live.name} · ${mmss(live.elapsed_s)} · ${live.size_mb.toFixed(1)} MB · ${live.topics} topics`
      : R.last ? `last: ${R.last.name} (${R.last.ok ? "saved" : "FAILED"})` : "idle";
    el.style.color = live ? "var(--danger)" : R.last && R.last.ok === false ? "var(--danger)" : "";
    $("#dg-rec").querySelector(".btn.danger").disabled = !holds || !!live;
    $("#dg-rec-name").disabled = !holds || !!live;
    [...$("#dg-rec").querySelectorAll(".btn")].find((b) => b.textContent === "■ Stop").disabled = !live || !!live.stopping;
    const auto = $("#dg-rec-auto"); auto.checked = st.auto; auto.disabled = !holds;
    for (const [id, v] of [["#dg-rec-maxmin", Math.round(st.max_s / 60)], ["#dg-rec-maxmb", Math.round(st.max_mb)]]) {
      const i = $(id); if (document.activeElement !== i) i.value = v; i.disabled = !holds;
    }
    const key = JSON.stringify([st.presets, this.rec && this.rec.presets, holds]);
    if (key !== this.recKey && this.rec) {
      this.recKey = key;
      $("#dg-rec-presets").replaceChildren(...this.rec.presets.map((p) => h("label", { class: "chk", title: p.note },
        h("input", { type: "checkbox", checked: st.presets.includes(p.id), disabled: !holds,
          onchange: (e) => cmd("record_configure", { presets: e.target.checked ? [...st.presets, p.id] : st.presets.filter((x) => x !== p.id) }) }),
        `${p.label} (${p.count})`)));
      this.recTopics();
    }
    if (live && this.recLive !== live.name) { this.recLive = live.name; this.recCatalog(); }
    if (!live && this.recLive) { this.recLive = null; setTimeout(() => this.recCatalog(), 500); }
  },
  async history() {
    const sel = $("#dg-hname"), r = await cmd("history", { n: 500, name: sel.value });
    if (!r.ok) return;
    this.histKey = S && S.history_last;
    if (r.names.join("|") !== this.hnames) {
      this.hnames = r.names.join("|");
      sel.replaceChildren(h("option", { value: "" }, "All programs"),
        ...r.names.map((n) => h("option", { value: n, selected: n === sel.value }, n)));
    }
    $("#dg-hfile").textContent = `The last ${r.keep} runs are kept (older ones are dropped). File: ${r.file}`;
    $("#dg-hist").replaceChildren(...(r.runs.length ? r.runs.map((e) => h("tr", {
      class: e.id && e.id === this.hsel ? "sel" : "", style: "cursor:pointer", title: "Click for details",
      onclick: () => this.historyOpen(e) },
      h("td", { class: "mono" }, new Date(e.start * 1000).toLocaleString()), h("td", {}, e.kind), h("td", {}, e.name),
      h("td", {}, e.operator ? e.operator.split(" @ ")[0] : "—"),
      h("td", { class: "num" }, e.duration_s ? e.duration_s.toFixed(1) + " s" : "—"),
      h("td", { class: "num" }, e.steps_total ? `${e.steps_done} of ${e.steps_total}` : "—"),
      h("td", { style: `color:var(--${e.outcome === "finished" ? "ok" : "danger"})` }, outcomeText(e)),
      h("td", { class: "mono small" }, e.recording ? e.recording.split("/").pop() : "")))
      : [h("tr", {}, h("td", { class: "muted" }, "No runs yet."))]));
  },
  async historyOpen(e) {
    this.hsel = e.id;
    const r = e.id ? await cmd("history_get", { id: e.id }) : { ok: true, run: e };
    const box = $("#dg-hdetail");
    if (!r.ok) { box.replaceChildren(h("div", { class: "muted small" }, r.msg || "not found")); return; }
    const x = r.run, kv = (k, v) => [h("span", { class: "k" }, k), h("span", { class: "v", style: "text-align:left" }, String(v))];
    box.replaceChildren(h("div", { class: "section", style: "margin-top:8px" }, `${x.kind} ${x.name}: ${outcomeText(x)}`),
      h("div", { class: "grid" },
        ...kv("Started", new Date(x.start * 1000).toLocaleString()), ...kv("Started by", x.operator || "—"),
        ...kv("Length", (x.duration_s || 0).toFixed(1) + " s"),
        ...kv("Steps", x.steps_total ? `${x.steps_done} of ${x.steps_total}` : "—"),
        ...kv("Result", x.result || "—"), ...kv("Stopped by", x.stopped_by || "—"),
        ...kv("F/T peak", x.peak_force_n == null ? "—" : `${x.peak_force_n} N, ${x.peak_torque_nm} Nm`),
        ...kv("Recording", x.recording || "—"), ...(x.log_dir ? kv("Policy log", x.log_dir) : [])),
      h("pre", { class: "log", style: "max-height:200px;margin-top:4px" },
        x.log && x.log.length ? x.log.join("\n") : "No log lines kept for this run."));
    this.history();
  },
  async activity() {
    const who = $("#dg-awho"), kind = $("#dg-akind"), r = await cmd("activity", { n: 300, who: who.value, kind: kind.value });
    if (!r.ok) return;
    this.actSeq = S && S.activity_seq;
    if (r.people.join("|") !== this.apeople) {
      this.apeople = r.people.join("|");
      who.replaceChildren(h("option", { value: "" }, "Everyone"), ...r.people.map((n) => h("option", { value: n, selected: n === who.value }, n)));
    }
    if (!kind.options[1]) kind.append(...r.kinds.map((k) => h("option", { value: k }, k)));
    $("#dg-afile").textContent = `Newest first, up to 300 events. Read-only. File: ${r.file}`;
    $("#dg-act").replaceChildren(...(r.events.length ? r.events.map((e) => h("tr", {},
      h("td", { class: "mono" }, e.time), h("td", { title: [e.key, e.role].filter(Boolean).join(" · ") }, e.who || "—"),
      h("td", {}, e.what), h("td", { class: "muted small" }, e.details)))
      : [h("tr", {}, h("td", { class: "muted" }, "Nothing yet."))]));
  },
  update(s) {
    if (s.history_last !== this.histKey && Date.now() - (this.histT || 0) > 1000) { this.histT = Date.now(); this.history(); }
    if (s.activity_seq !== this.actSeq && Date.now() - (this.actT || 0) > 1000) { this.actT = Date.now(); this.activity(); }
    $("#dg-ctrl").innerHTML = Object.entries(s.controllers || {}).sort().map(([n, st]) =>
      `<tr><td>${n}</td><td style="color:var(--${st === "active" ? "ok" : "muted"})">${st}</td></tr>`).join("") || "<tr><td class='muted'>not reported</td></tr>";
    const rows = [["Mode", s.mode_label], ["Command source", s.command_source],
      ["Joint state", s.joint_state_age == null ? "never received" : (s.joint_state_age * 1000).toFixed(0) + " ms"],
      ["Connections", Object.entries(s.connected || {}).map(([k, v]) => `${k} ${v ? "✓" : "✗"}`).join("  ")],
      ["Program", `${s.runner.program} · ${s.runner.state}`],
      ["Policy runner", s.policy.busy ? "busy " + (s.policy.current || "") : "idle"],
      ["Control lease", s.lease.client ? s.lease.name : "nobody"], ["Last refusal", s.last_refusal || "—"],
      ["Keep-out zones", `${s.keepout.boxes.filter((b) => b.enabled).length} enabled · ` +
        (s.keepout.status.state === "ok" ? "✓ " : s.keepout.status.state === "pending" ? "" : "⚠ ") + s.keepout.status.msg +
        (s.keepout.status.t ? ` (${new Date(s.keepout.status.t * 1000).toLocaleTimeString()})` : "")]];
    $("#dg-state").replaceChildren(...rows.flatMap(([k, v]) => [h("span", { class: "k" }, k),
      h("span", { class: "v", style: "text-align:left" }, String(v))]));
    this.recUpdate(s);
  },
};

// ================================================================== SETTINGS
// The GUI itself (not the robot): screen layouts, who you are (profiles), who may do what (roles).
pages.SETTINGS = {
  sub: "Layouts",
  ITEMS: ["Layouts", "Profiles", "Roles"],
  build(el) {
    this.nav = h("div", { class: "list subnav" }, this.ITEMS.map((n) =>
      h("div", { class: "item", "data-name": n, onclick: () => this.show(n) }, h("span", { class: "grow" }, n))));
    this.body = h("div", { class: "grow col" });
    el.append(h("div", { class: "cols" }, h("div", { class: "card subnav" }, this.nav), this.body));
    this.show("Layouts");
  },
  show(n) {
    if (!this.ITEMS.includes(n)) n = "Layouts";
    this.sub = n;
    this.nav.querySelectorAll(".item").forEach((it) => it.classList.toggle("on", it.dataset.name === n));
    if (n === "Roles") this.body.replaceChildren(pages.SETUP.rolesView());
    else if (n === "Profiles") this.profiles();
    else {                                  // made by OPERATE's workspace (views.js)
      this.lay = this.lay || pages.OPERATE.vg.setupView();
      this.lay.refresh();
      this.body.replaceChildren(h("div", { class: "col su-main" }, this.lay.el));
    }
  },
  // ---- Profiles: one per person; it keeps your setups and your screen layout
  async profiles() {
    const box = h("div", { class: "col su-main" });
    this.body.replaceChildren(box);
    const draw = (r) => {
      if (this.sub !== "Profiles") return;
      const me = localStorage.getItem("operator") || "";
      const list = r.profiles || [];
      const mine = list.find((p) => p.name.toLowerCase() === me.toLowerCase());
      const others = list.filter((p) => p !== mine && p.layout);
      const src = h("select", {}, others.map((p) => h("option", { value: p.name }, p.name)));
      const redraw = async () => { const q = await cmd("profile_list"); if (q.ok) draw(q); };
      box.replaceChildren(
        h("div", { class: "card" },
          h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "Profiles"),
            btn("+ New profile", "", async () => {
              const name = await askText("Name of the new profile (your name):", "");
              if (name && name.trim()) { const c = await cmd("profile_create", { name: name.trim() }); if (c.ok) draw(c); }
            })),
          note("A profile keeps one person's setups and screen layout.",
            "Pick who you are with the name in the header. A profile can be deleted only when it has no setups."),
          h("div", { class: "list" }, ...list.map((p) => h("div", { class: "item" },
            h("span", { class: "grow" }, p.name + (p === mine ? " (you)" : "")),
            h("span", { class: "muted small" }, `${p.setups} setup${p.setups === 1 ? "" : "s"}${p.layout ? " · layout" : ""}`),
            btn("Delete", "quiet", async () => {
              if (!(await ask(`Delete profile ${p.name}?`))) return;
              const c = await cmd("profile_delete", { name: p.name });
              if (c.ok) { if (p === mine) localStorage.removeItem("operator"); redraw(); }
            }, p.setups ? "Has setups: not empty" : "Delete this empty profile")))),
          list.length ? "" : h("div", { class: "muted" }, "No profiles yet.")),
        h("div", { class: "card" },
          h("div", { class: "section" }, "Copy a layout"),
          !mine ? h("div", { class: "muted" }, "Pick your profile first (the name in the header).")
            : !others.length ? h("div", { class: "muted" }, "No other profile has a saved layout.")
              : [note(`Replaces the layout of ${mine.name} with a copy of another one. The page reloads.`),
                h("div", { class: "row" }, "From", src,
                  btn("Copy", "primary", async () => {
                    if (!(await ask(`Replace the layout of ${mine.name} with ${src.value}'s?`))) return;
                    const c = await cmd("profile_prefs_copy", { name: mine.name, source: src.value });
                    if (c.ok) await profilePrefs(mine.name, true);
                  }))]));
    };
    const r = await cmd("profile_list");
    if (r.ok) draw(r);
  },
};

// ===================================================================== shell
function navTo(name, sub) {                  // sub: a SETUP item, e.g. "Cameras"
  if (pages[current] && pages[current].hide) pages[current].hide();
  if (typeof sub === "string" && pages[name]) pages[name].sub = sub;
  current = name;
  document.querySelectorAll("nav a").forEach((a) => a.classList.toggle("on", a.textContent === name));
  document.querySelectorAll(".page").forEach((p) => p.classList.toggle("on", p.dataset.page === name));
  if (pages[name].show) pages[name].show(pages[name].sub);
  location.hash = name;
  render();
}

// What the robot is doing now, for the header pill and the amber banner: [text, css class, page, banner text]
function activity(s) {
  const r = s.runner, pol = s.policy, f = s.force, B = s.batch;
  const bk = B && B.active ? `Batch ${Math.min(B.done + 1, B.total)}/${B.total}` : "";
  if (bk && B.phase === "waiting")
    return [`❚❚ ${bk}: waiting`, "warn", "PROGRAMS", "The batch waits for the person in control to start the next run."];
  if (r.state !== "IDLE") {
    const where = r.path ? r.path.map((x) => x + 1).join(".") : "";
    return [`${r.state === "PAUSED" ? "❚❚ Paused" : "▶"} ${bk ? bk + " · " : ""}program ${r.program || ""}${where ? " · " + where : ""}`,
      r.state === "PAUSED" ? "warn" : "run", "PROGRAMS", ""];
  }
  if (bk) return [`▶ ${bk}`, "run", "PROGRAMS", ""];
  if (s.teach) return [s.teach.mode === "path" && s.teach.phase === "recording" ? "● Teaching a path" : "Teaching",
    "warn", "PROGRAMS", "Teaching by hand on PROGRAMS: nothing moves by itself."];
  if (pol.busy) {
    const fb = pol.feedback;
    return pol.current
      ? [`▶ Policy ${pol.current}${fb && fb.elapsed ? " · " + fb.elapsed.toFixed(1) + " s" : ""}`, "run", "AUTOMATION", ""]
      : ["Preparing policy…", "warn", "AUTOMATION", ""];
  }
  if (s.payload && s.payload.running)
    return [`Payload calibration ${s.payload.done}/${s.payload.total}`, "run", "SETUP", ""];
  if (f && f.active) {
    const w = f.target || [], n = Math.hypot(w[0] || 0, w[1] || 0, w[2] || 0);
    return [`Force ${n.toFixed(1)} N${f.releasing ? " (releasing)" : ""}`, "warn", "OPERATE",
      `A force of ${n.toFixed(1)} N is being applied (${s.mode_label}). Release it on OPERATE → Force, or press STOP.`];
  }
  if (s.freedrive) return ["Freedrive", "warn", "OPERATE", "Freedrive is on: the arm can be moved by hand. End it on OPERATE."];
  if (s.jog && s.jog.active) return ["Jogging (Servo)", "run", "OPERATE", ""];
  if (s.busy) return [`Moving · ${s.command_source}`, "run", "OPERATE", ""];
  return ["Nothing", "idle", null, ""];
}

// ROBOT in the header: one plain answer (Ready / what is wrong) and a small line saying what to do
const nice = (x) => { x = String(x || "").replace(/_/g, " ").toLowerCase(); return x.charAt(0).toUpperCase() + x.slice(1); };
function robotSummary(s, progOff, healthy) {
  if (s.robot_mode === "?" || !s.robot_mode) return ["No data", "Waiting for the robot", ""];
  if (!["NORMAL", "REDUCED"].includes(s.safety_mode))
    return [nice(s.safety_mode), "See the Robot card", "bad"];
  if (s.robot_mode !== "RUNNING")
    return [nice(s.robot_mode), s.robot_mode === "POWER_OFF" ? "Power on from the Robot card" : "Release the brakes on the Robot card", "bad"];
  if (progOff && s.program_state === "PLAYING") return ["Not connected to the pendant program",
    "A program plays, but its External Control host is another PC: set it to this PC, then Play", "bad"];
  if (progOff) return [s.program_state === "PAUSED" ? "Pendant program paused" : "Pendant program stopped",
    `Press ${s.program_state === "PAUSED" ? "Resume" : "Play"} program on the Robot card`, "bad"];
  return ["Ready", `Running · ${nice(s.safety_mode)}`, healthy ? "ok" : ""];
}
function renderHeader(s) {
  // robot_program_running is only published once it changes: no value while RUNNING = External Control never ran
  const progOff = s.program_running === false || (s.program_running == null && s.robot_mode === "RUNNING");
  const healthy = s.robot_mode === "?" ? null : s.robot_mode === "RUNNING" && ["NORMAL", "REDUCED"].includes(s.safety_mode)
    && !progOff;
  const [stTxt, stSub, stCls] = robotSummary(s, progOff, healthy);
  $("#h-state-dot").className = "dot " + stCls;
  $("#h-state").textContent = stTxt;
  $("#h-state").className = "st-" + stCls;
  $("#h-state-sub").textContent = stSub;
  $("#h-stategrp").title = `Robot mode ${s.robot_mode} · safety ${s.safety_mode}` +
    (s.program_running == null ? "" : ` · pendant program ${progOff ? "not running" : "running"}`);
  const m = INFO.modes.find((x) => x.name === s.mode);
  $("#h-mode").textContent = m ? m.label : s.mode_label;
  $("#h-mode").title = s.mode_label;
  $("#h-mode").className = "val " + (s.unknown || s.mode === "unknown" ? "mode-bad" : m && (m.direct_torque || m.experimental) ? "mode-dt" : "");
  $("#h-speed").textContent = s.speed == null ? "—" : (s.speed * 100).toFixed(0) + "%";
  const ovr = $("#h-ovr");
  if (document.activeElement !== ovr && s.speed_override != null) ovr.value = Math.round(s.speed_override * 100);
  $("#h-ovr-val").textContent = ovr.value + "%";
  $("#h-ovr-val").className = +ovr.value < 100 ? "ovr-low" : "";
  gate(ovr, true);
  const [act, actCls, actPage, actBanner] = activity(s);
  const pill = $("#h-activity");
  pill.textContent = act;
  pill.className = "pill " + actCls;
  pill.onclick = actPage ? () => navTo(actPage) : null;
  const rec = s.record && s.record.recording, rp = $("#h-rec");
  rp.hidden = !rec;
  const rb = $("#h-recbtn");
  rb.innerHTML = rec ? '■<span class="lbl"> Stop rec</span>' : '●<span class="lbl"> Record</span>';
  rb.classList.toggle("danger", !!rec);
  rb.disabled = rec ? !!rec.stopping : !holds;
  if (rec) {
    rp.textContent = `● REC ${mmss(rec.elapsed_s)} · ${rec.size_mb.toFixed(1)} MB`;
    rp.title = `Recording ${rec.name}: ${rec.size_mb.toFixed(1)} MB, ${rec.topics} topics (${rec.source}). Click for details.`;
  }
  const bad = Object.entries(s.connected || {}).filter(([, v]) => !v).map(([k]) => k);
  $("#h-conn").innerHTML = bad.length ? icon("warn") + `<span class="ell">${bad.join(", ")} off</span>` : "";
  $("#h-conn").title = bad.length ? "Not connected: " + bad.join(", ") : "";
  $("#h-conn").hidden = !bad.length;
  document.body.classList.toggle("disconnected-parts", bad.length > 0);
  // TCP select
  const c = s.calibration, key = JSON.stringify([c.tcps, c.active_tcp]);
  if (key !== renderHeader.tcpKey) {
    renderHeader.tcpKey = key;
    $("#h-tcp").replaceChildren(...c.tcps.map((t) => h("option", { value: t.name, selected: t.name === c.active_tcp },
      t.name + (t.builtin ? "" : " · virtual"))));
  }
  gate($("#h-tcp"), s.idle);
  // frames for every frame select
  const fkey = JSON.stringify(c.frames);
  if (fkey !== renderHeader.fkey) {
    renderHeader.fkey = fkey;
    document.querySelectorAll("select[data-frames]").forEach((sel) => {
      const cur = sel.value;
      sel.replaceChildren(...c.frames.map((f) => h("option", { value: f }, f)));
      const vial = c.fixtures.find((f) => f.includes("vial"));
      sel.value = c.frames.includes(cur) ? cur : sel.id === "pg-sc-frame" && vial ? vial : c.frames[0];
    });
  }
  // lease
  const L = s.lease;
  // (states sent just before our own takeover still name the old holder: ignore them for 2 s)
  if (L.client && L.client !== MYID && holds && Date.now() - leaseAt > 2000) lostControl(`${L.name} took over`);
  $("#lease-who").textContent = holds ? `You (${L.name.split(" @ ")[0]})` : L.client ? `${L.name.split(" @ ")[0]} has control${L.away ? " (away)" : ""}` : "";
  $("#lease-view").hidden = holds;
  $("#lease-who").title = "Click to change your name";
  $("#lease-who").onclick = async () => { await operatorName(true); if (holds) takeLease(); };
  usersRender(s);
  batchAsk(s);
  // first time on this PC: nobody picked a robot yet, so ask the one in control (once per page load)
  if (holds && s.workcell?.robot_chosen === false && !robotAsked && pages.SETUP?.wc?.W.pickRobot) {
    robotAsked = true;
    pages.SETUP.wc.W.pickRobot(true);
  }
  document.body.classList.toggle("viewing", !holds);
  $("#lease-btn").textContent = holds ? "Release" : L.client ? "Take over" : "Take control";
  const viewer = INFO && INFO.role === "viewer";
  $("#lease-btn").disabled = viewer && !holds;
  $("#lease-btn").title = viewer ? "You are a Viewer: an Admin can make you an Operator (SETTINGS › Roles)" : "";
  $("#lease-btn").className = "btn " + (holds ? "quiet" : "primary");
  $("#lease-btn").style.color = holds ? "var(--on-chrome2)" : "";
  document.querySelectorAll("[data-freedrive]").forEach((b) => {
    b.textContent = s.freedrive ? "End freedrive" : "Freedrive";
    b.classList.toggle("on", s.freedrive);
    gate(b, s.freedrive || s.idle);
  });
  document.querySelectorAll("[data-lease]").forEach((b) => gate(b, true));
  document.querySelectorAll("[data-grip]").forEach((b) => {
    gate(b, !s.manual_refusal);
    if (holds) b.title = s.manual_refusal ? "Not now: " + s.manual_refusal : b.dataset.tip || "";
  });
  const banner = $("#banner");
  const inside = s.keepout && s.keepout.inside && s.keepout.inside.length
    ? `The robot is inside keep-out zone ${s.keepout.inside.join(", ")}: MoveIt will refuse every plan. ` +
      "Jog or freedrive out of it, or untick the box in SETUP." : "";
  // on OPERATE the Arm control card already shows freedrive (with its End button): no banner on top of the views
  const warn = (s.freedrive && current === "OPERATE" ? "" : actBanner) || inside;
  banner.className = !connected || s.unknown || srvError || netBad ? "on" : warn ? "on warn" : "";
  banner.textContent = !connected ? "Connection to the robot PC lost. Reconnecting… (the robot stops if this browser had control)"
    : netBad ? "Connection problem: the robot PC answers slowly or not at all. Still trying; if it lasts the robot stops and control is released."
    : srvError ? `The robot PC reports an error: ${srvError} (showing the last good state)`
    : s.unknown ? "CONTROL STATE UNKNOWN: GUI motion is disabled. " + (s.unknown_reason || "") : warn;
  document.body.classList.toggle("stale", !connected);
}

// SETUP > Parts changed the robot model: every 3D view that has loaded it reloads it (once per change)
let modelRev = null;
async function modelWatch(s) {
  const rev = s.workcell && s.workcell.model_rev;
  if (rev == null || rev === modelRev) return;
  const first = modelRev === null;
  modelRev = rev;
  if (first) return;
  const r = await fetch("/api/robot_model").then((x) => x.json()).catch(() => null);
  if (!r || !r.ok) return;
  for (const p of [pages.OPERATE, pages.PROGRAMS, pages.SETUP]) if (p && p.view && p.view.reload) p.view.reload(r);
}

function render() {
  if (!S || !INFO) return;
  renderHeader(S);
  if (holds && S.lease && S.lease.needs_switch) switchSettings();
  if (pages[current].update) pages[current].update(S);
}

async function boot() {
  INFO = await (await fetch("/api/info")).json();
  const me = localStorage.getItem("operator");
  // this profile's layout (changed in another browser: reload once). Stop here when reloading: building the pages
  // writes some of these keys (the F/T chart's open state), and the next load would differ again: a reload loop
  if (me && await profilePrefs(me, true)) return;
  $("#h-robot").replaceChildren(INFO.robot, ...(INFO.fake ? [" ", h("span", { class: "hchip fake", title: "Fake robot: no driver, nothing is sent to a robot" }, "FAKE")] : []));
  document.body.classList.toggle("fake", !!INFO.fake);
  $("#h-robot").title = `${INFO.ur_type} · ${INFO.robot_ip} · profile ${INFO.profile}`;
  document.title = `MoveIt GUI — ${INFO.robot}`;
  const nav = $("#nav"), main = $("#main");
  for (const name of Object.keys(pages)) {
    nav.append(h("a", { onclick: () => navTo(name) }, name));
    if (name === "OPERATE") nav.append(h("div", { class: "navlay", id: "nav-lay", hidden: true }));
    const el = h("div", { class: "page", "data-page": name });
    main.append(el);
    pages[name].build(el);
  }
  $("#stop").onclick = () => post("/api/stop");
  $("#lease-btn").onclick = () => (holds ? releaseLease() : takeLease());
  $("#lease-ask").onclick = askControl;
  $("#h-users").onclick = usersToggle;
  $("#h-tcp").onchange = (e) => cmd("set_tcp", { name: e.target.value });
  $("#h-ovr").oninput = (e) => ($("#h-ovr-val").textContent = e.target.value + "%");
  $("#h-ovr").onchange = (e) => { cmd("speed_override", { percent: +e.target.value }); e.target.blur(); };
  $("#logs-link").onclick = () => navTo("DIAGNOSTICS");
  $("#h-recbtn").onclick = async () => {
    if (S && S.record && S.record.recording) { cmd("record_stop"); return; }
    const note = await askText("Record everything\n\nNote for this run (optional):", "", { ok: "Record", maxLength: 200, placeholder: "note" });
    if (note != null) cmd("record_start", { preset: "all", note: note.trim() });
  };
  $("#h-rec").onclick = () => { navTo("DIAGNOSTICS"); $("#dg-rec").scrollIntoView({ block: "start" }); };
  document.addEventListener("keydown", (e) => { if (e.key === "Escape") post("/api/stop"); });

  const es = new EventSource("/api/events?bid=" + encodeURIComponent(BID) + "&name=" +
    encodeURIComponent(localStorage.getItem("operator") || ""));
  es.addEventListener("hello", (e) => { MYUID = JSON.parse(e.data).id; });
  let boot = null, logSeq = 0;
  es.addEventListener("state", (e) => {
    connected = true;
    const next = JSON.parse(e.data);
    if (next.error) { srvError = String(next.error); if (S) render(); return; }   // keep the last good state
    srvError = ""; S = next;
    if (S.boot && boot && S.boot !== boot && sessionStorage.getItem("retake")) { location.reload(); return; }
    if (S.boot && boot && S.boot !== boot && !$("#reload-bar")) {     // the GUI restarted: this page is old code
      document.body.prepend(h("div", { id: "reload-bar", class: "reload-bar" },
        "The GUI was restarted (maybe updated). This page still runs the old version. ",
        h("button", { class: "btn primary", onclick: () => location.reload() }, "Reload page")));
    }
    if (S.boot && boot && S.boot !== boot) logSeq = 0;               // sequence numbers restart with the server
    boot = boot || S.boot;
    modelWatch(S);
    render();
  });
  es.addEventListener("log", (e) => {
    const fresh = JSON.parse(e.data).filter(([seq]) => seq > logSeq);   // a reconnect resends the last 200 lines
    if (!fresh.length) return;
    logSeq = fresh[fresh.length - 1][0];
    const lines = fresh.map(([, line]) => line);
    logs.push(...lines);
    if (logs.length > 3000) logs.splice(0, logs.length - 3000);
    const last = lines[lines.length - 1];
    if (last && logs.seenFirst) say(last, !PROBLEM.test(last));     // the first batch is history: not an alarm
    else if (last) say(last, true);
    logs.seenFirst = true;
    if (current === "DIAGNOSTICS") pages.DIAGNOSTICS.show();
  });
  es.addEventListener("result", (e) => {
    const r = JSON.parse(e.data);
    holdJobDone(r.id);
    if (holdCancelled.has(r.id)) { holdCancelled.delete(r.id); myJobs.delete(r.id); toast(`${r.cmd}: stopped (button released)`, true); return; }
    if (myJobs.has(r.id)) {
      myJobs.delete(r.id);
      toast(`${r.cmd}: ${r.msg}`, r.ok);
      if (r.ok && r.cmd === "MoveJ") pages.OPERATE.resetJoints();      // the target is reached: fields follow again
      if (r.ok && r.cmd === "MovePose") pages.OPERATE.resetCart();
    }
    else say(`${r.cmd}: ${r.msg}`, r.ok);
  });
  let probing = false;
  es.onerror = async () => {                               // the heartbeat, not the stream, decides the lease
    connected = false; render();
    if (probing) return;                                   // the robot stack was stopped: show its Start page
    probing = true;
    try { const r = await fetch("/api/info", { cache: "no-store" }); if (r.status === 503 && (await r.json()).stack === "stopped") location.reload(); }
    catch (e) { /* down for a restart: EventSource retries */ }
    setTimeout(() => { probing = false; }, 3000);
  };
  navTo(Object.keys(pages).includes(location.hash.slice(1)) ? location.hash.slice(1) : "OPERATE");
  if (sessionStorage.getItem("retake")) {                 // back after switching to your settings: control again
    sessionStorage.removeItem("retake");
    takeLease();
  }
}
boot();
