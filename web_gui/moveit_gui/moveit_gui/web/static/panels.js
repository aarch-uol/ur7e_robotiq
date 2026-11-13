// OPERATE views besides cameras and the F/T chart (views.js shows one of each in a window):
//   plan  Motion planner: plan Manual's target, preview it again, execute or clear; the last plans this page made
//   jchart Joint chart: joint angles (or speeds) over the last 20 s
//   prog  Program run: what runs, where it is, Pause / Resume / Stop
//   log   Log: the last lines of the GUI log, with a filter
//   drv   Camera drivers: each driver from SETUP › Cameras with its state and Start / Stop
// The server owns every rule: these only show the state and send the same commands as the other pages.
import { icon } from "/static/icons.js";
import { axisRange, policyChart } from "/static/monitors.js";
import { STATE, ON } from "/static/drivers.js";

const COLORS = ["#d1495b", "#2e8b57", "#2f5fb3", "#c77d12", "#7b4fb3", "#14879c", "#6b7280"];

export function extraViews({ h, btn, cmd, pcmd, INFO, getS, op, navTo, logs }) {
  const box = (cls, ...kids) => h("div", { class: "xview " + cls }, ...kids);

  // ---------------------------------------------------------------- Motion planner
  const hist = [];                 // plans this page saw: {id, what, dur, at, end}
  let lastId = null;
  const pState = h("div", { class: "xp-state" });
  const pInfo = h("div", { class: "xp-info muted small" });
  const pHist = h("div", { class: "xp-hist" });
  const lease = (b) => (b.dataset.lease = "1", b);
  const planBtn = lease(btn("Plan", "", () => op.plan(), "Plan Manual's target (Joints or Pose tab) with its Speed and Accel."));
  const execBtn = lease(btn("Execute", "primary", () => op.execute(), "Run the plan, if the arm is still at its start"));
  const clearBtn = lease(btn("Clear", "quiet", () => cmd("plan_clear")));
  const replayBtn = btn("Preview again", "quiet", () => op.replayPreview(), "Play the plan in the 3D view again");
  const plan = {
    label: "Motion planner", min: 300,
    el: box("xplan", pState, h("div", { class: "row xp-btns" }, planBtn, replayBtn, h("span", { class: "grow" }), clearBtn, execBtn),
      pInfo, h("div", { class: "section" }, "This session"), pHist),
    async update(s) {
      const p = s.plan || {};
      if (p.what && p.id !== lastId) {
        lastId = p.id;
        hist.unshift({ id: p.id, what: p.what, dur: p.duration_s, at: new Date(), end: "" });
        hist.length = Math.min(hist.length, 8);
        detail(p.id);
      }
      if (!p.what && lastId != null) {
        const x = hist.find((y) => y.id === lastId);
        if (x && !x.end) x.end = s.busy ? "executed" : "cleared";
        lastId = null; pInfo.textContent = "";
      }
      pState.replaceChildren(p.what
        ? h("span", { class: "chip ok" }, `Plan · ${p.what} · ${(+p.duration_s).toFixed(1)} s`)
        : h("span", { class: "muted" }, s.busy ? "Moving…" : "No plan. Set a target in Manual, then Plan."));
      execBtn.hidden = clearBtn.hidden = replayBtn.hidden = !p.what;
      execBtn.disabled = !!s.motion_refusal;
      execBtn.title = s.motion_refusal ? "Cannot move: " + s.motion_refusal : "Run the plan, if the arm is still at its start";
      planBtn.disabled = !!s.busy;
      pHist.replaceChildren(...(hist.length ? hist.map((x) => h("div", { class: "xp-row" },
        h("span", { class: "mono muted" }, x.at.toTimeString().slice(0, 8)), h("span", { class: "grow" }, x.what),
        h("span", { class: "mono" }, (+x.dur).toFixed(1) + " s"), h("span", { class: "muted small" }, x.end || (x.id === lastId ? "ready" : ""))))
        : [h("div", { class: "muted small" }, "None yet.")]));
    },
  };
  async function detail(id) {          // the biggest joint move of the plan, from its points
    const r = await cmd("plan_get").catch(() => null);
    if (!r || !r.ok || !r.plan || r.id !== id) return;
    const pts = r.plan.points, a = pts[0].q, b = pts[pts.length - 1].q;
    let k = 0, m = 0;
    a.forEach((v, i) => { const d = Math.abs(b[i] - v); if (d > m) { m = d; k = i; } });
    pInfo.textContent = `${pts.length} points · biggest joint move J${k + 1} ${(m * 180 / Math.PI).toFixed(1)}° · ${r.plan.age_s.toFixed(0)} s old when shown`;
  }

  // ---------------------------------------------------------------- Joint chart
  const n = INFO.joint_names.length, WIN = 20;
  let t = [], q = INFO.joint_names.map(() => []), kind = "deg", raf = 0;
  const pick = new Set(INFO.joint_names.map((_, i) => i));
  const canvas = h("canvas", { class: "xj-canvas" });
  const chips = INFO.joint_names.map((nm, i) => {
    const b = h("button", { class: "ftc-chip", type: "button", "aria-pressed": "true", title: nm,
      onclick: () => { pick.has(i) ? pick.delete(i) : pick.add(i); b.setAttribute("aria-pressed", String(pick.has(i))); kick(); } },
      h("span", { class: "ftc-sw", style: `background:${COLORS[i % COLORS.length]}` }), `J${i + 1}`);
    return b;
  });
  const kindSel = h("span", { class: "tseg" }, [["deg", "Angle"], ["dps", "Speed"]].map(([k, label]) =>
    h("button", { type: "button", class: k === kind ? "on" : "", onclick: (e) => {
      kind = k; for (const x of e.currentTarget.parentNode.children) x.classList.toggle("on", x === e.currentTarget); kick();
    } }, label)));
  const jEl = box("xjoint", h("div", { class: "row xj-top" }, h("div", { class: "ftc-chips xj-chips" }, chips), h("span", { class: "grow" }), kindSel), canvas);
  const kick = () => { if (!raf) raf = requestAnimationFrame(() => { raf = 0; drawJ(); }); };
  function drawJ() {
    if (!canvas.isConnected) return;
    const dpr = devicePixelRatio || 1, Wd = canvas.clientWidth, Hd = canvas.clientHeight;
    if (!Wd || !Hd) return;
    if (canvas.width !== Math.round(Wd * dpr) || canvas.height !== Math.round(Hd * dpr)) { canvas.width = Math.round(Wd * dpr); canvas.height = Math.round(Hd * dpr); }
    const g = canvas.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, Wd, Hd);
    const now = t.length ? t[t.length - 1] : 0, L = 42, R = 8, T = 6, B = 18, pw = Wd - L - R, ph = Hd - T - B;
    let k0 = 0; while (k0 < t.length && t[k0] < now - WIN) k0++;
    const val = (i, k) => kind === "deg" ? q[i][k] : k > 0 && t[k] > t[k - 1] ? (q[i][k] - q[i][k - 1]) / (t[k] - t[k - 1]) : 0;
    const vs = []; for (const i of pick) for (let k = Math.max(k0, 1); k < t.length; k++) vs.push(val(i, k));
    const a = axisRange(vs, kind === "deg" ? 10 : 2);
    const X = (sec) => L + pw * (1 - sec / WIN), Y = (v) => T + ph * (1 - (v - a.lo) / (a.hi - a.lo));
    g.font = "10px system-ui, sans-serif"; g.lineWidth = 1; g.textBaseline = "middle"; g.textAlign = "right";
    for (let v = a.lo; v <= a.hi + a.step / 2; v += a.step) {
      const y = Math.round(Y(v)) + 0.5, zero = Math.abs(v) < a.step / 1e3;
      g.strokeStyle = zero ? "#808a96" : "#e2e5e9"; g.beginPath(); g.moveTo(L, y); g.lineTo(L + pw, y); g.stroke();
      g.fillStyle = "#515a65"; g.fillText((zero ? 0 : v).toFixed(a.step < 1 ? 1 : 0), L - 4, y);
    }
    g.save(); g.translate(9, T + ph / 2); g.rotate(-Math.PI / 2); g.textAlign = "center"; g.fillText(kind === "deg" ? "°" : "°/s", 0, 0); g.restore();
    g.textAlign = "center"; g.textBaseline = "alphabetic";
    for (let sec = WIN; sec >= 0; sec -= 5) { g.fillStyle = "#515a65"; g.fillText(sec ? `-${sec} s` : "now", Math.min(Math.max(X(sec), L + 10), L + pw - 10), Hd - 4); }
    g.save(); g.beginPath(); g.rect(L, T, pw, ph); g.clip(); g.lineWidth = 1.5;
    for (const i of pick) {
      g.strokeStyle = COLORS[i % COLORS.length]; g.beginPath();
      let first = true;
      for (let k = Math.max(k0, kind === "deg" ? 0 : 1); k < t.length; k++) { const x = X(now - t[k]), y = Y(val(i, k)); first ? g.moveTo(x, y) : g.lineTo(x, y); first = false; }
      g.stroke();
    }
    g.restore();
  }
  const jchart = {
    label: "Joint chart", min: 300, fill: true, el: jEl,
    update(s) {
      if (!s.joints_deg) return;
      const ts = performance.now() / 1000;
      t.push(ts); s.joints_deg.forEach((v, i) => q[i] && q[i].push(v));
      while (t.length && t[0] < ts - 65) { t.shift(); q.forEach((x) => x.shift()); }
      kick();
    },
    resize: kick,
  };
  new ResizeObserver(kick).observe(canvas);

  // ---------------------------------------------------------------- Program run
  const rState = h("div", { class: "xr-state" }), rWhere = h("div", { class: "mono small" }), rWhy = h("div", { class: "muted small" });
  const pauseBtn = lease(btn("Pause", "", () => (getS().runner.state === "PAUSED" ? pcmd("program_resume") : pcmd("program_pause"))));
  const stopBtn = lease(btn("■ Stop", "danger", () => pcmd("program_stop")));
  const prog = {
    label: "Program run", min: 260,
    el: box("xprog", rState, rWhere, rWhy, h("div", { class: "row" }, pauseBtn, stopBtn, h("span", { class: "grow" }),
      btn("Open PROGRAMS", "quiet", () => navTo("PROGRAMS")))),
    update(s) {
      const r = s.runner, on = r.state !== "IDLE";
      rState.replaceChildren(h("span", { class: "chip" + (r.state === "RUNNING" ? " ok" : r.state === "PAUSED" ? " warn" : "") },
        on ? r.state.toLowerCase() : "not running"), " ", h("b", {}, r.program || "(no program open)"));
      rWhere.textContent = on && r.path ? "step " + r.path.map((x) => x + 1).join(".") +
        (r.loops && r.loops.length ? " · loop " + r.loops.map((l) => l.join("/")).join(", ") : "") : "";
      rWhy.textContent = r.state === "PAUSED" && r.pause_reason ? r.pause_reason
        : !on && r.result ? (r.result.success ? "Last run finished: " : "Last run stopped: ") + (r.result.reason || "") : "";
      pauseBtn.hidden = stopBtn.hidden = !on;
      pauseBtn.innerHTML = icon(r.state === "PAUSED" ? "play" : "pause") + `<span>${r.state === "PAUSED" ? "Resume" : "Pause"}</span>`;
    },
  };

  // ---------------------------------------------------------------- Log
  const filter = h("input", { type: "search", placeholder: "Filter", class: "xl-filter", oninput: () => { shown = -1; logUpdate(); } });
  const errOnly = h("input", { type: "checkbox", onchange: () => { shown = -1; logUpdate(); } });
  const pre = h("div", { class: "xl-lines mono" });
  let shown = -1;
  function logUpdate() {
    if (!pre.isConnected || shown === logs.length) return;
    shown = logs.length;
    const f = filter.value.trim().toLowerCase(), bad = /fail|error|refus|cannot|lost|stopped|warn/i;
    const lines = logs.slice(-400).filter((l) => (!f || l.toLowerCase().includes(f)) && (!errOnly.checked || bad.test(l))).slice(-150);
    const stick = pre.scrollTop + pre.clientHeight >= pre.scrollHeight - 8;
    pre.replaceChildren(...lines.map((l) => h("div", { class: bad.test(l) ? "bad" : "" }, l)));
    if (stick) pre.scrollTop = pre.scrollHeight;
  }
  const log = {
    label: "Log", min: 300,
    el: box("xlog", h("div", { class: "row" }, filter, h("label", { class: "small muted" }, errOnly, " problems only"), h("span", { class: "grow" }),
      btn("All logs", "quiet", () => navTo("DIAGNOSTICS"))), pre),
    update: logUpdate,
  };

  // ---------------------------------------------------------------- Camera drivers
  const dList = h("div", { class: "xd-list" });
  let dSig = null;
  const drv = {
    label: "Camera drivers", min: 240,
    el: box("xdrv", dList, h("div", { class: "row" }, h("span", { class: "grow" }),
      btn("Set up…", "quiet", () => navTo("SETUP", "Cameras")))),
    update(s) {
      const ds = s.workcell?.drivers || [];
      const sig = JSON.stringify(ds);
      if (sig === dSig) return;
      dSig = sig;
      dList.replaceChildren(...(ds.length ? ds.map((d) => {
        const [txt, cls] = STATE[d.state] || STATE.stopped, on = ON.has(d.state);
        const b = d.state === "missing" ? "" : lease(btn(on || d.state === "stopping" ? "Stop" : "Start", on ? "" : "primary",
          () => cmd(on ? "drivers_stop" : "drivers_start", { name: d.name })));
        if (b && d.state === "stopping") b.disabled = true;
        return h("div", { class: "xd-row", title: d.msg || "" },
          h("span", { class: "dot " + (STATE[d.state] || STATE.stopped)[2] }),
          h("span", { class: "grow" }, h("div", {}, d.label || d.name), h("div", { class: "muted small" }, txt.toLowerCase() + (d.msg ? " · " + d.msg : ""))),
          b);
      }) : [h("div", { class: "muted small" }, "No camera drivers yet. Add one in SETUP › Cameras.")]));
    },
  };

  // ---------------------------------------------------------------- Policy window
  const polChart = policyChart();
  const polSel = h("select", { class: "grow", "aria-label": "Policy" },
    ...INFO.policies.map((p) => h("option", { value: p.name }, p.name)));
  const polDur = h("input", { type: "number", min: "0", step: "0.5", placeholder: "manifest default", style: "width:110px" });
  const polReq = h("div", { class: "col", style: "gap:4px" });
  const polReqSum = h("summary", { class: "small" }, "Checks");
  const polWarn = h("div", { class: "banner-warn hidden" });
  const polState = h("span", { class: "muted small" });
  const polStart = lease(btn("Start", "primary", () => cmd("policy_start", {
    name: polSel.value, duration: polDur.value === "" ? null : +polDur.value,
  }), "Prepare and start the selected policy"));
  const polStop = btn("■ Stop", "danger", () => cmd("policy_stop"), "Stop the policy; no control lease is needed");
  let polReqAt = 0, polReqKey = "";
  const policy = {
    label: "Policy", min: 320, fill: true,
    // Start/Stop first: the window can be short, and Stop must never scroll out of sight
    el: box("xpolicy", h("div", { class: "row" }, polSel, polDur, h("span", { class: "muted small" }, "s")),
      h("div", { class: "row" }, polStart, polStop, polState, h("span", { class: "grow" }),
        btn("Library", "quiet", () => navTo("SETUP", "Policies"))),
      polWarn, polChart.el, h("details", {}, polReqSum, polReq)),
    async update(s) {
      polChart.push(s);
      const p = s.policy || {}, busy = !!p.busy, current = p.current || "";
      polState.textContent = busy ? (current ? `Running · ${current}` : "Preparing…") : "Idle";
      polStart.dataset.policyBusy = busy ? "1" : "";
      polStart.disabled = busy;
      polWarn.classList.toggle("hidden", !(p.range_warnings?.length || p.step_warnings?.length));
      polWarn.textContent = [...new Set([...(p.range_warnings || []), ...(p.step_warnings || []).map((x) => `runner: ${x}`)])].join("\n");
      if (!polSel.value && INFO.policies.length) polSel.value = INFO.policies[0].name;
      if (!polSel.value || Date.now() - polReqAt < 3000) return;
      polReqAt = Date.now();
      const r = await cmd("policy_requirements", { name: polSel.value });
      if (!r.ok) return;
      const key = JSON.stringify(r.requirements);
      if (key === polReqKey) return;
      polReqKey = key;
      const nBad = r.requirements.filter((q) => q.state === "fail").length, nWarn = r.requirements.filter((q) => q.state === "warn").length;
      polReqSum.textContent = nBad ? `Checks: ${nBad} not ready` : nWarn ? `Checks: ${nWarn} warning${nWarn > 1 ? "s" : ""}` : "Checks: all ready";
      polReqSum.className = "small " + (nBad ? "bad" : nWarn ? "warn" : "ok");
      polReq.replaceChildren(...r.requirements.map((q) => h("div", { class: "row", title: q.detail || "" },
        h("span", { class: "dot " + (q.state === "ok" ? "ok" : q.state === "fail" ? "bad" : "warn") }),
        h("span", {}, q.label), h("span", { class: "muted small grow" }, q.detail || ""))));
    },
  };

  return { plan, jchart, prog, log, drv, policy };
}
