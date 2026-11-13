// Monitoring panels for OPERATE: a joint-limit monitor and a live F/T strip chart. Plain DOM + <canvas>, no libraries.

function el(tag, props = {}, ...kids) {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(props || {})) {
    if (k === "class") e.className = v;
    else if (k === "style") e.style.cssText = v;
    else if (k.startsWith("on")) e.addEventListener(k.slice(2), v);
    else if (v === true) e.setAttribute(k, "");
    else if (v !== false && v != null) e.setAttribute(k, v);
  }
  for (const c of kids.flat()) if (c != null && c !== false) e.append(c.nodeType ? c : document.createTextNode(c));
  return e;
}

// Distance to the nearest limit that turns the marker amber / red (degrees).
export const WARN_DEG = 10, ALARM_DEG = 3;
const UNKNOWN = [-360, 360];

/** "ok" | "warn" | "alarm" for angle q (deg) inside limits [lo, hi]. Past a limit is alarm. */
export function limitState(q, lo, hi) {
  const d = Math.min(q - lo, hi - q);
  return d <= ALARM_DEG ? "alarm" : d <= WARN_DEG ? "warn" : "ok";
}

/**
 * Joint monitor card. names: joint names; limits: [[lo, hi] | null] in degrees (null = unknown, shown as +-360).
 * Returns {el, update(state), setLimits(limits)}.
 */
export function jointMonitor(names, limits) {
  let lim = names.map((_, i) => (limits && limits[i]) || null);
  const rows = names.map((n, i) => {
    const short = n.replace(/^.*?(shoulder|elbow|wrist)/, "$1").replace(/_joint$/, "").replace(/_/g, " ");
    const v = el("span", { class: "jm-v mono" }, "—");
    const vel = el("span", { class: "jm-vel mono muted" }, "");
    const mark = el("div", { class: "jm-mark" });
    const track = el("div", { class: "jm-track" }, mark);
    const lo = el("span", { class: "jm-lim mono muted" }), hi = el("span", { class: "jm-lim mono muted" });
    const row = el("div", { class: "jm-row" },
      el("div", { class: "jm-head" }, el("span", { class: "jm-n", title: n }, short), v, vel),
      el("div", { class: "jm-bar" }, lo, track, hi));
    return { row, v, vel, mark, track, lo, hi };
  });
  const note = el("div", { class: "muted" });
  const paintLimits = () => {
    rows.forEach((r, i) => {
      const [lo, hi] = lim[i] || UNKNOWN;
      r.lo.textContent = Math.round(lo);
      r.hi.textContent = Math.round(hi);
      const span = hi - lo, w = Math.min(50, (WARN_DEG / span) * 100), a = Math.min(50, (ALARM_DEG / span) * 100);
      // Shaded warn and alarm zones at both ends of the track.
      r.track.style.background = `linear-gradient(to right, var(--danger-bg) 0 ${a}%, var(--warn-bg) ${a}% ${w}%, ` +
        `var(--sunken) ${w}% ${100 - w}%, var(--warn-bg) ${100 - w}% ${100 - a}%, var(--danger-bg) ${100 - a}% 100%)`;
    });
    const unknown = names.filter((_, i) => !lim[i]).length;
    note.textContent = unknown
      ? `Limits unknown for ${unknown} joint${unknown > 1 ? "s" : ""} (no URDF limits): showing +-360 deg.`
      : "Limits from the robot URDF. Amber: within 10 deg of a limit. Red: within 3 deg.";
  };
  paintLimits();
  const root = el("div", { class: "jm" }, rows.map((r) => r.row), note);
  return {
    el: root,
    unknownCount: () => lim.filter((x) => !x).length,
    setLimits(l) { lim = names.map((_, i) => (l && l[i]) || null); paintLimits(); },
    update(s) {
      const q = s.joints_deg, qd = s.joint_vel_dps;
      let worst = "ok";
      rows.forEach((r, i) => {
        if (!q || q[i] == null) { r.v.textContent = "—"; return; }
        const [lo, hi] = lim[i] || UNKNOWN;
        const st = limitState(q[i], lo, hi);
        if (st === "alarm" || (st === "warn" && worst === "ok")) worst = st;
        r.v.textContent = q[i].toFixed(1) + "°";
        r.row.dataset.state = st;
        r.mark.style.left = Math.max(0, Math.min(100, ((q[i] - lo) / (hi - lo)) * 100)) + "%";
        r.vel.textContent = qd && qd[i] != null ? `${qd[i].toFixed(1)} °/s` : "";
      });
      root.dataset.worst = worst;
    },
  };
}


// Series colours follow the conventional x/y/z order and the app's tokens: red, green, blue; magnitudes neutral.
const COLORS = { x: "#c23b32", y: "#1e8a57", z: "#2b5fb8", n: "#515a65" };
export const FT_SERIES = [
  { key: "Fx", i: 0, color: COLORS.x, unit: "N" }, { key: "Fy", i: 1, color: COLORS.y, unit: "N" },
  { key: "Fz", i: 2, color: COLORS.z, unit: "N" }, { key: "|F|", mag: [0, 1, 2], color: COLORS.n, unit: "N", dash: true },
  { key: "Tx", i: 3, color: COLORS.x, unit: "Nm", dash: true }, { key: "Ty", i: 4, color: COLORS.y, unit: "Nm", dash: true },
  { key: "Tz", i: 5, color: COLORS.z, unit: "Nm", dash: true }, { key: "|T|", mag: [3, 4, 5], color: COLORS.n, unit: "Nm", dash: [2, 3] },
];
const WINDOWS = [5, 10, 20, 60];
const MAX_KEEP_S = 60;

/** Nice tick step (1, 2, 5 x 10^k) for a range split into about n intervals. */
export function niceStep(range, n = 4) {
  const raw = range / n, p = Math.pow(10, Math.floor(Math.log10(raw))), f = raw / p;
  return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10) * p;
}

/** Value range of the series over the visible window, padded and widened to at least minSpan. */
export function axisRange(values, minSpan) {
  let lo = Infinity, hi = -Infinity;
  for (const v of values) { if (v < lo) lo = v; if (v > hi) hi = v; }
  if (lo === Infinity) { lo = -minSpan / 2; hi = minSpan / 2; }
  lo = Math.min(lo, 0); hi = Math.max(hi, 0);                 // keep 0 in view: the sign matters
  if (hi - lo < minSpan) { const m = (hi + lo) / 2; lo = m - minSpan / 2; hi = m + minSpan / 2; }
  const pad = (hi - lo) * 0.08;
  const step = niceStep(hi - lo + 2 * pad);
  return { lo: Math.floor((lo - pad) / step) * step, hi: Math.ceil((hi + pad) / step) * step, step };
}

/**
 * Live F/T strip chart. Pick any of Fx Fy Fz |F| Tx Ty Tz |T| (forces on the left axis in N, torques on the
 * right axis in Nm when both are shown), a 5 / 10 / 20 / 60 s window, Pause, Clear. push(state) takes the
 * 100 Hz samples in state.ft_samples (or state.wrench). Drawing runs on animation frames (~30 fps) only while
 * the card is open and on screen; a control change redraws at once. Returns {el, push(state), draw(), pick}.
 */
export function ftChart(storage = globalThis.localStorage) {
  let t = [], data = [[], [], [], [], [], []];     // Fx Fy Fz Tx Ty Tz
  let saved = {};
  try { saved = JSON.parse(storage.getItem("ftchart") || "{}"); } catch (e) { saved = {}; }
  const pick = new Set(Array.isArray(saved.pick) ? saved.pick : ["Fx", "Fy", "Fz"]);
  let win = WINDOWS.includes(saved.win) ? saved.win : 20;
  let paused = false, cmd = null, dirty = true, lastDraw = 0, raf = 0;
  const save = () => { try { storage.setItem("ftchart", JSON.stringify({ pick: [...pick], win })); } catch (e) { /* private mode */ } };
  const canvas = el("canvas", { class: "ftc-canvas", role: "img", "aria-label": "Live force and torque chart" });
  const readout = el("div", { class: "ftc-read" });          // empty: pushes the buttons right
  const chips = FT_SERIES.map((s) => {
    const b = el("button", { class: "ftc-chip", type: "button", "aria-pressed": String(pick.has(s.key)),
      title: `Show ${s.key} (${s.unit})`, onclick: () => {
        pick.has(s.key) ? pick.delete(s.key) : pick.add(s.key);
        b.setAttribute("aria-pressed", String(pick.has(s.key)));
        save(); kick(true);
      } }, el("span", { class: "ftc-sw", style: `background:${s.color}` }), s.key);
    return b;
  });
  const winSel = el("select", { "aria-label": "Time window", onchange: () => { win = +winSel.value; save(); kick(true); } },
    ...WINDOWS.map((w) => el("option", { value: String(w), ...(w === win ? { selected: true } : {}) }, `${w} s`)));
  const pauseBtn = el("button", { class: "btn quiet", type: "button", onclick: () => {
    paused = !paused; pauseBtn.textContent = paused ? "Resume" : "Pause"; kick(true);
  } }, "Pause");
  const clearBtn = el("button", { class: "btn quiet", type: "button", onclick: () => { t = []; data = data.map(() => []); kick(true); } }, "Clear");
  const body = el("div", { class: "ftc-body" },
    el("div", { class: "row ftc-chips" }, ...chips.slice(0, 4), el("span", { class: "ftc-gap" }), ...chips.slice(4)),
    canvas,
    el("div", { class: "row" }, readout, winSel, pauseBtn, clearBtn));
  const root = el("details", { class: "card ft-chart", ontoggle: () => kick(true) },
    el("summary", { class: "section" }, "F/T chart"), body);
  if (saved.open !== false) root.open = true;
  let shown = root.open;                     // opening it here fires a toggle too: store only real changes
  root.addEventListener("toggle", () => { if (root.open === shown) return; shown = root.open; try { storage.setItem("ftchart", JSON.stringify({ pick: [...pick], win, open: root.open })); } catch (e) { /* */ } });

  const visible = () => root.open && root.offsetParent !== null && !(globalThis.document && document.hidden);
  function kick(now) {
    dirty = true;
    if (now) { lastDraw = 0; }
    if (!raf && visible() && globalThis.requestAnimationFrame) raf = requestAnimationFrame(frame);
  }
  function frame(ts) {
    raf = 0;
    if (!dirty || !visible()) return;
    if (ts - lastDraw < 33) { raf = requestAnimationFrame(frame); return; }
    lastDraw = ts;
    draw();
  }

  function seriesValues(s, k) {
    return s.mag ? Math.hypot(data[s.mag[0]][k], data[s.mag[1]][k], data[s.mag[2]][k]) : data[s.i][k];
  }

  function draw() {
    dirty = false;
    const dpr = globalThis.devicePixelRatio || 1;
    const W = canvas.clientWidth, H = canvas.clientHeight;
    if (!W || !H) return;
    if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
      canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
    }
    const g = canvas.getContext("2d");
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    g.clearRect(0, 0, W, H);
    const shown = FT_SERIES.filter((s) => pick.has(s.key));
    const forces = shown.filter((s) => s.unit === "N"), torques = shown.filter((s) => s.unit === "Nm");
    const now = t.length ? t[t.length - 1] : 0;
    let k0 = 0;
    while (k0 < t.length && t[k0] < now - win) k0++;
    const L = 40, R = torques.length && forces.length ? 40 : 8, T = 6, B = 18, pw = W - L - R, ph = H - T - B;
    const step = Math.max(1, Math.floor((t.length - k0) / (pw * 2)));       // never more points than pixels
    const vals = (list) => { const out = []; for (const s of list) for (let k = k0; k < t.length; k += step) out.push(seriesValues(s, k)); return out; };
    const cmdFor = (s) => (cmd && s.i != null && cmd[s.i] ? cmd[s.i] : null);
    const axes = [];
    if (forces.length) axes.push({ unit: "N", list: forces, side: "L", ...axisRange([...vals(forces), ...forces.map(cmdFor).filter((v) => v != null)], 2) });
    if (torques.length) axes.push({ unit: "Nm", list: torques, side: forces.length ? "R" : "L", ...axisRange([...vals(torques), ...torques.map(cmdFor).filter((v) => v != null)], 0.2) });
    const X = (sec) => L + pw * (1 - sec / win);
    const Yof = (a) => (v) => T + ph * (1 - (v - a.lo) / (a.hi - a.lo));
    g.font = "10px system-ui, sans-serif"; g.lineWidth = 1;
    // grid from the first axis; labels on each axis' side
    axes.forEach((a, n) => {
      const Y = Yof(a);
      g.textBaseline = "middle"; g.textAlign = a.side === "L" ? "right" : "left";
      for (let v = a.lo; v <= a.hi + a.step / 2; v += a.step) {
        const y = Math.round(Y(v)) + 0.5, zero = Math.abs(v) < a.step / 1e3;
        if (n === 0 || zero) { g.strokeStyle = zero ? "#808a96" : "#e2e5e9"; g.beginPath(); g.moveTo(L, y); g.lineTo(L + pw, y); g.stroke(); }
        g.fillStyle = "#515a65";
        g.fillText((zero ? 0 : v).toFixed(a.step < 0.1 ? 2 : a.step < 1 ? 1 : 0), a.side === "L" ? L - 4 : L + pw + 4, y);
      }
      g.save(); g.translate(a.side === "L" ? 9 : W - 6, T + ph / 2); g.rotate(-Math.PI / 2); g.textAlign = "center";
      g.fillText(a.unit, 0, 0); g.restore();
    });
    g.textBaseline = "alphabetic"; g.textAlign = "center";
    const tick = win <= 5 ? 1 : win <= 10 ? 2 : win <= 20 ? 5 : 10;
    for (let sec = win; sec >= 0; sec -= tick) {
      const x = Math.round(X(sec)) + 0.5;
      g.strokeStyle = "#e2e5e9"; g.beginPath(); g.moveTo(x, T); g.lineTo(x, T + ph); g.stroke();
      g.fillStyle = "#515a65"; g.fillText(sec === 0 ? "now" : `-${sec} s`, Math.min(Math.max(x, L + 10), L + pw - 10), H - 4);
    }
    g.save(); g.beginPath(); g.rect(L, T, pw, ph); g.clip();
    for (const a of axes) {
      const Y = Yof(a);
      for (const s of a.list) {
        const c = cmdFor(s);
        if (c != null) {
          g.strokeStyle = s.color; g.globalAlpha = 0.6; g.setLineDash([2, 3]); g.lineWidth = 2;
          g.beginPath(); g.moveTo(L, Y(c)); g.lineTo(L + pw, Y(c)); g.stroke(); g.globalAlpha = 1;
        }
        g.strokeStyle = s.color; g.lineWidth = s.mag ? 1.2 : 1.5;
        g.setLineDash(s.dash === true ? [5, 3] : s.dash || []);
        g.beginPath();
        let first = true;
        for (let k = k0; k < t.length; k += step) {
          const x = X(now - t[k]), y = Y(seriesValues(s, k));
          first ? g.moveTo(x, y) : g.lineTo(x, y); first = false;
        }
        g.stroke();
      }
    }
    g.restore(); g.setLineDash([]);
    const empty = !shown.length ? "pick an axis above" : !t.length ? "no F/T data" : "";
    if (empty) { g.fillStyle = "#515a65"; g.textAlign = "center"; g.fillText(empty, L + pw / 2, T + ph / 2); }
  }

  function add(ts, w) {
    if (t.length && ts <= t[t.length - 1]) return;       // a reconnect may repeat samples
    t.push(ts); for (let i = 0; i < 6; i++) data[i].push(w[i]);
  }

  return {
    el: root,
    draw,
    pick,
    get window() { return win; },
    get length() { return t.length; },
    push(s) {
      const c = s.force && s.force.active && s.force.current ? s.force.current : null;
      if (String(c) !== String(cmd)) { cmd = c; dirty = true; }
      if (paused) { if (dirty) kick(); return; }
      const before = t.length;
      if (s.ft_samples && s.ft_samples.length) for (const r of s.ft_samples) add(r[0], r.slice(1));
      else if (s.wrench) add(s.t || Date.now() / 1000, s.wrench);
      if (t.length !== before) {
        const cut = t[t.length - 1] - MAX_KEEP_S;
        let n = 0;
        while (n < t.length && t[n] < cut) n++;
        if (n > 500 || (n && t.length > 7000)) { t = t.slice(n); data = data.map((d) => d.slice(n)); }
        dirty = true;
      }
      if (dirty) kick();
    },
  };
}

/** Keep runner samples for a rolling policy window. The buffer is exported so the small chart can be tested
 * without a browser canvas. Samples are the shared /policy_runner/step JSON objects. */
export function policyStepBuffer(windowSeconds = 30) {
  let rows = [];
  return {
    get length() { return rows.length; },
    get rows() { return rows.slice(); },
    push(samples) {
      for (const sample of Array.isArray(samples) ? samples : []) {
        if (!sample || !Number.isFinite(Number(sample.t))) continue;
        rows.push(sample);
      }
      if (rows.length) {
        const cut = Number(rows[rows.length - 1].t) - windowSeconds;
        let first = 0;
        while (first < rows.length && Number(rows[first].t) < cut) first++;
        if (first) rows = rows.slice(first);
      }
      return rows.length;
    },
    clear() { rows = []; },
  };
}

export function policyActionMeta(policy = {}, sample = {}) {
  const values = Array.isArray(sample?.command) ? sample.command : Array.isArray(sample?.action) ? sample.action : [];
  const raw = Array.isArray(policy?.action_names) ? policy.action_names : [];
  const names = values.map((_, i) => {
    const item = raw[i];
    return typeof item === "string" && item ? item : item && typeof item.name === "string" && item.name ? item.name : `a${i}`;
  });
  const rawUnits = policy?.action_units;
  const units = names.map((name, i) => {
    const item = raw[i];
    if (item && typeof item === "object" && item.unit) return String(item.unit);
    if (Array.isArray(rawUnits)) return rawUnits[i] ? String(rawUnits[i]) : "";
    if (rawUnits && typeof rawUnits === "object") return rawUnits[name] ? String(rawUnits[name]) : "";
    return "";
  });
  return { names, units };
}

export function policyChartStatus(state = {}, now = Date.now() / 1000) {
  const rows = Array.isArray(state?.policy_steps) ? state.policy_steps : [];
  const latest = rows.at(-1);
  const supplied = state?.policy?.step_age_s;
  const suppliedAge = typeof supplied === "number" ? supplied : NaN;
  const age = Number.isFinite(suppliedAge) ? suppliedAge : latest && Number.isFinite(Number(latest.t))
    ? Math.max(0, now - Number(latest.t)) : null;
  return { age, stale: !state?.policy?.busy || age == null || age > 2 };
}

/** Live policy chart: actions and F/T on their own scales; observation sigma is shown as bars. */
export function policyChart() {
  const buf = policyStepBuffer(30);
  const canvas = el("canvas", { class: "ftc-canvas", style: "height:260px", role: "img", "aria-label": "Live policy actions and force chart" });
  const note = el("div", { class: "ftc-read muted small" });
  const sigmaBox = el("div", { class: "policy-sigma" });
  const root = el("div", { class: "card policy-chart" },
    el("div", { class: "section" }, "Policy chart · last 30 s"), canvas, sigmaBox, note);
  let latestState = {};
  let dirty = true;
  // Two plots, each on its own scale: policy actions (~0.01) would be a flat line next to F/T (~10 N)
  function draw() {
    dirty = false;
    const W = canvas.clientWidth || 520, H = canvas.clientHeight || 260, dpr = globalThis.devicePixelRatio || 1;
    if (canvas.width !== Math.round(W * dpr) || canvas.height !== Math.round(H * dpr)) {
      canvas.width = Math.round(W * dpr); canvas.height = Math.round(H * dpr);
    }
    const g = canvas.getContext("2d");
    if (!g) return;
    g.setTransform(dpr, 0, 0, dpr, 0, 0); g.clearRect(0, 0, W, H);
    const rows = buf.rows, L = 40, R = 8, B = 16, gap = 14, now = rows.length ? Number(rows.at(-1).t) : 0;
    const meta = policyActionMeta(latestState.policy, rows.at(-1));
    const PLOTS = [
      { label: "action", names: meta.names.length ? meta.names : ["a0"],
        get: (r, i) => (Array.isArray(r.command) ? r.command : r.action)?.[i],
        colors: ["#d1495b", "#2e8b57", "#2f5fb3", "#c77d12", "#7b4fb3", "#14879c"] },
      { label: "F/T (N)", names: ["Fx", "Fy", "Fz"], get: (r, i) => r.ft?.[i],
        colors: ["#c77d12", "#7b4fb3", "#14879c"] },
    ];
    const ph = (H - B - gap * PLOTS.length) / PLOTS.length, pw = W - L - R;
    const X = (t) => L + pw * (1 - (now - t) / 30);
    g.font = "10px system-ui, sans-serif";
    PLOTS.forEach((P, n) => {
      const T = gap + n * (ph + gap);
      const vals = rows.flatMap((r) => P.names.map((_, i) => P.get(r, i)))
        .map((v) => typeof v === "number" && Number.isFinite(v) ? v : null).filter((v) => v != null);
      const a = axisRange(vals, n ? 1 : 0.01), Y = (v) => T + ph * (1 - (v - a.lo) / (a.hi - a.lo));
      g.textAlign = "right"; g.textBaseline = "middle"; g.strokeStyle = "#e2e5e9"; g.lineWidth = 1;
      const dec = a.step >= 1 ? 0 : Math.min(4, Math.ceil(-Math.log10(a.step)));
      for (let v = a.lo; v <= a.hi + a.step / 2; v += a.step) {
        const y = Math.round(Y(v)) + .5; g.beginPath(); g.moveTo(L, y); g.lineTo(L + pw, y); g.stroke();
        g.fillStyle = "#515a65"; g.fillText(v.toFixed(dec), L - 4, y);
      }
      g.textAlign = "left"; g.textBaseline = "alphabetic"; let lx = L;
      g.fillStyle = "#515a65"; g.fillText(P.label, lx, T - 3); lx += g.measureText(P.label).width + 10;
      P.names.forEach((name, i) => { g.fillStyle = P.colors[i % P.colors.length]; g.fillText("— " + name, lx, T - 3); lx += g.measureText("— " + name).width + 8; });
      g.save(); g.beginPath(); g.rect(L, T, pw, ph); g.clip();
      P.names.forEach((_, i) => {
        g.strokeStyle = P.colors[i % P.colors.length]; g.lineWidth = 1.4; g.beginPath(); let first = true;
        rows.forEach((r) => { const v = P.get(r, i); if (typeof v !== "number" || !Number.isFinite(v)) return; const x = X(Number(r.t)), y = Y(v); first ? g.moveTo(x, y) : g.lineTo(x, y); first = false; });
        g.stroke();
      });
      g.restore();
    });
    g.fillStyle = "#515a65"; g.textAlign = "center"; g.textBaseline = "alphabetic";
    for (let s = 30; s >= 0; s -= 10) g.fillText(s ? `-${s} s` : "now", Math.min(Math.max(L + pw * (1 - s / 30), L + 12), L + pw - 12), H - 3);
    const latest = rows.at(-1), sigma = latest?.obs_sigma || {};
    const bad = latest?.out_of_range || [];
    sigmaBox.replaceChildren(...Object.entries(sigma).filter(([, value]) => typeof value === "number" && Number.isFinite(value)).map(([name, value]) => {
      const n = value, width = Math.max(0, Math.min(100, n / 4 * 100));
      return el("div", { class: "policy-sigma-row", title: `${name}: ${n.toFixed(1)} sigma` },
        el("span", { class: "muted small" }, name), el("div", { class: "bar grow" },
          el("div", { style: `width:${width}%;${n > 4 ? "background:var(--danger)" : ""}` })),
        el("span", { class: "mono small" }, n.toFixed(1)));
    }));
    const status = policyChartStatus({ ...latestState, policy_steps: rows }, Date.now() / 1000);
    root.dataset.stale = String(status.stale);
    canvas.style.filter = status.stale ? "grayscale(1)" : "";
    root.style.opacity = status.stale ? "0.55" : "";
    const action = latest ? (Array.isArray(latest.command) ? latest.command : latest.action) : null;
    const actionText = meta.names.map((name, i) => {
      const v = action?.[i], shown = typeof v === "number" && Number.isFinite(v) ? v.toFixed(3) : "—";
      return `${name} ${shown}${meta.units[i] ? ` ${meta.units[i]}` : ""}`;
    }).join(" · ");
    const ageText = status.age == null ? "last step —" : `last step ${status.age.toFixed(1)} s ago`;
    note.textContent = latest ? `${actionText || "action —"} · ${ageText} · ` +
      `obs sigma ${Object.entries(sigma).filter(([, v]) => typeof v === "number" && Number.isFinite(v)).map(([k, v]) => `${k} ${v.toFixed(1)}`).join(", ") || "—"}` +
      (bad.length ? ` · out of range: ${bad.join(", ")}` : "") : "No policy step data";
  }
  return { el: root, get length() { return buf.length; }, get rows() { return buf.rows; }, draw,
    push(state) { latestState = state || {}; buf.push(latestState.policy_steps || []); draw(); }, clear() { buf.clear(); draw(); } };
}
