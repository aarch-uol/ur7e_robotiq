// OPERATE: the workspace and its layouts.
// Three columns. Left and right: cards (Manual, F/T sensor, Robot, ...) and views such as the F/T chart, stacked.
// Middle: the 3D view, pinned top left (never moved, minimised or closed), with a stack of views on its right and
// one below it (each may be empty). A view is a camera, the F/T chart or empty.
// A view is also one of the windows in panels.js: Motion planner, Joint chart, Program run, Log (one of each).
// Windows always fill their space; resize by the lines between them. A camera "fits to the picture" (default):
// its row / stack / window size follows the picture's shape, and that line is off; turn it off in its bar.
// Layouts: "Main" is built in and fixed (nothing moves, resizes or minimises). Your own layouts are made in
// SETUP › Layouts; in them you can resize (drag the lines) and minimise (a window becomes its title bar, in place;
// nothing else moves). "Edit" (from SETUP › Layouts) moves, adds and closes windows: drag ⠿ above / below a window,
// or onto the right or bottom edge of the 3D view. The switcher (numbers) is in the top bar when there are two or
// more layouts. Kept per browser (localStorage "op-layouts1").
import { icon } from "/static/icons.js";
import { ask } from "/static/dialog.js";

const KEY = "op-layouts1";
const MAIN_H = 560;
const MINW = { "3d": 420, ft: 260, cam: 240, empty: 220, plan: 300, jchart: 300, prog: 260, log: 300, drv: 240 }, MINH = 160, MAIN_MINH = 260;
const SIDE_MIN = 280, SIDE_MAX = 600, FOLD_W = 36, GAP = 8, MAX_VIEWS = 6, MAX_LAYOUTS = 9;
const TYPES = [["cam", "Camera"], ["ft", "F/T chart"], ["plan", "Motion planner"], ["jchart", "Joint chart"],
  ["prog", "Program run"], ["log", "Log"], ["drv", "Camera drivers"], ["empty", "Empty"]];
const ONE = ["ft", "plan", "jchart", "prog", "log", "drv"];                   // one window of each in a layout
const QUALITY = [["low", "Low", "640 px, 10 fps"], ["med", "Med", "960 px, 15 fps"], ["high", "High", "1280 px, 30 fps"]];   // server.PREVIEW
const typeName = (k) => (TYPES.find((x) => x[0] === k) || [k, k])[1];
const CARDS = ["manual", "fts", "robot", "arm", "grip", "joints"];
const clone = (o) => JSON.parse(JSON.stringify(o));
const base = (o) => Object.assign({ name: "", fixed: false, l: ["manual", "fts"], r: ["robot", "arm", "grip", "joints"],
  right: [], rw: [], bottom: [], bw: [], f: 0.62, bh: 300, tiles: {}, min: [], fold: { l: false, r: false },
  w: { l: 360, r: 340 }, h: {} }, o);
// 1 Main (the page as it always was): what you command left with the F/T chart, the 3D view, the robot right.
// The chart "att"ached to the F/T sensor card: it sits right under it, moves with it, never minimises.
const MAIN = base({ name: "Main", fixed: true, l: ["manual", "fts", "chart"], tiles: { chart: { type: "ft", att: true } }, h: { chart: 300 } });
// 2 and 3: ready-made layouts of your own (edit, rename or delete them in SETUP › Layouts)
const PRESETS = [
  base({ name: "Force", l: ["manual", "fts", "chart"], r: ["robot", "arm", "joints", "grip"], right: ["c1"], rw: [1], f: 0.66,
    tiles: { chart: { type: "ft", att: true }, c1: { type: "cam", pick: 0 } }, h: { chart: 320 }, min: ["grip"] }),
  base({ name: "Cameras", l: ["manual", "grip", "drv"], r: ["robot", "arm", "fts", "joints"], bottom: ["c1", "c2"], bw: [1, 1], bh: 340,
    tiles: { c1: { type: "cam", pick: 0 }, c2: { type: "cam", pick: 1 }, drv: { type: "drv" } } }),
];

// every card exactly once in a side column; tiles referenced exist; numbers sane (a stored layout may be old)
function sane(x) {
  const L = base(x && typeof x === "object" ? x : {});
  L.fixed = false;
  L.name = String(L.name || "Layout").slice(0, 24);
  for (const k of ["l", "r", "right", "bottom", "min"]) if (!Array.isArray(L[k])) L[k] = [];
  L.tiles = L.tiles && typeof L.tiles === "object" ? L.tiles : {};
  for (const [id, t] of Object.entries(L.tiles)) if (!t || !TYPES.some(([k]) => k === t.type)) delete L.tiles[id];
  const seen = new Set();
  for (const k of ["l", "r"]) L[k] = L[k].filter((id) => (CARDS.includes(id) || L.tiles[id]) && !seen.has(id) && seen.add(id));
  for (const k of ["right", "bottom"]) L[k] = L[k].filter((id) => L.tiles[id] && !seen.has(id) && seen.add(id));
  for (const id of CARDS) if (!seen.has(id)) { L.r.push(id); seen.add(id); }
  for (const id of Object.keys(L.tiles)) if (!seen.has(id)) delete L.tiles[id];
  const had = new Set();                                                // one of each (F/T chart, planner, ...)
  for (const id of seen) { const t = L.tiles[id]; if (t && ONE.includes(t.type)) { if (had.has(t.type)) t.type = "empty"; had.add(t.type); } }
  L.rw = L.right.map((_, i) => (+L.rw?.[i] > 0 ? +L.rw[i] : 1));
  L.bw = L.bottom.map((_, i) => (+L.bw?.[i] > 0 ? +L.bw[i] : 1));
  L.f = Math.min(0.9, Math.max(0.2, +L.f || 0.62));
  L.bh = Math.max(MINH, +L.bh || 300);
  L.fold = { l: !!L.fold?.l, r: !!L.fold?.r };
  L.w = { l: clampSide(L.w?.l, 360), r: clampSide(L.w?.r, 340) };
  L.h = L.h && typeof L.h === "object" ? L.h : {};
  delete L.rh; delete L.bwh;                                            // old: own window sizes (gone)
  L.min = L.min.filter((id) => seen.has(id) && !L.tiles[id]?.att);
  attach(L);
  return L;
}
// the attached F/T chart goes right under the F/T sensor card (wherever that is)
const attId = (L) => Object.keys(L.tiles).find((k) => L.tiles[k].att && L.tiles[k].type === "ft");
function attach(L) {
  const id = attId(L);
  if (!id) return;
  for (const k of ["l", "r", "right", "bottom"]) {
    const i = L[k].indexOf(id);
    if (i >= 0) { L[k].splice(i, 1); if (k === "right") L.rw.splice(i, 1); if (k === "bottom") L.bw.splice(i, 1); }
  }
  if (L.tiles[id].under) { L.bottom.push(id); L.bw.push(L.bw.length ? L.bw.reduce((a, b) => a + b, 0) / L.bw.length : 1); return; }
  const z = L.l.includes("fts") ? "l" : "r";
  L[z].splice(L[z].indexOf("fts") + 1, 0, id);
}
function clampSide(v, d) { return Math.max(SIDE_MIN, Math.min(SIDE_MAX, Math.round(+v || d))); }

export function workspace({ h, cmd, toast, cards, view3d, ftEl, ftBtn, header, navSlot, onEdit, extra = {} }) {
  // ---- state
  let stored = null;
  try { stored = JSON.parse(localStorage.getItem(KEY) || "null"); } catch (e) { stored = null; }
  const own = Array.isArray(stored?.list) ? stored.list.slice(0, MAX_LAYOUTS - 1).map(sane) : PRESETS.map(clone);
  // Main is fixed, but what is minimised and where its chart sits (beside F/T sensor or under the 3D view) is kept
  const main = clone(MAIN);
  if (Array.isArray(stored?.main?.min)) main.min = stored.main.min.filter((id) => CARDS.includes(id));
  if (stored?.main?.under) { main.tiles.chart.under = true; attach(main); }
  const W = { list: [main, ...own], cur: Math.min(Math.max(0, +stored?.cur || 0), own.length), edit: false, shown: false,
    cams: [], auto: { l: false, r: false }, drag: null, busy: false, ar: {}, mid: { w: 0, h: 0 }, ch: { w: 16, h: 60 } };
  const L = () => W.list[W.cur];
  const save = () => localStorage.setItem(KEY, JSON.stringify({ v: 1, cur: W.cur, list: W.list.slice(1),
    main: { min: W.list[0].min, under: !!W.list[0].tiles.chart?.under } }));
  const isTile = (id) => !!L().tiles[id];
  const isAtt = (id) => !!L().tiles[id]?.att;
  const isMin = (id) => !isAtt(id) && L().min.includes(id);
  const where = (id) => ["l", "r", "right", "bottom"].find((k) => L()[k].includes(id)) || null;
  const fd = (k) => (!L().fixed && L().fold[k]) || W.auto[k];
  const sideW = (k) => (fd(k) ? FOLD_W : L().w[k]);
  const allViews = () => [...L().l, ...L().r, ...L().right, ...L().bottom].filter(isTile);
  function take(id) {
    const k = where(id);
    if (!k) return;
    const i = L()[k].indexOf(id);
    L()[k].splice(i, 1);
    if (k === "right") L().rw.splice(i, 1);
    if (k === "bottom") L().bw.splice(i, 1);
  }
  function put(id, k, i) {
    const a = L()[k];
    i = i == null || i < 0 ? a.length : i;
    a.splice(i, 0, id);
    if (k === "right" || k === "bottom") {
      const w = L()[k === "right" ? "rw" : "bw"];
      w.splice(i, 0, w.length ? w.reduce((x, y) => x + y, 0) / w.length : 1);
    }
  }
  const stackMinW = () => Math.max(0, ...L().right.map((id) => MINW[L().tiles[id].type]));
  const midMinW = () => Math.max(MINW["3d"] + (L().right.length ? GAP + stackMinW() : 0),
    L().bottom.reduce((a, id) => a + MINW[L().tiles[id].type] + GAP, -GAP));

  // ---- cameras
  const camName = (topic) => (W.cams.find((c) => c.topic === topic) || { name: topic || "Camera" }).name;
  const topicOf = (t) => t.topic || (W.cams[t.pick || 0] || W.cams[0] || {}).topic || "";
  async function refreshCams() {
    const r = await cmd("camera_status").catch(() => null);
    if (!r || !r.ok) return;
    const known = new Set(r.cameras.map((c) => c.topic));
    // first what publishes, then the cameras of SETUP › Drivers & cameras, then the profile's list: "Camera 1" is a live one
    const rank = (c) => (c.publishers > 0 ? 0 : c.driver ? 1 : 2);
    const next = [...r.cameras, ...(r.topics || []).filter((t) => !known.has(t) && !/\/(compressed|compressedDepth|zstd|theora)$/.test(t))
      .map((t) => ({ name: t, topic: t, publishers: 1 }))].map((c, i) => [rank(c), i, c]).sort((a, b) => a[0] - b[0] || a[1] - b[1]).map((x) => x[2]);
    if (JSON.stringify(next) === JSON.stringify(W.cams)) return;
    W.cams = next;
    if (W.shown) render();
  }

  // ---- card frames: a grip and a minimise button in each card's own title row (the card itself is unchanged)
  const cardBits = {};
  for (const id of CARDS) {
    const c = cards[id], head = c.querySelector(".chead");
    const grip = h("span", { class: "lgrip ledit", title: "Drag to move", html: icon("grip") });
    const mn = h("button", { class: "lmin", type: "button", onclick: (e) => { e.stopPropagation(); toggleMin(id); } });
    head.prepend(grip); head.append(mn);
    head.addEventListener("click", (e) => { if (isMin(id) && !e.target.closest("button, select, input, a")) toggleMin(id); });
    grip.addEventListener("pointerdown", (e) => startDrag(e, id));
    c.classList.add("lpanel"); c.dataset.lid = id;
    cardBits[id] = { el: c, mn };
  }

  // ---- views (DOM kept per id while the layout is shown, so camera streams survive re-arranging)
  let tiles = new Map();
  const tool = (ic, tip, f) => h("button", { class: "vtool", type: "button", title: tip, html: icon(ic), onclick: f });
  function tileDom(id) {
    if (tiles.has(id)) return tiles.get(id);
    const T = { id, body: h("div", { class: "lbody" }), tools: h("span", { class: "vtools" }), img: null, rec: null };
    T.kind = h("select", { class: "lkind ledit", title: "What this window shows", onchange: () => setType(id, T.kind.value) },
      TYPES.map(([k, n]) => h("option", { value: k }, n)));
    T.topic = h("select", { class: "ltopic", title: "Camera topic", onchange: () => { L().tiles[id].topic = T.topic.value; save(); render(); } });
    T.topic.addEventListener("mousedown", () => refreshCams());
    T.q = h("select", { class: "lq", title: "Picture in this window: Low 640 px, 10 fps · Med 960 px, 15 fps · High 1280 px, 30 fps " +
      "(more data over the network). The camera itself is set in SETUP › Drivers & cameras.",
      onchange: () => { L().tiles[id].q = T.q.value; save(); render(); } },
      QUALITY.map(([k, n, d]) => h("option", { value: k, title: d }, n)));
    T.title = h("span", { class: "ltitle" });
    T.mn = h("button", { class: "lmin", type: "button", onclick: (e) => { e.stopPropagation(); toggleMin(id); } });
    T.x = h("button", { class: "lx ledit", type: "button", title: "Close this window", html: icon("close"),
      onclick: (e) => { e.stopPropagation(); closeTile(id); } });
    T.grip = h("span", { class: "lgrip ledit", title: "Drag to move", html: icon("grip") });
    T.grip.addEventListener("pointerdown", (e) => startDrag(e, id));
    T.dock = h("button", { class: "vtool", type: "button", onclick: (e) => {
      e.stopPropagation();
      const S = L(), t = S.tiles[id];
      t.under = !t.under; attach(S); save(); render();
    } });
    T.bar = h("div", { class: "lbar" }, T.grip, T.kind, T.title, T.topic, h("span", { class: "grow" }), T.tools, T.dock, T.mn, T.x);
    // fit to picture (cameras): the window takes the picture's shape; off: it fills its space (the picture letterboxes)
    T.fit = h("button", { class: "vtool", type: "button", html: icon("fit"), onclick: (e) => {
      e.stopPropagation();
      const t = L().tiles[id];
      if (t.fit === false) delete t.fit; else t.fit = false;
      save(); render();
    } });
    T.bar.addEventListener("click", (e) => { if (isMin(id) && !e.target.closest("button, select")) toggleMin(id); });
    T.rz = h("div", { class: "lrz", title: "Drag to resize" });
    T.rz.addEventListener("pointerdown", (e) => {
      if (L().fixed) return;
      const y0 = e.clientY, h0 = T.el.offsetHeight;
      track(e, (ev) => { L().h[id] = Math.round(Math.max(MINH, Math.min(700, h0 + ev.clientY - y0))); T.el.style.height = L().h[id] + "px"; });
    });
    T.el = h("div", { class: "lpanel ltile" }, T.bar, T.body, T.rz);
    T.el.dataset.lid = id;
    tiles.set(id, T);
    return T;
  }
  function stopCam(T) {
    if (T.rec) stopRec(T);
    if (T.img) { T.img.src = ""; T.img = null; }
  }
  function fill(T) {
    const t = L().tiles[T.id], live = !isMin(T.id), topic = t.type === "cam" ? topicOf(t) : "";
    const on = W.shown && live;
    T.kind.value = t.type;
    T.title.textContent = t.type === "cam" ? (topic ? camName(topic) : "Camera") : typeName(t.type);
    T.topic.hidden = t.type !== "cam" || !live || !W.cams.length;
    T.title.hidden = !T.topic.hidden;
    if (t.type === "cam" && W.cams.length) {
      const opts = W.cams.map((c) => h("option", { value: c.topic, title: c.topic }, c.name + (c.publishers > 0 ? "" : " (no data)")));
      if (topic && !W.cams.some((c) => c.topic === topic)) opts.unshift(h("option", { value: topic }, topic));
      T.topic.replaceChildren(...opts);
      T.topic.value = topic;
    }
    const q = QUALITY.some(([k]) => k === t.q) ? t.q : "low";
    const key = [t.type, topic, on, live, q].join("|");
    if (T.key === key) return;
    stopCam(T);
    T.key = key;
    T.tools.replaceChildren();
    if (!live) { T.body.replaceChildren(); return; }
    if (t.type === "ft") {
      T.body.replaceChildren(ftEl);
      chartRO.disconnect(); chartRO.observe(T.body);
      ftEl.open = true;
      ftEl.dispatchEvent(new Event("toggle"));
    } else if (t.type === "cam" && topic) {
      T.img = h("img", { class: "vcam", alt: "", draggable: "false",
        src: on ? "/api/camera.mjpeg?topic=" + encodeURIComponent(topic) + "&q=" + q + "&t=" + Date.now() : "" });
      T.msg = h("div", { class: "vmsg" }, "");
      T.img.onerror = () => { if (T.img && W.shown) { T.msg.textContent = "No picture"; T.msg.title = "Is the camera running? SETUP › Drivers & cameras"; } };
      T.img.onload = () => { T.msg.textContent = ""; aspect(T, topic); };
      T.recLbl = h("span", { class: "vrec" });
      T.body.replaceChildren(T.img, T.msg);
      T.recBtn = tool("record", "Record a clip of this camera (a WebM video downloads when you stop)",
        () => (T.rec ? stopRec(T) : startRec(T, topic)));
      T.q.value = q;
      T.tools.append(T.q, T.recLbl, T.fit, tool("camera", "Photo (downloads a JPEG)", () => photo(T, topic)), T.recBtn,
        tool("popout", "Open in its own browser window", () => window.open("/api/camera.mjpeg?topic=" + encodeURIComponent(topic) + "&q=" + q,
          "cam" + topic, "popup,width=680,height=520")));
    } else if (extra[t.type]) {
      T.body.replaceChildren(extra[t.type].el);
    } else if (t.type === "cam") {
      T.body.replaceChildren(h("div", { class: "vempty" }, "No camera yet. Start one in SETUP › Drivers & cameras."));
    } else {
      T.body.replaceChildren(h("div", { class: "vempty" }, "Empty. Pick what it shows in SETUP › Layouts › Edit."));
    }
  }
  // the picture's shape (width / height), per topic; a change redraws (an MJPEG stream may change size)
  function aspect(T, topic) {
    const im = T.img;
    if (!im || !im.naturalWidth || !im.naturalHeight) return;
    const a = im.naturalWidth / im.naturalHeight, o = W.ar[topic];
    if (!o || Math.abs(a - o) / o > 0.01) { W.ar[topic] = a; later(); }
  }
  setInterval(() => { for (const T of tiles.values()) if (T.img) aspect(T, topicOf(L().tiles[T.id] || {})); }, 2000);
  // a camera window that follows its picture: its shape, or 0 (fit off, no picture yet, minimised, not a camera)
  const fitAr = (id) => {
    const t = L().tiles[id];
    return t && t.type === "cam" && t.fit !== false && !isMin(id) ? W.ar[topicOf(t)] || 4 / 3 : 0;   // no picture yet: 4:3
  };
  const stamp = () => new Date().toISOString().slice(0, 19).replace("T", "_").replace(/:/g, "-");
  const fname = (topic, ext) => `${topic.replace(/^\//, "").replace(/\//g, "_")}_${stamp()}.${ext}`;
  function download(blob, name) {
    const a = h("a", { href: URL.createObjectURL(blob), download: name });
    document.body.append(a); a.click(); a.remove();
    setTimeout(() => URL.revokeObjectURL(a.href), 4000);
  }
  function frameCanvas(img) {
    const c = h("canvas", {});
    c.width = img.naturalWidth; c.height = img.naturalHeight;
    return c;
  }
  function photo(T, topic) {
    if (!T.img || !T.img.naturalWidth) { toast("No picture yet"); return; }
    const c = frameCanvas(T.img), name = fname(topic, "jpg");
    c.getContext("2d").drawImage(T.img, 0, 0);
    c.toBlob((b) => { download(b, name); toast("Photo downloaded: " + name, true); }, "image/jpeg", 0.92);
  }
  function startRec(T, topic) {
    if (!T.img || !T.img.naturalWidth) { toast("No picture yet"); return; }
    if (!window.MediaRecorder) { toast("This browser cannot record video"); return; }
    const c = frameCanvas(T.img), g = c.getContext("2d");
    const type = ["video/webm;codecs=vp9", "video/webm;codecs=vp8", "video/webm"].find((m) => MediaRecorder.isTypeSupported(m));
    const rec = new MediaRecorder(c.captureStream(15), type ? { mimeType: type } : {});
    const parts = [], t0 = Date.now(), name = fname(topic, "webm");
    rec.ondataavailable = (e) => e.data.size && parts.push(e.data);
    rec.onstop = () => { download(new Blob(parts, { type: "video/webm" }), name); toast("Clip downloaded: " + name, true); };
    const R = { rec, raf: 0 };
    const draw = () => {
      if (T.rec !== R) return;
      if (T.img) g.drawImage(T.img, 0, 0, c.width, c.height);
      const s = Math.round((Date.now() - t0) / 1000);
      T.recLbl.textContent = `${String(Math.floor(s / 60)).padStart(2, "0")}:${String(s % 60).padStart(2, "0")}`;
      R.raf = requestAnimationFrame(draw);
    };
    T.rec = R;
    rec.start(1000);
    T.recBtn.innerHTML = icon("stop"); T.recBtn.classList.add("on"); T.recBtn.title = "Stop and download the clip";
    draw();
  }
  function stopRec(T) {
    const R = T.rec;
    T.rec = null;
    cancelAnimationFrame(R.raf);
    R.rec.stop();
    T.recLbl.textContent = "";
    T.recBtn.innerHTML = icon("record"); T.recBtn.classList.remove("on"); T.recBtn.title = "Record a clip of this camera";
  }

  // the chart fills its window (its <details> does not stretch by CSS); attached beside F/T sensor: its own height
  const chartRO = new ResizeObserver(() => fitChart());
  function fitChart() {
    const c = ftEl.querySelector(".ftc-canvas"), body = ftEl.parentNode;
    if (!c || !body || !body.classList.contains("lbody")) return;
    if (body.closest(".lside") && body.closest(".latt")) { c.style.removeProperty("height"); c.style.removeProperty("flex"); return; }
    // what sits above the plot (wide: one line) and below it (narrow: the readout line)
    const box = c.parentNode, gap = parseFloat(getComputedStyle(box).rowGap) || 0, rc = c.getBoundingClientRect();
    const above = rc.top - body.getBoundingClientRect().top;
    const below = [...box.children].filter((k) => k !== c && k.offsetParent && k.getBoundingClientRect().top >= rc.bottom - 1)
      .reduce((a, k) => a + k.offsetHeight + gap, 0) + parseFloat(getComputedStyle(ftEl).paddingBottom);
    c.style.setProperty("height", Math.max(60, body.clientHeight - above - below - 2) + "px", "important");
    c.style.setProperty("flex", "none", "important");
  }

  // ---- changes
  function toggleMin(id) {
    if (isAtt(id)) return;
    const S = L();
    S.min = S.min.includes(id) ? S.min.filter((m) => m !== id) : [...S.min, id];
    save(); render();
  }
  function setType(id, v) {
    const S = L(), other = ONE.includes(v) && Object.keys(S.tiles).find((k) => k !== id && S.tiles[k].type === v);
    if (other && S.tiles[other].att) { toast("The F/T chart is open under F/T sensor (its Chart button)"); render(); return; }
    if (other) S.tiles[other].type = S.tiles[id].type;            // one of each: the two swap
    S.tiles[id].type = v;
    if (v === "cam") { delete S.tiles[id].topic; S.tiles[id].pick = 0; }
    save(); render();
  }
  function closeTile(id) {
    const T = tiles.get(id);
    if (T) { stopCam(T); tiles.delete(id); }
    take(id); delete L().tiles[id]; L().min = L().min.filter((m) => m !== id);
    save(); render();
  }
  function addView() {
    if (allViews().length >= MAX_VIEWS) { toast(`At most ${MAX_VIEWS} views in a layout`); return; }
    const S = L(); let n = 1;
    while (S.tiles["v" + n]) n++;
    S.tiles["v" + n] = { type: "empty" };
    put("v" + n, "right");
    save(); render();
  }
  // F/T sensor › Chart (your own layouts): the chart opens attached right under the F/T sensor card; again: it closes.
  // A chart already in another window moves there. Main always has it (no button).
  const chartUnder = () => attId(L()) || null;
  function toggleChart() {
    const S = L();
    if (S.fixed) return;
    const under = chartUnder();
    if (under) { closeTile(under); return; }
    let id = Object.keys(S.tiles).find((k) => S.tiles[k].type === "ft");
    if (id) take(id);
    else { id = "chart"; let n = 1; while (S.tiles[id]) id = "chart" + n++; S.tiles[id] = { type: "ft" }; }
    S.tiles[id].att = true;
    S.min = S.min.filter((m) => m !== id);
    attach(S);
    save(); render();
  }

  // ---- drag helpers (listen on window, so redrawing during a drag does not end it)
  function track(e, move) {
    e.preventDefault();
    document.body.classList.add("lresizing");
    const up = () => { window.removeEventListener("pointermove", move); document.body.classList.remove("lresizing"); save(); render(); };
    window.addEventListener("pointermove", move);
    window.addEventListener("pointerup", up, { once: true });
  }
  function line(dir, measure, apply, off) {
    const s = h("div", { class: "lline " + dir + (off || L().fixed ? " off" : ""), title: "Drag to resize" });
    s.addEventListener("pointerdown", (e) => {
      if (s.classList.contains("off")) return;
      const m = measure(), x0 = dir === "v" ? e.clientX : e.clientY;
      track(e, (ev) => { apply(m, (dir === "v" ? ev.clientX : ev.clientY) - x0); render(); });
    });
    return s;
  }

  // ---- building
  function panel(id, ctx) {
    const min = isMin(id);
    if (!isTile(id)) {
      const B = cardBits[id];
      B.el.classList.toggle("lmin-on", min);
      B.el.dataset.ctx = ctx;
      B.mn.hidden = false;
      B.mn.innerHTML = icon(min ? "restore" : "minus");
      B.mn.title = min ? "Show it again" : "Minimise (stays here; click its title to bring it back)";
      B.el.style.cssText = "";
      return B.el;
    }
    const T = tileDom(id);
    T.el.dataset.ctx = ctx;
    T.el.classList.toggle("lmin-on", min);
    T.el.style.cssText = "";
    const att = isAtt(id);
    T.mn.hidden = att;
    T.dock.hidden = !att;
    const under = !!L().tiles[id].under;
    T.dock.innerHTML = icon(under ? "dockSide" : "dockBottom");
    T.dock.title = under ? "Put the chart back under F/T sensor" : "Move the chart under the 3D view";
    const t = L().tiles[id], cam = t.type === "cam";
    T.fit.classList.toggle("fiton", cam && t.fit !== false);
    T.fit.title = t.fit === false ? "Fit to picture: off (the window fills its space). Click: fit the window to the picture"
      : "Fit to picture: on (the window takes the picture's shape). Click: fill the space instead";
    T.grip.hidden = T.kind.hidden = T.x.hidden = att;
    T.el.classList.toggle("latt", att);
    T.el.classList.toggle("lft", L().tiles[id].type === "ft");
    T.mn.innerHTML = icon(min ? "restore" : "minus");
    T.mn.title = min ? "Show it again" : "Minimise (stays here; click its bar to bring it back)";
    const side = ctx === "l" || ctx === "r", ar = side && fitAr(id);
    T.rz.hidden = !side || L().fixed || min || att || !!ar;
    if (side && !min && !att) T.el.style.height = (ar ? Math.round((L().w[ctx] - W.ch.w) / ar + W.ch.h) : L().h[id] || 240) + "px";
    fill(T);
    return T.el;
  }
  function stack(k, fitted) {
    const S = L(), ids = S[k], w = S[k === "right" ? "rw" : "bw"], col = k === "right";
    const box = h("div", { class: "lstack " + k, "data-stack": k });
    const els = ids.map((id) => panel(id, k));
    // fitted: all open windows follow their pictures; each gets the length its picture needs (lines between are off)
    const open = ids.map((id, i) => (fitted ? (col ? 1 / (fitAr(id) || 1e9) : fitAr(id)) : col && isMin(id) ? 0 : w[i]));
    const sum = open.reduce((a, b) => a + b, 0) || 1;
    els.forEach((e, i) => {
      if (i > 0) {
        const a = els[i - 1], b = e, ia = ids[i - 1], ib = ids[i];
        box.append(line(col ? "h" : "v", () => {
          const ra = a.getBoundingClientRect(), rb = b.getBoundingClientRect();
          return { la: col ? ra.height : ra.width, len: col ? ra.height + rb.height : ra.width + rb.width, tot: w[i - 1] + w[i],
            lo: col ? MINH : MINW[S.tiles[ia].type], hi: col ? MINH : MINW[S.tiles[ib].type] };
        }, (m, d) => {
          const f = Math.min(1 - m.hi / m.len, Math.max(m.lo / m.len, (m.la + d) / m.len));
          w[i - 1] = m.tot * f; w[i] = m.tot * (1 - f);
        }, !!fitted || (col && (isMin(ia) || isMin(ib)))));
      }
      e.style.flex = open[i] ? `${(open[i] / sum) * ids.length} 1 0` : "0 0 auto";
      if (!col) { e.style.minWidth = (fitted ? 0 : MINW[S.tiles[ids[i]].type]) + "px"; if (isMin(ids[i])) e.style.alignSelf = "flex-start"; }
      else if (!isMin(ids[i])) e.style.minHeight = (fitted ? 0 : MINH) + "px";
      box.append(e);
    });
    return box;
  }
  // a stack that fits its pictures: every open window is a camera that fits, and at least one is open
  const stackFit = (k) => { const a = L()[k].filter((id) => !isMin(id)); return a.length > 0 && a.every(fitAr); };
  // sizes that follow the pictures (not saved). The middle column's size comes from the last render (0: not yet).
  // a camera window's frame around its picture (bar, padding, borders), measured from one on screen
  function chrome() {
    for (const T of tiles.values()) {
      const im = T.img;
      if (im && im.isConnected && im.clientHeight > 20) { W.ch = { w: T.el.offsetWidth - im.clientWidth, h: T.el.offsetHeight - im.clientHeight }; return; }
    }
  }
  const midSize = () => { const m = el.querySelector(":scope > .lmid"); return m ? { w: m.clientWidth, h: m.clientHeight } : { w: 0, h: 0 }; };
  // the row under the 3D view, as tall as its pictures need for the whole width; 0: it does not fit (the 3D view
  // keeps at least half the height, e.g. one camera alone would be too tall), then the row keeps its own height
  // (its line) and the pictures letterbox
  function fitBottomH() {
    const S = L();
    if (!S.bottom.length || !stackFit("bottom")) return 0;
    const Wr = W.mid.w - GAP * (S.bottom.length - 1) - S.bottom.filter(isMin).length * 120, H = W.mid.h - GAP;
    if (Wr <= 0 || H <= 0) return 0;
    const sumAr = S.bottom.reduce((a, id) => a + fitAr(id), 0);
    const n = S.bottom.length - S.bottom.filter(isMin).length;
    const v = Math.round((Wr - n * W.ch.w) / sumAr + W.ch.h);
    return v >= MINH && v <= Math.min(H / 2, H - MAIN_MINH) ? v : 0;
  }
  // the stack beside the 3D view, as wide as its pictures need for the whole height; 0: it does not fit (the 3D view
  // keeps at least half the width), then the stack keeps its own width (its line)
  function fitRightW(bh) {
    const S = L();
    if (!stackFit("right") || !W.mid.h) return 0;
    const Ht = W.mid.h - (S.bottom.length ? bh + GAP : 0), open = S.right.filter((id) => !isMin(id));
    const Hp = Ht - GAP * (S.right.length - 1) - (S.right.length - open.length) * (W.ch.h - 10) - open.length * W.ch.h;
    const sumInv = open.reduce((a, id) => a + 1 / fitAr(id), 0), Wt = W.mid.w - GAP;
    const v = Math.round(Hp / sumInv + W.ch.w);
    return v >= Math.max(120, stackMinW() * 0.6) && v <= Math.min(Wt / 2, Wt - MINW["3d"] - GAP) ? v : 0;
  }
  const mainEl = h("div", { class: "lpanel lmain", "data-lid": "main", "data-ctx": "main" },
    h("div", { class: "lbody" }, view3d));
  function middle() {
    const S = L(), z = h("div", { class: "lmid", "data-zone": "m" }), top = h("div", { class: "ltop" });
    mainEl.style.cssText = `min-width:${MINW["3d"]}px;min-height:${MAIN_MINH}px`;
    top.append(mainEl);
    const fb = fitBottomH(), bh = fb || S.bh;
    if (S.right.length) {
      const sw = fitRightW(bh), st = stack("right", !!sw);
      st.style.minWidth = (sw ? 0 : stackMinW()) + "px";
      if (sw) { mainEl.style.flex = "1 1 0"; st.style.flex = `0 0 ${sw}px`; }
      else { mainEl.style.flex = `${S.f} 1 0`; st.style.flex = `${1 - S.f} 1 0`; }
      top.append(line("v", () => ({ W: top.clientWidth - GAP, x: mainEl.getBoundingClientRect().width }),
        (m, d) => { S.f = Math.min(1 - stackMinW() / m.W, Math.max(MINW["3d"] / m.W, (m.x + d) / m.W)); }, !!sw), st);
    } else mainEl.style.flex = "1 1 0";
    // Main: the 3D view as it always was, a set height (not the whole column)
    if (S.fixed && !S.bottom.length) top.style.cssText = `flex:0 0 auto;height:${MAIN_H}px`;
    z.append(top);
    if (S.bottom.length) {
      const st = stack("bottom", !!fb);
      st.style.flex = `0 0 ${bh}px`;
      z.append(line("h", () => ({ H: z.clientHeight - GAP, h: bh }), (m, d) => {
        S.bh = Math.round(Math.max(MINH, Math.min(m.H - MAIN_MINH, m.h - d)));
      }, !!fb), st);
    }
    z.style.minWidth = midMinW() + "px";
    return z;
  }
  function side(k) {
    const S = L(), z = h("div", { class: "lside " + k + (fd(k) ? " folded" : ""), "data-zone": k });
    if (fd(k)) {
      const names = S[k].map((id) => (isTile(id) ? typeName(S.tiles[id].type)
        : cards[id].querySelector(".ctitle")?.textContent || id)).join(" · ");
      z.append(h("button", { class: "lstrip", type: "button", disabled: W.auto[k],
        title: W.auto[k] ? "Hidden: the browser window is too small" : "Show this column",
        onclick: () => { S.fold[k] = false; save(); render(); } }, h("span", {}, names || "Empty")));
      // the cards stay in the page (hidden), so their values keep updating; views there pause
      z.append(h("div", { hidden: true }, S[k].filter((id) => !isTile(id)).map((id) => panel(id, k))));
      return z;
    }
    z.style.width = S.w[k] + "px";
    S[k].forEach((id) => z.append(panel(id, k)));
    if (!S[k].length) z.append(h("div", { class: "ldropzone" }, "Drop a window here"));
    return z;
  }
  function sideLine(k) {
    const s = h("div", { class: "lline v side" + (fd(k) || L().fixed ? " off" : ""), title: "Drag to resize" });
    s.addEventListener("pointerdown", (e) => {
      if (fd(k) || L().fixed) return;
      const x0 = e.clientX, w0 = L().w[k];
      track(e, (ev) => {
        L().w[k] = Math.round(Math.max(SIDE_MIN, Math.min(SIDE_MAX, room(k), w0 + (k === "l" ? 1 : -1) * (ev.clientX - x0))));
        render();
      });
    });
    return s;
  }
  const room = (k) => el.clientWidth - 2 * GAP - sideW(k === "l" ? "r" : "l") - midMinW();
  // a smaller browser window: the side columns give way first (right, then left), down to 280 px, then they fold by
  // themselves (not saved) and come back when there is room again
  function fit() {
    const S = L(), avail = el.clientWidth - 2 * GAP, need = midMinW();
    W.auto = { l: false, r: false };
    if (!avail) return;
    const want = { l: S.w.l, r: S.w.r };
    for (const k of ["r", "l"]) {
      const total = sideW("l") + sideW("r") + need;
      if (fd(k) || total <= avail) continue;
      const w = S.w[k] - (total - avail);
      if (w >= SIDE_MIN) S.w[k] = Math.round(w); else { S.w[k] = want[k]; W.auto[k] = true; }
    }
  }
  const el = h("div", { class: "lws" });
  function render() {
    const S = L();
    document.body.classList.toggle("ledit", W.edit && W.shown);
    el.classList.toggle("lfixed", S.fixed);
    fit();
    W.mid = midSize();
    chrome();
    const live = new Set(allViews());
    for (const [id, T] of tiles) if (!live.has(id)) { stopCam(T); tiles.delete(id); }
    if (!allViews().some((id) => S.tiles[id].type === "ft" && !isMin(id)) && ftEl.parentNode) ftEl.remove();
    el.replaceChildren(side("l"), sideLine("l"), middle(), sideLine("r"), side("r"));
    if (ftBtn) {
      const on = !!chartUnder();
      ftBtn.hidden = S.fixed;
      ftBtn.classList.toggle("pressed", on);
      ftBtn.title = on ? "Hide the F/T chart" : "Show the F/T chart under this card";
    }
    renderHeader();
    requestAnimationFrame(fitChart);
    // sizes that follow pictures need the middle column measured: the first render after a change draws again
    const now = midSize();
    const ch1 = W.ch; chrome();
    if ((now.w !== W.mid.w || now.h !== W.mid.h || W.ch.w !== ch1.w || W.ch.h !== ch1.h) && allViews().some(fitAr)) later();
  }
  const later = () => {
    if (!W.shown || W.busy || document.body.classList.contains("lresizing")) return;
    W.busy = true;
    requestAnimationFrame(() => { W.busy = false; render(); });
  };
  addEventListener("resize", later);

  // ---- layout numbers (two or more layouts) under OPERATE in the left nav; while editing, the edit toolbar in the
  // top bar (it needs the width)
  function renderHeader() {
    if (!header) return;
    const S = L();
    header.replaceChildren();
    header.className = "grp hlay editing";
    header.hidden = !W.shown || !W.edit;
    if (navSlot) {
      navSlot.replaceChildren();
      navSlot.hidden = !W.shown || W.edit || W.list.length < 2;
    }
    if (!W.edit) {
      if (navSlot && !navSlot.hidden) renderNums(navSlot);
      return;
    }
    if (W.edit) {
      const name = h("input", { type: "text", value: S.name, maxlength: "24", title: "Layout name",
        oninput: () => { S.name = name.value.trim() || "Layout"; save(); } });
      header.append(h("span", { class: "hl-lbl" }, `Editing ${W.cur + 1}`), name,
        h("button", { type: "button", html: icon("plus") + "<span>View</span>", title: "Add a view on the right of the 3D view", onclick: addView }),
        h("button", { type: "button", html: icon("reset") + "<span>Reset</span>", title: "Start again from the Main layout",
          onclick: () => { W.list[W.cur] = Object.assign(clone(MAIN), { name: S.name, fixed: false }); tiles.forEach(stopCam); tiles = new Map(); save(); render(); } }),
        h("span", { class: "hl-hint", title: "Drop above or below a window, or on the right or bottom edge of the 3D view" }, "Drag ⠿ to move"),
        h("button", { type: "button", class: "hl-done", html: icon("check") + "<span>Done</span>", onclick: () => { W.edit = false; save(); render(); } }));
      return;
    }
  }
  function renderNums(box) {
    const S = L();
    const tog = (k) => h("button", { type: "button", class: "hl-ico" + (fd(k) ? "" : " on"), disabled: S.fixed || W.auto[k],
      html: icon(k === "l" ? "panelL" : "panelR"),
      title: S.fixed ? "Main is fixed: both columns stay" : W.auto[k] ? "Hidden: the browser window is too small"
        : (fd(k) ? "Show" : "Hide") + ` the ${k === "l" ? "left" : "right"} column`,
      onclick: () => { S.fold[k] = !S.fold[k]; save(); render(); } });
    box.append(h("div", { class: "navlay-cap" }, "Layout"),
      h("div", { class: "navlay-row" }, ...W.list.map((x, i) => h("button", { type: "button", class: "hl-num" + (i === W.cur ? " on" : ""),
        title: `${i + 1}: ${x.name}` + (x.fixed ? " (fixed)" : ""), "aria-label": `Layout ${i + 1}: ${x.name}`,
        onclick: () => use(i) }, String(i + 1)))),
      h("div", { class: "navlay-row" }, tog("l"), tog("r")));
  }
  function use(i) {
    if (i === W.cur) return;
    tiles.forEach(stopCam); tiles = new Map();
    W.cur = i; W.edit = false;
    save(); render();
  }

  // ---- drag and drop (edit mode only)
  function startDrag(e, id) {
    if (!W.edit || isAtt(id)) return;
    e.preventDefault();
    const label = isTile(id) ? tileDom(id).title.textContent : cards[id].querySelector(".ctitle")?.textContent || id;
    W.drag = { id, ghost: h("div", { class: "lghost" }, label), tgt: h("div", { class: "ltarget" }), drop: null };
    document.body.append(W.drag.ghost, W.drag.tgt);
    document.body.classList.add("ldragging");
    (isTile(id) ? tileDom(id).el : cards[id]).classList.add("lsrc");
    window.addEventListener("pointermove", dragMove);
    window.addEventListener("pointerup", dragEnd, { once: true });
    dragMove(e);
  }
  function dragMove(e) {
    const D = W.drag;
    if (!D) return;
    D.ghost.style.left = e.clientX + 12 + "px"; D.ghost.style.top = e.clientY + 10 + "px";
    const hit = document.elementFromPoint(e.clientX, e.clientY), tile = isTile(D.id), S = L();
    const p = hit && hit.closest(".lpanel"), zone = hit && hit.closest("[data-zone]");
    let drop = null, rect = null, ln = false;
    if (p && el.contains(p) && p.dataset.lid !== D.id) {
      const r = p.getBoundingClientRect(), k = p.dataset.ctx;
      if (k === "main") {               // the pinned 3D view: its right or bottom part opens a stack there
        if (tile && !S.right.length && e.clientX > r.right - r.width * 0.35) { drop = { k: "right" }; rect = [r.right - r.width * 0.35, r.top, r.width * 0.35, r.height]; }
        else if (tile && !S.bottom.length && e.clientY > r.bottom - r.height * 0.35) { drop = { k: "bottom" }; rect = [r.left, r.bottom - r.height * 0.35, r.width, r.height * 0.35]; }
      } else if (k === "bottom") {      // a row: before or after (a vertical line)
        if (tile) { const before = e.clientX < r.left + r.width / 2; drop = { k, at: p.dataset.lid, before }; ln = true;
          rect = [before ? r.left - 6 : r.right + 2, r.top, 4, r.height]; }
      } else if (k !== "right" || tile) {   // columns: above or below (a horizontal line); cards stay in the side columns
        const before = e.clientY < r.top + r.height / 2; drop = { k, at: p.dataset.lid, before }; ln = true;
        rect = [r.left, before ? r.top - 6 : r.bottom + 2, r.width, 4];
      }
    } else if (zone && el.contains(zone) && (zone.dataset.zone === "l" || zone.dataset.zone === "r") && !fd(zone.dataset.zone)) {
      const k = zone.dataset.zone, ids = S[k].filter((i) => i !== D.id), last = ids[ids.length - 1];
      if (!last) { const b = zone.getBoundingClientRect(); drop = { k }; rect = [b.left, b.top, b.width, 80]; }
      else {
        const r = (isTile(last) ? tileDom(last).el : cards[last]).getBoundingClientRect();
        if (e.clientY > r.bottom) { drop = { k, at: last, before: false }; ln = true; rect = [r.left, r.bottom + 2, r.width, 4]; }
      }
    }
    D.drop = drop;
    D.tgt.style.display = rect ? "block" : "none";
    D.tgt.classList.toggle("line", ln);
    if (rect) Object.assign(D.tgt.style, { left: rect[0] + "px", top: rect[1] + "px", width: rect[2] + "px", height: rect[3] + "px" });
  }
  function dragEnd() {
    window.removeEventListener("pointermove", dragMove);
    const D = W.drag;
    W.drag = null;
    if (!D) return;
    D.ghost.remove(); D.tgt.remove();
    document.body.classList.remove("ldragging");
    document.querySelectorAll(".lsrc").forEach((x) => x.classList.remove("lsrc"));
    if (D.drop) {
      take(D.id);
      const a = L()[D.drop.k];
      put(D.id, D.drop.k, D.drop.at ? a.indexOf(D.drop.at) + (D.drop.before ? 0 : 1) : null);
      attach(L());
      save();
    }
    render();
  }

  // ---- SETUP › Layouts
  function newLayout(from, name) {
    if (W.list.length >= MAX_LAYOUTS) { toast(`At most ${MAX_LAYOUTS} layouts`); return null; }
    const c = sane(Object.assign(clone(from), { fixed: false }));
    let n = W.list.length + 1;
    while (W.list.some((x) => x.name === "Layout " + n)) n++;
    c.name = name && !W.list.some((x) => x.name === name) ? name.slice(0, 24) : "Layout " + n;
    W.list.push(c); save();
    return c;
  }
  function setupView() {
    const box = h("div", { class: "col" });
    const draw = () => {
      const views = (x) => [...x.right, ...x.bottom, ...x.l, ...x.r].filter((id) => x.tiles[id])
        .map((id) => (x.tiles[id].type === "empty" ? "empty view" : typeName(x.tiles[id].type).toLowerCase()).replace("f/t", "F/T"));
      const rows = W.list.map((x, i) => h("div", { class: "lrow" + (i === W.cur ? " cur" : "") },
        h("span", { class: "lnum" }, String(i + 1)),
        x.fixed ? h("b", { class: "lname" }, x.name) : h("input", { type: "text", class: "lname", value: x.name, maxlength: "24", title: "Rename",
          onchange: (e) => { x.name = e.target.value.trim() || "Layout"; save(); renderHeader(); } }),
        h("span", { class: "chip" }, x.fixed ? "Fixed" : "Your own"),
        h("span", { class: "muted small grow" }, "3D view" + (views(x).length ? ", " + views(x).join(", ") : "")),
        h("button", { class: "btn quiet", type: "button", title: "A copy of this layout you can change",
          onclick: () => { newLayout(x, x.name + " copy"); draw(); renderHeader(); } }, "Duplicate"),
        x.fixed ? h("span", { class: "lact" }) : h("span", { class: "lact" },
          h("button", { class: "btn", type: "button", title: "Open it in OPERATE to move, add or close windows",
            onclick: () => { if (W.cur !== i) { tiles.forEach(stopCam); tiles = new Map(); W.cur = i; } W.edit = true; save(); onEdit && onEdit(); } }, "Edit"),
          h("button", { class: "btn quiet", type: "button", title: "Delete this layout",
            onclick: async () => {
              if (!(await ask(`Delete layout ${i + 1} "${x.name}"?`, { ok: "Delete", danger: true }))) return;
              i = W.list.indexOf(x);
              if (i < 0) return;
              W.list.splice(i, 1);
              if (W.cur === i) { tiles.forEach(stopCam); tiles = new Map(); W.cur = 0; } else if (W.cur > i) W.cur--;
              save(); draw(); renderHeader();
            } }, "Delete"))));
      box.replaceChildren(
        h("div", { class: "card" },
          h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "Layouts"),
            h("button", { class: "btn primary", type: "button", disabled: W.list.length >= MAX_LAYOUTS,
              onclick: () => { newLayout(MAIN); draw(); renderHeader(); } }, "+ New layout")),
          h("div", { class: "muted small" }, "Window layouts for OPERATE. 1 (Main) is fixed. With two or more, their numbers show in the top bar to switch. Kept in this browser."),
          h("div", { class: "llist" }, rows)));
    };
    draw();
    return { el: box, refresh: draw };
  }

  return {
    el,
    toggleChart,
    setupView,
    enter() { W.shown = true; render(); refreshCams(); },
    update(s) {                          // a camera driver started or stopped: list the cameras again
      const sig = (s.workcell?.drivers || []).map((d) => d.name + ":" + d.state).join(" ");
      if (sig !== W.drvSig) { W.drvSig = sig; if (W.shown) refreshCams(); }
    },
    leave() { W.shown = false; W.edit = false; document.body.classList.remove("ledit"); tiles.forEach((T) => { stopCam(T); T.key = null; }); renderHeader(); },
  };
}
