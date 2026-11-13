// SETUP > Drivers & cameras: start / stop camera drivers and (later) vision pipelines, see their pictures and logs.
// Built by app.js with its helpers (ctx). The server (services.py) owns every rule; this only shows and asks.

const STATE = {   // state -> [chip text, chip class, dot class]
  stopped: ["Stopped", "", ""], waiting: ["Waiting…", "warn", "warn"], starting: ["Starting…", "warn", "warn"],
  running: ["Running", "ok", "ok"], no_data: ["No data", "warn", "warn"], stopping: ["Stopping…", "warn", "warn"],
  failed: ["Failed", "bad", "bad"], missing: ["Not installed", "", ""],
};
const ON = new Set(["waiting", "starting", "running", "no_data"]);

export function upTime(s) {
  if (s == null) return "";
  return s < 90 ? `${s} s` : s < 5400 ? `${Math.round(s / 60)} min` : `${(s / 3600).toFixed(1)} h`;
}

export function drivers(ctx) {
  const { h, btn, note, cmd, toast, lease, ask, askText } = ctx;
  const chip = (t, cls) => h("span", { class: "chip " + (cls || "") }, t);
  const list = h("div", { class: "list", id: "dr-list" });
  const others = h("div", { class: "list", id: "dr-others" });
  const right = h("div", { class: "card grow", id: "dr-right" });
  const D = { data: null, sel: null, topic: null, edit: null, sig: "", logT: 0 };

  const left = h("div", { class: "card", style: "width:340px" },
    h("div", { class: "section" }, "Drivers & pipelines"),
    note("Camera drivers and vision pipelines the GUI starts and stops for you.",
      "Each one is a ROS process in the robot container. Start starts what it needs first (a pipeline's camera); Stop " +
      "stops what needs it first. Ready = all its topics publish. A GUI restart leaves them running."),
    list,
    h("div", { class: "row" }, lease(btn("+ Add…", "primary", () => openEditor(null)))),
    h("div", { class: "section" }, "Other image topics"),
    others,
    h("div", { class: "row" }, h("input", { type: "text", id: "dr-topic", list: "dr-topics", placeholder: "any image topic…",
      style: "flex:1", onchange: (e) => pickTopic(e.target.value.trim()) }), h("datalist", { id: "dr-topics" }),
      btn("Rescan", "quiet", () => scan())));

  // ------------------------------------------------------------------ data
  async function load() {
    const r = await cmd("drivers_get");
    if (!r.ok) return;
    D.data = r;
    if (D.sel && !r.services.some((s) => s.name === D.sel)) D.sel = null;
    if (!D.sel && !D.topic && !D.edit && r.services.length) D.sel = r.services[0].name;
    fillList();
    if (!D.edit) fillRight();
    scan();
  }
  function took(r) {                        // a command answered with the fresh view
    if (r.services) { D.data = { ...D.data, services: r.services, error: r.error }; fillList(); if (!D.edit) fillRight(); }
  }
  async function scan() {
    const r = await cmd("camera_status");
    if (!r.ok) return;
    const mine = new Set((D.data?.services || []).flatMap((s) => s.outputs.map((o) => o.topic)));
    const rest = r.cameras.filter((c) => !c.driver && !mine.has(c.topic));
    others.replaceChildren(...(rest.length ? rest.map((c) => h("div", { class: "item" + (D.topic === c.topic ? " on" : ""),
      onclick: () => pickTopic(c.topic) },
      h("span", { class: "dot" + (c.publishers > 0 ? " ok" : ""), style: "margin-top:5px" }),
      h("div", { class: "grow" }, h("div", {}, c.name), h("div", { class: "muted small" }, c.topic)))) :
      [h("div", { class: "muted small" }, "none")]));
    document.getElementById("dr-topics")?.replaceChildren(...r.topics.map((t) => h("option", { value: t })));
  }

  // ------------------------------------------------------------------ the list
  function fillList() {
    const ss = D.data?.services || [];
    const kinds = D.data?.kinds || {};
    const groups = [...new Set(ss.map((s) => s.kind))];
    const row = (s) => {
      const [, , dot] = STATE[s.state] || STATE.stopped, on = ON.has(s.state);
      return h("div", { class: "item" + (D.sel === s.name && !D.topic ? " on" : ""), "data-name": s.name, onclick: () => pick(s.name) },
        h("span", { class: "dot " + dot, style: "margin-top:5px" }),
        h("div", { class: "grow" }, h("div", {}, s.name),
          h("div", { class: "muted small" }, s.type_label, " · ", (STATE[s.state] || STATE.stopped)[0].toLowerCase(),
            s.state === "running" && s.up_s != null ? ` ${upTime(s.up_s)}` : "")),
        s.state === "missing" ? "" : lease(btn(on || s.state === "stopping" ? "Stop" : "Start", on ? "" : "quiet",
          (e) => { e.stopPropagation(); (on ? stop : start)(s.name); })));
    };
    list.replaceChildren(...(ss.length ? (groups.length > 1
      ? groups.flatMap((k) => [h("div", { class: "muted small dr-grp" }, kinds[k] || k), ...ss.filter((s) => s.kind === k).map(row)])
      : ss.map(row)) : [h("div", { class: "muted" }, "Nothing set up yet. Press + Add… and pick your camera.")]),
      ...(D.data?.error ? [h("div", { class: "bad small" }, D.data.error)] : []));
  }
  function pick(name) { D.sel = name; D.topic = null; D.edit = null; fillList(); fillRight(); scan(); }
  function pickTopic(t) { if (!t) return; D.topic = t; D.edit = null; fillList(); fillRight(); scan(); }
  async function start(n) { const r = await cmd("drivers_start", { name: n }); took(r); }
  async function stop(n) { const r = await cmd("drivers_stop", { name: n }); took(r); }

  // ------------------------------------------------------------------ the right side: one service or one topic
  const preview = h("div", { class: "cam", id: "dr-cam" });
  const info = h("div", { class: "secondary small", id: "dr-caminfo" });
  const logBox = h("pre", { class: "dr-log", id: "dr-log" });
  function show(topic) {
    D.shown = topic || null;
    if (!topic) return preview.replaceChildren(h("span", { class: "muted" }, "No picture: start it first"));
    preview.replaceChildren(h("img", { src: "/api/camera.mjpeg?topic=" + encodeURIComponent(topic) + "&t=" + Date.now(), alt: "",
      onerror: () => preview.replaceChildren(h("span", { class: "muted" }, `${topic}: no picture yet`)) }));
  }
  function fillRight() {
    if (D.topic) {
      right.replaceChildren(h("div", { class: "row" }, h("span", { class: "title grow" }, D.topic)), info, preview);
      if (D.shown !== D.topic) show(D.topic);
      return;
    }
    const s = (D.data?.services || []).find((x) => x.name === D.sel);
    if (!s) { right.replaceChildren(h("div", { class: "muted" }, "Select a driver, or + Add… one.")); show(null); return; }
    const [txt, cls] = STATE[s.state] || STATE.stopped, on = ON.has(s.state);
    const imgs = s.outputs.filter((o) => o.image);
    if (!imgs.some((o) => o.topic === D.view)) D.view = imgs[0]?.topic || null;
    const live = on && D.view && s.outputs.find((o) => o.topic === D.view)?.publishers > 0;
    right.replaceChildren(
      h("div", { class: "row" }, h("span", { class: "title grow" }, s.name, h("span", { class: "muted" }, " · " + s.type_label)),
        chip(txt + (s.state === "running" && s.up_s != null ? " " + upTime(s.up_s) : ""), cls)),
      s.msg ? h("div", { class: s.state === "failed" ? "bad small" : "muted small" }, s.msg) : "",
      h("div", { class: "row" },
        s.state === "missing" ? "" : lease(btn(on || s.state === "stopping" ? "Stop" : "Start", on ? "" : "primary",
          () => (on ? stop : start)(s.name))),
        lease(btn("Edit…", "quiet", () => openEditor(s))),
        lease(btn("Delete", "quiet", () => del(s))),
        h("label", { class: "chk", title: "Start it when the GUI starts (after a robot stack restart, say)" },
          h("input", { type: "checkbox", checked: s.autostart, onchange: async (e) => took(await cmd("drivers_autostart", { name: s.name, on: e.target.checked })) }),
          "start with the GUI")),
      imgs.length > 1 ? h("div", { class: "seg" }, ...imgs.map((o) => h("button", { class: "btn" + (o.topic === D.view ? " on" : ""),
        onclick: () => { D.view = o.topic; fillRight(); } }, o.label))) : "",
      info, preview,
      h("div", { class: "section" }, "Topics"),
      h("div", { class: "list" }, ...s.outputs.map((o) => h("div", { class: "item" },
        h("span", { class: "dot" + (o.publishers > 0 ? " ok" : ""), style: "margin-top:5px" }),
        h("div", { class: "grow" }, h("div", {}, o.label, o.required ? "" : h("span", { class: "muted small" }, " (optional)")),
          h("div", { class: "muted small" }, o.topic))))),
      s.needs.length ? h("div", { class: "muted small" }, "Needs: " + s.needs.join(", ")) : "",
      h("details", { id: "dr-logbox", ontoggle: (e) => e.target.open && log(true) }, h("summary", {}, "Log"), logBox,
        h("div", { class: "muted small" }, "Command: ", h("code", {}, s.command))));
    const want = live ? D.view : null;
    if (D.shown !== want || !preview.firstChild) show(want);
  }
  async function log(force) {
    const box = document.getElementById("dr-logbox");
    if (!box?.open || !D.sel || (!force && Date.now() - D.logT < 2000)) return;
    D.logT = Date.now();
    const r = await cmd("drivers_log", { name: D.sel, lines: 80 });
    if (r.ok) { logBox.textContent = r.text || "(empty)"; logBox.scrollTop = logBox.scrollHeight; }
  }
  async function del(s) {
    if (!(await ask(`Delete "${s.name}"? (Its log files stay.)`))) return;
    const r = await cmd("drivers_delete", { name: s.name });
    if (r.ok) { D.sel = null; took(r); }
  }

  // ------------------------------------------------------------------ add / edit
  function openEditor(s) {
    D.edit = { replace: s ? s.name : null, type: s ? s.type : null };
    editor(s);
  }
  function editor(s) {
    const T = D.data?.types || [], E = D.edit;
    const t = T.find((x) => x.type === E.type);
    const pickType = h("div", { class: "dr-types" }, ...T.map((x) => h("button", {
      class: "dr-type" + (x.type === E.type ? " on" : ""), title: x.pkg ? "ROS package " + x.pkg : "",
      onclick: () => { E.type = x.type; editor(s); } },
      h("div", {}, x.label), h("div", { class: "muted small" }, x.help), x.installed ? "" : h("div", { class: "warn small" }, "not installed"))));
    const kids = [h("div", { class: "row" }, h("span", { class: "title grow" }, E.replace ? `Edit ${E.replace}` : "Add a driver or pipeline")),
      h("div", { class: "section" }, "1 · What is it?"), pickType];
    if (t) {
      const p = s?.type === t.type ? s.params : {};
      const others = (D.data?.services || []).filter((x) => x.name !== E.replace);
      kids.push(h("div", { class: "section" }, "2 · Settings"),
        !t.installed ? h("div", { class: "warn small" }, "Not in the robot image: " + t.install + ". You can set it up now; Start works once it is installed.") : "",
        h("div", { class: "dr-form" },
          h("label", {}, "Name"), h("input", { type: "text", id: "dr-name", value: s?.name || "", placeholder: t.kind === "camera" ? "e.g. wrist" : "e.g. midpoints" }),
          h("div", { class: "muted small span2" }, "lower_snake_case; it becomes the ROS namespace of its topics"),
          ...(t.kind ? [] : [h("label", {}, "Kind"), h("select", { id: "dr-kind" }, ...Object.entries(D.data.kinds).map(([k, l]) =>
            h("option", { value: k, selected: (s?.kind || "pipeline") === k }, l)))]),
          ...t.fields.flatMap((f) => {
            const v = p[f.key] ?? f.default, id = "dr-f-" + f.key;
            const inp = typeof f.default === "boolean" ? h("input", { type: "checkbox", id, checked: !!v })
              : f.choices.length ? h("select", { id }, ...f.choices.map(([cv, cl]) => h("option", { value: cv, selected: cv === v }, cl)))
                : h("input", { type: "text", id, value: v });
            return [h("label", {}, f.label), inp, f.hint ? h("div", { class: "muted small span2" }, f.hint) : ""];
          })),
        others.length ? h("div", {}, h("div", { class: "section" }, "3 · Start after (needs)"),
          h("div", { class: "row wrap" }, ...others.map((o) => h("label", { class: "chk" },
            h("input", { type: "checkbox", class: "dr-need", value: o.name, checked: (s?.needs || []).includes(o.name) }), o.name)))) : "",
        h("label", { class: "chk" }, h("input", { type: "checkbox", id: "dr-auto", checked: !!s?.autostart }), "start with the GUI"));
    }
    kids.push(h("div", { class: "row" }, t ? lease(btn("Save", "primary", () => saveEdit())) : "",
      btn("Cancel", "quiet", () => { D.edit = null; fillRight(); })));
    right.replaceChildren(...kids.filter(Boolean));
    show(null);
  }
  async function saveEdit() {
    const E = D.edit, t = D.data.types.find((x) => x.type === E.type);
    const val = (id) => document.getElementById(id);
    const params = Object.fromEntries(t.fields.map((f) => [f.key, typeof f.default === "boolean" ? val("dr-f-" + f.key).checked : val("dr-f-" + f.key).value]));
    const service = { name: val("dr-name").value.trim(), type: t.type, params, autostart: val("dr-auto").checked,
      kind: val("dr-kind")?.value, needs: [...right.querySelectorAll(".dr-need")].filter((c) => c.checked).map((c) => c.value) };
    const r = await cmd("drivers_save", { service, replace: E.replace });
    if (r.ok) { D.edit = null; D.sel = service.name; D.topic = null; took(r); scan(); }
  }

  // ------------------------------------------------------------------ live
  function update(s) {
    const st = (s.workcell?.drivers || []).map((d) => d.name + d.state + d.msg).join("|");
    if (st !== D.sig) { D.sig = st; load(); }
    else if (!D.edit && D.data) {                          // uptime and topics tick on
      if (Date.now() - (D.viewT || 0) > 3000) { D.viewT = Date.now(); load(); }
    }
    const c = s.camera, t = D.shown;
    info.textContent = c && t && c.topic === t
      ? (c.width ? `${c.width}×${c.height} · ` : "") + `${c.encoding || "—"} · ${c.rate != null ? c.rate.toFixed(1) : "—"} Hz` +
        (c.age != null && c.age > 2 ? ` · last picture ${c.age.toFixed(0)} s ago` : "") + (c.error ? " · " + c.error : "") : "";
    log(false);
  }
  function leave() { D.shown = null; preview.replaceChildren(); }

  return { view: h("div", { class: "cols" }, left, right), load, update, leave };
}

// the SETUP nav badge: "2/3" running, warn when one failed or has no data
export function driversBadge(rows) {
  if (!rows || !rows.length) return null;
  const on = rows.filter((r) => r.state === "running").length;
  const bad = rows.filter((r) => r.state === "failed" || r.state === "no_data");
  return [`${on}/${rows.length}`, bad.length ? "warn" : on ? "ok" : "",
    bad.length ? `problem: ${bad.map((r) => r.name).join(", ")}` : `${on} of ${rows.length} drivers running`];
}
