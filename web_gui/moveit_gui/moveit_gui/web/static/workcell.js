// SETUP > Parts, Camera calibration, Setups (and the restart buttons). Built by app.js with its helpers (ctx), so
// this file has no state of its own beyond the views. The server owns every rule; this only shows and asks.

export function workcell(ctx) {
  const { h, btn, note, cmd, toast, lease } = ctx;
  // the views are built once and only one is in the page at a time: look in the page, then in the hidden views
  const roots = [];
  const $ = (sel) => document.querySelector(sel) || roots.map((r) => r.querySelector(sel)).find(Boolean) || null;
  const num = (id, value, step = "1", w = "76px") => h("input", { type: "number", id, value, step, style: `width:${w}` });
  const v3 = (id, vals, step) => h("div", { class: "row" }, vals.map((v, i) => num(`${id}${i}`, v, step)));
  const read3 = (id) => [0, 1, 2].map((i) => +$(`#${id}${i}`).value || 0);
  const set3 = (id, v) => [0, 1, 2].forEach((i) => ($(`#${id}${i}`).value = +(+v[i]).toFixed(3)));
  const chip = (text, cls, title = "") => h("span", { class: "chip " + cls, title }, text);
  const STATUS = {
    live: ["live", "warn", "In TF, MoveIt's scene and the 3D views now. Restart the robot stack to put it in the robot model (RViz, recordings, self-collision)."],
    in_model: ["in model", "ok", "The running robot model has this part, as saved."],
    changed: ["restart", "warn", "Changed since the robot model was built: the robot model still has the old one until the stack restarts."],
    removed: ["removed", "warn", "Deleted, but the running robot model still has it until the stack restarts."],
  };
  const W = { parts: null, he: null, setups: null };

  // ================================================================ Parts
  function shapeText(p) {
    const s = p.shape;
    if (s.type === "box") return `box ${s.size_mm.map((x) => +x.toFixed(1)).join("×")} mm`;
    if (s.type === "cylinder") return `cylinder r${+s.radius_mm.toFixed(1)} × ${+s.length_mm.toFixed(1)} mm`;
    if (s.type === "sphere") return `sphere r${+s.radius_mm.toFixed(1)} mm`;
    if (s.type === "mesh") return `mesh ${s.file}`;
    return "frame only";
  }
  const COLL = { visual: "collides as shown", box: "collides as its bounding box", hull: "collides as its convex hull",
    none: "no collision (shown only)" };

  function buildParts(onChange) {
    const P = { editing: null, data: null };
    const shapeSel = h("select", { id: "pt-shape", onchange: () => shapeFields() },
      ["box", "cylinder", "sphere", "mesh", "none"].map((t) => h("option", { value: t }, t === "none" ? "none (frame only)" : t)));
    const fields = h("div", { class: "row wrap", id: "pt-dims" });
    const meshSel = h("select", { id: "pt-mesh", style: "min-width:140px" });
    const fileIn = h("input", { type: "file", accept: ".stl,.STL,.obj,.OBJ,.dae,.DAE", style: "display:none", onchange: (e) => upload(e.target.files[0]) });
    const isDae = (f) => /\.dae$/i.test(f || "");
    // where the mesh's own origin goes in the part frame: as exported, centred on the frame, or standing on it
    function meshPlace(how) {
      const i = P.data?.mesh_info?.[meshSel.value], k = +$("#pt-scale").value * 1000;
      if (!i || !i.min) return toast("the mesh cannot be read", false);
      const c = [0, 1, 2].map((a) => -(i.min[a] + i.max[a]) / 2 * k);
      if (how === "base") c[2] = -i.min[2] * k;
      set3("pt-moff", how === "none" ? [0, 0, 0] : c.map((x) => +x.toFixed(2)));
    }
    function meshSize() {
      const i = P.data?.mesh_info?.[meshSel.value], el = $("#pt-msize");
      if (!el) return;
      const sc = $("#pt-scale");
      if (sc) { sc.disabled = isDae(meshSel.value); if (sc.disabled) sc.value = "1"; }
      el.textContent = i && i.size ? `${i.size.map((x) => +(x * +$("#pt-scale").value * 1000).toFixed(1)).join(" × ")} mm` +
        (isDae(meshSel.value) ? " (the .dae's own unit)" : "") : "";
    }
    function shapeFields(sh) {
      const t = shapeSel.value, s = sh || {};
      const kids = [];
      if (t === "box") kids.push(h("span", { class: "k" }, "size X Y Z mm"), ...[0, 1, 2].map((i) => num(`pt-d${i}`, (s.size_mm || [40, 40, 20])[i], "0.5")));
      if (t === "cylinder") kids.push(h("span", { class: "k" }, "radius"), num("pt-d0", s.radius_mm ?? 20, "0.5"), h("span", { class: "k" }, "length mm"), num("pt-d1", s.length_mm ?? 50, "0.5"));
      if (t === "sphere") kids.push(h("span", { class: "k" }, "radius mm"), num("pt-d0", s.radius_mm ?? 15, "0.5"));
      if (t === "mesh") kids.push(meshSel, h("span", { class: "k" }, "units"),
        h("select", { id: "pt-scale", onchange: () => meshSize(), title: "The unit the file was exported in (a .dae carries its own)" },
          h("option", { value: "0.001" }, "mm"), h("option", { value: "1" }, "m"), h("option", { value: "0.0254" }, "inch")),
        btn("Upload…", "quiet", () => fileIn.click(), "An .stl, .obj or .dae exported from CAD (up to 20 MB). A .dae keeps its colours."),
        h("span", { class: "muted small", id: "pt-msize", style: "width:100%" }),
        h("span", { class: "k" }, "mesh origin at X Y Z mm"), v3("pt-moff", s.offset_mm || [0, 0, 0], "0.5"),
        h("div", { class: "row", style: "width:100%;gap:2px" }, h("span", { class: "muted small" }, "place it:"),
          btn("as exported", "quiet", () => meshPlace("none"), "The mesh's own origin on the part frame"),
          btn("centred", "quiet", () => meshPlace("center"), "Its bounding box centred on the part frame"),
          btn("standing on the frame", "quiet", () => meshPlace("base"), "Centred in X / Y, its lowest point on the frame (stacks like the other shapes)")));
      fields.replaceChildren(...kids);
      if (t === "mesh") {
        meshSel.replaceChildren(...(P.data?.meshes || []).map((m) => h("option", { value: m }, m)));
        meshSel.onchange = () => meshSize();
        if (s.file) meshSel.value = s.file;
        if (s.scale) fields.querySelector("#pt-scale").value = String(s.scale);
        meshSize();
      }
    }
    async function upload(f) {
      if (!f) return;
      if (f.size > 20 * 1024 * 1024) return toast("the mesh is over 20 MB: simplify it", false);
      const b64 = await new Promise((res) => { const r = new FileReader(); r.onload = () => res(r.result.split(",")[1]); r.readAsDataURL(f); });
      const r = await ctx.post("/api/upload", { filename: f.name, data_b64: b64 });
      fileIn.value = "";
      if (!r.ok) return;
      fill(r.parts);
      const i = r.mesh || {};
      shapeSel.value = "mesh"; shapeFields({ file: r.file, scale: i.scale });
      if ($("#pt-coll").value === "visual") $("#pt-coll").value = "hull";
      toast(i.unit === "m" ? `${r.file}: ${i.size.map((x) => +(x * 1000).toFixed(1)).join(" × ")} mm` :
        `${r.file}: ${r.size_file_units.join(" × ")} in file units, read as ${i.hint === "m" ? "metres" : "mm"}: check the size shown`, true);
    }
    const parentSel = h("select", { id: "pt-parent", style: "min-width:180px", title: "What it is bolted to: a link of the robot, or another part" });
    const presetSel = h("select", { id: "pt-preset", onchange: () => { const k = presetSel.value; presetSel.value = ""; if (k) startFrom(P.data.presets[k], k); } });
    const card = h("div", { class: "card su-main" },
      h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "Parts"),
        btn("+ New part", "quiet", () => startFrom({ parent: "tool0", shape: { type: "box", size_mm: [40, 40, 20] } }))),
      note("Things bolted to the robot or the cell, built piece on piece like the real tool. MoveIt plans around them at once.",
        "A part is a box, cylinder, sphere or mesh (.stl, .obj, .dae) on a parent (a robot link or another part), at an offset in mm / deg. " +
        "A shape sits on its frame and grows along +Z, so a part put on another one starts on its far face. Saved parts are " +
        "live at once: a TF frame, a MoveIt collision object (carried by the arm link they ride on) and the 3D views. " +
        "Restart the robot stack to write them into the robot model too (URDF / SRDF: RViz, recordings, self-collision). " +
        "Changing the tool changes the payload: redo SETUP › F/T + Payload."),
      h("table", { class: "su-ptable" }, h("thead", {}, h("tr", {}, ["Part", "On", "Shape", "Offset mm, °", "", ""].map((x) => h("th", {}, x)))),
        h("tbody", { id: "pt-list" })),
      h("div", { class: "muted small", id: "pt-sync" }),
      h("div", { class: "banner-warn row hidden", id: "pt-restart" },
        h("span", { class: "grow", id: "pt-restart-msg" }),
        lease(btn("Restart robot stack…", "", () => restart("stack")))),
      h("div", { class: "error", id: "pt-problems" }));
    const editor = h("div", { class: "card", style: "width:430px" },
      h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0", id: "pt-title" }, "New part"),
        h("span", { class: "muted small" }, "Start from"), presetSel),
      h("div", { class: "grid", style: "grid-template-columns:auto 1fr" },
        h("span", { class: "k" }, "Name"), h("input", { type: "text", id: "pt-name", placeholder: "spatula_holder" }),
        h("span", { class: "k" }, "On"), h("div", { class: "row" }, parentSel,
          btn("on far face", "quiet", () => { const p = P.data.parts.find((x) => x.name === parentSel.value); set3("pt-xyz", [0, 0, p ? p.far_face_mm : 0]); },
            "Put it on the far face of the parent part (where a stacked piece starts)")),
        h("span", { class: "k" }, "Offset X Y Z mm"), v3("pt-xyz", [0, 0, 0], "0.5"),
        h("span", { class: "k" }, "Rotate R P Y °"), v3("pt-rpy", [0, 0, 0], "1"),
        h("span", { class: "k" }, "Shape"), shapeSel,
        h("span", {}), fields,
        h("span", { class: "k" }, "Shape sits"), h("select", { id: "pt-anchor" }, h("option", { value: "base" }, "on the frame, along +Z"), h("option", { value: "center" }, "centred on the frame")),
        h("span", { class: "k" }, "Colour"), h("div", { class: "row" }, h("input", { type: "color", id: "pt-color", value: "#9aa0a6" }),
          h("label", { class: "chk", title: "A camera's frame (made by Camera calibration)" }, h("input", { type: "checkbox", id: "pt-cam" }), "camera")),
        h("span", { class: "k" }, "Collision"), h("select", { id: "pt-coll", title: "What MoveIt avoids (and carries with the arm). The 3D views show it with Collision ticked." },
          h("option", { value: "visual" }, "as shown"), h("option", { value: "hull" }, "convex hull (light; meshes)"),
          h("option", { value: "box" }, "bounding box"), h("option", { value: "none" }, "none (shown only)")),
        h("span", { class: "k" }, "Note"), h("input", { type: "text", id: "pt-note", placeholder: "optional" })),
      h("div", { class: "row" }, lease(btn("Save part", "primary", () => save())),
        lease(btn("TCP at its tip…", "", () => makeTcp(), "A virtual TCP at the far face of this part (it follows the part when you edit it)")),
        h("div", { class: "grow" }), h("span", { class: "muted small", id: "pt-hint" })),
      fileIn);
    shapeFields();

    function startFrom(d, label) {
      P.editing = null;
      $("#pt-title").textContent = label ? `New part (${label})` : "New part";
      $("#pt-name").value = "";
      parentSel.value = d.parent || "tool0";
      set3("pt-xyz", d.xyz_mm || [0, 0, 0]); set3("pt-rpy", d.rpy_deg || [0, 0, 0]);
      shapeSel.value = d.shape?.type || "none"; shapeFields(d.shape);
      $("#pt-anchor").value = d.anchor || "base"; $("#pt-color").value = d.color || "#9aa0a6";
      $("#pt-coll").value = d.collision === false ? "none" : d.collision === true || !d.collision ?
        (d.shape?.type === "mesh" ? "hull" : "visual") : d.collision;
      $("#pt-cam").checked = d.kind === "camera"; $("#pt-note").value = d.note || "";
      $("#pt-name").focus();
    }
    function edit(p) {
      startFrom(p);
      P.editing = p.name;
      $("#pt-title").textContent = `Edit ${p.name}`;
      $("#pt-name").value = p.name;
    }
    function shape() {
      const t = shapeSel.value, d = (i) => +$(`#pt-d${i}`).value;
      if (t === "box") return { type: t, size_mm: [d(0), d(1), d(2)] };
      if (t === "cylinder") return { type: t, radius_mm: d(0), length_mm: d(1) };
      if (t === "sphere") return { type: t, radius_mm: d(0) };
      if (t === "mesh") return { type: t, file: meshSel.value, scale: +$("#pt-scale").value, offset_mm: read3("pt-moff") };
      return { type: "none" };
    }
    async function save() {
      const name = $("#pt-name").value.trim();
      const r = await cmd("parts_save", { name, parent: parentSel.value, xyz_mm: read3("pt-xyz"), rpy_deg: read3("pt-rpy"),
        shape: shape(), anchor: $("#pt-anchor").value, color: $("#pt-color").value, collision: $("#pt-coll").value,
        kind: $("#pt-cam").checked ? "camera" : "part", note: $("#pt-note").value, replace: P.editing });
      if (r.ok) { fill(r.parts); P.editing = name; $("#pt-title").textContent = `Edit ${name}`; onChange(); }
    }
    async function del(p) {
      if (!window.confirm(`Delete part "${p.name}"?`)) return;
      const r = await cmd("parts_delete", { name: p.name });
      if (r.ok) { fill(r.parts); if (P.editing === p.name) startFrom({}); onChange(); }
    }
    async function makeTcp() {
      const n = P.editing || $("#pt-name").value.trim();
      if (!n) return toast("save the part first", false);
      const t = window.prompt(`Name of the TCP at the far face of "${n}" (it follows the part when you edit it):`, `${n}_tip`);
      if (t) cmd("parts_make_tcp", { name: n, tcp: t });
    }
    function fill(d) {
      P.data = d;
      const groups = [["Robot (moves with the arm)", d.arm_links], ["Robot (fixed)", d.links.filter((l) => !d.arm_links.includes(l))],
        ["Parts", d.parts.map((p) => p.name)]];
      const keep = parentSel.value;
      parentSel.replaceChildren(...groups.filter(([, ls]) => ls.length).map(([g, ls]) => h("optgroup", { label: g }, ls.map((l) => h("option", { value: l }, l)))));
      if (keep) parentSel.value = keep;
      presetSel.replaceChildren(h("option", { value: "" }, "—"), ...Object.keys(d.presets).map((k) => h("option", { value: k }, k)));
      if (shapeSel.value === "mesh") { const m = meshSel.value; shapeFields({ file: m, scale: +$("#pt-scale").value, offset_mm: read3("pt-moff") }); }
      $("#pt-list").replaceChildren(...(d.parts.length ? d.parts.map((p) => {
        const [lbl, cls, tip] = STATUS[p.status] || [p.status, "", ""];
        return h("tr", { class: P.editing === p.name ? "on" : "" },
          h("td", {}, h("span", { class: "dot", style: `background:${p.color}` }), " ", p.name, p.kind === "camera" ? " 📷" : ""),
          h("td", { class: "mono" }, p.parent), h("td", { title: COLL[p.collision] || "" }, shapeText(p),
            p.collision && p.collision !== "visual" && p.shape.type !== "none" ? h("span", { class: "muted small" }, ` · ${p.collision === "none" ? "no collision" : p.collision}`) : ""),
          h("td", { class: "mono small", style: "white-space:nowrap" }, `${p.xyz_mm.map((x) => +x.toFixed(1)).join(", ")}`, h("br"), `${p.rpy_deg.map((x) => +x.toFixed(1)).join(", ")}°`),
          h("td", {}, chip(lbl, cls, tip), p.meta?.source === "handeye" ? chip("calibrated", "ok", p.note || "") : ""),
          h("td", {}, h("div", { class: "row", style: "flex-wrap:nowrap;gap:2px" }, btn("Edit", "quiet", () => edit(p)),
            lease(btn("+", "quiet", () => startFrom({ parent: p.name, xyz_mm: [0, 0, p.far_face_mm], shape: { type: "box", size_mm: [30, 30, 20] } }),
              "Add a part stacked on the far face of this one")),
            lease(btn("✕", "quiet", () => del(p), "Delete")))));
      }) : [h("tr", {}, h("td", { colspan: 6, class: "muted" }, "No parts yet. + New part, or start from a preset."))]));
      d.removed.forEach((n) => $("#pt-list").append(h("tr", {}, h("td", { class: "muted" }, n), h("td", { colspan: 3, class: "muted" }, "deleted"), h("td", {}, chip(...STATUS.removed.slice(0, 2), STATUS.removed[2])), h("td"))));
      $("#pt-problems").textContent = [d.error, ...d.problems, d.model_ok ? "" : "Robot model: " + d.model_msg].filter(Boolean).join(" · ");
      sync(d.sync, d.restart_for);
    }
    function sync(sy, rf) {
      $("#pt-sync").textContent = sy ? `MoveIt / TF: ${sy.msg}` : "";
      $("#pt-sync").classList.toggle("error", sy && /error|unverified/.test(sy.state));
      $("#pt-restart").classList.toggle("hidden", !rf || !rf.length);
      if (rf && rf.length) $("#pt-restart-msg").textContent = `Not in the robot model yet: ${rf.join(", ")}. They work now (TF, MoveIt, 3D view); ` +
        "a robot stack restart writes them into the robot model (RViz, recordings, MoveIt self-collision).";
    }
    roots.push(card, editor);
    W.parts = { card, editor, load: () => cmd("parts_get").then((r) => r.ok && fill(r.parts)), fill,
      update: (s) => { const w = s.workcell; if (w) sync(w.parts_sync, w.restart_for); } };
    return W.parts;
  }

  // ================================================================ Restart
  async function restart(what) {
    const stack = what === "stack";
    const msg = stack
      ? "Restart the robot stack and the GUI?\n\nThe driver, controllers, MoveIt and the GUI stop and start again (about a minute), " +
        "with the parts in the robot model. The arm stays where it is (stopped, powered). On the pendant, press Play on the " +
        "External Control program again afterwards (unless headless). Every browser must take control again."
      : "Restart the GUI?\n\nThe robot stack keeps running. Every browser must take control again.";
    if (!window.confirm(msg)) return;
    const r = await cmd("restart", { what });
    if (r.ok) toast(r.msg, true, true);
  }

  // ================================================================ Camera calibration
  function buildCalib() {
    const C = { data: null, jpeg: null };
    const camSel = h("select", { id: "he-cam", style: "min-width:200px" });
    const mount = h("select", { id: "he-mount" }, h("option", { value: "eye_in_hand" }, "on the robot's tool (eye-in-hand)"),
      h("option", { value: "eye_to_hand" }, "fixed in the cell (eye-to-hand)"));
    const bf = (id, label, step) => [h("span", { class: "k" }, label), num(id, "", step, "70px")];
    const img = h("img", { id: "he-img", style: "max-width:100%;display:none;border-radius:6px" });
    const left = h("div", { class: "col su-main" },
      h("div", { class: "card" }, h("div", { class: "section" }, "1 · Camera and board"),
        note("Hand-eye calibration: where the camera is on the robot (or in the cell), from 10-15 views of a ChArUco board.",
          "Eye-in-hand: the board lies still in the cell, the camera on the tool looks at it from different angles. Eye-to-hand: " +
          "the board is fixed on the tool, the camera on its stand. The intrinsics come from the camera's own CameraInfo " +
          "(RealSense / ZED are factory calibrated). OpenCV solves it with Park's method; four other methods are shown as a cross-check. " +
          "The result becomes a camera part, so the camera's frames hang on the robot in TF (and in the robot model after a stack restart)."),
        h("div", { class: "grid", style: "grid-template-columns:auto 1fr" },
          h("span", { class: "k" }, "Camera"), h("div", { class: "row" }, camSel, h("input", { type: "text", id: "he-topic", placeholder: "or an image topic", style: "flex:1" })),
          h("span", { class: "k" }, "Mounted"), mount),
        h("div", { class: "row" }, lease(btn("Start", "primary", () => start(), "Read the camera's intrinsics and begin (clears the samples)")),
          h("span", { class: "muted small grow", id: "he-intr" })),
        h("div", { class: "section" }, "Board (ChArUco)"),
        h("div", { class: "row wrap" }, ...bf("he-bx", "squares X", "1"), ...bf("he-by", "Y", "1"), ...bf("he-bs", "square mm", "0.1"), ...bf("he-bm", "marker mm", "0.1"),
          h("select", { id: "he-bd" }, ["DICT_4X4_50", "DICT_5X5_100", "DICT_6X6_250"].map((d) => h("option", { value: d }, d)))),
        h("div", { class: "row" }, lease(btn("Save board", "quiet", () => saveBoard(), "After printing: enter the square size you MEASURE on the print")),
          btn("Print board…", "", () => printBoard(), "Opens the board at its true size: print at 100 % (no fit to page) and glue it flat on something rigid"))),
      h("div", { class: "card" }, h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "2 · Samples"), ctx.freedriveButton()),
        h("div", { class: "row wrap" },
          btn("Look", "quiet", () => look(), "Grab a frame and show what the board detector sees (records nothing)"),
          lease(btn("Capture here", "", () => capture(), "Take a sample where the arm is now (freedrive or jog between samples)")),
          h("span", { class: "muted small" }, "or"), h("span", { class: "k" }, "tilt °"), num("he-tilt", 20, "1", "56px"),
          lease(btn("Plan 12 poses", "", () => plan(), "Poses around this one, turned about the board (eye-in-hand) or the tool tip (eye-to-hand). Nothing moves yet")),
          lease(btn("Collect all…", "primary", () => collect(), "Move to each planned pose (MoveIt, collision-aware) and sample there"))),
        h("div", { class: "muted small", id: "he-status" }), img,
        h("div", { class: "row wrap", id: "he-samples" }),
        h("div", { class: "row" }, lease(btn("Clear samples", "quiet", () => cmd("handeye_drop").then((r) => r.ok && fill(r.he)))))));
    const right = h("div", { class: "card", style: "width:380px" },
      h("div", { class: "section" }, "3 · Result"),
      h("div", { class: "row" }, btn("Solve", "primary", () => cmd("handeye_solve").then((r) => r.ok && fill(r.he)))),
      h("div", { id: "he-result" }),
      h("div", { class: "row" }, h("input", { type: "text", id: "he-name", placeholder: "camera name, e.g. wrist_rs", style: "flex:1" }),
        lease(btn("Save + add to robot", "primary", () => saveCam(), "Keep the calibration and put the camera's frame on the robot as a part"))),
      h("div", { class: "section" }, "Saved calibrations"), h("div", { class: "list", id: "he-saved" }));

    async function start() {
      const opt = C.data?.cameras.find((c) => c.topic === camSel.value);
      const topic = $("#he-topic").value.trim() || camSel.value;
      const r = await cmd("handeye_start", { topic, info_topic: $("#he-topic").value.trim() ? "" : (opt?.info || ""), mount: mount.value });
      if (r.ok) fill(r.he);
    }
    async function saveBoard() {
      const r = await cmd("handeye_board", { squares_x: +$("#he-bx").value, squares_y: +$("#he-by").value, square_mm: +$("#he-bs").value,
        marker_mm: +$("#he-bm").value, dict: $("#he-bd").value });
      if (r.ok) fill(r.he);
    }
    async function printBoard() {
      const r = await cmd("handeye_board_svg");
      if (!r.ok) return;
      const url = URL.createObjectURL(new Blob([r.svg], { type: "image/svg+xml" }));
      window.open(url, "_blank");
    }
    function shot(r) {
      if (r && r.jpeg) { img.src = "data:image/jpeg;base64," + r.jpeg; img.style.display = ""; }
    }
    async function look() { const r = await cmd("handeye_look"); shot(r); if (r.ok || r.detection) status(r.detection); }
    async function capture() { const r = await cmd("handeye_capture"); shot(r); if (r.ok) fill(r.he); }
    async function plan() { const r = await cmd("handeye_plan", { tilt_deg: +$("#he-tilt").value }); if (r.ok) fill(r.he); }
    async function collect() {
      if (!ctx.fake() && !window.confirm(`Move through ${C.data?.poses || 0} poses and sample at each?\n\nThe arm turns up to ` +
        `±${$("#he-tilt").value}° about the board (or the tool tip). MoveIt plans each move around the keep-out zones and parts, ` +
        "but check the space around the arm. Keep a hand near STOP.")) return;
      cmd("handeye_collect");
    }
    async function saveCam() {
      const n = $("#he-name").value.trim();
      if (!n) return toast("name the camera (e.g. wrist_rs)", false);
      const r = await cmd("handeye_save", { name: n });
      if (r.ok) { fill(r.he); W.parts?.fill(r.parts); }
    }
    function status(d) {
      if (!d) return;
      $("#he-status").textContent = d.ok ? `Board: ${d.corners} corners, ${d.reproj_px} px reprojection, ${d.distance_mm} mm away` : d.why;
      $("#he-status").classList.toggle("error", !d.ok);
    }
    function fill(d) {
      C.data = d;
      const keep = camSel.value;
      camSel.replaceChildren(...d.cameras.map((c) => h("option", { value: c.topic }, c.name)));
      if (keep) camSel.value = keep;
      if (d.topic && !d.cameras.some((c) => c.topic === d.topic)) $("#he-topic").value = d.topic;
      else if (d.topic) camSel.value = d.topic;
      mount.value = d.mount;
      const b = d.board;
      [["he-bx", b.squares_x], ["he-by", b.squares_y], ["he-bs", b.square_mm], ["he-bm", b.marker_mm], ["he-bd", b.dict]].forEach(([id, v]) => {
        if (document.activeElement?.id !== id) $("#" + id).value = v; });
      const i = d.intrinsics;
      $("#he-intr").textContent = i ? `${d.topic} · frame ${d.frame} · fx ${i.fx.toFixed(1)} fy ${i.fy.toFixed(1)} · cx ${i.cx.toFixed(1)} cy ${i.cy.toFixed(1)}` +
        (i.D.some((x) => Math.abs(x) > 1e-9) ? " · distortion " + i.D.map((x) => x.toFixed(3)).join(" ") : " · no distortion") : "Not started";
      status(d.last);
      $("#he-samples").replaceChildren(...d.samples.map((s, k) => h("span", { class: "chip", title: `${s.corners} corners, ${s.reproj_px} px` },
        `#${k + 1} ${s.corners}c`, h("a", { class: "x", title: "Remove this sample", onclick: () => cmd("handeye_drop", { index: k }).then((r) => r.ok && fill(r.he)) }, " ×"))),
        h("span", { class: "muted small" }, `${d.samples.length} sample(s)` + (d.poses ? ` · ${d.poses} poses planned` : "") +
          (d.samples.length < d.min_samples ? ` · at least ${d.min_samples}, 12+ is better` : "")));
      const r = d.result;
      $("#he-result").replaceChildren(...(r ? [
        h("div", { class: "row" }, chip(r.verdict.toUpperCase(), r.verdict === "good" ? "ok" : r.verdict === "usable" ? "warn" : "err"),
          h("span", { class: "small" }, `the ${r.n} samples put the board within ${r.rms_mm} mm / ${r.rms_deg}° (rms)`)),
        h("div", { class: "grid small", style: "grid-template-columns:auto 1fr" },
          h("span", { class: "k" }, r.mount === "eye_in_hand" ? `${d.flange} → camera` : `${d.base} → camera`),
          h("span", { class: "mono" }, `${r.xyz_mm.join(", ")} mm`), h("span", { class: "k" }, "R P Y"), h("span", { class: "mono" }, `${r.rpy_deg.join(", ")} °`),
          h("span", { class: "k" }, "other methods"), h("span", { class: "mono" }, Object.entries(r.methods).map(([k, v]) => `${k} ${v.mm} mm / ${v.deg}°`).join(" · ")),
          h("span", { class: "k" }, "orientation spread"), h("span", {}, `${r.orientation_spread_deg}°`)),
        r.per_sample.some((p) => p.mm > 3 * Math.max(r.rms_mm, 0.5)) ? h("div", { class: "error small" }, "Outliers: " + r.per_sample.map((p, k) => p.mm > 3 * Math.max(r.rms_mm, 0.5) ? `#${k + 1} (${p.mm} mm)` : "").filter(Boolean).join(", ") + ": remove them and solve again") : "",
        note(r.verdict === "good" ? "Good: a few mm and tenths of a degree is what the robot and a printed board allow." :
          "More varied tilts, more samples, a flatter / better measured board, and checking the square size usually help.")]
        : [h("div", { class: "muted small" }, d.samples.length >= d.min_samples ? "Press Solve." : "Collect samples first.")]));
      $("#he-saved").replaceChildren(...(d.saved.length ? d.saved.map((c) => h("div", { class: "item" },
        h("div", { class: "grow" }, h("div", {}, `${c.name} · ${c.mount.replace(/_/g, "-")}`),
          h("div", { class: "muted small" }, `${c.verdict} · ${c.rms_mm} mm / ${c.rms_deg}° · ${c.samples} samples · ${c.saved}`),
          h("div", { class: "mono small" }, `${c.parent} → ${c.optical_frame}: ${c.xyz_mm.join(", ")} mm`)),
        lease(btn("Add to robot", "quiet", () => cmd("handeye_apply", { name: c.name }).then((r) => r.ok && W.parts?.fill(r.parts)))),
        lease(btn("✕", "quiet", () => window.confirm(`Delete calibration ${c.name}?`) && cmd("handeye_delete", { name: c.name }).then((r) => r.ok && fill(r.he))))))
        : [h("div", { class: "muted small" }, "None yet.")]));
    }
    roots.push(left, right);
    W.he = { left, right, load: () => cmd("handeye_get").then((r) => r.ok && fill(r.he)), fill,
      update: (s) => {
        const p = s.workcell?.he;
        if (!p) return;
        const key = JSON.stringify(p);
        if (key !== C.key) { C.key = key; W.he.load(); }
        if (p.running) $("#he-status").textContent = `Pose ${(p.pose ?? 0) + 1} of ${p.total} · ${p.got ?? 0} sampled`;
      } };
    return W.he;
  }

  // ================================================================ Setups
  function buildSetups() {
    const list = h("div", { class: "list", id: "st-list" }), autos = h("div", { class: "list", id: "st-auto" });
    const card = h("div", { class: "card su-main" },
      h("div", { class: "section" }, "Setups"),
      note("One physical configuration of the robot, kept by name: switch the whole set in one go.",
        "A setup holds the parts, TCPs and fixtures, keep-out zones, F/T bias, controller gains and payload, vial profiles, " +
        "camera calibrations and the last payload calibration. Programs and policies are shared by every setup. Loading one " +
        "restarts the GUI (and the robot stack when its parts differ from the running robot model); the files in use before " +
        "are kept as an automatic backup."),
      list,
      h("div", { class: "section" }, "Save the current configuration"),
      h("div", { class: "row" }, h("input", { type: "text", id: "st-name", placeholder: "e.g. scraping spatula", style: "width:180px" }),
        h("input", { type: "text", id: "st-note", placeholder: "note (what is mounted)", style: "flex:1" }),
        lease(btn("Save", "primary", () => save(false)))),
      h("details", {}, h("summary", { class: "muted small" }, "Automatic backups (before each load, newest 5)"), autos));
    const baseBox = h("div", { class: "list", id: "st-base" });
    const side = h("div", { class: "card", style: "width:320px" },
      h("div", { class: "section" }, "Base"),
      note("What is on the flange. Fixed parts of the robot model: parts go on top of them.",
        "Each base has its own URDF, SRDF (MoveIt's collision rules and groups) and controllers. Switching restarts the robot " +
        "stack. The usual way: build a tool on a base in SETUP › Parts, save it as a setup here, and switch by loading setups " +
        "(a setup puts its base back)."),
      baseBox,
      h("div", { class: "section" }, "Restart"),
      note("After changing parts, or when something is stuck."),
      h("div", { class: "row" }, lease(btn("Restart GUI…", "", () => restart("gui"), "The GUI only: the robot stack keeps running"))),
      h("div", { class: "row" }, lease(btn("Restart robot stack…", "", () => restart("stack"), "Driver, controllers, MoveIt and the GUI: puts the parts into the robot model"))),
      note("Refused while the robot moves, a program or policy runs, or a force is applied.",
        "The launcher (ur7e_gui.sh) does the restart; a GUI started by hand cannot restart itself."));
    async function save(replace) {
      const name = $("#st-name").value.trim();
      const r = await cmd("setup_save", { name, note: $("#st-note").value, replace });
      if (!r.ok && /exists/.test(r.msg || "") && window.confirm(`Replace setup "${name}" with the current configuration?`)) return save(true);
      if (r.ok) { fillBase(r.base); fill(r.setups); }
    }
    let B = null;
    const baseLabel = (n) => (B?.choices.find((c) => c.name === n) || { label: n || "?" }).label;
    function fillBase(b) {
      if (!b) return;
      B = b;
      baseBox.replaceChildren(...b.choices.map((c) => h("div", { class: "item" + (c.name === b.running ? " on" : "") },
        h("div", { class: "grow" }, h("div", {}, c.label, c.name === b.running ? h("span", { class: "chip ok" }, " running") : "",
          c.name === b.next && c.name !== b.running ? h("span", { class: "chip warn" }, " next start") : ""),
          h("div", { class: "muted small" }, c.text)),
        c.name === b.running ? "" : lease(btn("Switch…", "quiet", () => switchBase(c))))));
    }
    async function switchBase(c) {
      if (!window.confirm(`Switch the base to "${c.label}"?\n\n${c.text}.\n\nCheck first that this is what is really on the robot. ` +
        "The robot stack restarts with its URDF, SRDF and controllers. Your parts stay; parts on links this base does not have " +
        "are left out (SETUP › Parts lists them). If the active TCP is on the removed hardware, the GUI falls back to tool0.\n\n" +
        "Save the current configuration as a setup first if you want to come back to it.")) return;
      const r = await cmd("base_switch", { base: c.name, confirm: true });
      if (r.ok) toast(r.msg, true, true);
    }
    async function load(s) {
      const sw = s.base && B && s.base !== B.running ? `\n\nBase: ${baseLabel(B.running)} → ${baseLabel(s.base)} (check the hardware on the robot).` : "";
      if (!window.confirm(`Load setup "${s.name}"?\n\n${s.holds.join(", ")}${sw}\n\nThe GUI restarts (and the robot stack too if its parts ` +
        "or base differ from the running robot model). The current files are kept as an automatic backup. Afterwards: check the TCP, " +
        "and on the pendant press Play again if the stack restarted.")) return;
      const r = await cmd("setup_load", { name: s.name });
      if (r.ok) toast(r.msg, true, true);
    }
    const row = (s) => h("div", { class: "item" },
      h("div", { class: "grow" }, h("div", {}, s.name, s.base ? h("span", { class: "chip" + (B && s.base !== B.running ? " warn" : ""),
          title: B && s.base !== B.running ? "a different base from the running one: loading switches it" : "base" }, baseLabel(s.base)) : "",
          s.note ? h("span", { class: "muted" }, " · " + s.note) : ""),
        h("div", { class: "muted small" }, `${s.saved} · ${s.holds.join(", ") || "empty"}`), s.error ? h("div", { class: "error small" }, s.error) : ""),
      lease(btn("Load…", "", () => load(s))),
      lease(btn("✕", "quiet", () => window.confirm(`Delete setup "${s.name}"?`) && cmd("setup_delete", { name: s.name }).then((r) => r.ok && fill(r.setups)))));
    function fill(all) {
      const mine = all.filter((s) => !s.auto), auto = all.filter((s) => s.auto);
      list.replaceChildren(...(mine.length ? mine.map(row) : [h("div", { class: "muted small" }, "No setups saved yet.")]));
      autos.replaceChildren(...(auto.length ? auto.map(row) : [h("div", { class: "muted small" }, "None.")]));
    }
    roots.push(card, side);
    W.setups = { card, side, load: () => cmd("setup_list").then((r) => { if (r.ok) { fillBase(r.base); fill(r.setups); } }) };
    return W.setups;
  }

  return { buildParts, buildCalib, buildSetups, restart, W };
}
