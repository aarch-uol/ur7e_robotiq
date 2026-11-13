// SETUP > Parts, Camera calibration, Setups (and the restart buttons). Built by app.js with its helpers (ctx), so
// this file has no state of its own beyond the views. The server owns every rule; this only shows and asks.

export function meshPartDefaults(file, parent, existing = [], scale = 0.001) {
  let name = String(file || "mesh").replace(/\.[^.]+$/, "").toLowerCase().replace(/[^a-z0-9]+/g, "_")
    .replace(/^_+|_+$/g, "");
  if (!/^[a-z]/.test(name)) name = "part_" + name;
  name = (name || "mesh").slice(0, 40).replace(/_+$/g, "") || "mesh";
  const used = new Set(existing);
  if (used.has(name)) {
    const root = name.slice(0, 37).replace(/_+$/g, "");
    let n = 2;
    while (used.has(`${root}_${n}`)) n++;
    name = `${root}_${n}`;
  }
  return { name, parent: parent || "tool0", xyz_mm: [0, 0, 0], rpy_deg: [0, 0, 0],
    shape: { type: "mesh", file, scale }, anchor: "base", collision: "auto",
    collision_settings: { max_pieces: 8, max_triangles: 64, tolerance_mm: 0.5, concavity: 1.35 } };
}

export function workcell(ctx) {
  const { h, btn, note, cmd, toast, lease, ask, askText, navTo } = ctx;
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
  const COLL = { auto: "automatic light convex pieces (recommended for meshes)", visual: "uses the full visual shape",
    box: "uses one bounding box", hull: "uses one convex hull (may fill concave gaps)", none: "no collision (shown only)" };

  function buildParts(onChange) {
    const P = { editing: null, data: null, drafting: false, startPose: null, lastParent: "tool0", importing: false,
      meshRev: 0, collisionPreview: [], collisionInfo: null, previewSeq: 0 };
    const shapeSel = h("select", { id: "pt-shape", onchange: () => {
      shapeFields();
      if (shapeSel.value === "mesh" && $("#pt-coll").value === "visual") $("#pt-coll").value = "auto";
      collisionPreview();
    } },
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
      draftChanged();
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
        h("select", { id: "pt-scale", onchange: () => { meshSize(); collisionPreview(); }, title: "The unit the file was exported in (a .dae carries its own)" },
          h("option", { value: "0.001" }, "mm"), h("option", { value: "1" }, "m"), h("option", { value: "0.0254" }, "inch")),
        btn("Upload…", "quiet", () => { P.importing = false; fileIn.click(); },
          "Replace/select the mesh for this draft. An .stl, .obj or .dae exported from CAD (up to 20 MB)."),
        h("span", { class: "muted small", id: "pt-msize", style: "width:100%" }),
        h("span", { class: "k" }, "mesh origin at X Y Z mm"), v3("pt-moff", s.offset_mm || [0, 0, 0], "0.5"),
        h("div", { class: "row", style: "width:100%;gap:2px" }, h("span", { class: "muted small" }, "place it:"),
          btn("as exported", "quiet", () => meshPlace("none"), "The mesh's own origin on the part frame"),
          btn("centred", "quiet", () => meshPlace("center"), "Its bounding box centred on the part frame"),
          btn("standing on the frame", "quiet", () => meshPlace("base"), "Centred in X / Y, its lowest point on the frame (stacks like the other shapes)")));
      fields.replaceChildren(...kids);
      if (t === "mesh") {
        meshSel.replaceChildren(...(P.data?.meshes || []).map((m) => h("option", { value: m }, m)));
        meshSel.onchange = () => { meshSize(); collisionPreview(); };
        if (s.file) meshSel.value = s.file;
        if (s.scale) fields.querySelector("#pt-scale").value = String(s.scale);
        meshSize();
      }
    }
    async function upload(f) {
      if (!f) return;
      const importing = P.importing;
      P.importing = false;
      if (f.size > 20 * 1024 * 1024) return toast("the mesh is over 20 MB: simplify it", false);
      const b64 = await new Promise((res) => { const r = new FileReader(); r.onload = () => res(r.result.split(",")[1]); r.readAsDataURL(f); });
      const r = await ctx.post("/api/upload", { filename: f.name, data_b64: b64 });
      fileIn.value = "";
      if (!r.ok) return;
      P.meshRev++;
      fill(r.parts);
      const i = r.mesh || {};
      if (importing) {
        const d = meshPartDefaults(r.file, P.lastParent || parentSel.value || "tool0",
          r.parts.parts.map((p) => p.name), i.scale);
        startFrom(d);
        meshPlace("base");
        $("#pt-hint").textContent = "Mesh uploaded: not saved yet. Place it and press Save part.";
        collisionPreview();
      } else {
        shapeSel.value = "mesh"; shapeFields({ file: r.file, scale: i.scale });
        if (!P.editing && ($("#pt-coll").value === "visual" || $("#pt-coll").value === "hull")) $("#pt-coll").value = "auto";
        draftChanged();
        collisionPreview();
      }
      toast(i.unit === "m" ? `${r.file}: ${i.size.map((x) => +(x * 1000).toFixed(1)).join(" × ")} mm` :
        `${r.file}: ${r.size_file_units.join(" × ")} in file units, read as ${i.hint === "m" ? "metres" : "mm"}: check the size shown`, true);
    }
    const parentSel = h("select", { id: "pt-parent", style: "min-width:180px", title: "What it is bolted to: a link of the robot, or another part" });
    const presetSel = h("select", { id: "pt-preset", onchange: () => { const k = presetSel.value; presetSel.value = ""; if (k) startFrom(P.data.presets[k], k); } });
    const card = h("div", { class: "card su-main" },
      h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "Parts"),
        btn("+ New part", "quiet", () => startFrom({ parent: P.lastParent || "tool0", shape: { type: "box", size_mm: [40, 40, 20] } })),
        btn("Import mesh…", "quiet", () => {
          if (P.drafting) P.lastParent = parentSel.value || P.lastParent;   // no draft yet: the select shows its first option
          P.importing = true;
          fileIn.click();
        }, "Upload a CAD mesh and start placing it immediately")),
      note("Things bolted to the robot or the cell, built piece on piece like the real tool. MoveIt plans around them at once.",
        "A part is a box, cylinder, sphere or mesh (.stl, .obj, .dae) on a parent (a robot link or another part), at an offset in mm / deg. " +
        "A shape sits on its frame and grows along +Z, so a part put on another one starts on its far face. Saved parts are " +
        "live at once: a TF frame, a MoveIt collision object (carried by the arm link they ride on) and the 3D views. " +
        "Restart the robot stack to write them into the robot model too (URDF / SRDF: RViz, recordings, self-collision). " +
        "Changing the tool changes the payload: redo SETUP › F/T + payload."),
      h("table", { class: "su-ptable" }, h("thead", {}, h("tr", {}, ["Part", "On", "Shape", "Offset mm, °", "", ""].map((x) => h("th", {}, x)))),
        h("tbody", { id: "pt-list" })),
      h("div", { class: "muted small", id: "pt-sync" }),
      h("div", { class: "error", id: "pt-problems" }));
    const editor = h("div", { class: "card", style: "width:430px" },
      h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0", id: "pt-title" }, "New part"),
        h("span", { class: "muted small" }, "Start from"), presetSel),
      h("div", { class: "row wrap", style: "margin-bottom:6px" },
        h("select", { id: "pt-gmode", title: "Same gizmo as OPERATE › Goal TCP" },
          h("option", { value: "translate" }, "Move"), h("option", { value: "rotate" }, "Rotate")),
        h("select", { id: "pt-gspace", title: "Axes of the part itself, or the frame it is mounted on" },
          h("option", { value: "local" }, "Local"), h("option", { value: "parent" }, "Parent")),
        h("select", { id: "pt-snap", title: "Translation and rotation snapping while dragging" },
          h("option", { value: "0" }, "Snap off"), h("option", { value: "1" }, "Snap 1 mm · 1°"),
          h("option", { value: "5" }, "Snap 5 mm · 5°")),
        btn("Reset", "quiet", () => resetPose(), "Return to the saved/start pose"),
        btn("Cancel", "quiet", () => cancelDraft(), "Discard this draft and show the saved part again")),
      h("div", { class: "grid", style: "grid-template-columns:auto 1fr" },
        h("span", { class: "k" }, "Name"), h("input", { type: "text", id: "pt-name", placeholder: "spatula_holder" }),
        h("span", { class: "k" }, "On"), h("div", { class: "row" }, parentSel,
          btn("on far face", "quiet", () => { const p = P.data.parts.find((x) => x.name === parentSel.value); set3("pt-xyz", [0, 0, p ? p.far_face_mm : 0]); draftChanged(); },
            "Put it on the far face of the parent part (where a stacked piece starts)")),
        h("span", { class: "k" }, "Offset X Y Z mm"), v3("pt-xyz", [0, 0, 0], "0.5"),
        h("span", { class: "k" }, "Rotate R P Y °"), v3("pt-rpy", [0, 0, 0], "1"),
        h("span", { class: "k" }, "Shape"), shapeSel,
        h("span", {}), fields,
        h("span", { class: "k" }, "Shape sits"), h("select", { id: "pt-anchor" }, h("option", { value: "base" }, "on the frame, along +Z"), h("option", { value: "center" }, "centred on the frame")),
        h("span", { class: "k" }, "Colour"), h("div", { class: "row" }, h("input", { type: "color", id: "pt-color", value: "#9aa0a6" }),
          h("label", { class: "chk", title: "A camera's frame (made by Camera calibration)" }, h("input", { type: "checkbox", id: "pt-cam" }), "camera")),
        h("span", { class: "k" }, "Collision"), h("select", { id: "pt-coll", onchange: () => collisionPreview(),
          title: "What MoveIt avoids (and carries with the arm). The 3D views show it with Collision ticked." },
          h("option", { value: "auto" }, "auto (recommended)"), h("option", { value: "visual" }, "full visual mesh"),
          h("option", { value: "hull" }, "single convex hull"), h("option", { value: "box" }, "bounding box"),
          h("option", { value: "none" }, "none (shown only)")),
        h("span", {}), h("div", {},
          h("div", { class: "muted small", id: "pt-cinfo" }, ""),
          h("details", { id: "pt-cadv", style: "margin-top:4px" },
            h("summary", { class: "muted small" }, "Advanced"),
            h("div", { class: "row wrap", style: "margin-top:4px" },
              h("span", { class: "k" }, "max pieces"), num("pt-cpieces", 8, "1", "58px"),
              h("span", { class: "k" }, "triangles / piece"), num("pt-ctris", 64, "8", "62px"),
              h("span", { class: "k" }, "tolerance mm"), num("pt-ctol", 0.5, "0.1", "58px"),
              btn("Regenerate", "quiet", () => collisionPreview(true), "Rebuild the draft collision with these settings")))),
        h("span", { class: "k" }, "Note"), h("input", { type: "text", id: "pt-note", placeholder: "optional" })),
      h("div", { class: "row" }, lease(btn("Save part", "primary", () => save())),
        lease(btn("TCP at its tip…", "", () => makeTcp(), "A virtual TCP at the far face of this part (it follows the part when you edit it)")),
        h("div", { class: "grow" }), h("span", { class: "muted small", id: "pt-hint" })),
      h("div", { class: "muted small", style: "margin-top:6px" },
        h("div", {}, "Active now: draft in 3D; after Save: TF + MoveIt collision + 3D view."),
        h("div", { id: "pt-model-state" }, "Robot model (URDF): after Restart robot stack.")),
      fileIn);
    shapeFields();
    editor.addEventListener("input", () => draftChanged());
    editor.addEventListener("change", (e) => {
      if (e.target === parentSel) P.lastParent = parentSel.value || P.lastParent;
      draftChanged();
    });

    function draftChanged() {
      P.drafting = true;
      onChange();
    }
    function startFrom(d, label, editing = null) {
      P.editing = editing;
      P.drafting = true;
      $("#pt-title").textContent = editing ? `Edit ${editing}` : label ? `New part (${label})` : "New part";
      $("#pt-name").value = editing || d.name || "";
      parentSel.value = d.parent || "tool0";
      set3("pt-xyz", d.xyz_mm || [0, 0, 0]); set3("pt-rpy", d.rpy_deg || [0, 0, 0]);
      shapeSel.value = d.shape?.type || "none"; shapeFields(d.shape);
      $("#pt-anchor").value = d.anchor || "base"; $("#pt-color").value = d.color || "#9aa0a6";
      $("#pt-coll").value = d.collision === false ? "none" : d.collision === true || !d.collision ?
        (d.shape?.type === "mesh" ? "auto" : "visual") : d.collision;
      const cs = { max_pieces: 8, max_triangles: 64, tolerance_mm: 0.5, ...(d.collision_settings || {}) };
      $("#pt-cpieces").value = cs.max_pieces; $("#pt-ctris").value = cs.max_triangles; $("#pt-ctol").value = cs.tolerance_mm;
      P.collisionPreview = []; P.collisionInfo = d.collision_info || null;
      $("#pt-cinfo").textContent = collisionSummary(P.collisionInfo);
      $("#pt-cam").checked = d.kind === "camera"; $("#pt-note").value = d.note || "";
      P.lastParent = parentSel.value || P.lastParent;
      P.startPose = { xyz_mm: read3("pt-xyz"), rpy_deg: read3("pt-rpy") };
      $("#pt-hint").textContent = editing ? "Draft: drag it or type the pose, then Save part." : "Not saved yet.";
      $("#pt-name").focus();
      onChange();
      if ($("#pt-coll").value === "auto" && d.shape?.type === "mesh") collisionPreview();
    }
    function edit(p) {
      startFrom(p, null, p.name);
    }
    function resetPose() {
      if (!P.startPose) return;
      set3("pt-xyz", P.startPose.xyz_mm); set3("pt-rpy", P.startPose.rpy_deg);
      draftChanged();
    }
    function cancelDraft() {
      P.drafting = false;
      P.editing = null;
      P.startPose = null;
      $("#pt-title").textContent = "New part";
      $("#pt-name").value = "";
      parentSel.value = P.lastParent || "tool0";
      set3("pt-xyz", [0, 0, 0]); set3("pt-rpy", [0, 0, 0]);
      shapeSel.value = "none"; shapeFields({ type: "none" });
      $("#pt-anchor").value = "base"; $("#pt-color").value = "#9aa0a6";
      $("#pt-coll").value = "visual"; $("#pt-cam").checked = false; $("#pt-note").value = "";
      P.collisionPreview = []; P.collisionInfo = null; $("#pt-cinfo").textContent = "";
      $("#pt-hint").textContent = "Draft cancelled.";
      onChange();
    }
    function shape() {
      const t = shapeSel.value, d = (i) => +$(`#pt-d${i}`).value;
      if (t === "box") return { type: t, size_mm: [d(0), d(1), d(2)] };
      if (t === "cylinder") return { type: t, radius_mm: d(0), length_mm: d(1) };
      if (t === "sphere") return { type: t, radius_mm: d(0) };
      if (t === "mesh") return { type: t, file: meshSel.value, scale: +$("#pt-scale").value, offset_mm: read3("pt-moff") };
      return { type: "none" };
    }
    function collisionSettings() {
      return { max_pieces: +$("#pt-cpieces").value || 8, max_triangles: +$("#pt-ctris").value || 64,
        tolerance_mm: +$("#pt-ctol").value || 0, concavity: 1.35 };
    }
    function collisionSummary(i) {
      if (!i) return "";
      const over = Math.round((i.volume_ratio - 1) * 100);
      return `Collision: ${i.pieces} convex piece${i.pieces === 1 ? "" : "s"}, ${i.triangles} triangles, ` +
        `${over >= 0 ? "+" : ""}${over} % summed volume over the part (visual mesh ${i.visual_triangles.toLocaleString()} triangles)`;
    }
    async function collisionPreview(force = false) {
      if (!P.drafting || shapeSel.value !== "mesh" || $("#pt-coll").value !== "auto" || !meshSel.value) {
        P.collisionPreview = []; P.collisionInfo = null;
        if ($("#pt-cinfo")) $("#pt-cinfo").textContent = "";
        onChange();
        return;
      }
      const seq = ++P.previewSeq;
      $("#pt-cinfo").textContent = "generating collision…";
      const r = await cmd("parts_collision_preview", { file: meshSel.value, scale: +$("#pt-scale").value,
        collision_settings: collisionSettings(), force });
      if (seq !== P.previewSeq || !r.ok) return;
      if (r.generating) return pollCollisionPreview(r.collision_job, seq);
      P.collisionPreview = r.preview || []; P.collisionInfo = r.info || null;
      $("#pt-cinfo").textContent = collisionSummary(P.collisionInfo);
      P.meshRev++;
      onChange();
    }
    async function pollCollisionPreview(job, seq) {
      if (seq !== P.previewSeq) return;
      const r = await cmd("parts_collision_preview_status", { job });
      if (seq !== P.previewSeq || !r.ok) return;
      if (r.state === "generating") return setTimeout(() => pollCollisionPreview(job, seq), 150);
      if (r.state === "error") {
        $("#pt-cinfo").textContent = "Collision generation failed: " + r.msg;
        return;
      }
      P.collisionPreview = r.preview || []; P.collisionInfo = r.info || null;
      $("#pt-cinfo").textContent = collisionSummary(P.collisionInfo);
      P.meshRev++;
      onChange();
    }
    async function save() {
      if (!P.drafting) return toast("start or edit a part first", false);
      const name = $("#pt-name").value.trim();
      const r = await cmd("parts_save", { name, parent: parentSel.value, xyz_mm: read3("pt-xyz"), rpy_deg: read3("pt-rpy"),
        shape: shape(), anchor: $("#pt-anchor").value, color: $("#pt-color").value, collision: $("#pt-coll").value,
        collision_settings: $("#pt-coll").value === "auto" ? collisionSettings() : null,
        kind: $("#pt-cam").checked ? "camera" : "part", note: $("#pt-note").value, replace: P.editing });
      if (r.ok) {
        fill(r.parts);
        if (r.generating) {
          $("#pt-hint").textContent = "generating collision…";
          return waitSaved(name);
        }
        P.editing = name;
        P.drafting = false;
        P.startPose = { xyz_mm: read3("pt-xyz"), rpy_deg: read3("pt-rpy") };
        $("#pt-title").textContent = `Edit ${name}`;
        $("#pt-hint").textContent = "Saved: active now in TF, MoveIt collision and the 3D view.";
        onChange();
      }
    }
    async function waitSaved(name) {
      const r = await cmd("parts_get");
      if (!r.ok) return;
      fill(r.parts);
      const j = r.parts.collision_jobs?.[name];
      if (j?.state === "generating") return setTimeout(() => waitSaved(name), 250);
      if (j?.state === "error") return toast(`collision generation failed: ${j.msg}`, false, true);
      const p = r.parts.parts.find((x) => x.name === name);
      if (!p) return toast("collision generation ended but the part was not saved", false, true);
      P.editing = name; P.drafting = false;
      P.startPose = { xyz_mm: read3("pt-xyz"), rpy_deg: read3("pt-rpy") };
      $("#pt-title").textContent = `Edit ${name}`;
      $("#pt-hint").textContent = "Saved: active now in TF, MoveIt collision and the 3D view.";
      onChange();
    }
    async function del(p) {
      if (!(await ask(`Delete part "${p.name}"?`))) return;
      const r = await cmd("parts_delete", { name: p.name });
      if (r.ok) { fill(r.parts); if (P.editing === p.name) startFrom({}); onChange(); }
    }
    async function makeTcp() {
      const n = P.editing || $("#pt-name").value.trim();
      if (!n) return toast("save the part first", false);
      const t = (await askText(`Name of the TCP at the far face of "${n}" (it follows the part when you edit it):`, `${n}_tip`));
      if (t) return cmd("parts_make_tcp", { name: n, tcp: t });
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
    function draft() {
      if (!P.drafting || !P.data) return null;
      return { editing: P.editing, parent: parentSel.value, xyz_mm: read3("pt-xyz"), rpy_deg: read3("pt-rpy"),
        shape: shape(), anchor: $("#pt-anchor").value, color: $("#pt-color").value,
        mode: $("#pt-gmode").value, space: $("#pt-gspace").value, snap: +$("#pt-snap").value || 0,
        mesh_rev: P.meshRev, collision_preview: P.collisionPreview };
    }
    function poseFromGizmo(p) {
      if (!P.drafting) return;
      set3("pt-xyz", p.xyz_mm); set3("pt-rpy", p.rpy_deg);
      onChange();
    }
    function sync(sy, rf) {
      $("#pt-sync").textContent = sy ? `MoveIt / TF: ${sy.msg}` : "";
      $("#pt-sync").classList.toggle("error", sy && /error|unverified/.test(sy.state));
      $("#pt-model-state").textContent = rf && rf.length
        ? "Robot model (URDF): pending Restart robot stack."
        : "Robot model (URDF): matches the saved Parts.";
    }
    roots.push(card, editor);
    W.parts = { card, editor, load: () => cmd("parts_get").then((r) => r.ok && fill(r.parts)), fill,
      draft, poseFromGizmo,
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
    if (!(await ask(msg))) return;
    const r = await cmd("restart", { what });
    if (r.ok) toast(r.msg, true, true);
  }
  async function stackStop() {
    if (!(await ask("Stop the robot stack?\n\nFor when nobody uses the robot. The driver, controllers, MoveIt, the cameras and this GUI " +
      "stop, so the PC is idle. The arm stays where it is. The address shows a page with Start robot stack; " +
      "anyone (not a Viewer) can start it again there (about a minute).", { ok: "Stop stack" }))) return;
    const r = await cmd("stack_stop", { who: localStorage.getItem("operator") || "" });
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
    const stepBtn = (b, step) => { b.dataset.step = step; return b; };
    const left = h("div", { class: "col su-main" },
      h("div", { class: "card" }, h("div", { class: "section" }, "1 · Camera"),
        note("Finds where a camera is on the robot (or in the cell) from 10-15 pictures of a printed ChArUco board.",
          "Eye-in-hand: the board lies still in the cell, the camera on the tool looks at it from different angles. Eye-to-hand: " +
          "the board is fixed on the tool, the camera on its stand. The intrinsics come from the camera's own CameraInfo " +
          "(RealSense / ZED are factory calibrated). OpenCV solves it with Park's method; four other methods are shown as a cross-check. " +
          "The result becomes a camera part, so the camera's frames hang on the robot in TF (and in the robot model after a stack restart)."),
        h("div", { class: "grid", style: "grid-template-columns:auto 1fr" },
          h("span", { class: "k" }, "Camera"), h("div", { class: "row" }, camSel, h("input", { type: "text", id: "he-topic", placeholder: "or an image topic", style: "flex:1" })),
          h("span", { class: "k" }, "Mounted"), mount),
        h("div", { class: "row" }, lease(stepBtn(btn("Start", "", () => start(), "Read the camera's intrinsics and begin (clears the samples)"), 1)),
          h("span", { class: "muted small grow", id: "he-intr" }))),
      h("div", { class: "card" }, h("div", { class: "section" }, "2 · Board"),
        h("div", { class: "row" }, h("span", { class: "small grow", id: "he-board-sum" }, "—"),
          btn("Print board…", "quiet", () => printBoard(), "Opens the board at its true size: print at 100 % (no fit to page) and glue it flat on something rigid")),
        h("details", { id: "he-board-edit" }, h("summary", { class: "muted small" }, "Change the board (measure the printed squares with a ruler)"),
          h("div", { class: "row wrap" }, ...bf("he-bx", "squares X", "1"), ...bf("he-by", "Y", "1"), ...bf("he-bs", "square mm", "0.1"), ...bf("he-bm", "marker mm", "0.1"),
            h("select", { id: "he-bd" }, ["DICT_4X4_50", "DICT_5X5_100", "DICT_6X6_250"].map((d) => h("option", { value: d }, d)))),
          h("div", { class: "row" }, lease(btn("Save board", "", () => saveBoard(), "Enter the square size you MEASURE on the print"))))),
      h("div", { class: "card" }, h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "3 · Pictures of the board"),
          btn("Look", "quiet", () => look(), "Grab a frame and show what the board detector sees (records nothing)")),
        h("div", { class: "grid", style: "grid-template-columns:auto 1fr;align-items:center" },
          h("span", { class: "k" }, "Automatic"),
          h("div", { class: "row wrap" }, h("span", { class: "muted small" }, "the robot moves to 12 poses around this one, tilted"), num("he-tilt", 20, "1", "56px"), h("span", { class: "muted small" }, "°"),
            lease(btn("Plan poses", "quiet", () => plan(), "Poses around this one, turned about the board (eye-in-hand) or the tool tip (eye-to-hand). Nothing moves yet")),
            lease(stepBtn(btn("Collect all…", "", () => collect(), "Move to each planned pose (MoveIt, collision-aware) and take a picture there"), 2))),
          h("span", { class: "k" }, "By hand"),
          h("div", { class: "row wrap" }, h("span", { class: "muted small" }, "move the arm yourself, then"), ctx.freedriveButton(),
            lease(btn("Capture here", "", () => capture(), "Take a picture where the arm is now (freedrive or jog between pictures)")))),
        h("div", { class: "muted small", id: "he-status" }), img,
        h("div", { class: "row wrap", id: "he-samples" }),
        h("div", { class: "row" }, lease(btn("Clear pictures", "quiet", () => cmd("handeye_drop").then((r) => r.ok && fill(r.he)))))));
    const right = h("div", { class: "card", style: "width:380px" },
      h("div", { class: "section" }, "4 · Solve and save"),
      h("div", { class: "row" }, stepBtn(btn("Solve", "", () => cmd("handeye_solve").then((r) => r.ok && fill(r.he))), 3)),
      h("div", { id: "he-result" }),
      h("div", { class: "row" }, h("input", { type: "text", id: "he-name", placeholder: "camera name, e.g. wrist_rs", style: "flex:1" }),
        lease(stepBtn(btn("Save + add to robot", "", () => saveCam(), "Keep the calibration and put the camera's frame on the robot as a part"), 4))),
      note("Saving puts the camera on the robot now (TF, 3D view, SETUP › Parts). The robot model gets it at the next robot stack restart."),
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
      if (!ctx.fake() && !(await ask(`Move through ${C.data?.poses || 0} poses and sample at each?\n\nThe arm turns up to ` +
        `±${$("#he-tilt").value}° about the board (or the tool tip). MoveIt plans each move around the keep-out zones and parts, ` +
        "but check the space around the arm. Keep a hand near STOP."))) return;
      return cmd("handeye_collect");
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
      $("#he-board-sum").textContent = `${b.squares_x} × ${b.squares_y} squares of ${b.square_mm} mm, markers ${b.marker_mm} mm, ${b.dict}`;
      // the next step's button is the blue one: start, collect, solve, save
      const step = !d.intrinsics ? 1 : d.samples.length < d.min_samples ? 2 : !d.result ? 3 : 4;
      [left, right].forEach((el) => el.querySelectorAll("[data-step]").forEach((x) => x.classList.toggle("primary", +x.dataset.step === step)));
      const i = d.intrinsics;
      $("#he-intr").textContent = i ? `${d.topic} · frame ${d.frame} · fx ${i.fx.toFixed(1)} fy ${i.fy.toFixed(1)} · cx ${i.cx.toFixed(1)} cy ${i.cy.toFixed(1)}` +
        (i.D.some((x) => Math.abs(x) > 1e-9) ? " · distortion " + i.D.map((x) => x.toFixed(3)).join(" ") : " · no distortion") : "Not started";
      status(d.last);
      $("#he-samples").replaceChildren(...d.samples.map((s, k) => h("span", { class: "chip", title: `${s.corners} corners, ${s.reproj_px} px` },
        `#${k + 1} ${s.corners}c`, h("a", { class: "x", title: "Remove this sample", onclick: () => cmd("handeye_drop", { index: k }).then((r) => r.ok && fill(r.he)) }, " ×"))),
        h("span", { class: "muted small" }, `${d.samples.length} picture(s)` + (d.poses ? ` · ${d.poses} poses planned` : "") +
          (d.samples.length < d.min_samples ? ` · at least ${d.min_samples}, 12+ is better` : d.samples.length < 12 ? " · 12+ is better" : " · enough")));
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
        : [h("div", { class: "muted small" }, d.samples.length >= d.min_samples ? "Press Solve." : "Take pictures of the board first (step 3).")]));
      $("#he-saved").replaceChildren(...(d.saved.length ? d.saved.map((c) => h("div", { class: "item" },
        h("div", { class: "grow" }, h("div", {}, `${c.name} · ${c.mount.replace(/_/g, "-")}`),
          h("div", { class: "muted small" }, `${c.verdict} · ${c.rms_mm} mm / ${c.rms_deg}° · ${c.samples} samples · ${c.saved}`),
          h("div", { class: "mono small" }, `${c.parent} → ${c.optical_frame}: ${c.xyz_mm.join(", ")} mm`)),
        lease(btn("Add to robot", "quiet", () => cmd("handeye_apply", { name: c.name }).then((r) => r.ok && W.parts?.fill(r.parts)))),
        lease(btn("✕", "quiet", async () => (await ask(`Delete calibration ${c.name}?`)) && cmd("handeye_delete", { name: c.name }).then((r) => r.ok && fill(r.he))))))
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
    // the internal names of what a setup holds, in plain words (several files can be one thing to the user)
    const LABEL = { "TCPs, fixtures, vision": "TCPs + fixtures", "controller gains + payload": "gains + payload",
      "vial profiles": "vials", "flask profiles": "flasks", "part meshes": "parts", "parts URDF/SRDF": "", "last payload calibration": "payload calibration" };
    const words = (keys) => [...new Set((keys || []).map((k) => LABEL[k] ?? k).filter(Boolean))];
    const list = h("div", { class: "list", id: "st-list" }), autos = h("div", { class: "list", id: "st-auto" });
    const cur = h("div", { class: "st-cur", id: "st-current" }, "—");
    // whose setups the list shows: a person's profile, or "" (the shared ones, saved before profiles existed)
    let OWNER = null, PROFILES = [];
    const ownerSel = h("select", { id: "st-owner", title: "Whose setups to show. Save puts the setup here." });
    ownerSel.onchange = () => { OWNER = ownerSel.value; W.setups.load(); };
    const ownerName = (o) => o || "Shared";
    const isCur = (s) => CUR?.name === s.name && (CUR.owner || "") === (OWNER || "");
    const steps = h("div", { class: "list", id: "st-steps" });
    const guide = h("details", { class: "st-guide" }, h("summary", { class: "section", style: "cursor:pointer" }, "Make a new setup, step by step"), steps,
      h("div", { class: "muted small", style: "margin-top:6px" }, "To switch later: Load… on a setup. Its camera holder, calibration and cameras come back."));
    const card = h("div", { class: "card su-main" },
      h("div", { class: "section" }, "Setups"),
      note("One setup per tool or camera holder, saved by name. Load one to switch.",
        "A setup keeps the parts (your holder in the robot model), TCPs, camera calibrations and which cameras run, F/T bias, " +
        "payload, gains, keep-out zones and vials. Programs and policies are shared by every setup. Loading one restarts the GUI " +
        "(and the robot stack when its parts or base differ from the running robot model); the files in use before are kept as an automatic backup."),
      cur,
      h("div", { class: "row" }, h("span", { class: "muted" }, "Profile"), ownerSel, h("span", { class: "grow" }),
        lease(btn("Copy from another profile…", "quiet", () => copyFrom(), "Copy someone's setup into this profile"))),
      list,
      guide,
      h("div", { class: "section" }, "Save what is on the robot now"),
      h("div", { class: "row" }, h("input", { type: "text", id: "st-name", placeholder: "name, e.g. scraping spatula", style: "width:200px" }),
        h("input", { type: "text", id: "st-note", placeholder: "note (what is mounted)", style: "flex:1" }),
        lease(btn("Save", "primary", () => save(false)))),
      h("div", { class: "muted small", id: "st-present" }),
      h("details", {}, h("summary", { class: "muted small" }, "Automatic backups (before each load, newest 5)"), autos));
    const baseBox = h("div", { class: "list", id: "st-base" }), robotBox = h("div", { id: "st-robot" });
    const side = h("div", { class: "card", style: "width:320px" },
      h("div", { class: "section" }, "Robot"),
      note("What this PC runs: the arm (real or fake), gripper, F/T sensor and cameras.",
        "Hardware… lists what the GUI knows: what is ready on this PC can be picked, the rest says what is missing. " +
        "Switching restarts the robot stack (about a minute). Kept for the next starts; ur7e_gui.sh --fake or --robot-ip win for one start."),
      robotBox,
      h("div", { class: "section" }, "On the flange (base)"),
      note("The fixed hardware in the robot model. Your parts go on top of it.",
        "Each base has its own URDF, SRDF (MoveIt's collision rules and groups) and controllers. Switching restarts the robot " +
        "stack. A setup remembers its base: loading the setup puts it back."),
      baseBox,
      h("div", { class: "section" }, "Restart"),
      h("div", { class: "row" }, lease(btn("Restart GUI…", "", () => restart("gui"), "The GUI only: the robot stack keeps running")),
        lease(btn("Restart robot stack…", "", () => restart("stack"), "Driver, controllers, MoveIt and the GUI: puts the parts into the robot model"))),
      h("div", { class: "row" }, lease(btn("Stop robot stack…", "", () => stackStop(), "Nobody uses the robot: stop everything, leave a Start page"))),
      note("Refused while the robot moves, a program or policy runs, or a force is applied.",
        "The launcher (ur7e_gui.sh) does the restart; a GUI started by hand cannot restart itself."));
    let B = null, CUR = null;
    let RB = null;
    const robotLabel = (n) => (RB?.robots.find((r) => r.name === n) || { label: n }).label;
    const optLabel = (list, n) => (list?.find((x) => x.name === n) || { label: n }).label;
    function fillRobot(v) {
      if (!v) return;
      RB = v;
      if (B) fillBase(B);
      const run = v.running;
      robotBox.replaceChildren(run
        ? h("div", {},
            h("div", { class: "row" }, h("span", { class: "grow" }, h("b", {}, robotLabel(run.robot)), ` · ${run.mode}${run.mode === "real" ? " · " + run.ip : ""}`),
              lease(btn("Hardware…", "primary", () => setupRobot(false)))),
            h("div", { class: "muted small" }, `Gripper: ${optLabel(v.grippers, v.gripper)} · F/T: ${optLabel(v.ft_sensors, v.ft)} · ` +
              `Cameras: ${v.cameras.length ? v.cameras.map((c) => c.name).join(", ") : "none"}`),
            h("div", { class: "muted small", title: "Your own ROS 2 programs need  export ROS_DOMAIN_ID=" + run.domain }, `ROS domain ${run.domain} · ` +
              optLabel(v.nets, run.net) + ` · continuous jog ${run.servo || "off"}`))
        : h("div", { class: "muted small" }, "Started by hand, not by ur7e_gui.sh: set the robot up there."));
    }
    // Set up this robot: arm (real / fake, IP), gripper, F/T sensor, cameras; one Apply, one restart
    async function setupRobot(first) {
      const r0 = await cmd("robot_view");
      if (!r0.ok) return;
      const v = r0.robot;
      fillRobot(v);
      if (B) fillBase(B);
      const cur = v.running || v.next;
      const ready = (r) => !r.missing.length;
      let arm = v.robots.find((r) => r.name === cur.robot && ready(r)) ? cur.robot : (v.robots.find(ready) || {}).name;
      let mode = cur.mode, gripper = v.gripper, ft = v.ft, net = cur.net || "lab";
      const domain = h("input", { type: "number", min: 0, max: v.domain_max, step: 1, value: cur.domain ?? "42", style: "width:70px" });
      const ip = h("input", { type: "text", value: cur.ip || v.next.ip, style: "width:140px" });
      const servo = h("input", { type: "checkbox", checked: (cur.servo || v.next.servo || "on") === "on" });
      const ftIp = h("input", { type: "text", value: cur.ft_ip || v.next.ft_ip, style: "width:140px" });
      const camChecks = {};
      const sec = (n, t) => h("div", { class: "section", style: "margin:10px 0 4px" }, `${n}. ${t}`);
      const radio = (group, name, on, label, sub, enabled, set, extra) => h("label", { class: "item" + (on ? " on" : "") + (enabled ? "" : " muted"),
          style: enabled ? "cursor:pointer" : "" },
        h("input", { type: "radio", name: group, value: name, checked: on, disabled: !enabled, onchange: () => { set(name); paint(); } }),
        h("div", { class: "grow" }, h("div", {}, label, extra || ""), sub ? h("div", { class: "small" }, sub) : ""));
      const body = h("div", { style: "display:flex;flex-direction:column;gap:2px" });
      const foot = h("div", { class: "muted small", style: "margin-top:8px" });
      function paint() {
        const A = v.robots.find((r) => r.name === arm) || { grippers: ["none"], ft_sensors: ["none"] };
        if (!A.grippers.includes(gripper)) gripper = A.grippers[0];                     // another arm: its own gripper
        if (!A.ft_sensors.includes(ft)) ft = A.ft_sensors.includes("none") ? "none" : A.ft_sensors[0];
        const real = mode === "real";
        body.replaceChildren(
          sec(1, "Robot arm"),
          h("div", { class: "list" }, ...v.robots.map((r) => radio("su-arm", r.name, r.name === arm, r.label, ready(r) ? r.text : "",
            ready(r), (n) => (arm = n),
            [" ", ready(r) ? h("span", { class: "chip ok" }, "ready") : h("span", { class: "chip warn" }, "not set up"),
              ready(r) ? "" : h("ul", { class: "small", style: "margin:4px 0 0 16px;padding:0" }, ...r.missing.map((m) => h("li", {}, m)))]))),
          h("div", { class: "row", style: "margin-top:4px" }, ...v.modes.map((m) => h("label", { title: m.text },
            h("input", { type: "radio", name: "su-mode", value: m.name, checked: m.name === mode, onchange: () => { mode = m.name; paint(); } }), ` ${m.label} `)),
            real ? h("span", {}, " Robot IP ", ip) : h("span", { class: "muted small" }, " no robot: a simulated arm")),
          arm === "ur7e" ? h("label", { class: "row", style: "cursor:pointer;margin-top:4px" }, servo, " Continuous jog (MoveIt Servo)",
            h("span", { class: "muted small" }, " hold-to-jog buttons; off: step jog only")) : "",
          sec(2, "Gripper"),
          h("div", { class: "list" }, ...A.grippers.map((g) => radio("su-grip", g, g === gripper, optLabel(v.grippers, g),
            v.grippers.find((x) => x.name === g)?.text, true, (n) => (gripper = n)))),
          sec(3, "Force/torque sensor"),
          h("div", { class: "list" }, ...A.ft_sensors.map((f) => radio("su-ft", f, f === ft, optLabel(v.ft_sensors, f),
            v.ft_sensors.find((x) => x.name === f)?.text, true, (n) => (ft = n)))),
          ft !== "none" && real ? h("div", { class: "row" }, "F/T sensor IP ", ftIp) : "",
          sec(4, "ROS 2"),
          h("div", { class: "row" }, "Domain ID ", domain, h("span", { class: "muted small" }, " your own ROS 2 programs use the same number")),
          h("div", { class: "list" }, ...v.nets.map((n) => radio("su-net", n.name, n.name === net, n.label, n.text, true, (x) => (net = x)))),
          sec(5, "Cameras"),
          v.cameras.length ? h("div", { class: "small" }, "On this robot: " + v.cameras.map((c) => `${c.name} (${c.label})`).join(", ") +
            ". Edit or remove them in SETUP › Cameras.") : h("div", { class: "muted small" }, "None yet. Tick the ones to add:"),
          h("div", { class: "list" }, ...v.camera_types.filter((t) => !v.cameras.some((c) => c.type === t.type)).map((t) => {
            const box = camChecks[t.type] || (camChecks[t.type] = h("input", { type: "checkbox", disabled: !t.installed }));
            return h("label", { class: "item" + (t.installed ? "" : " muted"), style: t.installed ? "cursor:pointer" : "" }, box,
              h("div", { class: "grow" }, h("div", {}, "Add ", t.label, " ", t.installed ? "" : h("span", { class: "chip warn" }, "not installed")),
                h("div", { class: "small" }, t.installed ? t.help : t.install)));
          })));
        updateFoot();
      }
      function updateFoot() {
        const real = mode === "real";
        const same = cur && arm === cur.robot && mode === cur.mode && (!real || ip.value.trim() === cur.ip) &&
          gripper === v.gripper && ft === v.ft && (!real || ft === "none" || ftIp.value.trim() === cur.ft_ip) &&
          String(domain.value).trim() === String(cur.domain) && net === cur.net &&
          (servo.checked ? "on" : "off") === (cur.servo || "off");
        foot.textContent = same ? "Cameras are added without a restart." : "Apply restarts the robot stack and the GUI (about a minute).";
      }
      body.addEventListener("input", updateFoot);
      paint();
      const form = h("div", {}, body, foot);
      const title = first ? "Set up this robot" : "Robot hardware";
      const text = (first ? "Nothing is set up on this PC yet. Pick what you have; you can change it later in SETUP › Setups.\n" : "") +
        "Fake: no robot, nothing is sent anywhere. Real: nothing moves until you command it.";
      const c = await ask(`${title}\n${text}`, { content: form, wide: true, ok: "Apply", cancel: first ? "Later" : "Cancel",
        result: () => ({ robot: arm, mode, ip: ip.value.trim(), gripper, ft, ft_ip: ftIp.value.trim(), domain: String(domain.value).trim(), net,
          servo: servo.checked ? "on" : "off",
          cameras: Object.entries(camChecks).filter(([, b]) => b.checked).map(([k]) => k) }) });
      if (!c) return;
      if (!c.robot) return toast("no robot is ready on this PC yet", false);
      const r = await cmd("robot_setup", { ...c, confirm: true });
      if (r.ok) { toast(r.msg, true, !!r.restart); if (r.robot) fillRobot(r.robot); }
    }
    async function save(replace, name, nt) {
      name = name ?? $("#st-name").value.trim();
      nt = nt ?? $("#st-note").value;
      if (!name) return toast("name the setup first", false);
      const r = await cmd("setup_save", { name, note: nt, replace, owner: OWNER || "" });
      if (!r.ok && /exists/.test(r.msg || "") && (await ask(`Replace setup "${name}" with what is on the robot now?`))) return save(true, name, nt);
      if (r.ok) { fillBase(r.base); fill(r.setups, r.current); }
    }
    // copy a setup from another profile (or Shared) into the one shown; the robot's files don't change
    async function copyFrom() {
      const others = ["", ...PROFILES.map((p) => p.name)].filter((o) => o !== (OWNER || ""));
      const from = h("select", {}, ...others.map((o) => h("option", { value: o }, ownerName(o))));
      const pick = h("select", { style: "min-width:200px" });
      const as = h("input", { type: "text", placeholder: "same name", maxlength: 40, style: "width:200px" });
      const fillPick = async () => {
        const r = await cmd("setup_list", { owner: from.value });
        const l = (r.ok ? r.setups : []).filter((x) => !x.auto);
        pick.replaceChildren(...(l.length ? l.map((x) => h("option", { value: x.name }, x.name + (x.note ? " · " + x.note : "")))
          : [h("option", { value: "" }, "no setups")]));
      };
      from.onchange = fillPick;
      await fillPick();
      const form = h("div", { style: "display:flex;flex-direction:column;gap:8px" }, h("div", { class: "row" }, "From", from), h("div", { class: "row" }, "Setup", pick),
        h("div", { class: "row" }, "Save as", as));
      const c = await ask(`Copy a setup into ${ownerName(OWNER)}\nA copy: later changes in either one don't touch the other. ` +
        "The robot keeps running what it runs now; Load… the copy to use it.", { content: form, ok: "Copy", result: () => ({ from: from.value, name: pick.value, as: as.value.trim() }) });
      if (!c || !c.name) return;
      const go = (replace) => cmd("setup_copy", { source_owner: c.from, name: c.name, owner: OWNER || "", new_name: c.as, replace });
      let r = await go(false);
      if (!r.ok && /already has/.test(r.msg || "") && (await ask(`Replace "${c.as || c.name}" in ${ownerName(OWNER)} with the copy?`))) r = await go(true);
      if (r.ok) { toast(r.msg, true); fill(r.setups, r.current); }
    }
    const baseLabel = (n) => (B?.choices.find((c) => c.name === n) || { label: n || "?" }).label;
    function fillBase(b) {
      if (!b) return;
      B = b;
      if (RB?.running && RB.running.robot !== "ur7e")       // these bases are the UR7e's parts
        return baseBox.replaceChildren(h("div", { class: "muted small" }, "Gripper and F/T sensor: Robot › Hardware… above."));
      baseBox.replaceChildren(...b.choices.map((c) => h("div", { class: "item" + (c.name === b.running ? " on" : "") },
        h("div", { class: "grow" }, h("div", {}, c.label, c.name === b.running ? h("span", { class: "chip ok" }, " running") : "",
          c.name === b.next && c.name !== b.running ? h("span", { class: "chip warn" }, " next start") : ""),
          h("div", { class: "muted small" }, c.text)),
        c.name === b.running ? "" : lease(btn("Switch…", "quiet", () => switchBase(c))))));
    }
    async function switchBase(c) {
      if (!(await ask(`Switch the base to "${c.label}"?\n\n${c.text}.\n\nCheck first that this is what is really on the robot. ` +
        "The robot stack restarts (about a minute). Parts on hardware this base does not have are left out.\n\n" +
        "Save the current setup first if you want to come back to it."))) return;
      const r = await cmd("base_switch", { base: c.name, confirm: true });
      if (r.ok) toast(r.msg, true, true);
    }
    async function load(s) {
      const pv = await cmd("setup_preview", { name: s.name, owner: OWNER || "" });
      if (!pv.ok) return;
      const ch = words(pv.changes);
      const lines = [ch.length ? `Changes: ${ch.join(", ")}` : "Nothing differs from what is in use now.",
        pv.base ? `Base: ${pv.base.from_label} → ${pv.base.to_label}. Check that this hardware is really on the robot.` : "",
        pv.restart === "stack" ? "Restarts the robot stack and the GUI (about a minute). Then press Play on the pendant (unless headless)."
          : "Restarts the GUI (a few seconds)."].filter(Boolean);
      const unsaved = CUR?.name && (CUR.changed?.length || CUR.base_changed) && !isCur(s)
        ? `\n\nNot saved in "${CUR.name}": ${words(CUR.changed).concat(CUR.base_changed ? ["base"] : []).join(", ")} (kept in an automatic backup).` : "";
      if (!(await ask(`Load setup "${s.name}"?\n\n${lines.join("\n")}${unsaved}`))) return;
      const r = await cmd("setup_load", { name: s.name, owner: OWNER || "" });
      if (r.ok) toast(r.msg, true, true);
    }
    const row = (s) => h("div", { class: "item" + (isCur(s) ? " on" : "") },
      h("div", { class: "grow" }, h("div", {}, s.name, isCur(s) ? h("span", { class: "chip ok" }, "in use") : "",
          s.base ? h("span", { class: "chip" + (B && s.base !== B.running ? " warn" : ""),
          title: B && s.base !== B.running ? "a different base from the running one: loading switches it" : "base" }, baseLabel(s.base)) : "",
          s.note ? h("span", { class: "muted" }, " · " + s.note) : ""),
        s.cameras?.length ? h("div", { class: "muted small" }, "Cameras: " + s.cameras.map(camLabel).join(", ")) : "",
        h("div", { class: "muted small" }, `${s.saved} · ${words(s.holds).join(", ") || "empty"}` + (s.copied_from ? ` · copied from ${s.copied_from}` : "")), s.error ? h("div", { class: "error small" }, s.error) : ""),
      lease(btn("Load…", "", () => load(s), "Shows what changes first")),
      lease(btn("✕", "quiet", async () => (await ask(`Delete setup "${s.name}"?`)) && cmd("setup_delete", { name: s.name, owner: OWNER || "" }).then((r) => r.ok && fill(r.setups, r.current)))));
    function fillCurrent(c) {
      CUR = c ? { ...c, note: (c.owner || "") === (OWNER || "") ? (W.setups.all || []).find((s) => s.name === c.name)?.note || "" : "" } : null;
      if (!c) return;
      $("#st-present").textContent = c.present?.length ? "Saves: " + [B ? "base " + baseLabel(B.running) : "", ...words(c.present)].filter(Boolean).join(" · ")
        : "Nothing to save yet.";
      const who = c.settings_owner ? h("div", { class: "small", title: "The robot runs the settings of the one in " +
        "control. Taking control switches to your own; theirs stay in their profile." },
        "Settings in use: ", h("b", {}, `${c.settings_owner}'s`)) : "";
      if (!c.name) {
        cur.replaceChildren(who, h("b", {}, "No setup loaded."), " The robot runs what was set up last. Save it below to keep it.");
        return;
      }
      const ch = words(c.changed).concat(c.base_changed ? ["base"] : []);
      const mine = (c.owner || "") === (OWNER || "");
      cur.replaceChildren(who, h("div", { class: "row" }, h("span", { class: "grow" }, "In use: ", h("b", {}, c.owner ? `${c.owner} / ${c.name}` : c.name), " ",
          ch.length ? chip("changed", "warn") : chip("saved", "ok")),
        ch.length && mine ? lease(btn(`Save into "${c.name}"`, "primary", async () => (await ask(`Save the changes (${ch.join(", ")}) into "${c.name}"?`)) &&
          save(true, c.name, CUR.note))) : ""),
        h("div", { class: "muted small" }, ch.length ? `Changed since saved: ${ch.join(", ")}` : "Nothing changed since it was saved or loaded."));
    }
    // the step list: where each step stands now, and a button to its page
    function fillSteps(g) {
      if (!g) return;
      const step = (n, title, done, now, page, sub) => h("div", { class: "item" + (done ? " on" : "") },
        h("div", { class: "grow" }, h("div", {}, `${n}. ${title} `, done ? chip("done", "ok") : chip("to do", "warn")),
          h("div", { class: "muted small" }, now)),
        page ? btn(sub || page, "quiet", () => navTo(page, sub)) : "");
      const run = g.cameras.filter((c) => g.on.includes(c.name));
      steps.replaceChildren(
        step(1, "Put your camera holder in the robot model", g.parts.length > 0,
          g.parts.length ? "Parts now: " + g.parts.join(", ") : "Import its mesh (.stl, .obj, .dae) on the tool. Then Restart robot stack.", "SETUP", "Parts"),
        step(2, "Add your camera and start it", run.length > 0,
          run.length ? "Running: " + run.map((c) => c.label).join(", ") : g.cameras.length ? "Start yours (others: Stop)." : "Add it with + Add…", "SETUP", "Cameras"),
        step(3, "Calibrate the camera", g.calibrations.length > 0,
          g.calibrations.length ? "Calibrated: " + g.calibrations.join(", ") : "Eye-in-hand: Start, Plan poses, Collect all…, Solve, Save + add to robot.", "SETUP", "Camera"),
        step(4, "Save it under your name", !!CUR?.name && !(CUR.changed?.length || CUR.base_changed),
          CUR?.name ? `In use: ${CUR.name}` : "Type a name below (e.g. alice_d405) and press Save.", "", ""));
    }
    function fillOwners(profiles, owner) {
      PROFILES = profiles || PROFILES;
      if (owner != null) OWNER = owner;
      ownerSel.replaceChildren(...["", ...PROFILES.map((p) => p.name)].map((o) => h("option", { value: o }, ownerName(o))));
      ownerSel.value = OWNER || "";
    }
    function fill(all, current) {
      W.setups.all = all;
      if (current) fillCurrent(current);
      const mine = all.filter((s) => !s.auto), auto = all.filter((s) => s.auto);
      list.replaceChildren(...(mine.length ? mine.map(row) : [h("div", { class: "muted small" }, "No setups saved yet: follow the steps below.")]));
      if (!mine.length) guide.open = true;
      autos.replaceChildren(...(auto.length ? auto.map(row) : [h("div", { class: "muted small" }, "None.")]));
    }
    roots.push(card, side);
    W.pickRobot = setupRobot;
    let G = null;
    const camLabel = (n) => (G?.cameras.find((c) => c.name === n) || { label: n }).label;
    W.setups = { card, side, all: [],
      load: async () => {
        if (OWNER == null) {                // first time: this browser's profile (top right), else the shared setups
          const p = await cmd("profile_list");
          const me = (localStorage.getItem("operator") || "").toLowerCase();
          OWNER = (p.ok && p.profiles.find((x) => x.name.toLowerCase() === me)?.name) || "";
        }
        const r = await cmd("setup_list", { owner: OWNER });
        if (!r.ok && OWNER) { OWNER = ""; return W.setups.load(); }      // the profile was deleted
        if (r.ok) { G = r.guide; fillOwners(r.profiles, r.owner); fillBase(r.base); fill(r.setups, r.current); fillSteps(r.guide); }
        cmd("robot_view").then((x) => x.ok && fillRobot(x.robot));
      },
      showOwner: (o) => { OWNER = o || ""; W.setups.load(); },
      update: (s) => {                      // the state's short summary changed (a save, load or edit elsewhere): read it again
        const k = JSON.stringify(s.workcell?.setup || null);
        if (k !== W.setups.key) { const first = W.setups.key == null; W.setups.key = k; if (!first) W.setups.load(); }
      } };
    return W.setups;
  }

  return { buildParts, buildCalib, buildSetups, restart, W };
}
