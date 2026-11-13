// SETUP > Flask Geometry: touch a round-bottom flask with the TCP, fit it, save it as the "flask" fixture and a
// named profile, then check what a tool reaches through the neck and make a scrape program (opened unsaved in
// PROGRAMS). Built by app.js with its helpers (ctx). The server owns every rule; this only shows and asks.

// what the shared SETUP 3D view draws for this page (base frame, mm): pure, so it is tested without a browser
export function flaskShapes(d, reach) {
  const out = { spheres: [], cylinders: [], points: [], lines: [] };
  if (!d) return out;
  const geo = (d.result && d.result.draw) || (d.fixture && d.fixture.draw);
  if (geo) {
    out.spheres.push({ c: geo.sphere.c, r: geo.sphere.r, color: 0x60a5fa, opacity: 0.16 });
    out.cylinders.push({ a: geo.neck.a, b: geo.neck.b, r: geo.neck.r, color: 0x60a5fa, opacity: 0.22 });
    out.points.push({ p: geo.pivot, color: 0xf59e0b, size: 3.5, onTop: true });
  }
  const COL = { rim: 0x2b5fb8, bulb: 0x7c3aed, neck: 0x0f766e };
  for (const [k, pts] of Object.entries(d.points || {}))
    for (const p of pts) out.points.push({ p: p.map((x) => x * 1000), color: COL[k] ?? 0x2b5fb8, size: 2.5 });
  if (d.hint && d.hint.xyz) out.points.push({ p: d.hint.xyz.map((x) => x * 1000), color: 0xf59e0b, size: 4 });
  for (const p of (reach && reach.poses) || []) {
    const color = p.ok ? 0x16a34a : 0xdc2626;
    const pts = p.pts || [p.tip, p.tail];               // tip, (elbow of a bent tool,) back up the shaft
    for (let i = 1; i < pts.length; i++) out.lines.push({ a: pts[i - 1], b: pts[i], color });
    out.points.push({ p: p.tip, color, size: 1.6 });
  }
  return out;
}

export function flaskPage(ctx) {
  const { h, btn, note, cmd, pcmd, toast, ask, askText, lease, freedriveButton } = ctx;
  const F = { data: null, reach: null };
  const num = (value, step, w = "70px", ph = "") => h("input", { type: "number", value, step, placeholder: ph, style: `width:${w}` });
  const fmt = (v, d = 1) => (v == null || Number.isNaN(+v) ? "—" : (+v).toFixed(d));
  // fit numbers (sent with every flask command so the page and the server agree)
  const tip = num("0", "0.1"), wall = num("1.5", "0.1"), neckId = num("", "0.5", "80px", "optional");
  const fitArgs = () => ({ tip_mm: tip.value === "" ? 0 : +tip.value, wall_mm: wall.value === "" ? 1.5 : +wall.value,
    neck_id_mm: neckId.value === "" ? "" : +neckId.value });
  for (const i of [tip, wall, neckId]) i.addEventListener("change", () => run("flask_get"));
  const run = (name, args = {}) => cmd(name, { ...fitArgs(), ...args }).then((r) => { if (r.ok && r.flask) show(r.flask); return r; });

  // ---- touch points
  const KINDS = [["rim", "Rim", "3 or more, spread around the outside of the rim"],
    ["bulb", "Bulb", "4 or more (6–10 is better), spread around and down to the bottom"],
    ["neck", "Neck side", "Optional: the outside of the neck, between the bulb and the rim"]];
  const lists = {}, counts = {};
  const groups = KINDS.map(([k, label, tipText]) => {
    lists[k] = h("div", { class: "row wrap" });
    counts[k] = h("span", { class: "muted small" }, "");
    return h("div", { class: "col", style: "gap:4px" },
      h("div", { class: "row" }, h("span", { style: "width:80px", title: tipText }, label), counts[k], h("div", { class: "grow" }),
        lease(btn("Capture", "", () => run("flask_capture", { kind: k }), `Add where the TCP is now as a ${label.toLowerCase()} point. Nothing moves`))),
      lists[k]);
  });
  const hint = h("div", { class: "small", style: "font-weight:600" }, "");
  const fakeBtn = lease(btn("Fake touches", "quiet", () => run("flask_fake_touches"),
    "Simulated robot only: touches on a 250 ml flask 5 cm below the TCP"));
  const touchCard = h("div", { class: "card" },
    h("div", { class: "row" }, h("span", { class: "section grow", style: "margin:0" }, "Touch points"), freedriveButton()),
    note("In freedrive, touch the flask from outside with the TCP tip and capture each point.",
      "Use the active TCP: its point must be the centre of the ball tip. Rim: the outside of the rim. Bulb: all around " +
      "and down to the bottom, not only one side. Neck side points make the neck size better; without them the neck is " +
      "taken from the rim."),
    h("div", { class: "row wrap" }, h("span", { class: "k" }, "Tip radius mm"), tip, h("span", { class: "k" }, "Wall mm"), wall,
      h("span", { class: "k", title: "Measure it with a caliper if you can; otherwise it is the outer neck minus the wall" }, "Neck ID mm"), neckId),
    hint, ...groups,
    h("div", { class: "row" }, lease(btn("Reset", "quiet", async () => (await ask("Forget every touch point?")) && run("flask_reset"))),
      h("div", { class: "grow" }), fakeBtn));

  // ---- fit result + save
  const res = h("div", { class: "grid" }), warns = h("div", { class: "col", style: "gap:2px" }), err = h("div", { class: "error" });
  const pname = h("input", { type: "text", placeholder: "flask name, e.g. 250 ml A", style: "flex:1" });
  const save = lease(btn("Save flask", "primary", async () => {
    const n = pname.value.trim() || (await askText("Name of this flask (kept in the flask profiles list):", ""));
    if (!n) return;
    const w = (F.data && F.data.result && F.data.result.warnings) || [];
    if (w.length && !(await ask(`The fit has warnings:\n\n${w.join("\n")}\n\nSave it anyway?`))) return;
    run("flask_save", { name: n });
  }, "Save as a flask profile and make it the fixture \"flask\" that scrape programs use"));
  const fitCard = h("div", { class: "card" }, h("div", { class: "section" }, "Flask"), res, warns, err,
    note("Saved, the flask is the fixture \"flask\": its origin is the bulb centre, Z up the neck.",
      "Sizes are outside sizes minus the tip radius; the inner radius is the bulb minus the wall. The closest size is a " +
      "typical catalogue size, not a check. RMS is how far the touches are from the fitted shape."),
    h("div", { class: "row" }, pname, save));

  // ---- profiles
  const plist = h("tbody", {});
  const profCard = h("div", { class: "card" }, h("div", { class: "section" }, "Flask profiles"),
    h("table", { class: "su-ptable" }, h("thead", {}, h("tr", {}, ["Name", "Size", "OD mm", "Neck ID mm", "Centre mm", "Source", "In use", ""]
      .map((x) => h("th", {}, x)))), plist),
    note("Use makes a profile the fixture \"flask\". Refused while a program runs.", "Saved per robot. Deleting a profile keeps the fixture."));

  // ---- scrape generator
  const toolSel = h("select", { onchange: () => bentRow.classList.toggle("hidden", toolSel.value !== "bent") },
    h("option", { value: "straight" }, "Straight"), h("option", { value: "bent" }, "Bent"));
  const shaftR = num("3", "0.5"), shaftL = num("250", "5"), bendL = num("20", "0.5"), bendA = num("30", "1");
  const sugOut = h("div", { class: "small muted" }, "");
  const bentRow = h("div", { class: "row hidden" }, "bend length", bendL, "mm  angle", bendA, "°",
    lease(btn("Suggest bend", "quiet", async () => {
      const r = await cmd("flask_suggest_bend", { shaft_radius_mm: +shaftR.value, shaft_length_mm: +shaftL.value,
        theta_min_deg: +thMin.value, theta_max_deg: +thMax.value });
      if (!r.ok) return;
      const g = r.suggestion;
      bendL.value = g.length_mm.toFixed(1); bendA.value = g.angle_deg.toFixed(1);
      sugOut.textContent = `Suggested: ${fmt(g.length_mm)} mm at ${fmt(g.angle_deg)}° (fits ${fmt(g.length_range_mm[0])}–` +
        `${fmt(g.length_range_mm[1])} mm, ${fmt(g.angle_range_deg[0])}–${fmt(g.angle_range_deg[1])}°). ` +
        (g.band ? `Whole rings ${fmt(g.band[0])}–${fmt(g.band[1])}°` : "No whole ring") +
        (g.meets ? ": covers your polar range." : `: not all of ${fmt(g.target[0])}–${fmt(g.target[1])}°.`);
    }, "A bend that passes the neck and reaches far up the bulb, for this shaft and your polar range")));
  const tcpSel = h("select", {});
  const thMin = num("0", "1"), thMax = num("15", "1"), rings = num("3", "1"), step = num("15", "1");
  const pattern = h("select", {}, h("option", { value: "down" }, "Top to bottom"), h("option", { value: "rings" }, "Rings"));
  const press = num("0.5", "0.1"), force = num("", "0.5", "70px", "off"), speed = num("10", "1"), appr = num("50", "5");
  const reachOut = h("div", { class: "col small", style: "gap:2px" });
  const scrapeArgs = () => ({ tool: { kind: toolSel.value, shaft_radius_mm: +shaftR.value, shaft_length_mm: +shaftL.value,
      ...(toolSel.value === "bent" ? { bend_length_mm: +bendL.value, bend_angle_deg: +bendA.value } : {}) },
    tcp: tcpSel.value, theta_min_deg: +thMin.value, theta_max_deg: +thMax.value, rings: +rings.value, step_deg: +step.value,
    pattern: pattern.value, press_mm: +press.value || 0 });
  const showReach = (r) => {
    F.reach = r; ctx.repaint();
    const total = r.reachable + r.unreachable;
    reachOut.replaceChildren(
      h("div", {}, r.band ? `Whole rings reach ${fmt(r.band[0])}–${fmt(r.band[1])}° from the bottom.` : "No whole ring is reachable with this tool."),
      h("div", { class: r.unreachable ? "warn" : "good" }, `${r.reachable} of ${total} poses reachable (green), ${r.unreachable} not (red).`),
      ...r.reasons.map(([why, n]) => h("div", { class: "muted" }, `${n} × ${why}`)));
  };
  const scrapeCard = h("div", { class: "card" }, h("div", { class: "section" }, "Flask scrape"),
    note("The tool goes in through the neck and pivots there. Top to bottom (like the vial): one stroke per angle " +
      "around, from the highest polar angle down to the lowest. Rings: round the axis, from the bottom up.",
      "0° is the bottom of the bulb. Heights is how many polar angles from min to max. Press depth is how far past the inner wall the tip is aimed. Speed limits every " +
      "linear move. Force (optional) presses along the tool in a force mode. A straight tool only reaches a small band " +
      "near the bottom; a bent tool (bend length and angle of the end) reaches higher. Suggest bend proposes one."),
    h("div", { class: "grid", style: "grid-template-columns:auto 1fr" },
      h("span", { class: "k" }, "Tool"), h("div", { class: "row" }, toolSel, "shaft r", shaftR, "length", shaftL, "mm"),
      h("span", { class: "k" }, ""), h("div", { class: "col", style: "gap:2px" }, bentRow, sugOut),
      h("span", { class: "k" }, "TCP"), tcpSel,
      h("span", { class: "k" }, "Polar ° min / max"), h("div", { class: "row" }, thMin, thMax),
      h("span", { class: "k" }, "Pattern"), pattern,
      h("span", { class: "k" }, "Heights / step °"), h("div", { class: "row" }, rings, step),
      h("span", { class: "k" }, "Press mm / force N"), h("div", { class: "row" }, press, force),
      h("span", { class: "k" }, "Speed mm/s / approach mm"), h("div", { class: "row" }, speed, appr)),
    h("div", { class: "row" },
      lease(btn("Check reach", "", () => { reachOut.replaceChildren(h("div", { class: "muted" }, "Checking… (a bent tool takes a few seconds)"));
        cmd("flask_reach", scrapeArgs()).then((r) => (r.ok ? showReach(r.reach) : reachOut.replaceChildren())); },
        "Which poses the tool reaches through the neck. Nothing moves")),
      lease(btn("Make program", "primary", async () => {
        if (!(await ctx.guardProgram("make a flask scrape program"))) return;
        const r = await pcmd("program_flask_scrape", { ...scrapeArgs(), speed_mm_s: +speed.value,
          force_n: force.value === "" ? null : +force.value, approach_mm: +appr.value || 50 });
        if (r.ok) { showReach(r.reach); ctx.openProgram(r); }
      }, "Make the program and open it in PROGRAMS, unsaved. It does not run"))),
    reachOut);

  const view = h("div", { class: "col su-main" }, touchCard, fitCard, profCard, scrapeCard);

  function show(d) {
    F.data = d;
    const n = (k) => (d.points[k] || []).length;
    for (const [k] of KINDS) {
      const need = k === "rim" ? 3 : k === "bulb" ? 4 : 0;
      counts[k].textContent = need ? `${n(k)} / ${need}+` : String(n(k));
      counts[k].className = "small " + (n(k) >= need && (need || n(k)) ? "good" : "muted");
      lists[k].replaceChildren(...(d.points[k] || []).map((p, i) => h("span", { class: "chip", title: p.map((x) => (x * 1000).toFixed(1)).join(", ") + " mm" },
        `${i + 1}: ${p.map((x) => (x * 1000).toFixed(0)).join(" ")}`,
        (() => { const x = h("span", { class: "x", title: "Remove" }, " ×"); x.onclick = () => run("flask_remove", { kind: k, index: i }); return x; })())));
    }
    hint.textContent = d.hint ? d.hint.text : "";
    const r = d.result;
    const rows = r ? [["Bulb OD", fmt(r.od_mm) + " mm"], ["Inner radius", fmt(r.r_inner_mm) + " mm"], ["Neck bore", fmt(r.neck_id_mm) + " mm"],
      ["Neck OD", fmt(r.neck_od_mm) + " mm"], ["Neck length", fmt(r.neck_length_mm) + " mm"], ["Closest size", r.size],
      ["RMS rim / bulb / neck", `${fmt(r.rim_rms_mm, 2)} / ${fmt(r.bulb_rms_mm, 2)} / ${fmt(r.neck_rms_mm, 2)} mm`],
      ["Neck tilt / offset", `${fmt(r.tilt_deg)}° / ${fmt(r.axis_offset_mm)} mm`]] : [["Fit", "—"]];
    res.replaceChildren(...rows.flatMap(([k, v]) => [h("span", { class: "k" }, k), h("span", { class: "v" }, v)]));
    warns.replaceChildren(...((r && r.warnings) || []).map((w) => h("div", { class: "warn small" }, "⚠ " + w)));
    err.textContent = d.error || "";
    F.ok = !!r;
    for (const [i, key] of [[tip, "tip_mm"], [wall, "wall_mm"], [neckId, "neck_id_mm"]])
      if (document.activeElement !== i && d.settings) i.value = d.settings[key] ?? "";
    plist.replaceChildren(...(d.profiles || []).map((e) => h("tr", { class: e.in_use_as.length ? "on" : "" },
      h("td", { style: "white-space:nowrap" }, e.name), h("td", { style: "white-space:nowrap" }, e.size), h("td", { class: "num" }, fmt(e.od_mm)), h("td", { class: "num" }, fmt(e.neck_id_mm)),
      h("td", { class: "num mono small" }, e.centre_mm.join(", ")),
      h("td", { class: "muted small" }, e.source + (e.saved ? " · " + e.saved : "") + (e.warnings ? ` · ${e.warnings} warning(s)` : "")),
      h("td", { class: e.in_use_as.length ? "good" : "muted" }, e.in_use_as.join(", ") || "—"),
      h("td", {}, lease(btn("Use", "", async () => (await ask(`Use flask "${e.name}" (${e.size}) as the fixture "flask"?`)) &&
          run("flask_profile_use", { name: e.name }), `Make "${e.name}" the fixture "flask"`)),
        lease(btn("Delete", "quiet", async () => (await ask(`Delete flask profile "${e.name}"? The fixture made from it stays.`)) &&
          run("flask_profile_delete", { name: e.name })))))));
    if (!(d.profiles || []).length) plist.replaceChildren(h("tr", {}, h("td", { colspan: "8", class: "muted" }, "None saved yet.")));
    ctx.repaint();
  }

  // the server keeps the fit numbers: take them from it (the page's defaults must not overwrite them)
  function load() { return cmd("flask_get").then((r) => { if (r.ok) show(r.flask); return r; }); }

  // state at 10 Hz: the TCP list and what may be pressed
  let tcpKey = "";
  function update(s) {
    const c = s.calibration || {}, key = JSON.stringify([c.tcps?.map((t) => t.name), c.active_tcp]);
    if (key !== tcpKey) {
      const cur = tcpSel.value;
      tcpKey = key;
      tcpSel.replaceChildren(...(c.tcps || []).map((t) => h("option", { value: t.name }, t.name + (t.name === c.active_tcp ? " (active)" : ""))));
      tcpSel.value = (c.tcps || []).some((t) => t.name === cur) ? cur : c.active_tcp;
    }
    fakeBtn.classList.toggle("hidden", !ctx.fake());
    ctx.gate(save, F.ok);
  }

  return { view, load, update, shapes: () => flaskShapes(F.data, F.reach) };
}
