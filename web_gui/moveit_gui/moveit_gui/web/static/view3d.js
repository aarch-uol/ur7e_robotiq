// The 3D view of OPERATE: the robot from its URDF (/api/robot_model, meshes from /mesh/...), posed from the joint
// states, like RViz. Also: a draggable goal for the TCP (MotionPlanning-style), a ghost robot that previews a
// plan, the measured F/T and the commanded force as arrows. Display only: every command goes through app.js.
//
// Frames: ROS is z-up; the scene's world is the URDF root. Poses that leave this file are in `baseLink`.

import * as THREE from "three";
import { OrbitControls } from "/static/vendor/three/examples/jsm/controls/OrbitControls.js";
import { TransformControls } from "/static/vendor/three/examples/jsm/controls/TransformControls.js";
import { STLLoader } from "/static/vendor/three/examples/jsm/loaders/STLLoader.js";
import { ColladaLoader } from "/static/vendor/three/examples/jsm/loaders/ColladaLoader.js";

const D2R = Math.PI / 180;
const TRAIL_S = 10;                        // seconds of TCP trail
const LABEL_H = 0.032;                   // label height, fraction of the view (sizeAttenuation off)
const FRAME_AXIS = 0.05;                   // TF frame axes: 5 cm
const N_PER_M = 40;                        // arrow length: 40 N per metre (10 N = 25 cm)

function originMatrix(xyz, rpy) {
  // URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll) = three.js Euler order "ZYX"
  const m = new THREE.Matrix4().makeRotationFromEuler(new THREE.Euler(rpy[0], rpy[1], rpy[2], "ZYX"));
  m.setPosition(xyz[0], xyz[1], xyz[2]);
  return m;
}

// Pose (mm, deg; roll-pitch-yaw as ROS) <-> Matrix4 (m)
export function poseToMatrix(xyzMm, rpyDeg) {
  return originMatrix(xyzMm.map((v) => v / 1000), rpyDeg.map((v) => v * D2R));
}
export function matrixToPose(m) {
  const p = new THREE.Vector3(), q = new THREE.Quaternion(), s = new THREE.Vector3();
  m.decompose(p, q, s);
  const e = new THREE.Euler().setFromQuaternion(q, "ZYX");
  return { xyz_mm: [p.x * 1000, p.y * 1000, p.z * 1000], rpy_deg: [e.x / D2R, e.y / D2R, e.z / D2R] };
}

// A Part pose is always the pose of its FRAME in its parent frame. Shape anchoring / mesh offsets are visual
// children of that frame and deliberately never enter these two conversions.
export function partFrameWorld(parentWorld, pose) {
  return parentWorld.clone().multiply(poseToMatrix(pose.xyz_mm, pose.rpy_deg));
}
export function partPoseFromWorld(parentWorld, world) {
  return matrixToPose(parentWorld.clone().invert().multiply(world));
}
export function partGeometryOffsetMm(d) {
  const s = d.shape || {}, t = s.type;
  if (t === "mesh") return (s.offset_mm || [0, 0, 0]).slice();
  if (d.anchor !== "base") return [0, 0, 0];
  if (t === "box") return [0, 0, (s.size_mm?.[2] || 0) / 2];
  if (t === "cylinder") return [0, 0, (s.length_mm || 0) / 2];
  if (t === "sphere") return [0, 0, s.radius_mm || 0];
  return [0, 0, 0];
}

// Wavefront OBJ: positions and faces only (polygons as fans), the way the GUI's Python side reads it.
function parseObj(text) {
  const v = [], out = [];
  for (const line of text.split("\n")) {
    const p = line.trim().split(/\s+/);
    if (p[0] === "v" && p.length >= 4) v.push([+p[1], +p[2], +p[3]]);
    else if (p[0] === "f" && p.length >= 4) {
      const idx = p.slice(1).map((t) => { const k = parseInt(t, 10); return k > 0 ? k - 1 : v.length + k; });
      for (let i = 1; i < idx.length - 1; i++)
        for (const k of [idx[0], idx[i], idx[i + 1]]) { const q = v[k]; if (q) out.push(q[0], q[1], q[2]); else out.push(0, 0, 0); }
    }
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(out, 3));
  g.computeVertexNormals();
  return g;
}

class MeshCache {
  constructor() { this.stl = new STLLoader(); this.dae = new ColladaLoader(); this.cache = new Map(); }
  url(uri) {
    const m = /^package:\/\/([^/]+)\/(.+)$/.exec(uri);
    if (m) return `/mesh/${m[1]}/${m[2]}`;
    const f = /^file:\/\/.*\/parts\/meshes\/([^/]+)$/.exec(uri);     // a mesh uploaded in SETUP > Parts
    if (f) return `/mesh/_parts/${f[1]}`;
    const c = /^file:\/\/.*\/parts\/generated\/hulls\/([^/]+)$/.exec(uri); // its collision hull
    if (c) return `/mesh/_parts/hulls/${c[1]}`;
    const a = /^file:\/\/.*\/parts\/generated\/collision\/([^/]+)$/.exec(uri);
    return a ? `/mesh/_parts/collision/${a[1]}` : null;
  }
  // uploaded meshes can be replaced under the same name: forget them when the model changes
  forgetParts() { for (const k of [...this.cache.keys()]) if (k.startsWith("/mesh/_parts/")) this.cache.delete(k); }
  load(uri) {
    const url = this.url(uri);
    if (!url) return Promise.resolve(null);
    if (!this.cache.has(url)) {
      const ext = url.split(".").pop().toLowerCase();
      this.cache.set(url, new Promise((resolve) => {
        if (ext === "stl") {
          this.stl.load(url, (g) => resolve({ kind: "stl", geometry: g }), undefined, () => resolve(null));
        } else if (ext === "dae") {
          this.dae.load(url, (d) => {
            d.scene.rotation.set(0, 0, 0);    // keep the file's own (z-up) axes, as RViz does; unit scale stays
            resolve({ kind: "dae", scene: d.scene });
          }, undefined, () => resolve(null));
        } else if (ext === "obj") {
          fetch(url).then((r) => (r.ok ? r.text() : Promise.reject())).then((t) => resolve({ kind: "stl", geometry: parseObj(t) }))
            .catch(() => resolve(null));
        } else resolve(null);
      }));
    }
    return this.cache.get(url);
  }
}

// One posable instance of the robot. ghost: one translucent material for everything.
// layer "collision": the links' collision shapes instead of their visuals (MoveIt's view of the robot)
class RobotInstance {
  constructor(model, meshes, ghost, color = 0x5b8def, layer = "visual") {
    this.model = model;
    this.stubbed = new Set();
    this.root = new THREE.Group();
    this.links = {};
    this.joints = {};                       // name -> {def, motion: Group}
    this.ghost = ghost;
    this.layer = layer;
    this.material = ghost ? new THREE.MeshStandardMaterial({ color, transparent: true, opacity: 0.35,
                                                             depthWrite: false }) : null;
    const linkGroup = (name) => {
      if (!this.links[name]) {
        const g = new THREE.Group();
        g.name = name;
        this.links[name] = g;
      }
      return this.links[name];
    };
    this.root.add(linkGroup(model.root));
    for (const j of model.joints) {
      const origin = new THREE.Group();
      origin.matrixAutoUpdate = false;
      origin.matrix.copy(originMatrix(j.xyz, j.rpy));
      const motion = new THREE.Group();
      origin.add(motion);
      motion.add(linkGroup(j.child));
      linkGroup(j.parent).add(origin);
      this.joints[j.name] = { def: j, motion, axis: new THREE.Vector3(...j.axis).normalize() };
    }
    const shapes = layer === "collision" ? (model.collisions || {}) : model.links;
    for (const [name, visuals] of Object.entries(shapes)) {
      for (const v of visuals) this.addVisual(linkGroup(name), v, meshes);
    }
  }

  addVisual(parent, v, meshes) {
    const holder = new THREE.Group();
    holder.userData.linkVisual = true;
    holder.matrixAutoUpdate = false;
    holder.matrix.copy(originMatrix(v.xyz, v.rpy));
    parent.add(holder);
    const color = v.rgba ? new THREE.Color(v.rgba[0], v.rgba[1], v.rgba[2]) : new THREE.Color(0xb8bcc4);
    const mat = this.material || new THREE.MeshStandardMaterial({ color, metalness: 0.15, roughness: 0.7 });
    const g = v.geometry;
    if (g.type === "box") holder.add(new THREE.Mesh(new THREE.BoxGeometry(...g.size), mat));
    else if (g.type === "sphere") holder.add(new THREE.Mesh(new THREE.SphereGeometry(g.radius, 24, 16), mat));
    else if (g.type === "cylinder") {
      const c = new THREE.Mesh(new THREE.CylinderGeometry(g.radius, g.radius, g.length, 32), mat);
      c.rotation.x = Math.PI / 2;           // three.js cylinders run along y, URDF ones along z
      holder.add(c);
    } else if (g.type === "mesh") {
      meshes.load(g.uri).then((r) => {
        if (!r) { if (this.layer === "visual") this.stub(parent, mat); return; }
        let obj;
        if (r.kind === "stl") obj = new THREE.Mesh(r.geometry, mat);
        else {
          obj = r.scene.clone(true);
          if (this.material) obj.traverse((o) => { if (o.isMesh) o.material = this.material; });
        }
        obj.scale.multiply(new THREE.Vector3(...g.scale));
        holder.add(obj);
      });
    }
  }

  setLinkGeometryVisible(name, visible) {
    const g = this.links[name];
    if (!g) return;
    for (const c of g.children) if (c.userData.linkVisual) c.visible = visible;
  }

  // A mesh could not be loaded (package not installed on the robot PC): draw the link as rods from its origin to
  // its child joints, so the arm is still visible and posed correctly.
  stub(linkGroup, mat) {
    if (this.stubbed.has(linkGroup.name)) return;
    this.stubbed.add(linkGroup.name);
    const m = this.material || new THREE.MeshStandardMaterial({ color: 0x9aa3ad, metalness: 0.1, roughness: 0.8 });
    linkGroup.add(new THREE.Mesh(new THREE.SphereGeometry(0.035, 16, 12), m));
    for (const j of this.model.joints.filter((x) => x.parent === linkGroup.name)) {
      const end = new THREE.Vector3(...j.xyz), len = end.length();
      if (len < 0.01) continue;
      const rod = new THREE.Mesh(new THREE.CylinderGeometry(0.025, 0.025, len, 16), m);
      rod.position.copy(end.clone().multiplyScalar(0.5));
      rod.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), end.clone().normalize());
      linkGroup.add(rod);
    }
    if (this.onStub) this.onStub(linkGroup.name);
  }

  // values: {jointName: position}; mimic joints follow their leader
  setJoints(values) {
    for (const { def, motion, axis } of Object.values(this.joints)) {
      if (def.type === "fixed") continue;
      let q = values[def.name];
      if (def.mimic && values[def.mimic.joint] != null) q = values[def.mimic.joint] * def.mimic.multiplier + def.mimic.offset;
      if (q == null) continue;
      if (def.type === "prismatic") motion.position.copy(axis).multiplyScalar(q);
      else motion.quaternion.setFromAxisAngle(axis, q);
    }
  }

  worldMatrix(link) {
    const g = this.links[link];
    if (!g) return null;
    g.updateWorldMatrix(true, false);
    return g.matrixWorld.clone();
  }
}

export class View3D {
  constructor(el, { baseLink, onTargetChange }) {
    this.el = el;
    el.view3d = this;                        // reachable from the element (diagnostics, tests)
    this.baseLink = baseLink;
    this.onTargetChange = onTargetChange || (() => {});
    this.meshes = new MeshCache();
    this.robot = null;
    this.ghost = null;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xeef0f3);
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.01, 50);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(1.7, -1.5, 1.25);
    this.renderer = new THREE.WebGLRenderer({ antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    el.append(this.renderer.domElement);
    this.show = View3D.savedShow();
    this.showBar();
    this.orbit = new OrbitControls(this.camera, this.renderer.domElement);
    this.orbit.target.set(0.15, 0, 0.4);
    this.orbit.update();
    this.orbit.addEventListener("change", () => this.draw());

    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8a8f99, 0.9));
    const sun = new THREE.DirectionalLight(0xffffff, 0.8);
    sun.position.set(2, -1.5, 3);
    this.scene.add(sun);
    const grid = new THREE.GridHelper(2, 20, 0xb0b6c0, 0xd2d6dc);
    grid.rotation.x = Math.PI / 2;           // GridHelper lies in x-z; ours is the floor, x-y
    this.scene.add(grid);
    this.scene.add(new THREE.AxesHelper(0.15));

    // Goal of the TCP, dragged with the gizmo (MotionPlanning's interactive marker)
    this.target = new THREE.Group();
    this.target.add(new THREE.AxesHelper(0.08));
    this.target.add(new THREE.Mesh(new THREE.SphereGeometry(0.008, 16, 12),
                                   new THREE.MeshBasicMaterial({ color: 0xf59e0b })));
    this.target.visible = false;
    this.scene.add(this.target);
    this.gizmo = new TransformControls(this.camera, this.renderer.domElement);
    this.gizmo.setSpace("local");
    this.gizmo.setSize(0.8);
    this.gizmoOwner = null;
    this.gizmoDragging = false;
    this.gizmo.addEventListener("dragging-changed", (e) => {
      this.orbit.enabled = !e.value;
      this.gizmoDragging = e.value;
      if (this.gizmoOwner === "part") this.partDragChanged(e.value);
    });
    this.gizmo.addEventListener("objectChange", () => {
      if (this.gizmoOwner === "target") this.onTargetChange(this.targetPose());
      else if (this.gizmoOwner === "part") this.partGizmoChanged();
    });
    this.gizmo.addEventListener("change", () => this.draw());
    this.scene.add(this.gizmo);

    // SETUP > Parts draft: the geometry is a child of the part frame; a separate proxy lets TransformControls
    // present either the part's local axes or its parent's axes without changing the saved frame convention.
    this.partDraftFrame = new THREE.Group();
    this.partDraftFrame.visible = false;
    this.partDraftVisual = new THREE.Group();
    this.partDraftFrame.add(this.partDraftVisual);
    this.partDraftCollision = new THREE.Group();
    this.partDraftCollision.visible = this.show.collision;
    this.partDraftFrame.add(this.partDraftCollision);
    this.scene.add(this.partDraftFrame);
    this.partGizmoProxy = new THREE.Group();
    this.partGizmoProxy.visible = false;
    this.scene.add(this.partGizmoProxy);
    this.partDraft = null;
    this.partShapeKey = "";
    this.partShapeRev = 0;
    this.partMeshRev = -1;
    this.partHiddenLink = null;
    this.partSpace = "local";
    this.partSnap = 0;
    this.partDrag = null;

    this.ftArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.1, 0xdc2626, 0.03, 0.018);
    this.ftArrow.visible = false;
    this.cmdArrow = new THREE.ArrowHelper(new THREE.Vector3(0, 0, 1), new THREE.Vector3(), 0.1, 0x16a34a, 0.03, 0.018);
    this.cmdArrow.visible = false;
    this.scene.add(this.ftArrow, this.cmdArrow);

    this.preview = null;                     // {points:[{t,q}], names, t0}
    this.frameGroup = new THREE.Group();     // TF frames overlay
    this.scene.add(this.frameGroup);
    this.labels = new Map();                 // frame name -> sprite material (text texture, cached)
    this.pathLine = null;                    // planned TCP path while a preview plays
    this.trailOn = true;
    this.trail = [];                         // [{p: Vector3, t: ms}] actual TCP, last TRAIL_S seconds
    this.trailLine = null;
    this.follow = false;
    new ResizeObserver(() => this.resize()).observe(el);
    this.resize();
  }

  // What of the robot to draw: its visual meshes, its collision shapes (translucent green, what MoveIt checks),
  // or both. Shared by every 3D view of the GUI (localStorage), switched from the corner of each view.
  showBar() {
    const bar = document.createElement("div");
    bar.className = "v3show";
    for (const [k, label, tip] of [["visual", "Visual", "The robot's visual meshes"],
      ["collision", "Collision", "The collision shapes MoveIt plans with (robot links and parts)"]]) {
      const l = document.createElement("label");
      l.title = tip;
      const c = document.createElement("input");
      c.type = "checkbox";
      c.checked = this.show[k];
      c.dataset.k = k;
      c.onchange = () => this.setShow({ [k]: c.checked });
      l.append(c, " " + label);
      bar.append(l);
    }
    this.el.append(bar);
    this.showEl = bar;
    window.addEventListener("storage", (e) => { if (e.key === "v3-show") this.setShow(View3D.savedShow(), false); });
    window.addEventListener("v3-show", (e) => { if (e.detail !== this) this.setShow(View3D.savedShow(), false); });
  }
  static savedShow() {
    try { return { visual: true, collision: false, ...JSON.parse(localStorage.getItem("v3-show") || "{}") }; }
    catch { return { visual: true, collision: false }; }
  }
  setShow(s, save = true) {
    this.show = { ...this.show, ...s };
    if (save) {                                            // every 3D view of the GUI follows (this tab and others)
      localStorage.setItem("v3-show", JSON.stringify(this.show));
      window.dispatchEvent(new CustomEvent("v3-show", { detail: this }));
    }
    if (this.showEl) for (const c of this.showEl.querySelectorAll("input")) c.checked = this.show[c.dataset.k];
    if (this.robot) {
      if (this.show.collision && !this.colBot) {           // built on first use (it loads the collision meshes)
        this.colBot = new RobotInstance(this.model, this.meshes, true, 0x2fbf71, "collision");
        this.colBot.material.opacity = 0.45;
        this.scene.add(this.colBot.root);
        if (this.robot.values) this.colBot.setJoints(this.robot.values);
        if (this.partHiddenLink) this.colBot.setLinkGeometryVisible(this.partHiddenLink, false);
      }
      this.robot.root.visible = this.show.visual;
      if (this.colBot) this.colBot.root.visible = this.show.collision;
      if (this.partDraftCollision) this.partDraftCollision.visible = this.show.collision;
    }
    this.draw();
  }

  async load(model) {
    if (this.robot) return;
    this.model = model;
    this.robot = new RobotInstance(model, this.meshes, false);
    this.ghost = new RobotInstance(model, this.meshes, true);
    this.ghost.root.visible = false;
    // RViz MotionPlanning's orange "goal state": follows the goal marker through IK
    this.goalBot = new RobotInstance(model, this.meshes, true, 0xf28c28);
    this.goalBot.root.visible = false;
    this.scene.add(this.robot.root, this.ghost.root, this.goalBot.root);
    this.setShow({}, false);
    this.draw();
  }

  // The robot model changed (a part saved in SETUP > Parts): rebuild the three instances, keep what was shown.
  async reload(model) {
    if (!this.robot) return this.load(model);
    const vis = [this.ghost.root.visible, this.goalBot.root.visible];
    const q = this.robot.values || null;
    for (const r of [this.robot, this.ghost, this.goalBot, this.colBot]) {
      if (!r) continue;
      this.scene.remove(r.root);
      r.root.traverse((o) => { if (o.geometry) o.geometry.dispose(); });
    }
    this.robot = null;
    this.colBot = null;
    this.meshes.forgetParts();
    await this.load(model);
    [this.ghost.root.visible, this.goalBot.root.visible] = vis;
    if (q) this.robot.setJoints(q);
    this.placePartDraft();
    if (this.partHiddenLink) this.hidePartLink(this.partHiddenLink, true);
    this.draw();
  }

  resize() {
    const w = this.el.clientWidth || 300, h = this.el.clientHeight || 300;
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.draw();
  }

  draw() {
    if (this._pending) return;
    this._pending = true;
    requestAnimationFrame(() => {
      this._pending = false;
      if (this.preview) this.stepPreview();
      this.renderer.render(this.scene, this.camera);
    });
  }

  base() { return (this.robot && this.robot.worldMatrix(this.baseLink)) || new THREE.Matrix4(); }

  // tcpOff: {link, xyz_mm, rpy_deg}, the active TCP as a fixed offset from a model link (state.tcp_off). Paths are
  // drawn at link * offset, so a virtual TCP (e.g. a spatula tip, not in the URDF) gets its own line
  setJoints(values, tcpOff) {
    if (!this.robot) return;
    this.robot.setJoints(values);
    this.robot.values = values;
    if (this.colBot) this.colBot.setJoints(values);
    if (tcpOff !== undefined) {
      const ok = tcpOff && this.robot.links[tcpOff.link];
      this.tcpOff = ok ? { link: tcpOff.link, M: poseToMatrix(tcpOff.xyz_mm, tcpOff.rpy_deg) } : null;
    }
    this.placePartDraft();
    this.draw();
  }

  // ---------------------------------------------------------------- goal
  attachGizmo(owner, object) {
    if (this.gizmoOwner === owner && this.gizmo.object === object) return;
    this.gizmo.detach();
    this.gizmoOwner = owner;
    if (object) this.gizmo.attach(object);
  }
  detachGizmo(owner) {
    if (owner && this.gizmoOwner !== owner) return;
    this.gizmo.detach();
    this.gizmoOwner = null;
  }
  showTarget(on) {
    this.target.visible = on;
    if (on) {
      this.clearPartDraft();
      this.gizmo.setSpace("local");
      this.gizmo.setTranslationSnap(null);
      this.gizmo.setRotationSnap(null);
      this.attachGizmo("target", this.target);
    } else this.detachGizmo("target");
    this.draw();
  }
  setGizmoMode(mode) { this.gizmo.setMode(mode); this.draw(); }
  // pose: {xyz_mm, rpy_deg} in baseLink
  setTarget(pose) {
    const m = this.base().multiply(poseToMatrix(pose.xyz_mm, pose.rpy_deg));
    m.decompose(this.target.position, this.target.quaternion, this.target.scale);
    this.draw();
  }
  targetPose() {
    this.target.updateMatrixWorld(true);
    return matrixToPose(this.base().invert().multiply(this.target.matrixWorld));
  }

  // ---------------------------------------------------------- SETUP Parts draft
  hidePartLink(name, hide) {
    for (const r of [this.robot, this.colBot]) if (r) r.setLinkGeometryVisible(name, !hide);
  }
  clearPartDraft() {
    if (this.partHiddenLink) this.hidePartLink(this.partHiddenLink, false);
    this.partHiddenLink = null;
    this.partDraft = null;
    this.partDraftFrame.visible = false;
    this.partGizmoProxy.visible = false;
    this.partDrag = null;
    this.partShapeRev++;
    if (this.gizmoOwner === "part") this.detachGizmo("part");
    this.draw();
  }
  setPartDraft(draft, onChange) {
    if (!draft) return this.clearPartDraft();
    if (this.gizmoOwner === "target") this.showTarget(false);
    if (this.partHiddenLink && this.partHiddenLink !== draft.editing) this.hidePartLink(this.partHiddenLink, false);
    this.partHiddenLink = draft.editing || null;
    if (this.partHiddenLink) this.hidePartLink(this.partHiddenLink, true);
    this.partDraft = { ...draft, onChange };
    this.partSpace = draft.space === "parent" ? "parent" : "local";
    this.partSnap = +draft.snap || 0;
    this.partDraftFrame.visible = true;
    if (draft.mesh_rev !== this.partMeshRev) {
      this.partMeshRev = draft.mesh_rev;
      this.meshes.forgetParts();
    }
    const shapeKey = JSON.stringify([draft.shape, draft.anchor, draft.color, draft.mesh_rev]);
    if (shapeKey !== this.partShapeKey) {
      this.partShapeKey = shapeKey;
      this.buildPartDraftShape(draft);
    }
    this.gizmo.setMode(draft.mode === "rotate" ? "rotate" : "translate");
    const step = this.partSnap / 1000;
    this.gizmo.setTranslationSnap(step || null);
    this.gizmo.setRotationSnap(this.partSnap ? this.partSnap * D2R : null);
    this.placePartDraft();
    if (!this.gizmoDragging) {
      this.syncPartGizmo();
      this.attachGizmo("part", this.partGizmoProxy);
    }
    this.draw();
  }
  buildPartDraftShape(d) {
    this.partShapeRev++;
    const rev = this.partShapeRev;
    for (const o of [...this.partDraftVisual.children]) {
      this.partDraftVisual.remove(o);
      o.traverse((x) => { if (x.geometry) x.geometry.dispose(); });
    }
    for (const o of [...this.partDraftCollision.children]) {
      this.partDraftCollision.remove(o);
      o.traverse((x) => { if (x.geometry) x.geometry.dispose(); });
    }
    const mat = new THREE.MeshStandardMaterial({ color: 0xf59e0b, transparent: true, opacity: 0.48,
                                                 depthWrite: false, metalness: 0.05, roughness: 0.65 });
    const s = d.shape || {}, off = partGeometryOffsetMm(d).map((x) => x / 1000);
    let obj = null;
    if (s.type === "box") obj = new THREE.Mesh(new THREE.BoxGeometry(...s.size_mm.map((x) => x / 1000)), mat);
    else if (s.type === "sphere") obj = new THREE.Mesh(new THREE.SphereGeometry(s.radius_mm / 1000, 24, 16), mat);
    else if (s.type === "cylinder") {
      obj = new THREE.Mesh(new THREE.CylinderGeometry(s.radius_mm / 1000, s.radius_mm / 1000, s.length_mm / 1000, 32), mat);
      obj.rotation.x = Math.PI / 2;
    } else if (s.type === "mesh" && s.file) {
      this.meshes.load(`file:///parts/meshes/${s.file}`).then((r) => {
        if (!r || rev !== this.partShapeRev || !this.partDraft) return;
        let mesh;
        if (r.kind === "stl") mesh = new THREE.Mesh(r.geometry, mat);
        else {
          mesh = r.scene.clone(true);
          mesh.traverse((o) => {
            if (!o.isMesh || !o.material) return;
            const mats = Array.isArray(o.material) ? o.material : [o.material];
            const copy = mats.map((m) => { const q = m.clone(); q.transparent = true; q.opacity = 0.58; q.depthWrite = false; return q; });
            o.material = Array.isArray(o.material) ? copy : copy[0];
          });
        }
        mesh.position.set(...off);
        mesh.scale.multiplyScalar(+s.scale || 1);
        this.partDraftVisual.add(mesh);
        this.draw();
      });
    }
    if (obj) {
      obj.position.set(...off);
      this.partDraftVisual.add(obj);
    }
    const cm = new THREE.MeshStandardMaterial({ color: 0x2fbf71, transparent: true, opacity: 0.42,
                                                depthWrite: false, metalness: 0, roughness: 0.8 });
    for (const g of d.collision_preview || []) {
      this.meshes.load(g.uri).then((r) => {
        if (!r || rev !== this.partShapeRev || !this.partDraft) return;
        let mesh;
        if (r.kind === "stl") mesh = new THREE.Mesh(r.geometry, cm);
        else {
          mesh = r.scene.clone(true);
          mesh.traverse((o) => { if (o.isMesh) o.material = cm; });
        }
        mesh.position.set(...off);
        mesh.scale.multiply(new THREE.Vector3(...g.scale));
        this.partDraftCollision.add(mesh);
        this.draw();
      });
    }
    const ax = new THREE.AxesHelper(0.06);
    ax.material.depthTest = false;
    ax.renderOrder = 6;
    this.partDraftVisual.add(ax);
  }
  partParentWorld() {
    return this.partDraft && this.robot && this.robot.worldMatrix(this.partDraft.parent);
  }
  placePartDraft() {
    if (!this.partDraft || !this.partDraftFrame.visible) return;
    const parent = this.partParentWorld();
    if (!parent) { this.partDraftFrame.visible = false; return; }
    this.partDraftFrame.visible = true;
    const M = partFrameWorld(parent, this.partDraft);
    M.decompose(this.partDraftFrame.position, this.partDraftFrame.quaternion, this.partDraftFrame.scale);
    if (!this.gizmoDragging) this.syncPartGizmo();
  }
  syncPartGizmo() {
    if (!this.partDraft || !this.partDraftFrame.visible) return;
    const parent = this.partParentWorld();
    if (!parent) return;
    this.partDraftFrame.updateMatrixWorld(true);
    const M = this.partDraftFrame.matrixWorld, p = new THREE.Vector3().setFromMatrixPosition(M);
    let q = new THREE.Quaternion().setFromRotationMatrix(M);
    if (this.partSpace === "parent") q = new THREE.Quaternion().setFromRotationMatrix(parent);
    this.partGizmoProxy.position.copy(p);
    this.partGizmoProxy.quaternion.copy(q);
    this.partGizmoProxy.scale.set(1, 1, 1);
    this.partGizmoProxy.visible = true;
    this.gizmo.setSpace("local");
  }
  partDragChanged(on) {
    if (!this.partDraft) return;
    if (on) {
      const parent = this.partParentWorld();
      this.partGizmoProxy.updateMatrixWorld(true);
      this.partDrag = parent ? { parent: parent.clone(), pose: poseToMatrix(this.partDraft.xyz_mm, this.partDraft.rpy_deg) } : null;
    } else {
      this.partDrag = null;
      this.syncPartGizmo();
    }
  }
  partGizmoChanged() {
    const d = this.partDraft, parent = this.partParentWorld();
    if (!d || !parent) return;
    this.partGizmoProxy.updateMatrixWorld(true);
    let pose;
    if (this.partSpace === "local") {
      pose = partPoseFromWorld(parent, this.partGizmoProxy.matrixWorld);
    } else if (this.gizmo.getMode() === "translate") {
      const p = new THREE.Vector3().setFromMatrixPosition(this.partGizmoProxy.matrixWorld).applyMatrix4(parent.clone().invert());
      pose = { xyz_mm: [p.x * 1000, p.y * 1000, p.z * 1000], rpy_deg: d.rpy_deg.slice() };
    } else {
      const start = this.partDrag?.pose || poseToMatrix(d.xyz_mm, d.rpy_deg);
      const qp = new THREE.Quaternion().setFromRotationMatrix(parent);
      const qg = new THREE.Quaternion().setFromRotationMatrix(this.partGizmoProxy.matrixWorld);
      const delta = qp.clone().invert().multiply(qg);
      const qs = new THREE.Quaternion().setFromRotationMatrix(start);
      const T = new THREE.Matrix4().compose(new THREE.Vector3(...d.xyz_mm.map((x) => x / 1000)),
        delta.multiply(qs), new THREE.Vector3(1, 1, 1));
      pose = matrixToPose(T);
    }
    d.xyz_mm = pose.xyz_mm.slice();
    d.rpy_deg = pose.rpy_deg.slice();
    const M = partFrameWorld(parent, pose);
    M.decompose(this.partDraftFrame.position, this.partDraftFrame.quaternion, this.partDraftFrame.scale);
    if (d.onChange) d.onChange(pose);
    this.draw();
  }

  // ------------------------------------------------------------- preview
  // plan: {points:[{t, q:[rad]}]} in jointNames order; plays in a loop until clearPreview()
  playPreview(plan, jointNames, extra, keepPath = false) {
    if (!this.ghost || !plan || !plan.points.length) return;
    this.preview = { points: plan.points, names: jointNames, extra: extra || {}, t0: performance.now() };
    if (!keepPath) this.setPath(this.planPath(plan.points, jointNames, extra || {}));
    this.ghost.root.visible = true;
    this.draw();
  }
  clearPreview() {
    this.preview = null;
    this.setPath(null);
    if (this.ghost) this.ghost.root.visible = false;
    this.draw();
  }
  stepPreview() {
    const p = this.preview, pts = p.points, T = pts[pts.length - 1].t;
    const hold = 0.8;                          // show the goal a moment before looping
    let t = ((performance.now() - p.t0) / 1000) % (T + hold);
    if (t > T) t = T;
    let i = 1;
    while (i < pts.length - 1 && pts[i].t < t) i++;
    const a = pts[i - 1], b = pts[i], s = b.t > a.t ? Math.min(1, Math.max(0, (t - a.t) / (b.t - a.t))) : 1;
    const v = { ...p.extra };
    p.names.forEach((n, k) => (v[n] = a.q[k] + (b.q[k] - a.q[k]) * s));
    this.ghost.setJoints(v);
    requestAnimationFrame(() => this.draw());
  }

  // ----------------------------------------------------------- path lines
  tcpLink(robot) {
    if (this.pathLink && robot.links[this.pathLink]) return this.pathLink;
    return ["robotiq_tcp", "tool0", "flange", "wrist_3_link"].find((n) => robot.links[n]) || null;
  }
  // TCP path of the plan, through the ghost's forward kinematics (restores the ghost pose afterwards)
  planPath(points, names, extra) {
    const off = this.tcpOff && this.ghost && this.ghost.links[this.tcpOff.link] ? this.tcpOff : null;
    const link = off ? off.link : this.ghost && this.tcpLink(this.ghost);
    if (!link) return null;
    const pts = [], step = Math.max(1, Math.ceil(points.length / 400));
    for (let i = 0; i < points.length; i += step) pts.push(points[i]);
    if (pts[pts.length - 1] !== points[points.length - 1]) pts.push(points[points.length - 1]);
    const out = [];
    for (const pt of pts) {
      const v = { ...extra };
      names.forEach((n, k) => (v[n] = pt.q[k]));
      this.ghost.setJoints(v);
      const W = this.ghost.worldMatrix(link).clone();
      out.push(new THREE.Vector3().setFromMatrixPosition(off ? W.multiply(off.M) : W));   // the active TCP
    }
    return out;
  }
  setPath(pts) {
    if (this.pathLine) {
      this.scene.remove(this.pathLine);
      this.pathLine.geometry.dispose();
      this.pathLine = null;
    }
    if (pts && pts.length > 1) {
      this.pathLine = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color: 0x2b5fb8, depthTest: false }));
      this.pathLine.renderOrder = 5;
      this.scene.add(this.pathLine);
    }
    this.draw();
  }
  // one part of the path drawn over it (PROGRAMS step-through: the move being shown), world points, or null
  setHighlight(pts) {
    if (this.hiLine) { this.scene.remove(this.hiLine); this.hiLine.traverse((o) => o.geometry && o.geometry.dispose()); this.hiLine = null; }
    if (pts && pts.length) {
      const g = new THREE.Group();
      if (pts.length > 1) {
        const l = new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
          new THREE.LineBasicMaterial({ color: 0xf59e0b, depthTest: false }));
        l.renderOrder = 6; g.add(l);
      }
      const end = new THREE.Mesh(new THREE.SphereGeometry(0.008, 14, 10),
        new THREE.MeshBasicMaterial({ color: 0xf59e0b, depthTest: false }));
      end.position.copy(pts[pts.length - 1]); end.renderOrder = 6; g.add(end);
      this.hiLine = g; this.scene.add(g);
    }
    this.draw();
  }
  // a path given in the base frame, mm (PROGRAMS: the program's TCP path)
  setPathMm(ptsMm) {
    const B = this.base();
    this.setPath(ptsMm ? ptsMm.map(([x, y, z]) => new THREE.Vector3(x / 1000, y / 1000, z / 1000).applyMatrix4(B)) : null);
  }
  setTrail(on) {
    this.trailOn = on;
    this.trail = [];
    this.rebuildTrail();
  }
  // pose: the live TCP {xyz_mm, rpy_deg} in baseLink (10 Hz)
  pushTcp(pose) {
    if (!pose) return;
    const now = performance.now();
    const p = new THREE.Vector3().setFromMatrixPosition(this.base().multiply(poseToMatrix(pose.xyz_mm, pose.rpy_deg)));
    if (this.follow && this.lastTcp) {
      const d = p.clone().sub(this.lastTcp);
      this.orbit.target.add(d);
      this.camera.position.add(d);
      this.orbit.update();
    }
    this.lastTcp = p;
    if (!this.trailOn) return;
    const last = this.trail[this.trail.length - 1];
    if (!last || last.p.distanceToSquared(p) > 1e-8) this.trail.push({ p, t: now });
    while (this.trail.length && now - this.trail[0].t > TRAIL_S * 1000) this.trail.shift();
    this.rebuildTrail();
  }
  rebuildTrail() {
    if (this.trailLine) {
      this.scene.remove(this.trailLine);
      this.trailLine.geometry.dispose();
      this.trailLine = null;
    }
    const n = this.trail.length;
    if (this.trailOn && n > 1) {
      const bg = this.scene.background, col = new THREE.Color(0xd9480f), c = [];
      const now = performance.now();
      for (const { t } of this.trail) {
        const a = Math.max(0, 1 - (now - t) / (TRAIL_S * 1000));       // 1 = fresh
        c.push(bg.r + (col.r - bg.r) * a, bg.g + (col.g - bg.g) * a, bg.b + (col.b - bg.b) * a);
      }
      const g = new THREE.BufferGeometry().setFromPoints(this.trail.map((q) => q.p));
      g.setAttribute("color", new THREE.Float32BufferAttribute(c, 3));
      this.trailLine = new THREE.Line(g, new THREE.LineBasicMaterial({ vertexColors: true }));
      this.scene.add(this.trailLine);
    }
    this.draw();
  }

  // --------------------------------------------------------------- camera
  setFollow(on) {
    this.follow = on;
    if (on && this.lastTcp) {
      const d = this.lastTcp.clone().sub(this.orbit.target);
      this.orbit.target.add(d);
      this.camera.position.add(d);
      this.orbit.update();
    }
  }
  // name: top | front | side | iso. Keeps the orbit target and the distance to it.
  setCameraPreset(name) {
    const dist = Math.max(0.5, this.camera.position.distanceTo(this.orbit.target));
    const dir = { top: [0, 0, 1], front: [0, -1, 0], side: [1, 0, 0], iso: [0.62, -0.55, 0.56] }[name];
    if (!dir) return;
    const v = new THREE.Vector3(...dir).normalize().multiplyScalar(dist);
    if (name === "top") v.y = -1e-3 * dist;           // keep "up" (z) well-defined: x to the right, y up
    this.camera.position.copy(this.orbit.target).add(v);
    this.orbit.update();
    this.draw();
  }

  // ------------------------------------------------------- TF frames overlay
  // frames: [{name, xyz_mm, rpy_deg}] in baseLink (checked ones only); axes ~5 cm plus a name label
  setFrames(frames) {
    for (const o of this.frameGroup.children) o.traverse((x) => { if (x.geometry) x.geometry.dispose(); });
    this.frameGroup.clear();
    const B = this.base();
    for (const f of frames) {
      if (!f.xyz_mm) continue;
      const g = new THREE.Group();
      new THREE.Matrix4().multiplyMatrices(B, poseToMatrix(f.xyz_mm, f.rpy_deg)).decompose(g.position, g.quaternion, g.scale);
      const ax = new THREE.AxesHelper(FRAME_AXIS);
      ax.material.depthTest = false;
      ax.renderOrder = 6;
      g.add(ax);
      const sp = new THREE.Sprite(this.labelMaterial(f.name));
      // constant on-screen size (readable when zoomed out), anchored at its lower-left corner next to the origin
      sp.scale.set(LABEL_H * sp.material.userData.aspect, LABEL_H, 1);
      sp.center.set(-0.05, -0.2);
      sp.position.set(0, 0, 0);
      sp.renderOrder = 7;
      g.add(sp);
      this.frameGroup.add(g);
    }
    this.draw();
  }
  labelMaterial(text) {
    let m = this.labels.get(text);
    if (m) return m;
    const c = document.createElement("canvas"), x = c.getContext("2d");
    const font = "bold 28px sans-serif";
    x.font = font;
    c.width = Math.ceil(x.measureText(text).width) + 12;
    c.height = 40;
    x.font = font;
    x.textBaseline = "middle";
    x.fillStyle = "rgba(255,255,255,0.75)";
    x.fillRect(0, 4, c.width, 32);
    x.fillStyle = "#1f2733";
    x.fillText(text, 6, 21);
    const tex = new THREE.CanvasTexture(c);
    tex.colorSpace = THREE.SRGBColorSpace;
    m = new THREE.SpriteMaterial({ map: tex, depthTest: false, transparent: true, sizeAttenuation: false });
    m.userData.aspect = c.width / c.height;
    this.labels.set(text, m);
    return m;
  }

  // ------------------------------------------------------- keep-out zones
  // boxes: [{name, size_mm, enabled, base: {xyz_mm, rpy_deg}}] (box centre in baseLink). Enabled: translucent red
  // with edges; disabled: dashed grey edges only. Rebuilt only when the list changes (state arrives at 10 Hz).
  setZones(boxes, on) {
    const key = on ? JSON.stringify([!!this.robot, (boxes || []).map((b) => [b.name, b.size_mm, b.enabled, b.base])]) : "";
    if (key === this.zonesKey && this.zones) return;
    this.zonesKey = key;
    if (!this.zones) { this.zones = new THREE.Group(); this.scene.add(this.zones); }
    for (const o of this.zones.children) o.traverse((x) => { if (x.geometry) x.geometry.dispose(); });
    this.zones.clear();
    if (!on) { this.draw(); return; }
    const B = this.base();
    for (const b of boxes || []) {
      if (!b.base) continue;                   // frame unresolvable: listed as an error on SETUP
      const g = new THREE.Group();
      g.name = "zone:" + b.name;
      new THREE.Matrix4().multiplyMatrices(B, poseToMatrix(b.base.xyz_mm, b.base.rpy_deg)).decompose(g.position, g.quaternion, g.scale);
      const geo = new THREE.BoxGeometry(...b.size_mm.map((v) => v / 1000));
      if (b.enabled) {
        const fill = new THREE.Mesh(geo, new THREE.MeshBasicMaterial({ color: 0xef4444, transparent: true, opacity: 0.22,
                                                                        depthWrite: false, side: THREE.DoubleSide }));
        fill.renderOrder = 2;
        g.add(fill);
      }
      const edges = new THREE.LineSegments(new THREE.EdgesGeometry(geo), b.enabled
        ? new THREE.LineBasicMaterial({ color: 0xc2410c })
        : new THREE.LineDashedMaterial({ color: 0x8b95a1, dashSize: 0.02, gapSize: 0.015 }));
      if (!b.enabled) edges.computeLineDistances();
      g.add(edges);
      const sp = new THREE.Sprite(this.labelMaterial(b.name + (b.enabled ? "" : " (off)")));
      sp.scale.set(LABEL_H * sp.material.userData.aspect, LABEL_H, 1);
      sp.center.set(0.5, -0.3);
      sp.position.set(0, 0, b.size_mm[2] / 2000);
      sp.renderOrder = 7;
      g.add(sp);
      this.zones.add(g);
    }
    this.draw();
  }

  // ------------------------------------------------------------ waypoints
  // markers: [{xyz_mm, rpy_deg, selected}] in baseLink (program pose waypoints)
  setMarkers(markers, line = true) {
    if (!this.markers) { this.markers = new THREE.Group(); this.scene.add(this.markers); }
    this.markers.clear();
    const pts = [];
    for (const mk of markers) {
      const g = new THREE.Group();
      const m = this.base().multiply(poseToMatrix(mk.xyz_mm, mk.rpy_deg));
      m.decompose(g.position, g.quaternion, g.scale);
      g.add(new THREE.Mesh(new THREE.SphereGeometry(mk.selected ? 0.012 : 0.007, 14, 10),
        new THREE.MeshBasicMaterial({ color: mk.selected ? 0xf59e0b : 0x2b5fb8 })));
      if (mk.selected) g.add(new THREE.AxesHelper(0.06));
      this.markers.add(g);
      pts.push(g.position.clone());
    }
    if (line && pts.length > 1)        // waypoint order (not the motion: setPathMm draws that)
      this.markers.add(new THREE.Line(new THREE.BufferGeometry().setFromPoints(pts),
        new THREE.LineBasicMaterial({ color: 0x7f8ea3 })));
    this.draw();
  }
  // ---------------------------------------------------------- goal state
  // The orange goal robot: IK for the goal marker (the TCP pose), seeded from the arm's current joints, within
  // `limits` ({name: [lo, hi] rad}). Damped least squares on the robot model in the page: instant, nothing is sent
  // to the robot, no collision check (Plan does that). Returns {ok, q} and keeps the last reachable pose shown.
  showGoalState(on, names, seed, limits, tcpPose) {
    const g = this.goalBot;
    if (!g) return null;
    if (!on) { g.root.visible = false; this.goalQ = null; this.draw(); return null; }
    // the active TCP relative to the link the solver drives (a virtual TCP is an offset from it): the exact
    // offset from the state when there is one, else measured from the live TCP pose
    const exact = this.tcpOff && this.robot.links[this.tcpOff.link] ? this.tcpOff : null;
    const link = exact ? exact.link : this.tcpLink(this.robot);
    if (!link || !tcpPose) return null;
    const off = exact ? exact.M.clone() : this.robot.worldMatrix(link).clone().invert()
      .multiply(this.base().multiply(poseToMatrix(tcpPose.xyz_mm, tcpPose.rpy_deg)));
    this.target.updateMatrixWorld(true);
    const want = this.target.matrixWorld.clone().multiply(off.clone().invert());
    const start = this.goalQ && this.goalQ.length === names.length ? this.goalQ : names.map((n) => seed[n] ?? 0);
    const fk = (qq) => { const v = { ...seed }; names.forEach((n, i) => (v[n] = qq[i])); g.setJoints(v); return g.worldMatrix(link); };
    const lim = names.map((n) => (limits && limits[n]) || [-2 * Math.PI, 2 * Math.PI]);
    let q = solveIK(fk, want, start, lim);
    if (!q) {
      // like RViz's KDL solver: restart from nudged seeds (a stretched or folded arm is singular), keep the one
      // closest to the arm's current joints
      const cur = names.map((n) => seed[n] ?? 0), best = [];
      const kicks = [[0, 0, 0.6, 0, 0, 0], [0, 0, -0.6, 0, 0, 0], [0, -0.4, 0.8, -0.4, 0, 0], [0, 0.4, -0.8, 0.4, 0, 0],
        [0, 0, 0.6, 0, 0.6, 0], [0, 0, -0.6, 0, -0.6, 0]];
      for (const k of kicks) {
        const r = solveIK(fk, want, cur.map((v, i) => v + (k[i] || 0)), lim);
        if (r) best.push(r);
      }
      const dist = (a) => a.reduce((acc, v, i) => acc + (v - cur[i]) ** 2, 0);
      q = best.sort((a, b) => dist(a) - dist(b))[0] || null;
    }
    if (q) this.goalQ = q;
    if (this.goalQ) {
      const v = { ...seed };
      names.forEach((n, i) => (v[n] = this.goalQ[i]));
      g.setJoints(v);
      g.root.visible = true;
    }
    this.draw();
    return { ok: !!q, q: this.goalQ };
  }

  // PROGRAMS preview playback: a joint trajectory {points: [{t, q}]} for playPreview() from the program's TCP poses
  // ([x, y, z mm, qx, qy, qz, qw] in the base frame), known joints (MoveJ targets) and per move `linear`.
  // Joint moves are interpolated in joint space (like the planner); linear moves are cut into <= 10 mm / 5 deg
  // steps solved with IK, each seeded from the last. Kinematics only, for seeing the program, not checking it.
  programTrajectory(poses, joints, linear, names, seed, limits) {
    const g = this.ghost, link = this.tcpLink(this.robot);
    if (!g || !link || !poses || poses.length < 2) return null;
    const base = this.base();
    const M = (p) => base.clone().multiply(new THREE.Matrix4().compose(new THREE.Vector3(p[0] / 1000, p[1] / 1000, p[2] / 1000),
      new THREE.Quaternion(p[3], p[4], p[5], p[6]), new THREE.Vector3(1, 1, 1)));
    // the active TCP relative to the link IK drives: the robot drawn at `seed` and the first pose are the same moment
    const v0 = { ...seed };
    g.setJoints(v0);
    const offInv = g.worldMatrix(link).clone().invert().multiply(M(poses[0])).invert();
    const lim = names.map((n) => (limits && limits[n]) || [-2 * Math.PI, 2 * Math.PI]);
    const fk = (qq) => { const v = { ...seed }; names.forEach((n, i) => (v[n] = qq[i])); g.setJoints(v); return g.worldMatrix(link); };
    let q = names.map((n) => seed[n] ?? 0), t = 0, fails = 0;
    const out = [{ t: 0, q: q.slice() }], ends = [0];       // ends[i]: the point where move i is done
    const ik = (Mtcp) => {
      const r = solveIK(fk, Mtcp.clone().multiply(offInv), q, lim, 80);
      if (r) q = r; else fails++;
    };
    for (let i = 1; i < poses.length; i++) {
      const A = new THREE.Vector3(poses[i - 1][0], poses[i - 1][1], poses[i - 1][2]).multiplyScalar(0.001);
      const B = new THREE.Vector3(poses[i][0], poses[i][1], poses[i][2]).multiplyScalar(0.001);
      const qa = new THREE.Quaternion(...poses[i - 1].slice(3)), qb = new THREE.Quaternion(...poses[i].slice(3));
      const dist = A.distanceTo(B), ang = qa.angleTo(qb);
      if (!linear[i]) {                                       // joint space: one target, interpolated by playPreview
        const q0 = q.slice();
        if (joints[i]) q = joints[i].slice(); else ik(M(poses[i]));
        const dq = Math.max(...q.map((v, k) => Math.abs(v - q0[k])));
        t += Math.max(0.3, dq / 0.8);
        out.push({ t, q: q.slice() });
        ends.push(out.length - 1);
        continue;
      }
      const n = Math.min(60, Math.max(1, Math.ceil(Math.max(dist / 0.01, ang / 0.087))));
      for (let k = 1; k <= n; k++) {
        const s = k / n;
        const P = A.clone().lerp(B, s), Q = qa.clone().slerp(qb, s);
        ik(base.clone().multiply(new THREE.Matrix4().compose(P, Q, new THREE.Vector3(1, 1, 1))));
        t += Math.max(0.02, dist / n / 0.1 + ang / n / 0.6);   // shown at ~100 mm/s, ~35 deg/s
        out.push({ t, q: q.slice() });
      }
      ends.push(out.length - 1);
    }
    return { points: out, ends, fails, duration: t };
  }

  // a still ghost of the arm at `values` ({joint: rad}), or null to hide it
  showGhost(values) {
    if (!this.ghost || this.preview) return;
    this.ghost.root.visible = !!values;
    if (values) this.ghost.setJoints(values);
    this.draw();
  }

  // --------------------------------------------------------------- arrows
  // Measured F/T: force [N] in sensorFrame, drawn at the sensor
  setMeasuredForce(f, sensorFrame) {
    const m = f && this.robot && this.robot.worldMatrix(sensorFrame);
    this.placeArrow(this.ftArrow, m, f && new THREE.Vector3(f[0], f[1], f[2]).applyMatrix3(new THREE.Matrix3().setFromMatrix4(m || new THREE.Matrix4())));
  }
  // Commanded force [N]: in the TCP's axes (frame "tool") or base axes, drawn at the TCP pose (baseLink)
  setCommandedForce(f, frame, tcpPose) {
    if (!f || !tcpPose) return this.placeArrow(this.cmdArrow, null, null);
    const T = this.base().multiply(poseToMatrix(tcpPose.xyz_mm, tcpPose.rpy_deg));
    const R = new THREE.Matrix3().setFromMatrix4(frame === "tool" ? T : this.base());
    this.placeArrow(this.cmdArrow, T, new THREE.Vector3(f[0], f[1], f[2]).applyMatrix3(R));
  }
  placeArrow(arrow, m, v) {
    const n = v ? v.length() : 0;
    if (!m || n < 0.3) { if (arrow.visible) { arrow.visible = false; this.draw(); } return; }
    arrow.visible = true;
    arrow.position.setFromMatrixPosition(m);
    arrow.setDirection(v.clone().normalize());
    arrow.setLength(Math.min(0.5, n / N_PER_M), 0.03, 0.018);
    this.draw();
  }
}


// Damped least-squares IK. fk(q) -> world Matrix4 of the driven link; want: Matrix4; q0: start; lim: [[lo, hi]].
// Returns q, or null when it does not converge (out of reach, or only past a joint limit).
export function solveIK(fk, want, q0, lim, iters = 120) {
  const q = q0.slice(), n = q.length;
  const pW = new THREE.Vector3().setFromMatrixPosition(want), qW = new THREE.Quaternion().setFromRotationMatrix(want);
  const err = (T) => {
    const p = new THREE.Vector3().setFromMatrixPosition(T), r = new THREE.Quaternion().setFromRotationMatrix(T);
    const d = qW.clone().multiply(r.invert());
    if (d.w < 0) { d.x = -d.x; d.y = -d.y; d.z = -d.z; d.w = -d.w; }
    const s = Math.sqrt(d.x * d.x + d.y * d.y + d.z * d.z), a = 2 * Math.atan2(s, d.w), k = s > 1e-9 ? a / s : 2;
    return [pW.x - p.x, pW.y - p.y, pW.z - p.z, d.x * k, d.y * k, d.z * k];
  };
  const clamp = () => q.forEach((v, i) => (q[i] = Math.min(lim[i][1], Math.max(lim[i][0], v))));
  clamp();
  for (let it = 0; it < iters; it++) {
    const e = err(fk(q));
    const pe = Math.hypot(e[0], e[1], e[2]), re = Math.hypot(e[3], e[4], e[5]);
    if (pe < 2e-4 && re < 2e-3) return q;
    const h = 1e-4, J = [...Array(6)].map(() => new Array(n).fill(0));
    for (let j = 0; j < n; j++) {
      const qq = q.slice(); qq[j] += h;
      const e2 = err(fk(qq));
      for (let r = 0; r < 6; r++) J[r][j] = (e[r] - e2[r]) / h;      // d(pose)/dq = -d(err)/dq
    }
    // dq = J^T (J J^T + l^2 I)^-1 e, the step limited so a far goal does not swing the arm
    const l2 = 0.01, A = [...Array(6)].map((_, r) => [...Array(6)].map((_, c) =>
      J[r].reduce((acc, x, k) => acc + x * J[c][k], 0) + (r === c ? l2 : 0)));
    const y = solve6(A, e);
    if (!y) return null;
    let dq = [...Array(n)].map((_, j) => J.reduce((acc, row, r) => acc + row[j] * y[r], 0));
    const m = Math.max(...dq.map(Math.abs));
    if (m > 0.3) dq = dq.map((v) => v * 0.3 / m);
    dq.forEach((v, j) => (q[j] += v));
    clamp();
  }
  return null;
}

function solve6(A, b) {
  const n = b.length, M = A.map((row, i) => [...row, b[i]]);
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(M[r][c]) > Math.abs(M[p][c])) p = r;
    if (Math.abs(M[p][c]) < 1e-12) return null;
    [M[c], M[p]] = [M[p], M[c]];
    for (let r = 0; r < n; r++) if (r !== c) {
      const f = M[r][c] / M[c][c];
      for (let k = c; k <= n; k++) M[r][k] -= f * M[c][k];
    }
  }
  return M.map((row, i) => row[n] / row[i]);
}
