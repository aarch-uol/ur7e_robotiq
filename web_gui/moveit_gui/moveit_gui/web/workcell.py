"""SETUP > Parts, Camera calibration, Setups, and the restart buttons: the web commands (a WebService mixin).

None of these move the robot except the hand-eye pose run (handeye_collect), which goes through MoveIt like the
payload calibration: claimed with core.begin_motion, refused unless idle in a MoveIt mode, stopped by STOP.
"""
from __future__ import annotations

import base64
import re
import threading
import time
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from .. import bases as B
from .. import handeye as he
from .. import parts as P
from .. import transforms as tf
from ..calibration import TcpDef, config_dir
from ..setups import Setups

RESTART_STACK, RESTART_GUI = 75, 76
PARTS_VERIFY_S = 10.0


class WorkcellCommands:
    # ------------------------------------------------------------------ init / tick
    def _workcell_init(self):
        prof = self.core.profile.name
        self.parts = P.PartStore(config_dir() / "parts" / f"{prof}.yaml")
        self.cams = he.CameraCalibrations(config_dir() / "cameras" / f"{prof}.yaml")
        self.setups = Setups(config_dir(), prof)
        self.restart_code: Optional[int] = None
        self.model_rev = 0                                  # +1 when the 3D views must reload the robot model
        self.on_restart = None                              # set by the server's main()
        self.parts_sync = {"state": "waiting", "msg": "waiting for the robot model", "sent": None, "at": 0.0}
        self._parts_sent: Dict[str, str] = {}               # object id -> attached link ("" = world)
        self._parts_sig = None
        self._parts_busy = False
        self._tcp_checked = None
        self.he: dict = {"topic": "", "info_topic": "", "mount": "eye_in_hand", "K": None, "D": None,
                         "frame": "", "samples": [], "result": None, "poses": [], "last": None,
                         "progress": {"running": False, "done": 0, "total": 0, "error": ""}}

    def _base_model(self) -> dict:
        xml = self.core.bridge.robot_description()
        if xml != self._model_xml:
            from ..robot_model import model_or_error
            self._model_xml, self._model = xml, model_or_error(xml)
            self.model_rev += 1
        return self._model

    def part_mesh_file(self, rel: str) -> Optional[Path]:
        """/mesh/_parts/<file>: an uploaded mesh; /mesh/_parts/hulls/<file>.hull.stl: its collision hull."""
        rel = rel or ""
        if rel.startswith("hulls/"):
            f = rel[len("hulls/"):]
            if not f.endswith(".hull.stl") or not P.MESH_FILE.match(f[:-len(".hull.stl")]):
                return None
            p = self.parts.generated_dir / "hulls" / f
        else:
            if not P.MESH_FILE.match(rel):
                return None
            p = self.parts.mesh_dir / rel
        return p if p.is_file() else None

    def _parts_tick(self):
        """From tick(): keep TF + MoveIt's scene in step with the live parts. Never while the arm moves (a scene
        change can abort a running trajectory), never blocking the tick (it runs on its own thread)."""
        if self._parts_busy:
            return
        m = self._base_model()
        if not m.get("ok"):
            self.parts_sync.update(state="waiting", msg="waiting for the robot model")
            return
        if self._tcp_checked != self.model_rev and self.core.idle():
            self._tcp_checked = self.model_rev
            self._tcp_check(m)
        frames, objects = P.live_plan(self.parts, m)
        sig = repr([(a, b, np.round(T, 6).tolist()) for a, b, T in frames] +
                   [(o["id"], o["link"], np.round(o["T_link_obj"], 6).tolist(), o["geom"], o["attach"]) for o in objects])
        due = time.monotonic() - self.parts_sync["at"] > PARTS_VERIFY_S
        if sig == self._parts_sig and not due:
            return
        if not self.core.idle() or self.core.force_active:
            if sig != self._parts_sig:
                self.parts_sync.update(state="pending", msg="parts changed: sent to MoveIt when the robot is idle")
            return
        self._parts_busy = True

        def run():
            try:
                self._parts_apply(sig, frames, objects)
            except Exception as e:
                self._log_error("parts", e)
                self.parts_sync.update(state="error", msg=f"{type(e).__name__}: {e}", at=time.monotonic())
            finally:
                self._parts_busy = False
        threading.Thread(target=run, daemon=True, name="parts-sync").start()

    def _parts_apply(self, sig, frames, objects):
        b = self.core.bridge
        want = {o["id"]: (o["link"] if o["attach"] else "") for o in objects}
        have = b.parts_scene_ids()
        stale = {i: l for i, l in {**self._parts_sent, **(have or {})}.items() if i not in want}
        missing = [o for o in objects if have is None or have.get(o["id"]) != want[o["id"]] or sig != self._parts_sig]
        if frames or missing or stale:
            ok, msg = b.parts_publish(frames, missing, sorted(stale.items()))
            if not ok:
                self.parts_sync.update(state="error", msg=msg, at=time.monotonic())
                return
        self._parts_sent = want
        changed = sig != self._parts_sig
        self._parts_sig = sig
        if have is None:
            state, msg = "unverified", (f"{len(frames)} frame(s), {len(objects)} collision object(s) sent; MoveIt's "
                                        "planning scene could not be read back (move_group not up?)")
        else:
            again = b.parts_scene_ids() if missing else have
            lost = [i for i in want if again is None or again.get(i) != want[i]]
            state = "ok" if not lost else "unverified"
            msg = (f"{len(frames)} frame(s) in TF, {len(objects)} collision object(s) in MoveIt" +
                   (f"; not confirmed yet: {', '.join(lost)}" if lost else ""))
        if changed or state != self.parts_sync["state"]:
            b.log(f"parts: {msg}")
        self.parts_sync.update(state=state, msg=msg, at=time.monotonic())

    def workcell_state(self) -> dict:
        """Small, in every state broadcast: what the pages need to refresh themselves."""
        m = self._model if getattr(self, "_model", None) else {}
        st = P.status(self.parts, m) if m.get("ok") else {}
        return {"parts_sync": {k: self.parts_sync[k] for k in ("state", "msg")},
                "parts_n": len(self.parts.parts),
                "restart_for": sorted(n for n, s in st.items() if s in ("live", "changed", "removed")),
                "he": {**self.he["progress"], "samples": len(self.he["samples"]),
                       "solved": self.he["result"] is not None},
                "restarting": self.restart_code, "model_rev": self.model_rev}

    # ------------------------------------------------------------------ parts
    def _robot_links(self) -> set:
        m = self._base_model()
        return {k for k in m.get("links", {}) if k not in P.model_parts(m)} if m.get("ok") else set()

    def parts_view(self) -> dict:
        m = self._base_model()
        st = P.status(self.parts, m) if m.get("ok") else {}
        out = []
        for n, p in self.parts.ordered().items():
            lo, hi = self.parts.extent_m(p)
            out.append({"name": n, **p.to_yaml(), "status": st.get(n, "live"), "far_face_mm": round(hi * 1000, 2)})
        removed = [n for n, s in st.items() if s == "removed"]
        restart = [n for n, s in st.items() if s in ("live", "changed", "removed")]
        links = sorted(self._robot_links())
        arm = [l for l in links if m.get("ok") and P.rides_on_arm(m, l)]
        return {"parts": out, "removed": removed, "restart_for": restart, "links": links, "arm_links": arm,
                "meshes": self.parts.meshes(), "mesh_info": {f: self.parts.mesh_info(f) for f in self.parts.meshes()},
                "presets": {k: v for k, v in P.PRESETS.items()},
                "problems": P.problems(self.parts, m) if m.get("ok") else [], "error": self.parts.error,
                "sync": {k: v for k, v in self.parts_sync.items() if k != "sent"},
                "model_ok": bool(m.get("ok")), "model_msg": m.get("msg", "")}

    def cmd_parts_get(self):
        return self._ok(parts=self.parts_view())
    cmd_parts_get.read_only = True

    def _parts_saved(self, what: str) -> dict:
        m = self._base_model()
        if m.get("ok"):
            P.write_generated(self.parts, m)
            self._update_part_tcps()
        self._parts_sig = None                               # re-sync now
        self.model_rev += 1
        self.core.bridge.log(f"parts: {what}")
        return self._ok(what, parts=self.parts_view())

    def cmd_parts_save(self, name, parent, xyz_mm=None, rpy_deg=None, shape=None, anchor="base", color="#9aa0a6",
                       collision=None, kind="part", note="", replace=None):
        old = self.parts.parts.get(replace or name)
        meta = dict(old.meta) if old else {}
        if old and meta.get("source") == "handeye" and (list(map(float, xyz_mm or [])) != old.xyz_mm or
                                                        list(map(float, rpy_deg or [])) != old.rpy_deg):
            meta = {k: v for k, v in meta.items() if k not in ("source", "camera")}
            meta["note"] = "offset edited after the hand-eye calibration"
        p = P.make_part(name, parent, xyz_mm, rpy_deg, shape, anchor, color, collision, kind, note, meta)
        self.parts.put(p, self._robot_links(), replace=replace or (name if old else None))
        return self._parts_saved(f"part {p.name} saved")

    def cmd_parts_delete(self, name):
        self.parts.delete(str(name))
        return self._parts_saved(f"part {name} deleted")

    def cmd_parts_mesh_upload(self, filename, data_b64):
        try:
            data = base64.b64decode(data_b64, validate=True)
        except Exception:
            raise P.PartError("the upload is not base64") from None
        f = self.parts.add_mesh(filename, data)
        info = self.parts.mesh_info(f)
        users = [n for n, p in self.parts.parts.items() if p.shape.get("file") == f]
        if users:                                            # replaced a mesh parts use: regenerate the model
            m = self._base_model()
            if m.get("ok"):
                P.write_generated(self.parts, m)
            self._parts_sig = None
            self.model_rev += 1
        return self._ok(f"mesh {f} uploaded" + (f" (used by {', '.join(users)})" if users else ""),
                        mesh=info, file=f, size_file_units=[round(float(v), 4) for v in info.get("size", [])],
                        parts=self.parts_view())

    def cmd_parts_make_tcp(self, name, tcp, at="far_face"):
        """A virtual TCP at a part (its far face or its frame), on the planning link, kept in step with the part."""
        p = self.parts.parts.get(str(name))
        if p is None:
            raise P.PartError(f"no part '{name}'")
        T = self._part_tcp_T(p, at)
        if not self.core.idle():
            raise P.PartError("the robot is busy: change TCPs while idle")
        tcp = str(tcp).strip()
        self.core.store.put_tcp(TcpDef(tcp, "virtual", parent=self.core.profile.planning_link, T_parent_tcp=T,
                                       meta={"method": f"from part {p.name} ({at})", "part": p.name, "part_at": at}),
                                replace=tcp in self.core.store.tcps)
        xyz, rpy = tf.T_to_mm_deg(T)
        self.core.bridge.log(f"TCP {tcp} from part {p.name}: {[round(v, 2) for v in xyz]} mm from "
                             f"{self.core.profile.planning_link}")
        return self._ok(f"TCP {tcp} saved at {p.name} ({'far face' if at == 'far_face' else 'frame'})")

    def _part_tcp_T(self, p: P.Part, at: str) -> np.ndarray:
        m = self._base_model()
        if not m.get("ok"):
            raise P.PartError("no robot model yet")
        link, T_link_part = P.anchor(self.parts, m, p.name)
        T_pl_link = P.fixed_T(m, self.core.profile.planning_link, link)
        if T_pl_link is None:
            raise P.PartError(f"{p.name} is not rigidly on {self.core.profile.planning_link} (it sits on {link}, "
                              f"which moves relative to it): a TCP there would be wrong")
        T = T_pl_link @ T_link_part
        if at == "far_face":
            E = np.eye(4)
            E[2, 3] = self.parts.extent_m(p)[1]
            T = T @ E
        return T

    def _update_part_tcps(self):
        for t in list(self.core.store.tcps.values()):
            pn = (t.meta or {}).get("part")
            if not pn or t.type != "virtual":
                continue
            p = self.parts.parts.get(pn)
            if p is None:
                continue                                     # the part is gone: the TCP keeps its last offset
            try:
                T = self._part_tcp_T(p, t.meta.get("part_at", "far_face"))
            except P.PartError:
                continue
            if t.T_parent_tcp is None or not np.allclose(T, t.T_parent_tcp, atol=1e-7):
                self.core.store.put_tcp(TcpDef(t.name, "virtual", parent=self.core.profile.planning_link,
                                               T_parent_tcp=T, meta=dict(t.meta)), replace=True)
                self.core.bridge.log(f"TCP {t.name} follows part {pn}")

    # ------------------------------------------------------------------ restart
    def _restart_refusal(self) -> Optional[str]:
        c = self.core
        if not c.idle() or c.force_active:
            return "the robot is busy (motion, program, policy or force): stop it first"
        if getattr(self, "payload_progress", {}).get("running") or self.he["progress"]["running"]:
            return "a calibration is running"
        return None

    def _restart(self, code: int, why: str):
        self.restart_code = code
        self.core.bridge.log(f"RESTART ({'robot stack + GUI' if code == RESTART_STACK else 'GUI'}): {why}")
        cb = self.on_restart
        if cb is not None:
            threading.Timer(0.8, cb).start()                 # let this reply reach the browser first

    def cmd_restart(self, what="gui"):
        why = self._restart_refusal()
        if why:
            raise P.PartError(why)
        if self.on_restart is None:
            raise P.PartError("this GUI was not started by the launcher (ur7e_gui.sh): restart it by hand")
        code = RESTART_STACK if what == "stack" else RESTART_GUI
        self._restart(code, "asked in the GUI")
        return self._ok("restarting the robot stack and the GUI: this page reconnects by itself (about a minute)"
                        if code == RESTART_STACK else "restarting the GUI: this page reconnects by itself")

    def _tcp_link(self, name: str) -> Optional[str]:
        """The robot-model link (a profile TCP's link) a TCP hangs on in the end, None when its chain ends on some
        other TF frame (a camera's, say: not the robot model's business)."""
        tcps, seen = self.core.store.tcps, set()
        while name in tcps and name not in seen:
            seen.add(name)
            t = tcps[name]
            if t.builtin:
                return t.link
            name = t.parent
        return None

    def _tcp_check(self, m: dict):
        """After a base switch the active TCP may hang on a link the robot no longer has (robotiq_tcp without the
        gripper): fall back to the flange, so motion never plans for a frame that is not there."""
        st = self.core.store
        link = self._tcp_link(st.active_tcp)
        have = set(P.merged_model(self.parts, m).get("links", {}))
        if not link or link in have:
            return
        flange = next((n for n, t in st.tcps.items() if t.builtin and t.link == self.core.profile.planning_link), None)
        if flange:
            old = st.active_tcp
            st.set_active_tcp(flange)
            self.core.bridge.log(f"active TCP '{old}' is on {link}, which this robot model does not have "
                                 f"(base '{B.running(m)}'): using '{flange}'")

    # ------------------------------------------------------------------ end-effector base (bases.py)
    def base_view(self) -> dict:
        m = self._base_model()
        return B.view(config_dir(), m if m.get("ok") else None)

    def running_base(self) -> str:
        m = self._base_model()
        return B.running(m if m.get("ok") else None)

    def base_lacks(self, mode: Optional[str] = None, gripper: bool = False) -> Optional[str]:
        """Why a mode (or the gripper) cannot work on the running base, None when it can."""
        b = self.running_base()
        label = B.BASES[b][0]
        if gripper and not B.has_gripper(b):
            return f"the robot runs the '{label}' base: no gripper (switch it in SETUP › Setups)"
        if mode:
            md = self.core.profile.modes.get(mode)
            if md is None:
                return None
            if md.requires_ft and not B.has_ft(b):
                return f"{md.label} needs the F/T sensor, and the robot runs the '{label}' base"
            m = self._base_model()
            links = m.get("links", {}) if m.get("ok") else None
            need = {md.wrench.get("frame"), md.wrench.get("point")} - {None, "", "world", self.core.profile.base_link}
            gone = sorted(l for l in need if links is not None and l not in links)
            if gone:
                return f"{md.label} works at {', '.join(gone)}, which the '{label}' base does not have"
        return None

    def cmd_base_switch(self, base, confirm=False):
        """Make `base` the end-effector base and restart the robot stack with it (new URDF, SRDF, controllers)."""
        base = B.check(base)
        if base == self.running_base() and (B.read_next(config_dir()) or B.DEFAULT) == base:
            return self._ok(f"the robot already runs the '{B.BASES[base][0]}' base")
        why = self._restart_refusal()
        if why:
            raise P.PartError(why)
        if self.core.mode not in ("unknown",) and not self.core.modes.mode_supports_moveit(self.core.mode):
            raise P.PartError(f"change to Position first (now '{self.core.mode}')")
        if self.on_restart is None:
            raise P.PartError("switching the base restarts the robot stack, and this GUI was not started by the "
                              "launcher (ur7e_gui.sh): run  ur7e_gui.sh --base " + base)
        if not confirm:
            raise P.PartError("confirm the switch (the robot stack restarts)")
        B.write_next(config_dir(), base)
        self._restart(RESTART_STACK, f"base switched to {base}")
        return self._ok(f"base '{B.BASES[base][0]}': restarting the robot stack and the GUI (about a minute)",
                        restart="stack")

    # ------------------------------------------------------------------ setups
    def cmd_setup_list(self):
        return self._ok(setups=self.setups.list(), base=self.base_view())
    cmd_setup_list.read_only = True

    def cmd_setup_save(self, name, note="", replace=False):
        holds = self.setups.save(str(name), str(note or ""), bool(replace), base=self.running_base())
        self.core.bridge.log(f"setup '{name}' saved: {', '.join(holds) or 'nothing to save yet'}")
        return self._ok(f"setup '{name}' saved ({len(holds)} item(s))", setups=self.setups.list(), base=self.base_view())

    def cmd_setup_delete(self, name):
        self.setups.delete(str(name))
        return self._ok(f"setup '{name}' deleted", setups=self.setups.list(), base=self.base_view())

    def cmd_setup_load(self, name):
        why = self._restart_refusal()
        if why:
            raise P.PartError(why)
        if self.core.mode not in ("unknown",) and not self.core.modes.mode_supports_moveit(self.core.mode):
            raise P.PartError(f"change to Position first (now '{self.core.mode}'): the setup changes gains and payload")
        if self.on_restart is None:
            raise P.PartError("loading a setup restarts the GUI, and this GUI was not started by the launcher "
                              "(ur7e_gui.sh)")
        r = self.setups.load(str(name), base=self.running_base())
        # the new files are in place: does the running robot model have the setup's parts, on the setup's base?
        new = P.PartStore(self.parts.path)
        m = self._base_model()
        stack = bool(m.get("ok")) and any(s != "in_model" for s in P.status(new, m).values())
        base = r.get("base")
        if base and base != self.running_base():
            B.write_next(config_dir(), base)
            stack = True
        elif base:
            B.write_next(config_dir(), base)                 # keep the next start on it too
        self._restart(RESTART_STACK if stack else RESTART_GUI, f"setup '{name}' loaded")
        return self._ok(f"setup '{name}' loaded (the files in use before are kept as '{r['backup']}'): restarting "
                        f"{'the robot stack and ' if stack else ''}the GUI" +
                        (f" on the '{B.BASES[base][0]}' base" if base else ""), restart="stack" if stack else "gui")

    # ------------------------------------------------------------------ hand-eye
    def he_view(self) -> dict:
        h = self.he
        r = h["result"]
        cams = [{"name": c.get("name", c["topic"]), "topic": c["topic"], "info": c.get("info", "")}
                for c in self.core.profile.cameras if not str(c.get("topic", "")).endswith(("depth", "depth_registered"))]
        res = None
        if r:
            xyz, rpy = tf.T_to_mm_deg(r["X"])
            res = {k: v for k, v in r.items() if k not in ("X", "T_board")}
            res.update(xyz_mm=[round(v, 2) for v in xyz], rpy_deg=[round(v, 3) for v in rpy])
        return {"board": self.cams.board, "cameras": cams, "saved": self.cams.view(), "topic": h["topic"],
                "info_topic": h["info_topic"], "mount": h["mount"], "frame": h["frame"],
                "intrinsics": None if h["K"] is None else {"fx": h["K"][0], "fy": h["K"][4], "cx": h["K"][2],
                                                           "cy": h["K"][5], "D": h["D"][:5]},
                "samples": [{"corners": s.corners, "reproj_px": round(s.reproj_px, 3)} for s in h["samples"]],
                "poses": len(h["poses"]), "result": res, "last": h["last"], "progress": dict(h["progress"]),
                "flange": self.core.profile.planning_link, "base": self.core.profile.base_link,
                "min_samples": he.MIN_SAMPLES}

    def cmd_handeye_get(self):
        return self._ok(he=self.he_view())
    cmd_handeye_get.read_only = True

    def cmd_handeye_board(self, squares_x, squares_y, square_mm, marker_mm, dict="DICT_5X5_100"):
        b = self.cams.set_board({"squares_x": squares_x, "squares_y": squares_y, "square_mm": square_mm,
                                 "marker_mm": marker_mm, "dict": dict})
        self.he["samples"], self.he["result"] = [], None
        return self._ok("board saved (samples cleared)", he=self.he_view(), board=b)

    def cmd_handeye_board_svg(self):
        return self._ok(svg=he.board_svg(self.cams.board))
    cmd_handeye_board_svg.read_only = True

    def cmd_handeye_start(self, topic, info_topic="", mount="eye_in_hand"):
        if self.he["progress"]["running"]:
            raise he.HandEyeError("a pose run is in progress")
        if mount not in he.MOUNTS:
            raise he.HandEyeError(f"mount is one of {', '.join(he.MOUNTS)}")
        topic = str(topic).strip()
        info_topic = str(info_topic or "").strip() or re.sub(r"/[^/]+$", "/camera_info", topic)
        info, why = self.core.bridge.camera_info(info_topic)
        if info is None:
            raise he.HandEyeError(f"{why}. The camera's CameraInfo gives the intrinsics (RealSense / ZED publish it "
                                  f"next to the image)")
        if not any(info["K"]):
            raise he.HandEyeError(f"{info_topic}: the camera matrix is all zeros (camera not calibrated)")
        self.he.update(topic=topic, info_topic=info_topic, mount=mount, K=info["K"], D=info["D"],
                       frame=info["frame_id"], samples=[], result=None, poses=[], last=None)
        self.core.bridge.log(f"hand-eye: {mount} with {topic} ({info['width']}x{info['height']}, frame "
                             f"{info['frame_id']})")
        return self._ok(f"calibrating {topic} ({mount.replace('_', '-')})", he=self.he_view())

    def _he_require(self):
        if not self.he["topic"] or self.he["K"] is None:
            raise he.HandEyeError("pick the camera first (Start)")

    def _he_look(self) -> tuple:
        """Grab one frame and detect the board: (Detection, preview jpeg base64 or None)."""
        import cv2
        img, frame = self.core.bridge.grab_image(self.he["topic"])
        if img is None:
            return he.Detection(False, why=frame), None
        d = he.detect(img, self.cams.board, self.he["K"], self.he["D"])
        vis = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        for x, y in d.points:
            cv2.circle(vis, (int(x), int(y)), max(2, img.shape[1] // 320), (40, 200, 40), -1)
        if d.ok:
            cv2.drawFrameAxes(vis, np.asarray(self.he["K"]).reshape(3, 3), np.asarray(self.he["D"]),
                              cv2.Rodrigues(d.T_cam_board[:3, :3])[0], d.T_cam_board[:3, 3],
                              self.cams.board["square_mm"] * 2 / 1000.0, 3)
        k = max(1, int(np.ceil(vis.shape[1] / 800)))
        ok, jpg = cv2.imencode(".jpg", vis[::k, ::k], [cv2.IMWRITE_JPEG_QUALITY, 75])
        return d, base64.b64encode(jpg.tobytes()).decode() if ok else None

    def _he_last(self, d: he.Detection) -> dict:
        self.he["last"] = {"ok": d.ok, "corners": d.corners, "reproj_px": None if not d.ok else round(d.reproj_px, 3),
                           "why": d.why, "distance_mm": None if not d.ok else
                           round(float(np.linalg.norm(d.T_cam_board[:3, 3])) * 1000)}
        return self.he["last"]

    def cmd_handeye_look(self):
        """Grab a frame and show what the detector sees. Records nothing."""
        self._he_require()
        d, jpg = self._he_look()
        return self._ok(("board: " + f"{d.corners} corners, {d.reproj_px:.2f} px") if d.ok else d.why,
                        detection=self._he_last(d), jpeg=jpg)
    cmd_handeye_look.read_only = True

    def _flange(self) -> Optional[np.ndarray]:
        p = self.core.profile
        return self.core.bridge.lookup_transform(p.base_link, p.planning_link)

    def _he_capture_here(self) -> tuple:
        b = self.core.bridge
        if b.is_stationary() is False:
            return False, "the arm is moving: capture when it is still", None, None
        T0 = self._flange()
        d, jpg = self._he_look()
        T1 = self._flange()
        self._he_last(d)
        if T0 is None or T1 is None:
            return False, "no flange pose from TF", d, jpg
        if np.linalg.norm(T0[:3, 3] - T1[:3, 3]) > 0.0005:
            return False, "the arm moved while the image was taken: hold still", d, jpg
        if not d.ok:
            return False, d.why, d, jpg
        self.he["samples"].append(he.Sample(T1, d.T_cam_board, d.corners, d.reproj_px))
        self.he["result"] = None
        return True, f"sample {len(self.he['samples'])}: {d.corners} corners, {d.reproj_px:.2f} px", d, jpg

    def cmd_handeye_capture(self):
        self._he_require()
        if self.he["progress"]["running"]:
            raise he.HandEyeError("a pose run is in progress")
        ok, msg, d, jpg = self._he_capture_here()
        if not ok:
            raise he.HandEyeError(msg)
        return self._ok(msg, he=self.he_view(), jpeg=jpg)

    def cmd_handeye_drop(self, index=None):
        s = self.he["samples"]
        if index is None:
            s.clear()
        elif 0 <= int(index) < len(s):
            s.pop(int(index))
        self.he["result"] = None
        return self._ok("samples cleared" if index is None else f"sample {int(index) + 1} removed", he=self.he_view())

    def cmd_handeye_plan(self, tilt_deg=20.0):
        """12 flange poses around the current one, turned about the board (eye-in-hand) or the tool tip
        (eye-to-hand). Nothing moves until Collect."""
        self._he_require()
        tilt = float(tilt_deg)
        if not 5 <= tilt <= 35:
            raise he.HandEyeError("tilt is 5 to 35 deg")
        T = self._flange()
        if T is None:
            raise he.HandEyeError("no flange pose from TF")
        if self.he["mount"] == "eye_in_hand":
            d, _ = self._he_look()
            self._he_last(d)
            if not d.ok:
                raise he.HandEyeError(f"the board must be in view to plan the poses around it: {d.why}")
            # the board centre, assuming the camera looks roughly along the flange's +Z (as wrist cameras do)
            dist = float(np.linalg.norm(d.T_cam_board[:3, 3]))
            if self.he["result"] is not None:
                pivot = (T @ self.he["result"]["X"] @ d.T_cam_board @ np.r_[self._board_centre(), 1])[:3]
            else:
                pivot = T[:3, 3] + T[:3, 2] * dist
        else:
            pivot = (self.core.geo.current_pose(self.core.profile.base_link, self.core.store.active_tcp).T())[:3, 3]
        self.he["poses"] = he.sample_poses(T, pivot, tilt)
        return self._ok(f"{len(self.he['poses'])} poses planned (±{tilt:g} deg about "
                        f"{'the board' if self.he['mount'] == 'eye_in_hand' else 'the tool tip'})", he=self.he_view(),
                        poses=[tf.T_to_mm_deg(P_) for P_ in self.he["poses"]])

    def _board_centre(self) -> np.ndarray:
        b = self.cams.board
        return np.array([b["squares_x"] * b["square_mm"] / 2000.0, b["squares_y"] * b["square_mm"] / 2000.0, 0.0])

    def cmd_handeye_collect(self, settle_s=0.8):
        """Move to each planned pose (MoveIt, collision-aware) and take a sample there."""
        self._he_require()
        poses = list(self.he["poses"])
        if not poses:
            raise he.HandEyeError("plan the poses first")
        if self.he["progress"]["running"] or not self.core.idle():
            raise he.HandEyeError("robot is busy")
        if not self.core.modes.mode_supports_moveit(self.core.mode):
            raise he.HandEyeError(f"MoveIt motion is not available in '{self.core.mode}'. Change to Position first.")
        why = self.core.begin_motion("camera calibration")
        if why:
            raise he.HandEyeError(why)
        prog = self.he["progress"]
        prog.update(running=True, done=0, total=len(poses), error="", got=0)
        b, link = self.core.bridge, self.core.profile.planning_link
        vel = max(0.01, 0.1 * self.core.speed_override)
        epoch = getattr(b, "stop_epoch", 0)

        def job():
            got, skipped = 0, []
            try:
                for k, T in enumerate(poses):
                    prog["pose"] = k
                    if getattr(b, "stop_epoch", 0) != epoch:
                        skipped.append("the rest: STOP")
                        break
                    if not b.move_pose(link, T, False, vel, vel):
                        if getattr(b, "stop_epoch", 0) != epoch:
                            skipped.append("the rest: STOP")
                            break
                        skipped.append(f"{k + 1} (no plan / not reached)")
                        prog["done"] = k + 1
                        continue
                    t_end = time.monotonic() + 3.0
                    time.sleep(float(settle_s))
                    while b.is_stationary() is False and time.monotonic() < t_end:
                        time.sleep(0.1)
                    ok, msg, _, _ = self._he_capture_here()
                    if ok:
                        got += 1
                    else:
                        skipped.append(f"{k + 1} ({msg})")
                    prog.update(done=k + 1, got=got)
                n = len(self.he["samples"])
                msg = f"{got} of {len(poses)} poses sampled ({n} sample(s) in all)"
                if skipped:
                    msg += "; skipped " + ", ".join(skipped[:6]) + ("…" if len(skipped) > 6 else "")
                if n >= he.MIN_SAMPLES:
                    try:
                        r = self._he_solve()
                        msg += f". Result: {r['verdict']} ({r['rms_mm']} mm, {r['rms_deg']} deg)"
                    except he.HandEyeError as e:
                        msg += f". Not solved: {e}"
                return True, msg
            finally:
                prog["running"] = False
                self.core.end_motion()
        return self._submit("camera calibration", job)

    def _he_solve(self) -> dict:
        r = he.solve(self.he["samples"], self.he["mount"])
        self.he["result"] = r
        return r

    def cmd_handeye_solve(self):
        self._he_require()
        r = self._he_solve()
        return self._ok(f"{r['verdict']}: board placed within {r['rms_mm']} mm / {r['rms_deg']} deg (rms) by "
                        f"{r['n']} samples", he=self.he_view())

    def cmd_handeye_save(self, name, as_part=True):
        r = self.he["result"]
        if r is None:
            raise he.HandEyeError("solve first")
        p = self.core.profile
        e = self.cams.save(str(name), self.he["topic"], self.he["info_topic"], self.he["frame"], p.planning_link,
                           p.base_link, r, self.he["samples"], self.he["K"])
        msg = f"camera {name} saved ({e['verdict']})"
        if as_part:
            msg += "; " + self._camera_part(str(name))
        return self._ok(msg, he=self.he_view(), parts=self.parts_view())

    def cmd_handeye_apply(self, name):
        if str(name) not in self.cams.cameras:
            raise he.HandEyeError(f"no camera calibration '{name}'")
        return self._ok(self._camera_part(str(name)), parts=self.parts_view())

    def cmd_handeye_delete(self, name):
        self.cams.delete(str(name))
        return self._ok(f"camera calibration {name} deleted (its part, if any, stays: delete it in Parts)",
                        he=self.he_view())

    def _camera_part(self, name: str) -> str:
        """Put the calibrated camera into the robot model as a part. The camera driver publishes its own TF tree
        (e.g. camera_link -> camera_color_optical_frame): the part is that tree's root, placed so the optical frame
        lands where the calibration says. Without a driver tree, the part is the optical frame itself."""
        e = self.cams.cameras[name]
        T_par_opt = self.cams.T_parent_optical(name)
        optical = e.get("optical_frame") or f"{name}_optical_frame"
        b = self.core.bridge
        root, T_root_opt = optical, np.eye(4)
        cur, hops = optical, 0
        while hops < 10:
            par = b.tf_parent(cur) if hasattr(b, "tf_parent") else None
            if not par or par in self._robot_links() or par in self.parts.parts and \
                    self.parts.parts[par].meta.get("camera") != name:
                break
            cur, hops = par, hops + 1
        if cur != optical:
            T = b.lookup_transform(cur, optical)
            if T is not None:
                root, T_root_opt = cur, T
        if not P.NAME.match(root):
            root = re.sub(r"[^a-z0-9_]", "_", root.lower()).lstrip("_0123456789") or f"{name}_link"
        xyz, rpy = tf.T_to_mm_deg(T_par_opt @ tf.inv(T_root_opt))
        old = self.parts.parts.get(root)
        part = P.make_part(root, e["parent"], xyz, rpy, {"type": "none"}, kind="camera",
                           note=f"hand-eye {e['saved']} ({e['verdict']}, {e['rms_mm']} mm)",
                           meta={"source": "handeye", "camera": name})
        self.parts.put(part, self._robot_links(), replace=root if old else None)
        self._parts_saved(f"camera part {root} from calibration {name}")
        return (f"part '{root}' on {e['parent']}" + (f" (the camera driver's frame above {optical})"
                                                      if root != optical else "") +
                ": live now, in the robot model after a stack restart")
