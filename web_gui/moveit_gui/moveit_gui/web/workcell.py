"""SETUP > Parts, Camera calibration, Setups, and the restart buttons: the web commands (a WebService mixin).

None of these move the robot except the hand-eye pose run (handeye_collect), which goes through MoveIt like the
payload calibration: claimed with core.begin_motion, refused unless idle in a MoveIt mode, stopped by STOP.
"""
from __future__ import annotations

import base64
import json
import os
import re
import shutil
import tempfile
import threading
import time
import zlib
from pathlib import Path
from typing import Dict, List, Optional

import numpy as np

from .. import bases as B
from .. import ft_axes as FA
from .. import handeye as he
from .. import parts as P
from .. import robots as R
from .. import services as SV
from .. import transforms as tf
from ..calibration import TcpDef, config_dir
from ..setups import SetupError, Setups, members as S_members
from ..user_profiles import Profiles, ProfileError
from ..worksets import WorkSets

RESTART_STACK, RESTART_GUI = 75, 76
PARTS_VERIFY_S = 10.0
SETUP_CHECK_S = 5.0          # how often the state compares the files in use with the loaded setup



def tcp_frames(tcps: dict, model: dict, other_frames=()) -> list:
    """Static TF [(parent, name, T)] for the virtual TCPs, so nodes outside the GUI (a policy runner's ee_frame,
    e.g. spatula_tip) can look them up. Only TCPs whose parent is a model link, a part frame or another published
    TCP, and whose name is no link or part frame already. TF only: MoveIt's geometry does not change."""
    links = set(model.get("links", {}))
    taken = links | set(other_frames)
    virt = {n: t for n, t in tcps.items() if t.type == "virtual" and t.T_parent_tcp is not None and n not in taken}
    out, done = [], set()
    for _ in range(len(virt)):                      # parents first (a TCP on a TCP)
        for n, t in virt.items():
            if n not in done and (t.parent in taken or t.parent in done):
                out.append((t.parent, n, np.asarray(t.T_parent_tcp, float)))
                done.add(n)
    return out

class WorkcellCommands:
    # ------------------------------------------------------------------ init / tick
    def _workcell_init(self):
        prof = self.core.profile.name
        self.parts = P.PartStore(config_dir() / "parts" / f"{prof}.yaml")
        self.cams = he.CameraCalibrations(config_dir() / "cameras" / f"{prof}.yaml")
        self.setups = Setups(config_dir(), prof)            # the shared setups (made before profiles existed)
        self.profiles = Profiles(config_dir())
        self.worksets = WorkSets(config_dir(), prof, self.profiles)   # whose settings are in use (worksets.py)
        self._settings_lock = threading.RLock()             # one durable settings transaction at a time
        self.worksets.recover(self.core.bridge.log)
        self.core.settings_owner = self.worksets.active                # named in the policy runs' log
        self.restart_code: Optional[int] = None
        self._restart_lock = threading.Lock()                # HTTP handlers run in threads: one restart decision
        self.model_rev = 0                                  # +1 when the 3D views must reload the robot model
        self.on_restart = None                              # set by the server's main()
        self.parts_sync = {"state": "waiting", "msg": "waiting for the robot model", "sent": None, "at": 0.0}
        self._parts_sent: Dict[str, str] = {}               # object id -> attached link ("" = world)
        self._parts_sig = None
        self._parts_busy = False
        self._tcp_checked = None
        self._collision_jobs: Dict[str, dict] = {}
        self._restore_auto_collisions()
        self._drivers_init()
        self.he: dict = {"topic": "", "info_topic": "", "mount": "eye_in_hand", "K": None, "D": None,
                         "frame": "", "samples": [], "result": None, "poses": [], "last": None,
                         "ee_link": "", "base_link": "", "method": "park", "sensor_frame": "",
                         "progress": {"running": False, "done": 0, "total": 0, "error": ""}}

    def _base_model(self) -> dict:
        xml = self.core.bridge.robot_description()
        if xml != self._model_xml:
            from ..robot_model import model_or_error
            self._model_xml, self._model = xml, model_or_error(xml)
            self.model_rev += 1
        return self._model

    def _restore_auto_collisions(self):
        """Rebuild missing/stale generated auto pieces after config restore without blocking service startup."""
        todo = [p for p in self.parts.parts.values()
                if p.collision == "auto" and self.parts.auto_collision_cached(p) is None]
        if not todo:
            return
        key = "startup"
        self._collision_jobs[key] = {"state": "generating", "msg": "regenerating saved collisions…"}

        def run():
            try:
                for p in todo:
                    self.parts.auto_collision_files(p)
                m = self._base_model()
                if m.get("ok"):
                    P.write_generated(self.parts, m)
                self._parts_sig = None
                self.model_rev += 1
                self._collision_jobs[key] = {"state": "done", "msg": "saved collisions regenerated"}
            except Exception as e:
                self._collision_jobs[key] = {"state": "error", "msg": str(e)}
                self._log_error("parts collision", e)
        threading.Thread(target=run, daemon=True, name="parts-collision-restore").start()

    def part_mesh_file(self, rel: str) -> Optional[Path]:
        """/mesh/_parts/<file>: uploaded mesh; hulls/ and collision/ are generated collision meshes."""
        rel = rel or ""
        if rel.startswith("hulls/"):
            f = rel[len("hulls/"):]
            if not f.endswith(".hull.stl") or not P.MESH_FILE.match(f[:-len(".hull.stl")]):
                return None
            p = self.parts.generated_dir / "hulls" / f
        elif rel.startswith("collision/"):
            f = rel[len("collision/"):]
            if "/" in f or "\\" in f or not re.match(r"^[A-Za-z0-9_.-]+\.stl$", f, re.I):
                return None
            p = self.parts.generated_dir / "collision" / f
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
        frames = frames + tcp_frames(self.core.store.tcps, m, {child for _, child, _ in frames})
        sig = repr([(a, b, np.round(T, 6).tolist()) for a, b, T in frames] +
                   [(o["id"], o["link"],
                     [(np.round(x["T_link_obj"], 6).tolist(), x["geom"]) for x in o.get("geoms", [])],
                     o["attach"]) for o in objects])
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
                "restarting": self.restart_code, "model_rev": self.model_rev,
                "cams_n": len(self.cams.cameras),
                "drivers": self.drivers_state(),
                "setup": self._setup_summary(), "base_next": self._base_next(),
                "robot_chosen": R.chosen(config_dir()) or R.running(self.core.fake) is None}   # by hand: never asked

    def _base_next(self) -> str:
        """The base the next robot stack start uses when it differs from the running one, else ""."""
        try:
            v = self.base_view()
            return v["next"] if v.get("next") and v["next"] != v.get("running") else ""
        except Exception:
            return ""

    # ------------------------------------------------------------------ drivers & pipelines (services.py)
    def _drivers_init(self):
        c, prof = self.core, self.core.profile.name
        here = config_dir() / "services"
        run = here / "run" if c.fake else Path("/tmp/moveit_gui_services") / prof   # /tmp: gone with the container
        logs = Path(os.environ["MOVEIT_GUI_LOG_DIR"]) / "services" if os.environ.get("MOVEIT_GUI_LOG_DIR") \
            else here / "logs"
        b = c.bridge
        # Real drivers with a real robot, and with a fake one started with --real-cameras; simulated ones otherwise.
        # The ZED runs in its own container (ur7e_gui.sh starts it and says where its runner listens).
        real = not c.fake or getattr(b, "real", None) is not None
        zed = os.environ.get("MOVEIT_GUI_ZED_DIR", "").strip() or None

        def have(pkg):
            if pkg == "zed_wrapper" and zed:
                return SV.remote_alive(zed)
            return SV.have_package(pkg) if real else True

        def route(s):
            if s.type == "zed" and zed and real:
                return lambda argv, log: SV.RemoteProc.start(zed, s.name, argv, log)
            return SV.spawn_process if real else SV.spawn_fake
        self.drivers = SV.Services(here / f"{prof}.yaml", run, logs, spawn=SV.spawn_fake, route=route,
                                   remote_root=zed, have=have, publishers=b.topic_publishers,
                                   on_change=None if real else self._driver_changed, log=b.log)

    def _driver_changed(self, s, up: bool):
        """Fake mode: a started fake driver 'publishes' its outputs, so the page and the preview work."""
        b = self.core.bridge
        for o in s.outputs():
            if o["image"]:
                (b.fake_cameras.__setitem__(o["topic"], ("rgb8", 640, 480, 15.0)) if up
                 else b.fake_cameras.pop(o["topic"], None))
            else:
                (b.fake_published.add if up else b.fake_published.discard)(o["topic"])

    def drivers_state(self) -> list:
        """Small, in every state: [{name, label, kind, state, msg}] (the full view is drivers_get)."""
        return [{k: r[k] for k in ("name", "label", "kind", "state", "msg")} for r in self.drivers.view()["services"]]

    def drivers_stop_all(self, wait: float = 6.0):
        """The GUI closes: stop the drivers, except on a GUI-only restart (the next GUI adopts them)."""
        if self.restart_code == RESTART_GUI:
            return
        self.drivers.stop_all(wait=wait)

    def cmd_drivers_get(self):
        return self._ok(**self.drivers.view(), types=SV.Services.types_view(self.drivers.have), kinds=SV.KINDS)
    cmd_drivers_get.read_only = True

    def cmd_drivers_save(self, service, replace=None):
        s = self.drivers.save(dict(service or {}), replace or None)
        return self._ok(f"{s.name} saved", **self.drivers.view())

    def cmd_drivers_delete(self, name):
        self.drivers.delete(str(name))
        return self._ok(f"{name} deleted", **self.drivers.view())

    def cmd_drivers_start(self, name):
        return self._ok(self.drivers.start(str(name)), **self.drivers.view())

    def cmd_drivers_stop(self, name):
        return self._ok(self.drivers.stop(str(name)), **self.drivers.view())

    def cmd_drivers_autostart(self, name, on=True):
        self.drivers.set_autostart(str(name), bool(on))
        return self._ok(f"{name}: {'starts' if on else 'does not start'} with the GUI", **self.drivers.view())

    def cmd_drivers_log(self, name, lines=60):
        return self._ok(**self.drivers.log_tail(str(name), max(5, min(400, int(lines)))))
    cmd_drivers_log.read_only = True

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
            ci = self.parts.auto_collision_info(p)
            out.append({"name": n, **p.to_yaml(), "status": st.get(n, "live"), "far_face_mm": round(hi * 1000, 2),
                        "collision_info": ci})
        removed = [n for n, s in st.items() if s == "removed"]
        restart = [n for n, s in st.items() if s in ("live", "changed", "removed")]
        links = sorted(self._robot_links())
        arm = [l for l in links if m.get("ok") and P.rides_on_arm(m, l)]
        return {"parts": out, "removed": removed, "restart_for": restart, "links": links, "arm_links": arm,
                "meshes": self.parts.meshes(), "mesh_info": {f: self.parts.mesh_info(f) for f in self.parts.meshes()},
                "presets": {k: v for k, v in P.PRESETS.items()},
                "problems": P.problems(self.parts, m) if m.get("ok") else [], "error": self.parts.error,
                "sync": {k: v for k, v in self.parts_sync.items() if k != "sent"},
                "collision_jobs": dict(self._collision_jobs),
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
                       collision=None, collision_settings=None, kind="part", note="", replace=None):
        old = self.parts.parts.get(replace or name)
        meta = dict(old.meta) if old else {}
        if old and meta.get("source") == "handeye" and (list(map(float, xyz_mm or [])) != old.xyz_mm or
                                                        list(map(float, rpy_deg or [])) != old.rpy_deg):
            meta = {k: v for k, v in meta.items() if k not in ("source", "camera")}
            meta["note"] = "offset edited after the hand-eye calibration"
        p = P.make_part(name, parent, xyz_mm, rpy_deg, shape, anchor, color, collision,
                        collision_settings=collision_settings, kind=kind, note=note, meta=meta)
        rep = replace or (name if old else None)
        links = self._robot_links()
        self.parts.check(p, links, replace=rep)
        if p.collision != "auto":
            self.parts.put(p, links, replace=rep)
            return self._parts_saved(f"part {p.name} saved")
        if self.parts.auto_collision_cached(p) is not None:
            self.parts.put(p, links, replace=rep)
            return self._parts_saved(f"part {p.name} saved")
        if any(j.get("state") == "generating" for j in self._collision_jobs.values()):
            raise P.PartError("another automatic collision is being generated")
        self._collision_jobs[p.name] = {"state": "generating", "msg": "generating collision…"}

        def generate():
            try:
                _, info = self.parts.auto_collision_files(p)
                self.parts.put(p, links, replace=rep)
                self._parts_saved(f"part {p.name} saved")
                self._collision_jobs[p.name] = {"state": "done", "msg": "collision generated", "info": info}
            except Exception as e:
                self._collision_jobs[p.name] = {"state": "error", "msg": str(e)}
                self._log_error("parts collision", e)
        threading.Thread(target=generate, daemon=True, name="parts-collision").start()
        return self._ok("generating collision…", generating=True, collision_job=p.name, parts=self.parts_view())

    def cmd_parts_collision_preview(self, file, scale=None, collision_settings=None, force=False):
        """Generate/reuse auto collision for an uploaded mesh without blocking the HTTP request."""
        f = str(file)
        info = self.parts.mesh_info(f)
        if info.get("error"):
            raise P.PartError(f"mesh {f} is missing")
        shape = {"type": "mesh", "file": f, "scale": info["scale"] if scale is None else scale}
        p = P.make_part("collision_preview", "tool0", shape=shape, collision="auto",
                        collision_settings=collision_settings)
        cached = None if force else self.parts.auto_collision_cached(p)
        if cached is not None:
            paths, result = cached
            return self._ok("collision generated", info=result,
                            preview=[{"uri": q.resolve().as_uri(), "scale": [p.shape["scale"]] * 3} for q in paths])
        _, manifest = self.parts._auto_cache(p)
        key = "preview:" + manifest.name
        job = self._collision_jobs.get(key)
        if job and job.get("state") == "generating":
            return self._ok("generating collision…", generating=True, collision_job=key)
        self._collision_jobs[key] = {"state": "generating", "msg": "generating collision…"}

        def generate():
            try:
                paths, result = self.parts.auto_collision_files(p, force=bool(force))
                self._collision_jobs[key] = {
                    "state": "done", "msg": "collision generated", "info": result,
                    "preview": [{"uri": q.resolve().as_uri(), "scale": [p.shape["scale"]] * 3} for q in paths],
                }
            except Exception as e:
                self._collision_jobs[key] = {"state": "error", "msg": str(e)}
                self._log_error("parts collision preview", e)
        threading.Thread(target=generate, daemon=True, name="parts-collision-preview").start()
        return self._ok("generating collision…", generating=True, collision_job=key)
    cmd_parts_collision_preview.read_only = True

    def cmd_parts_collision_preview_status(self, job):
        j = self._collision_jobs.get(str(job))
        if not j:
            raise P.PartError("no such collision preview")
        return self._ok(j.get("msg", ""), **{k: v for k, v in j.items() if k != "msg"})
    cmd_parts_collision_preview_status.read_only = True

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
        auto_users = [n for n in users if self.parts.parts[n].collision == "auto"]
        if auto_users:
            key = f"mesh:{f}"
            self._collision_jobs[key] = {"state": "generating", "msg": "regenerating collision…"}

            def regenerate():
                try:
                    for n in auto_users:
                        self.parts.auto_collision_files(self.parts.parts[n], force=True)
                    m = self._base_model()
                    if m.get("ok"):
                        P.write_generated(self.parts, m)
                    self._parts_sig = None
                    self.model_rev += 1
                    self._collision_jobs[key] = {"state": "done", "msg": "collision regenerated"}
                except Exception as e:
                    self._collision_jobs[key] = {"state": "error", "msg": str(e)}
                    self._log_error("parts collision", e)
            threading.Thread(target=regenerate, daemon=True, name="parts-collision").start()
        elif users:                                         # hull/box/visual stay cheap
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
        why = self.verified_standstill()
        if why:
            return why + ": stop it first"
        return None

    def _restart(self, code: int, why: str) -> bool:
        with self._restart_lock:
            if self.restart_code is not None:
                if code == RESTART_STACK and self.restart_code != RESTART_STACK:
                    self.restart_code = RESTART_STACK        # a stack stop/restart outranks a pending GUI-only restart
                    self.core.bridge.log(f"RESTART upgraded to robot stack + GUI: {why}")
                return False
            self.restart_code = code
        self.core.bridge.log(f"RESTART ({'robot stack + GUI' if code == RESTART_STACK else 'GUI'}): {why}")
        cb = self.on_restart
        if cb is not None:
            threading.Timer(0.8, cb).start()                 # let this reply reach the browser first
        return True

    def cmd_restart(self, what="gui"):
        why = self._restart_refusal()
        if why:
            raise P.PartError(why)
        if self.on_restart is None:
            raise P.PartError("this GUI was not started by the launcher (ur7e_gui.sh): restart it by hand")
        code = RESTART_STACK if what == "stack" else RESTART_GUI
        if not self._restart(code, "asked in the GUI"):
            return self._ok("a robot-stack restart is already pending" if self.restart_code == RESTART_STACK
                            else "a GUI restart is already pending")
        return self._ok("restarting the robot stack and the GUI: this page reconnects by itself (about a minute)"
                        if code == RESTART_STACK else "restarting the GUI: this page reconnects by itself")

    def cmd_stack_stop(self, who=""):
        """Stop the robot stack until someone presses Start: nothing of the robot runs, only a small start page
        (web/idle.py). For when nobody uses the robot."""
        why = self._restart_refusal()
        if why:
            raise P.PartError(why)
        if self.on_restart is None:
            raise P.PartError("this GUI was not started by the launcher (ur7e_gui.sh): stop it there")
        if self.restart_code == RESTART_STACK:
            return self._ok("the robot stack is already stopping")
        from .idle import marker
        marker(config_dir()).write_text(json.dumps({"by": str(who or "")[:24], "at": time.strftime("%Y-%m-%d %H:%M")}))
        self._restart(RESTART_STACK, f"robot stack stopped{' by ' + str(who)[:24] if who else ''}")
        return self._ok("stopping the robot stack: this page shows Start when it is down")

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

    # ------------------------------------------------------------------ F/T reading axes (ft_axes.py)
    def ft_axes_view(self) -> dict:
        b = self.core.ft_bias
        if not b.available:
            return {"available": False, "why": "no F/T broadcaster in this profile"}
        run = b.axes if b.live is not None else None
        nxt = FA.read_next(config_dir(), b.node)
        return {"available": True, "node": b.node, "running": FA.label(run), "next": FA.label(nxt),
                "running_name": FA.rotation_deg(run) if run else None, "next_name": FA.rotation_deg(nxt),
                "pending": run is not None and list(run) != list(nxt), "choices": FA.CHOICES,
                "why": "" if run is not None else (b.error or "the F/T broadcaster was not read yet")}

    def cmd_ft_axes_get(self):
        return self._ok(ft_axes=self.ft_axes_view())
    cmd_ft_axes_get.read_only = True

    def cmd_ft_axes_save(self, x, y, restart=False, confirm=False):
        """Keep the reading axes for the next stack start; with restart, restart the robot stack now."""
        b = self.core.ft_bias
        if not b.available:
            raise FA.FtAxesError("no F/T broadcaster in this profile")
        m = FA.mapping(x, y)
        if restart:
            why = self._restart_refusal()
            if why:
                raise FA.FtAxesError(why)
            if self.core.mode not in ("unknown",) and not self.core.modes.mode_supports_moveit(self.core.mode):
                raise FA.FtAxesError(f"change to Position first (now '{self.core.mode}')")
            if self.on_restart is None:
                raise FA.FtAxesError("this GUI was not started by the launcher (ur7e_gui.sh): the axes are used at "
                                     "the next stack start")
            if not confirm:
                raise FA.FtAxesError("confirm the restart of the robot stack")
        FA.write_next(config_dir(), m, b.node)
        lb = FA.label(m)
        txt = f"X reads sensor {lb['x']}, Y {lb['y']}, Z {lb['z']}"
        self.core.bridge.log(f"F/T reading axes set: {txt} (next stack start)")
        if restart:
            self._restart(RESTART_STACK, f"F/T reading axes set: {txt}")
            return self._ok("F/T reading axes saved: restarting the robot stack and the GUI (about a minute)",
                            restart="stack", ft_axes=self.ft_axes_view())
        return self._ok("F/T reading axes saved: used after the next robot stack restart",
                        ft_axes=self.ft_axes_view())

    # ------------------------------------------------------------------ robot: arm, real / fake, gripper, F/T, cameras
    def robot_view(self) -> dict:
        return R.view(config_dir(), self.core.fake)

    def robot_setup_view(self) -> dict:
        """Everything the Set up pop-up shows: robots, gripper / F/T on the running base, cameras."""
        v = self.robot_view()
        base = self.running_base()
        cams = [x for x in self.drivers.view()["services"] if x["kind"] == "camera"]
        types = [t for t in SV.Services.types_view(self.drivers.have) if t["kind"] == "camera"]
        robot = (v["running"] or v["next"])["robot"]
        return {**v, "base": base, **R.parts_of(robot, base), "cameras": [{"name": c["label"], "type": c["type"],
                "label": c["type_label"], "state": c["state"]} for c in cams],
                "camera_types": [{k: t[k] for k in ("type", "label", "help", "installed", "install")} for t in types]}

    def cmd_robot_view(self):
        return self._ok(robot=self.robot_setup_view())
    cmd_robot_view.read_only = True

    def cmd_robot_setup(self, robot, mode, ip, gripper, ft, ft_ip=None, cameras=(), confirm=False, domain=None, net=None,
                        servo=None):
        """Set up this PC's robot in one go: arm, real / fake, IPs, gripper, F/T sensor (the base), ROS domain and
        network, and cameras to add. Restarts the robot stack when any of those but the cameras changes."""
        new = R.check(robot, mode, ip, ft_ip, domain, net, servo)
        label = R.ROBOTS[new["robot"]]["label"]
        gone = R.missing(new["robot"])
        if gone:
            raise R.RobotError(f"{label} is not set up on this PC: " + "; ".join(gone))
        base = R.base_of(new["robot"], str(gripper), str(ft))
        run = R.running(self.core.fake)
        if run is None:
            raise R.RobotError("setting up the robot restarts the robot stack, and this GUI was not started by the "
                               "launcher (ur7e_gui.sh)")
        types = {t["type"]: t for t in SV.Services.types_view(self.drivers.have) if t["kind"] == "camera"}
        add = [str(c) for c in (cameras or [])]
        for c in add:
            if c not in types:
                raise R.RobotError(f"no camera type '{c}' (one of {', '.join(types)})")
            if not types[c]["installed"]:
                raise R.RobotError(f"{types[c]['label']} is not installed on this PC: {types[c]['install']}")
        restart = new != run or base != self.running_base()
        if restart:
            why = self._restart_refusal()
            if why:
                raise R.RobotError(why)
            if not self.core.fake and self.core.mode not in ("unknown",) \
                    and not self.core.modes.mode_supports_moveit(self.core.mode):
                raise R.RobotError(f"change to Position first (now '{self.core.mode}')")
            if not confirm:
                raise R.RobotError("confirm the setup (the robot stack restarts)")
        added = []
        for c in add:
            rows = self.drivers.view()["services"]
            if any(x["type"] == c for x in rows):           # a second one would fight the first for the camera:
                continue                                    # add it in Drivers & cameras, with its serial number
            have = {x["name"] for x in rows}
            name = next(n for n in [c] + [f"{c}_{i}" for i in range(2, 99)] if n not in have)
            self.drivers.save({"name": name, "label": types[c]["label"], "type": c, "params": {}, "autostart": True})
            added.append(types[c]["label"])
        R.write_next(config_dir(), **new)
        B.write_next(config_dir(), base)
        parts = [label, new["mode"], B.BASES[base][0].lower(), f"ROS domain {new['domain']}", R.NETS[new["net"]][0].lower(),
                 f"continuous jog {new['servo']}"] + ([f"camera {', '.join(added)} added"] if added else [])
        if restart:
            self._restart(RESTART_STACK, "robot set up: " + ", ".join(parts))
            return self._ok(", ".join(parts) + ": restarting the robot stack and the GUI (about a minute)",
                            restart="stack", robot=self.robot_setup_view())
        return self._ok("saved: " + ", ".join(parts) + " (nothing to restart)", robot=self.robot_setup_view())

    # ------------------------------------------------------------------ profiles
    def _setups(self, owner="") -> Setups:
        """The setups of profile `owner` ("" = the shared ones)."""
        if not str(owner or "").strip():
            return self.setups
        o = self.profiles.find(owner)
        return Setups(config_dir(), self.core.profile.name, owner=o, root=self.profiles.setups_dir(o))

    # ---- whose settings are in use (worksets.py): the robot runs the settings of the one in control
    def _profile_of(self, lease_name: str) -> str:
        """The profile a lease name ("Cenk @ 100.x.y.z") belongs to, "" if none."""
        n = str(lease_name or "").split(" @ ")[0].strip()
        try:
            return self.profiles.find(n) if n else ""
        except ProfileError:
            return ""

    def _claim_settings(self, lease):
        """Settings that are nobody's yet (from before profiles) become the first taker's, without a restart."""
        me = lease.profile
        if me and not self.worksets.active() and not self.worksets.has(me):
            with self._settings_lock:
                if not self.worksets.active() and not self.worksets.has(me):
                    self.worksets.save(me, base=self.running_base())
                    self.worksets.set_active(me)
            self.core.bridge.log(f"web: the settings in use are now {me}'s (kept in their profile)")

    def _needs_switch(self, owner: Optional[str] = None) -> bool:
        """The one in control does not have their own settings in use (they must switch first)."""
        L = self.lease
        if not L.token or not self.profiles.names():
            return False
        owner = self.worksets.active() if owner is None else owner
        return (L.profile or "").lower() != owner.lower()

    def _switch_refusal(self) -> str:
        act = self.worksets.active()
        if not self.lease.profile:
            return (f"pick your profile first (your name in the header): the robot has {act}'s settings in use, "
                    "and only they can change them")
        return (f"the robot has {act or 'other'}'s settings in use: switch to your own settings first "
                "(Use my settings, in the pop-up or SETUP › Setups)")

    def _own(self, owner) -> str:
        """`owner` ("" = Shared) checked against the one in control: nobody changes another person's things."""
        o = str(owner or "").strip()
        if not o:
            return ""
        o = self.profiles.find(o)
        mine = self.lease.profile if self.lease.token else ""
        if o.lower() != (mine or "").lower():
            raise ProfileError(f"that belongs to {o}: you can only change your own"
                               + (f" ({mine})" if mine else " (pick your profile first)"))
        return o

    def settings_view(self) -> dict:
        t, v = getattr(self, "_settings_cache", (0.0, None))
        if v is None or time.monotonic() - t > 2.0:
            v = {"owner": self.worksets.active(), "have": [n for n in self.profiles.names() if self.worksets.has(n)]}
            self._settings_cache = (time.monotonic(), v)
        return v

    def cmd_profile_activate(self, start="", owner="", name=""):
        """Put the settings of the one in control in place: the ones in use go back to their profile, the GUI
        restarts (and the robot stack when the parts or the base differ). A profile without settings of its own
        starts from `start`: "" = a copy of the settings in use, "setup" = setup `name` of `owner` ("" = Shared)."""
        caller_lease = self.lease
        caller_token = caller_lease.token

        def check_lease():
            with self._lease_lock:
                if self.lease is not caller_lease or not self.lease.holds(caller_token):
                    raise P.PartError("control changed: settings switch refused")

        check_lease()
        me = caller_lease.profile
        if not me:
            raise ProfileError("pick your profile first (your name in the header)")
        act = self.worksets.active()
        if act.lower() == me.lower():
            return self._ok("your settings are in use already")
        why = self._restart_refusal()
        if why:
            raise P.PartError(why)
        snap = self.core.bridge.snapshot()
        if self.core.mode == "none" and (snap.controllers_age is None or
                                          snap.controllers_age > self.core.CONTROLLERS_STALE_S):
            raise P.PartError("the controller list is stale: cannot switch settings safely")
        if self.core.mode == "unknown":
            raise P.PartError("control mode is unknown: cannot switch settings safely")
        if self.core.mode != "none" and not self.core.modes.mode_supports_moveit(self.core.mode):
            raise P.PartError(f"change to Position first (now '{self.core.mode}'): the switch changes gains and "
                              "payload")
        if self.on_restart is None:
            raise P.PartError("switching settings restarts the GUI, and this GUI was not started by the launcher "
                              "(ur7e_gui.sh)")
        base_now = self.running_base()
        src = None
        if not self.worksets.has(me) and start == "setup":
            S = self._setups(owner)
            info = S.info(str(name))                       # raises SetupError: no such setup
            if info.get("profile") and info["profile"] != self.core.profile.name:
                raise ProfileError(f"setup '{name}' is for robot profile {info['profile']}")
            src = (S, str(name), str(info.get("base") or ""))
        with self._settings_lock:
            tx = None
            committed = False
            try:
                check_lease()
                tx = self.worksets.begin_switch(me)
                if act:
                    self.worksets.save(act, base=base_now)     # old owner's save is part of the transaction
                if self.worksets.has(me):
                    info = self.worksets.info(me)
                    source, base = self.worksets._dir(me) / "files", str(info.get("base") or "")
                    setup = info.get("setup") or {}
                    how = "your settings"
                    preserve_setup = False
                elif src is not None:
                    S, n, base = src
                    source = S._path(n) / "files"
                    setup = {"name": n, **({"owner": S.owner} if S.owner else {})}
                    how = f"setup '{n}'" + (f" of {S.owner}" if S.owner else " (Shared)")
                    preserve_setup = False
                else:
                    source, base = tx / "old" / "files", ""
                    setup = None
                    how = f"a copy of {act + chr(39) + 's' if act else 'the'} settings"
                    preserve_setup = True
                self.worksets.stage_switch(tx, source, setup, preserve_setup)
                self.worksets.apply_switch(tx)
                self.worksets.save(me, base=base or base_now)  # persist before active.yaml can change
                m = self._base_model()
                stack = bool(m.get("ok")) and any(st != "in_model" for st in
                                                   P.status(P.PartStore(self.parts.path), m).values())
                check_lease()          # a takeover mid-switch rolls it back (before base_next is written)
                if base and base in B.BASES and base != base_now:
                    B.write_next(config_dir(), base)
                    stack = True
                self.worksets.commit_switch(tx)             # active.yaml is the final publication
                committed = True
            except Exception as e:
                if tx is not None and not committed:
                    try:
                        self.worksets.abort_switch(tx)
                    except Exception as rollback:
                        self.core.bridge.log(f"profile switch rollback FAILED: {rollback}")
                if isinstance(e, OSError):
                    raise ProfileError(f"could not switch settings: {e}") from None
                raise
        self._restart(RESTART_STACK if stack else RESTART_GUI, f"{me}'s settings in use ({how})")
        kept = f"{act}'s are kept in their profile; " if act else ""
        return self._ok(f"{me}'s settings in use ({how}): {kept}restarting {'the robot stack and ' if stack else ''}"
                        "the GUI", restart="stack" if stack else "gui")

    def cmd_profile_list(self):
        return self._ok(profiles=self.profiles.list(), in_use=self.setups.in_use(), settings=self.settings_view())
    cmd_profile_list.read_only = True

    def cmd_profile_create(self, name):
        n = self.profiles.create(str(name))
        self.core.bridge.log(f"profile '{n}' ready")
        return self._ok(f"profile '{n}' ready", name=n, profiles=self.profiles.list())
    cmd_profile_create.read_only = True       # picking who you are needs no control of the robot

    def cmd_profile_delete(self, name):
        n = self.profiles.find(str(name))
        if n.lower() == self.worksets.active().lower():
            raise ProfileError(f"{n}'s settings are in use: someone else must switch to theirs first")
        if self.worksets.has(n) and n.lower() != (self.lease.profile or "").lower():
            raise ProfileError(f"profile '{n}' still has its own settings: only {n} can delete it")
        self.profiles.delete(n)
        return self._ok(f"profile '{n}' deleted", profiles=self.profiles.list())

    def cmd_profile_prefs_get(self, name):
        return self._ok(prefs=self.profiles.prefs(str(name)))
    cmd_profile_prefs_get.read_only = True

    def cmd_profile_prefs_save(self, name, prefs=None):
        n = self.profiles.find(str(name))
        mine = str(self.lease.profile or "").strip()
        if not mine or n.lower() != mine.lower():
            raise ProfileError("you can only save your own profile layout")
        self.profiles.save_prefs(n, prefs or {})
        return self._ok()

    def cmd_profile_prefs_copy(self, name, source):
        n = self.profiles.find(str(name))
        mine = str(self.lease.profile or "").strip()
        if not mine or n.lower() != mine.lower():
            raise ProfileError("you can only copy into your own profile layout")
        d = self.profiles.copy_prefs(str(source), n)
        return self._ok(f"layout copied from {self.profiles.find(str(source))}", prefs=d)

    # ------------------------------------------------------------------ setups
    def setup_current(self) -> dict:
        """The setup in use and what changed since it was saved or loaded (read from disk: call it rarely)."""
        u = self.setups.in_use()
        try:
            S = self._setups(u["owner"])
        except ProfileError:
            S = self.setups
        n = S.current()
        out = {"name": n, "owner": S.owner if n else "", "changed": [], "base_changed": False,
               "present": self.setups.present(), "settings_owner": self.worksets.active()}
        if n:
            try:
                out["changed"] = S.changes(n)
                b = str(S.info(n).get("base") or "")
                out["base_changed"] = bool(b) and b != self.running_base()
            except Exception as e:
                out["error"] = str(e)
        self._setup_cache = (time.monotonic(), out)
        return out

    def _setup_summary(self) -> dict:
        t, v = getattr(self, "_setup_cache", (0.0, None))
        if v is None or time.monotonic() - t > SETUP_CHECK_S:
            try:
                v = self.setup_current()
            except Exception as e:
                v = {"name": "", "changed": [], "base_changed": False, "present": [], "error": str(e)}
                self._setup_cache = (time.monotonic(), v)
        return {k: v.get(k, "") for k in ("name", "owner", "changed", "base_changed", "settings_owner")}

    def cmd_setup_list(self, owner=""):
        S = self._setups(owner)
        return self._ok(setups=S.list(), owner=S.owner, base=self.base_view(), current=self.setup_current(),
                        profiles=self.profiles.list(), robot=self.robot_view(), guide=self.setup_guide())
    cmd_setup_list.read_only = True

    def cmd_setup_copy(self, source_owner, name, owner="", new_name="", replace=False):
        """Copy a setup from another profile (or Shared) into profile `owner` (your own)."""
        with self._settings_lock:
            S = self._setups(self._own(owner))
            n = S.copy_from(self._setups(source_owner), str(name), str(new_name or ""), bool(replace))
        return self._ok(f"setup '{name}' copied to {S.owner or 'Shared'} as '{n}'", setups=S.list(), owner=S.owner,
                        current=self.setup_current())

    def cmd_setup_preview(self, name, owner=""):
        """What loading setup `name` changes: the files, the base, and what restarts."""
        S = self._setups(owner)
        info = S.info(str(name))
        changes = S.changes(str(name))
        d = S._path(str(name)) / "files"
        rel = S_members(self.core.profile.name)["parts"]
        m = self._base_model()
        stack = bool(m.get("ok")) and any(st != "in_model" for st in P.status(P.PartStore(d / rel), m).values())
        b, run = str(info.get("base") or ""), self.running_base()
        base = {"from": run, "to": b, "from_label": B.BASES.get(run, (run,))[0],
                "to_label": B.BASES.get(b, (b,))[0]} if b and b != run else None
        return self._ok(changes=changes, base=base, restart="stack" if (stack or base) else "gui",
                        holds=info.get("holds", []), note=info.get("note", ""), saved=info.get("saved", ""))
    cmd_setup_preview.read_only = True

    def _cameras_on(self) -> list:
        """The camera drivers this robot runs now: running, or started with the GUI."""
        return [x["name"] for x in self.drivers.view()["services"]
                if x["kind"] == "camera" and (x["state"] not in ("stopped", "failed", "missing") or x["autostart"])]

    def _setup_load_snapshot(self) -> Path:
        snap = self.worksets.snapshot()
        try:
            meta = {"base": False, "drivers": False,
                    "autostart": {n: bool(s.autostart) for n, s in self.drivers.services.items()}}
            for source, name, key in ((config_dir() / B.FILE, "base", "base"),
                                      (self.drivers.path, "drivers.yaml", "drivers")):
                if source.is_file():
                    shutil.copy2(source, snap / name)
                    meta[key] = True
            (snap / "load.json").write_text(json.dumps(meta))
            return snap
        except Exception:
            self.worksets.drop(snap)
            raise

    def _restore_setup_load(self, snap: Path):
        self.worksets.restore(snap)
        meta = json.loads((snap / "load.json").read_text())
        for target, name, key in ((config_dir() / B.FILE, "base", "base"),
                                  (self.drivers.path, "drivers.yaml", "drivers")):
            if (snap / name).is_file():
                target.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(snap / name, target)
            elif not meta.get(key) and target.is_file():
                target.unlink()
        for n, on in meta.get("autostart", {}).items():
            if n in self.drivers.services:
                self.drivers.services[n].autostart = bool(on)

    def setup_guide(self) -> dict:
        """What the Setups page's step list shows: the parts, the cameras and their calibrations now."""
        cams = [{"name": x["name"], "label": x["label"], "state": x["state"]}
                for x in self.drivers.view()["services"] if x["kind"] == "camera"]
        return {"parts": list(self.parts.ordered()), "cameras": cams, "on": self._cameras_on(),
                "calibrations": [c["name"] for c in self.cams.view()]}

    def cmd_setup_save(self, name, note="", replace=False, owner=""):
        with self._settings_lock:
            S = self._setups(self._own(owner))
            holds = S.save(str(name), str(note or ""), bool(replace), base=self.running_base(), cameras=self._cameras_on())
        who = f" in {S.owner}" if S.owner else ""
        self.core.bridge.log(f"setup '{name}'{who} saved: {', '.join(holds) or 'nothing to save yet'}")
        return self._ok(f"setup '{name}' saved{who} ({len(holds)} item(s))", setups=S.list(), owner=S.owner,
                        base=self.base_view(), current=self.setup_current())

    def cmd_setup_delete(self, name, owner=""):
        S = self._setups(self._own(owner))
        S.delete(str(name))
        return self._ok(f"setup '{name}' deleted", setups=S.list(), owner=S.owner, base=self.base_view(),
                        current=self.setup_current())

    def cmd_setup_load(self, name, owner=""):
        why = self._restart_refusal()
        if why:
            raise P.PartError(why)
        if self.core.mode not in ("unknown",) and not self.core.modes.mode_supports_moveit(self.core.mode):
            raise P.PartError(f"change to Position first (now '{self.core.mode}'): the setup changes gains and payload")
        if self.on_restart is None:
            raise P.PartError("loading a setup restarts the GUI, and this GUI was not started by the launcher "
                              "(ur7e_gui.sh)")
        with self._settings_lock:
            snap = None
            try:
                snap = self._setup_load_snapshot()
                # anyone's setup can be loaded into your settings; the backup of the files in use goes to your profile
                mine = self.lease.profile if self.lease.token else ""
                r = self._setups(owner).load(str(name), base=self.running_base(), cameras=self._cameras_on(),
                                             backup_to=self._setups(mine) if mine else None)
                if r.get("cameras") is not None:              # its cameras start with the GUI, the others stop
                    for x in self.drivers.view()["services"]:
                        if x["kind"] != "camera":
                            continue
                        on = x["name"] in r["cameras"]
                        self.drivers.set_autostart(x["name"], on)
                        if not on and x["state"] not in ("stopped", "failed", "missing"):
                            self.drivers.stop(x["name"])
                # the new files are in place: does the running robot model have the setup's parts, on the setup's base?
                new = P.PartStore(self.parts.path)
                m = self._base_model()
                stack = bool(m.get("ok")) and any(s != "in_model" for s in P.status(new, m).values())
                base = r.get("base")
                if base and base != self.running_base():
                    B.write_next(config_dir(), base)
                    stack = True
                elif base:
                    B.write_next(config_dir(), base)             # keep the next start on it too
                self._restart(RESTART_STACK if stack else RESTART_GUI, f"setup '{name}' loaded")
            except Exception as e:
                if snap is not None:
                    try:
                        self._restore_setup_load(snap)
                    except Exception as rollback:
                        self.core.bridge.log(f"setup load rollback FAILED: {rollback}")
                if isinstance(e, OSError):
                    raise SetupError(f"could not load setup: {e}") from None
                raise
            finally:
                if snap is not None:
                    self.worksets.drop(snap)
        return self._ok(f"setup '{name}' loaded (the files in use before are kept as '{r['backup']}'): restarting "
                        f"{'the robot stack and ' if stack else ''}the GUI" +
                        (f" on the '{B.BASES[base][0]}' base" if base else ""), restart="stack" if stack else "gui")

    # ------------------------------------------------------------------ hand-eye
    def he_view(self) -> dict:
        h = self.he
        r = h["result"]
        cams = [{"name": c.get("name", c["topic"]), "topic": c["topic"], "info": c.get("info", "")}
                for c in self.core.profile.cameras if not str(c.get("topic", "")).endswith(("depth", "depth_registered"))]
        mine = []                                          # colour pictures of SETUP > Drivers & cameras first
        for d in self.drivers.view()["services"]:
            info = next((o["topic"] for o in d["outputs"] if o["topic"].endswith("camera_info")), "")
            mine += [{"name": f"{d['name']} ({o['label']})", "topic": o["topic"], "info": info} for o in d["outputs"]
                     if o["image"] and info and "depth" not in o["topic"]]
        cams = mine + [c for c in cams if all(m["topic"] != c["topic"] for m in mine)]
        res = None
        T_so = self._he_T_sensor_optical()
        if r:
            xyz, rpy = tf.T_to_mm_deg(r["X"] @ tf.inv(T_so))         # shown and edited in the picked camera frame
            res = {k: v for k, v in r.items() if k not in ("X", "T_board")}
            res.update(xyz_mm=[round(v, 2) for v in xyz], rpy_deg=[round(v, 3) for v in rpy])
        return {"board": self.cams.board, "cameras": cams, "saved": self.cams.view(), "topic": h["topic"],
                "info_topic": h["info_topic"], "mount": h["mount"], "frame": h["frame"],
                "intrinsics": None if h["K"] is None else {"fx": h["K"][0], "fy": h["K"][4], "cx": h["K"][2],
                                                           "cy": h["K"][5], "D": h["D"][:5]},
                "samples": [{"corners": s.corners, "reproj_px": round(s.reproj_px, 3)} for s in h["samples"]],
                "poses": len(h["poses"]), "result": res, "last": h["last"], "progress": dict(h["progress"]),
                "flange": self._he_ee(), "base": self._he_base(), "links": self._he_links(),
                "default_ee": self.core.profile.planning_link, "default_base": self.core.profile.base_link,
                "method": h["method"], "methods": list(he.METHODS),
                "sensor": self._he_sensor(), "sensor_frames": self._he_sensor_frames(),
                "T_sensor_optical": [round(float(v), 7) for v in T_so.reshape(-1)],
                "min_samples": he.MIN_SAMPLES, "good_samples": he.GOOD_SAMPLES}

    def _he_sensor(self) -> str:
        return self.he["sensor_frame"] or self.he["frame"]

    def _he_sensor_frames(self) -> list:
        """The camera driver's own TF tree (e.g. d405_link > d405_color_frame > d405_color_optical_frame): the frames
        the result can be given in, like MoveIt's sensor frame. Stops at the robot (a camera part hangs on it)."""
        opt = self.he["frame"]
        if not opt:
            return []
        par = self.core.bridge.tf_parents() if hasattr(self.core.bridge, "tf_parents") else {}
        stop = set(self._robot_links()) | {n for n, q in self.parts.parts.items() if not (q.meta or {}).get("camera")}
        root, hops = opt, 0
        while root in par and par[root] not in stop and hops < 10:
            root, hops = par[root], hops + 1
        kids: Dict[str, list] = {}
        for c, p_ in par.items():
            kids.setdefault(p_, []).append(c)
        out, todo = [], [root]
        while todo and len(out) < 50:
            f = todo.pop()
            out.append(f)
            todo += kids.get(f, [])
        return sorted(set(out) | {opt})

    def _he_T_sensor_optical(self) -> np.ndarray:
        s, opt = self._he_sensor(), self.he["frame"]
        if not opt or s == opt:
            return np.eye(4)
        T = self.core.bridge.lookup_transform(s, opt)
        return T if T is not None else np.eye(4)

    def cmd_handeye_sensor(self, frame=""):
        """The camera frame the result is given, edited and saved in (default: the optical frame of the image)."""
        self._he_require()
        frame = str(frame or "").strip()
        if frame and frame != self.he["frame"]:
            if frame not in self._he_sensor_frames():
                raise he.HandEyeError(f"{frame} is not in the camera's TF tree")
            if self.core.bridge.lookup_transform(frame, self.he["frame"]) is None:
                raise he.HandEyeError(f"no TF {frame} -> {self.he['frame']}")
        self.he["sensor_frame"] = "" if frame == self.he["frame"] else frame
        return self._ok(f"results in {self._he_sensor()}", he=self.he_view())

    def _he_ee(self) -> str:
        return self.he["ee_link"] or self.core.profile.planning_link

    def _he_base(self) -> str:
        return self.he["base_link"] or self.core.profile.base_link

    def _he_links(self) -> list:
        """Frames a camera can be calibrated against: the robot's links and the parts (not camera parts)."""
        parts = [n for n, q in self.parts.parts.items() if not (q.meta or {}).get("camera")]
        p = self.core.profile
        return sorted(set(self._robot_links()) | set(parts) | {p.planning_link, p.base_link, self._he_ee(), self._he_base()})

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

    def cmd_handeye_start(self, topic, info_topic="", mount="eye_in_hand", ee_link="", base_link=""):
        if self.he["progress"]["running"]:
            raise he.HandEyeError("a pose run is in progress")
        if mount not in he.MOUNTS:
            raise he.HandEyeError(f"mount is one of {', '.join(he.MOUNTS)}")
        p = self.core.profile
        ee, base = str(ee_link or "").strip() or p.planning_link, str(base_link or "").strip() or p.base_link
        if ee == base:
            raise he.HandEyeError("the end-effector link and the base link must differ")
        if self.core.bridge.lookup_transform(base, ee) is None:
            raise he.HandEyeError(f"no transform {base} -> {ee} in TF: pick links of this robot")
        topic = str(topic).strip()
        info_topic = str(info_topic or "").strip() or re.sub(r"/[^/]+$", "/camera_info", topic)
        info, why = self.core.bridge.camera_info(info_topic)
        if info is None:
            raise he.HandEyeError(f"{why}. The camera's CameraInfo gives the intrinsics (RealSense / ZED publish it "
                                  f"next to the image)")
        if not any(info["K"]):
            raise he.HandEyeError(f"{info_topic}: the camera matrix is all zeros (camera not calibrated)")
        self.he.update(topic=topic, info_topic=info_topic, mount=mount, K=info["K"], D=info["D"],
                       size=(int(info["width"]), int(info["height"])),
                       ee_link="" if ee == p.planning_link else ee, base_link="" if base == p.base_link else base,
                       frame=info["frame_id"], samples=[], result=None, poses=[], last=None)
        if self.he["sensor_frame"] and self.he["sensor_frame"] not in self._he_sensor_frames():
            self.he["sensor_frame"] = ""
        self.core.bridge.log(f"hand-eye: {mount} with {topic} ({info['width']}x{info['height']}, frame "
                             f"{info['frame_id']}), {base} -> {ee}")
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
        if d.quads:                                         # the ArUco markers found, blue
            cv2.polylines(vis, [np.asarray(q, np.int32) for q in d.quads], True, (230, 140, 30),
                          max(1, img.shape[1] // 640))
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
        self.he["last"] = {"ok": d.ok, "corners": d.corners, "markers": d.markers, "reproj_px": None if not d.ok else round(d.reproj_px, 3),
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

    @staticmethod
    def _he_depth_topic(color: str) -> str:
        """The depth image registered to a colour topic: RealSense aligned_depth_to_color, ZED depth_registered."""
        if "/color/" in color:                                          # RealSense: .../color/image_raw
            return color.split("/color/")[0] + "/aligned_depth_to_color/image_raw"
        if "/rgb/" in color:                                            # ZED: .../rgb/color/rect/image
            return color.split("/rgb/")[0] + "/depth/depth_registered"
        return ""

    def cmd_handeye_cloud(self, depth_topic="", stride=4, max_m=1.5):
        """The camera's point cloud for the 3D view, in the optical frame, with where the camera's parent link was
        when it was taken. The page places it with the calibration (or the pose being edited), so a good result
        puts the cloud on the real table and parts. Packed: int16 x, y, z in mm, then uint8 r, g, b; zlib, base64."""
        self._he_require()
        color = self.he["topic"]
        depth = str(depth_topic or "").strip() or self._he_depth_topic(color)
        if not depth:
            raise he.HandEyeError("no depth topic known for this camera: type it")
        stride = int(min(16, max(1, int(stride))))
        p, b = self.core.profile, self.core.bridge
        T0 = b.lookup_transform(p.base_link, self._he_ee() if self.he["mount"] == "eye_in_hand" else self._he_base())
        rgb, d, why = b.grab_rgbd(color, depth)
        T1 = b.lookup_transform(p.base_link, self._he_ee() if self.he["mount"] == "eye_in_hand" else self._he_base())
        if rgb is None:
            raise he.HandEyeError(why)
        if T0 is None or T1 is None:
            raise he.HandEyeError("no TF for the camera's parent link")
        H, W = d.shape
        K = np.asarray(self.he["K"], float).reshape(3, 3)
        if rgb.shape[:2] != d.shape:                       # colour at another size: pick its pixels by scale
            ys = (np.arange(H) * rgb.shape[0] / H).astype(int)
            xs = (np.arange(W) * rgb.shape[1] / W).astype(int)
            rgb = rgb[ys][:, xs]
        cw, ch = self.he.get("size") or (W, H)
        sx, sy = W / float(cw), H / float(ch)
        fx, fy, cx, cy = K[0, 0] * sx, K[1, 1] * sy, K[0, 2] * sx, K[1, 2] * sy   # K is for the colour size
        v, u = np.mgrid[0:H:stride, 0:W:stride]
        z = d[v, u]
        ok = np.isfinite(z) & (z > 0.05) & (z < float(max_m))
        u, v, z = u[ok], v[ok], z[ok]
        xyz = np.stack([(u - cx) * z / fx, (v - cy) * z / fy, z], axis=1)
        q = np.clip(np.round(xyz * 1000), -32767, 32767).astype("<i2")
        col = rgb[v, u].astype(np.uint8)
        blob = zlib.compress(q.tobytes() + col.tobytes(), 3)
        moved = float(np.linalg.norm(T0[:3, 3] - T1[:3, 3])) > 0.002
        return self._ok("", n=int(len(q)), data=base64.b64encode(blob).decode(), stride=stride, depth_topic=depth,
                        T_view_parent=[round(float(x), 7) for x in T1.reshape(-1)], moving=moved,
                        mount=self.he["mount"], frame=self.he["frame"])
    cmd_handeye_cloud.read_only = True

    def _flange(self) -> Optional[np.ndarray]:
        """T_base_ee of the links picked at Start (default: the profile's base and planning link)."""
        return self.core.bridge.lookup_transform(self._he_base(), self._he_ee())

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
            if self._he_base() != self.core.profile.base_link:                 # into the picked base frame
                T_b_b0 = self.core.bridge.lookup_transform(self._he_base(), self.core.profile.base_link)
                if T_b_b0 is None:
                    raise he.HandEyeError(f"no TF {self._he_base()} -> {self.core.profile.base_link}")
                pivot = (T_b_b0 @ np.r_[pivot, 1])[:3]
        poses = he.sample_poses(T, pivot, tilt)
        p = self.core.profile
        if self._he_ee() != p.planning_link or self._he_base() != p.base_link:   # MoveIt moves the planning link
            T_b0_b = self.core.bridge.lookup_transform(p.base_link, self._he_base())
            T_ee_pl = self.core.bridge.lookup_transform(self._he_ee(), p.planning_link)
            if T_b0_b is None or T_ee_pl is None:
                raise he.HandEyeError("no TF between the picked links and the planning link")
            poses = [T_b0_b @ P_ @ T_ee_pl for P_ in poses]
        self.he["poses"] = poses
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
        r = he.solve(self.he["samples"], self.he["mount"], self.he["method"])
        self.he["result"] = r
        return r

    def cmd_handeye_solve(self, method=None):
        self._he_require()
        if method is not None:
            if method not in he.METHODS:
                raise he.HandEyeError(f"solver is one of {', '.join(he.METHODS)}")
            self.he["method"] = method
        r = self._he_solve()
        return self._ok(f"{r['verdict']}: board placed within {r['rms_mm']} mm / {r['rms_deg']} deg (rms) by "
                        f"{r['n']} samples", he=self.he_view())

    def cmd_handeye_save(self, name, as_part=True, xyz_mm=None, rpy_deg=None):
        """Save the result; with xyz_mm / rpy_deg, the pose as adjusted by hand (checked against the point cloud)."""
        r = self.he["result"]
        if r is None:
            raise he.HandEyeError("solve first")
        if xyz_mm is not None or rpy_deg is not None:
            try:
                xyz, rpy = [float(v) for v in xyz_mm], [float(v) for v in rpy_deg]
            except (TypeError, ValueError):
                raise he.HandEyeError("the adjusted pose is x, y, z in mm and roll, pitch, yaw in deg") from None
            if len(xyz) != 3 or len(rpy) != 3 or not all(np.isfinite(xyz + rpy)):
                raise he.HandEyeError("the adjusted pose is x, y, z in mm and roll, pitch, yaw in deg")
            X = tf.mm_deg_to_T(xyz, rpy) @ self._he_T_sensor_optical()          # typed in the camera frame
            dmm = float(np.linalg.norm(X[:3, 3] - r["X"][:3, 3]) * 1000)
            ddeg = float(np.degrees(tf.rotation_angle(r["X"][:3, :3].T @ X[:3, :3])))
            if dmm > 0.05 or ddeg > 0.005:
                r = {**r, "X": X, "edited": {"mm": round(dmm, 2), "deg": round(ddeg, 3)}}
                self.he["result"] = r
        p = self.core.profile
        old = self.cams.cameras.get(str(name))
        e = self.cams.save(str(name), self.he["topic"], self.he["info_topic"], self.he["frame"], self._he_ee(),
                           self._he_base(), r, self.he["samples"], self.he["K"])
        if r.get("loaded") and not self.he["samples"] and old:     # re-saved after a check: keep its samples
            e.update(sample_data=old.get("sample_data", []), samples=old.get("samples", 0))
        e["sensor_frame"] = self._he_sensor()
        e["T_parent_sensor"] = he._T(r["X"] @ tf.inv(self._he_T_sensor_optical()))
        self.cams._write()
        msg = f"camera {name} saved ({e['verdict']}" + (f", adjusted by hand {r['edited']['mm']} mm / "
                                                         f"{r['edited']['deg']} deg" if r.get("edited") else "") + ")"
        if as_part:
            msg += "; " + self._camera_part(str(name))
        return self._ok(msg, he=self.he_view(), parts=self.parts_view())

    def cmd_handeye_load(self, name):
        """Open a saved camera calibration to check it against the point cloud and adjust it by hand."""
        e = self.cams.cameras.get(str(name))
        if e is None:
            raise he.HandEyeError(f"no camera calibration '{name}'")
        p = self.core.profile
        ee = e.get("ee_link") or (e["parent"] if e["mount"] == "eye_in_hand" else p.planning_link)
        base = e.get("base_link") or (e["parent"] if e["mount"] == "eye_to_hand" else p.base_link)
        self.cmd_handeye_start(e["topic"], e.get("info_topic", ""), e["mount"], ee_link=ee, base_link=base)
        self.he["method"] = e.get("method", "park")
        self.he["sensor_frame"] = e.get("sensor_frame", "") if e.get("sensor_frame") in self._he_sensor_frames() else ""
        self.he["result"] = {"X": self.cams.T_parent_optical(str(name)), "mount": e["mount"], "method": self.he["method"],
                             "verdict": e["verdict"], "rms_mm": e["rms_mm"], "rms_deg": e["rms_deg"],
                             "per_sample": [], "methods": e.get("methods", {}), "orientation_spread_deg": "—",
                             "n": e["samples"], "loaded": str(name)}
        return self._ok(f"camera {name} opened: check it with the point cloud, adjust, Save", he=self.he_view())

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
