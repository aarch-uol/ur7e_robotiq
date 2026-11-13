"""PROGRAMS > Teach & more > Teach by hand (WebService mixin). The maths is in moveit_gui/teach.py.

Teaching only reads where the arm is and edits the open program: it never moves the robot (freedrive, or jogging,
is the operator's hands). It needs control like every program edit, and ends when control changes hands. It is
refused while a program, batch or policy runs, and a program run is refused while teaching.
"""
from __future__ import annotations

import time
from typing import List, Optional

import numpy as np

from .. import cart_speed
from .. import transforms as tf
from ..geometry import PoseSpec
from .. import replay
from ..program import ProgramWaypoint, RunState, Step, path_label, walk
from ..teach import (JOINT_HZ, MAX_JOINT_SAMPLES, SAMPLE_HZ, Sampler, StillDetector, TaughtPath, TeachError,
                     check_tolerances, suggested_speed)

MOVES = {"joints": "MoveJ", "pose": "MoveJ to pose", "linear": "MoveL"}
MAX_PREVIEW_POINTS = 500
INSERT_KINDS = ("replay", "movel")   # a path goes in as one Replay step (the motion as taught) or as MoveL points

PRESS_LIFT_M = 0.010     # a pressed path starts and ends 10 mm off the surface


class TeachCommands:
    """Needs from WebService: core, program, lease, _edit_lock, _snapshot, _record_edit, _editable, _insert_step,
    _path, _reply, _ok and _err (a CommandError)."""

    def _teach_init(self):
        self.teach: Optional[dict] = None

    def _teach_refusal(self) -> Optional[str]:
        c = self.core
        if c.runner.state != RunState.IDLE or c.batch.active:
            return "a program is running: stop it first"
        if c.policies.busy:
            return "a policy is running: stop it first"
        return None

    def _teach_owner(self) -> str:
        return str(self.lease.uid or self.lease.token or "")

    def _teach_session(self, mode: Optional[str] = None) -> dict:
        s = self.teach
        if s is None:
            raise self._err("not teaching: press Record first")
        if mode and s["mode"] != mode:
            raise self._err(f"this is {s['mode']} teaching")
        return s

    def _taught_name(self, taken=()) -> str:
        k = 1
        while f"taught_{k:03d}" in self.program.waypoints or f"taught_{k:03d}" in taken:
            k += 1
        return f"taught_{k:03d}"

    # ---------------------------------------------------------------- commands
    def cmd_teach_start(self, mode="waypoints", move="linear", frame=None, auto=False, path=None, where="after",
                        pos_tol_mm=1.0, rot_tol_deg=2.0):
        """Begin teaching. waypoints: Mark adds a waypoint + a move step (move: joints | pose | linear), auto=True
        also marks when the arm stood still ~1 s after it moved. path: samples the joints (100 Hz) and the TCP
        (20 Hz) until Finish."""
        if self.teach is not None:
            raise self._err("already teaching: finish or cancel first")
        why = self._teach_refusal()
        if why:
            raise self._err(why)
        self._editable()
        if mode not in ("waypoints", "path"):
            raise self._err("mode is waypoints or path")
        if move not in MOVES:
            raise self._err("move is joints, pose or linear")
        c = self.core
        frame = str(frame or c.profile.base_link)
        c.geo.T_base_frame(frame)                        # an unknown / untaught frame is refused now (GeometryError)
        tcp = c.store.active_tcp
        T0 = c.geo.current_T_base_tcp(tcp)
        try:
            pos_tol_mm, rot_tol_deg = check_tolerances(pos_tol_mm, rot_tol_deg)
        except TeachError as e:
            raise self._err(str(e)) from None
        start = None
        if path not in (None, []):
            start = self._path(path)
        s = {"mode": mode, "move": move if mode == "waypoints" else "linear", "frame": frame, "tcp": tcp,
             "auto": bool(auto) and mode == "waypoints", "owner": self._teach_owner(), "by": self.lease.name,
             "t0": time.monotonic(), "start": start, "where": where if where in ("after", "before", "into") else "after",
             "anchor": None, "marks": 0, "names": [], "samples": [], "phase": "recording", "full": False,
             "error": "", "pos_tol_mm": pos_tol_mm, "rot_tol_deg": rot_tol_deg, "taught": None, "simp": None,
             "sampler": None, "detector": None, "joints": [], "tf_missed": 0, "replay": None}
        if mode == "path":
            every, tick = max(1, int(round(JOINT_HZ / SAMPLE_HZ))), [0]

            def get():                          # the joints every time, the TCP every 5th time (20 Hz)
                n, tick[0] = tick[0], tick[0] + 1
                js, T = self._teach_joint_sample(), None
                if n % every == 0:
                    try:
                        T = np.asarray(c.geo.current_T_base_tcp(tcp), float)
                    except Exception as e:          # noqa: BLE001 - TF late for a moment: that TCP sample is missed
                        s["tf_missed"] += 1
                        s["tf_error"] = str(e)
                if js is None and T is None:
                    raise TeachError("no fresh joint state")
                return js, T

            def on(t, x):
                js, T = x
                if T is not None:
                    s["samples"].append((t, T))
                if js is not None and (not s["joints"] or js[0] > s["joints"][-1][0]):
                    s["joints"].append(js)

            s["sampler"] = Sampler(get, on, hz=JOINT_HZ, max_samples=MAX_JOINT_SAMPLES,
                                   on_full=lambda: s.update(full=True))
        elif s["auto"]:
            det = StillDetector()
            det.reset(T0)
            s["detector"] = det
            s["sampler"] = Sampler(lambda: c.geo.current_T_base_tcp(tcp),
                                   lambda t, T: det.feed(t, T) and self._teach_auto_mark(s), hz=10.0,
                                   max_samples=10 ** 9)
        self.teach = s
        if s["sampler"]:
            s["sampler"].start()
        what = ("path: move the arm, then Finish" if mode == "path" else
                f"waypoints ({MOVES[s['move']]}): move the arm, then Mark" + (" (or stand still 1 s)" if s["auto"] else ""))
        c.bridge.log(f"teach: {self.lease.name or 'operator'} started teaching {what}, {tcp} in {frame}")
        return self._ok(f"teaching {what}", teach=self.teach_view())

    def _teach_joint_sample(self):
        """(time stamp s, joints) of the last joint state, None when there is none or it is stale."""
        b, c = self.core.bridge, self.core
        age = b.joint_state_age() if hasattr(b, "joint_state_age") else None
        if age is None or age > c.profile.joint_state_timeout_s:
            return None
        if hasattr(b, "joint_sample"):
            got = b.joint_sample()
            return None if got is None else (float(got[0]), [float(x) for x in got[1]])
        snap = b.snapshot()
        return (time.monotonic() - age, list(snap.joints)) if snap.joints else None

    def _teach_mark(self, s: dict):
        """One waypoint + move step at the arm's position now. The caller holds the edit lock."""
        c = self.core
        name = self._taught_name()
        if s["move"] == "joints":
            snap = c.last_snap
            if not snap.joints or snap.joint_state_age is None or snap.joint_state_age > c.profile.joint_state_timeout_s:
                raise self._err("no fresh joint state to mark")
            self.program.waypoints[name] = ProgramWaypoint(name, joints=list(snap.joints))
            st = Step("movej", {"target": name, "velocity": 0.1})
        else:
            pose = c.geo.current_pose(s["frame"], s["tcp"])
            self.program.waypoints[name] = ProgramWaypoint(name, pose=pose)
            st = Step("move_pose", {"target": name, "cartesian": s["move"] == "linear", "velocity": 0.1})
        path = self._teach_place(st, s)
        s["anchor"], s["marks"] = st, s["marks"] + 1
        s["names"].append(name)
        if s["detector"] is not None:
            try:
                s["detector"].reset(c.geo.current_T_base_tcp(s["tcp"]))
            except Exception:                            # noqa: BLE001 - the next sample sets it
                pass
        return name, path

    def _teach_place(self, st: Step, s: dict):
        """After the last taught step (wherever it is now), else where the selection was at the start, else the end."""
        anchor = s["anchor"]
        at = next((p for p, x in walk(self.program.steps) if x is anchor), None) if anchor is not None else None
        if at is not None:
            return self._insert_step(st, at, "after")
        start, s["start"] = s["start"], None
        if start is not None:
            try:
                self.program.step_at(start)
                return self._insert_step(st, start, s["where"])
            except Exception:                            # noqa: BLE001 - the program changed meanwhile: at the end
                pass
        return self._insert_step(st, None, "after")

    def cmd_teach_mark(self):
        s = self._teach_session("waypoints")
        self._editable()
        name, path = self._teach_mark(s)
        return self._reply(f"marked {name} ({MOVES[s['move']]})", path)

    def _teach_auto_mark(self, s: dict):
        """From the sampler thread: the arm stood still. Same edit as Mark, with undo and a new revision."""
        if self.teach is not s or self._teach_owner() != s["owner"] or self._teach_refusal():
            return
        try:
            with self._edit_lock:
                if self.teach is not s:
                    return
                before = self._snapshot()
                name, _ = self._teach_mark(s)
                self._record_edit(before)
            self.core.bridge.log(f"teach: marked {name} (the arm stood still)")
        except Exception as e:                           # noqa: BLE001 - shown in the teach panel
            s["error"] = str(e)

    def cmd_teach_finish(self):
        s = self._teach_session()
        if s["sampler"]:
            s["sampler"].stop()
        if s["mode"] == "waypoints":
            self._teach_end(f"taught {s['marks']} waypoint{'s' * (s['marks'] != 1)}")
            return self._ok(f"taught {s['marks']} waypoint{'s' * (s['marks'] != 1)}", teach=None)
        if s["phase"] == "recording":
            try:
                s["taught"] = TaughtPath(list(s["samples"]))
            except TeachError as e:
                self._teach_end("path too short")
                raise self._err(str(e)) from None
            s["phase"] = "review"
            s["simp"] = s["taught"].simplify(s["pos_tol_mm"], s["rot_tol_deg"])
            try:
                s["replay"] = replay.from_recording(list(s["joints"]))
            except replay.ReplayError as e:          # e.g. no joint states: only MoveL points then
                s["replay"] = {"error": str(e)}
        v = self.teach_view()
        r = v["result"]
        return self._ok(f"{r['points']} points from {r['samples']} samples, max {r['max_mm']:.1f} mm / "
                        f"{r['max_deg']:.1f}°", teach=v)

    def cmd_teach_simplify(self, pos_tol_mm=1.0, rot_tol_deg=2.0):
        s = self._teach_session("path")
        if s["phase"] != "review":
            raise self._err("finish the recording first")
        try:
            s["simp"] = s["taught"].simplify(pos_tol_mm, rot_tol_deg)
        except TeachError as e:
            raise self._err(str(e)) from None
        s["pos_tol_mm"], s["rot_tol_deg"] = float(pos_tol_mm), float(rot_tol_deg)
        return self._ok(f"{len(s['simp'].keep)} points", teach=self.teach_view())

    def cmd_teach_insert(self, pos_tol_mm=None, rot_tol_deg=None, speed_mm_s=None, path=None, where="after",
                         press_n=None, press_dir="tool", kind="replay", speed_pct=None):
        """kind "replay" (default): one Replay step, the joint motion as taught (replay.py), speed_pct % of the
        taught timing (100 = as taught).

        kind "movel": the simplified path, a folder with a MoveJ to pose to its start, then MoveL steps
        (optional Max mm/s), waypoints taught_NNN in the teaching frame.

        press_n (N, optional): replay the path pressing with this force in a force mode (Admittance): switch to it,
        MoveJ to 10 mm before the start, move until contact, apply the force, the MoveL steps, force off, back
        off 10 mm, back to the default mode. press_dir: "tool" (tool +Z) or "down" (base -Z)."""
        s = self._teach_session("path")
        if s["phase"] != "review":
            raise self._err("finish the recording first")
        self._editable()
        if kind not in INSERT_KINDS:
            raise self._err("insert the path as replay or movel")
        if kind == "replay":
            return self._teach_insert_replay(s, speed_mm_s, press_n, speed_pct, path, where)
        c = self.core
        if pos_tol_mm is not None or rot_tol_deg is not None:
            try:
                s["simp"] = s["taught"].simplify(pos_tol_mm if pos_tol_mm is not None else s["pos_tol_mm"],
                                                 rot_tol_deg if rot_tol_deg is not None else s["rot_tol_deg"])
            except TeachError as e:
                raise self._err(str(e)) from None
        try:
            mm_s = cart_speed.optional_mm_s(speed_mm_s, c.profile.max_cart_speed_mm_s)
        except ValueError as e:
            raise self._err(str(e)) from None
        press = self._press_plan(press_n, press_dir)
        T_frame_base = tf.inv(c.geo.T_base_frame(s["frame"]))
        names: List[str] = []
        kids = []
        if press:
            T0 = s["taught"].T[s["simp"].keep[0]]
            T_lift = T0.copy()
            if press["dir"] == "tool":                     # back along the tool's +Z
                T_lift[:3, 3] -= PRESS_LIFT_M * T0[:3, 2]
            else:                                           # straight up
                T_lift[2, 3] += PRESS_LIFT_M
            name = self._taught_name(names)
            names.append(name)
            self.program.waypoints[name] = ProgramWaypoint(
                name, pose=PoseSpec.from_T(s["frame"], s["tcp"], T_frame_base @ T_lift))
            axis = [0.0, 0.0, 1.0] if press["dir"] == "tool" else [0.0, 0.0, -1.0]
            frame = "tool" if press["dir"] == "tool" else "base"
            kids += [Step("controller", {"mode": press["mode"]}),
                     Step("move_pose", {"target": name, "cartesian": False, "velocity": 0.1}),
                     Step("move_until_contact", {"direction": axis, "frame": frame,
                                                 "distance_mm": 2 * PRESS_LIFT_M * 1000,
                                                 "force_n": round(min(3.0, press["n"] / 2), 2), "velocity": 0.01}),
                     Step("force", {"wrench": [x * press["n"] for x in axis] + [0.0, 0.0, 0.0], "frame": frame})]
        for n, k in enumerate(s["simp"].keep):
            if press and n == 0:
                continue                                    # the contact move took the place of the start
            name = self._taught_name(names)
            names.append(name)
            pose = PoseSpec.from_T(s["frame"], s["tcp"], T_frame_base @ s["taught"].T[k])
            self.program.waypoints[name] = ProgramWaypoint(name, pose=pose)
            p = {"target": name, "cartesian": n > 0, "velocity": 0.1}
            if n > 0 and mm_s > 0:
                p["speed_mm_s"] = round(mm_s, 1)
            kids.append(Step("move_pose", p))
        if press:
            kids += [Step("force_off", {}),
                     Step("move_relative", {"offset_mm": [-x * PRESS_LIFT_M * 1000 for x in axis], "frame": frame,
                                            "cartesian": True, "velocity": 0.02})]
            if press["back"]:
                kids.append(Step("controller", {"mode": press["back"]}))
        title = f"Taught path {time.strftime('%H:%M')}" + (f", pressing {press['n']:g} N" if press else "")
        folder = Step("folder", {"name": title}, kids)
        at = self._insert_step(folder, self._path(path) if path not in (None, []) else None, where)
        n = len(kids)
        self._teach_end(f"inserted {n} points")
        c.bridge.log(f"teach: path inserted as {n} steps at {path_label(at)} (max {s['simp'].max_mm:.1f} mm / "
                     f"{s['simp'].max_deg:.1f}° off the taught path" + (f", {mm_s:g} mm/s" if mm_s else "") + ")")
        return self._reply(f"inserted the path: {n} points in folder {path_label(at)}", at)

    def _teach_insert_replay(self, s: dict, speed_mm_s, press_n, speed_pct, path, where):
        if press_n not in (None, "", 0):
            raise self._err("pressing a path needs MoveL points (Insert as: MoveL points)")
        if speed_mm_s not in (None, "", 0):
            raise self._err("Max mm/s is for MoveL points: a replay keeps the taught timing (Speed %)")
        rp = s["replay"] or {}
        if "points" not in rp:
            raise self._err(rp.get("error") or "no joint motion recorded")
        try:
            speed = 1.0 if speed_pct in (None, "") else float(speed_pct) / 100.0
            if not replay.SPEED_RANGE[0] <= speed <= replay.SPEED_RANGE[1]:
                raise ValueError
        except (TypeError, ValueError):
            raise self._err(f"speed must be {replay.SPEED_RANGE[0] * 100:g}..{replay.SPEED_RANGE[1] * 100:g} %") from None
        title = f"Taught path {time.strftime('%H:%M')}"
        params = {"name": title, "velocity": 0.1}
        if speed != 1.0:
            params["speed"] = round(speed, 3)
        params["points"] = rp["points"]
        st = Step("replay", params)
        at = self._insert_step(st, self._path(path) if path not in (None, []) else None, where)
        n = len(rp["points"])
        self._teach_end(f"inserted a replay of {n} points")
        self.core.bridge.log(f"teach: path inserted as a replay at {path_label(at)}: {n} points from "
                             f"{rp['samples']} joint samples, {rp['duration_s']:.1f} s")
        return self._reply(f"inserted the path: replay of {n} points, {rp['duration_s']:.1f} s, at step "
                           f"{path_label(at)}", at)

    def _press_plan(self, press_n, press_dir) -> Optional[dict]:
        """None (no force), or the force mode, force, direction and the mode to go back to after."""
        if press_n in (None, "", 0):
            return None
        try:
            n = float(press_n)
        except (TypeError, ValueError):
            raise self._err("press force: a number in N") from None
        prof = self.core.profile
        lim = float((prof.force or {}).get("max_force_n", 20.0))
        if not 0 < n <= lim:
            raise self._err(f"press force must be in (0, {lim:g}] N (the profile limit)")
        if press_dir not in ("tool", "down"):
            raise self._err("press direction is tool or down")
        modes = {k: m for k, m in prof.modes.items() if m.wrench.get("topic") and m.supports_moveit}
        if not modes:
            raise self._err("no control mode of this robot takes a force while moving (e.g. Admittance)")
        cur = self.core.modes.current_mode()
        mode = cur if cur in modes else next(iter(modes))
        back = prof.default_mode if prof.default_mode != mode else None
        return {"n": n, "dir": press_dir, "mode": mode, "back": back}

    def cmd_teach_cancel(self):
        if self.teach is None:
            return self._ok("not teaching")
        self._teach_end("cancelled")
        return self._ok("teaching cancelled", teach=None)

    # -------------------------------------------------------------- internals
    def _teach_end(self, why: str):
        s, self.teach = self.teach, None
        if s and s["sampler"]:
            s["sampler"].stop()
        if s:
            self.core.bridge.log(f"teach: ended ({why})")

    def _teach_tick(self):
        """Control changed hands, or a program started: teaching ends (marks already made stay)."""
        s = self.teach
        if s is None:
            return
        if self._teach_owner() != s["owner"]:
            self._teach_end("control changed hands")
        elif s["phase"] == "recording" and self._teach_refusal():
            self._teach_end(self._teach_refusal())

    def teach_view(self) -> Optional[dict]:
        s = self.teach
        if s is None:
            return None
        sp = s["sampler"]
        v = {"mode": s["mode"], "move": s["move"], "move_label": MOVES[s["move"]], "frame": s["frame"],
             "tcp": s["tcp"], "auto": s["auto"], "by": s["by"], "phase": s["phase"],
             "elapsed_s": round(time.monotonic() - s["t0"], 1), "samples": len(s["samples"]),
             "missed": (sp.missed if sp else 0) + s["tf_missed"], "joint_samples": len(s["joints"]), "marks": s["marks"], "last": s["names"][-1] if s["names"] else "",
             "full": s["full"], "error": s["error"] or (sp.last_error if sp and sp.missed else "") or s.get("tf_error", ""),
             "pos_tol_mm": s["pos_tol_mm"], "rot_tol_deg": s["rot_tol_deg"], "result": None}
        if s["phase"] == "review" and s["simp"] is not None:
            tp, sm = s["taught"], s["simp"]
            st = tp.stats()
            step = max(1, int(np.ceil(len(tp) / MAX_PREVIEW_POINTS)))
            raw = [[round(float(x) * 1000.0, 1) for x in p] for p in tp.P[::step]]
            raw.append([round(float(x) * 1000.0, 1) for x in tp.P[-1]])
            kept = []
            for k in sm.keep:
                xyz, rpy = tf.T_to_mm_deg(tp.T[k])
                kept.append({"xyz_mm": [round(x, 1) for x in xyz], "rpy_deg": [round(x, 1) for x in rpy]})
            v["result"] = {"samples": len(tp), "points": len(sm.keep), "max_mm": round(sm.max_mm, 2),
                           "max_deg": round(sm.max_deg, 2), **st,
                           "speed_mm_s": suggested_speed(st["avg_mm_s"], self.core.profile.max_cart_speed_mm_s),
                           "raw_mm": raw, "kept": kept,
                           "replay": {k: v for k, v in (s["replay"] or {}).items() if k != "points"}
                           | ({"points": len(s["replay"]["points"])} if "points" in (s["replay"] or {}) else {})}
        return v
