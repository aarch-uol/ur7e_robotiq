"""SETUP > F/T + payload > Match simulation: the web commands (a WebService mixin). Logic in ft_match.py.

Nothing here moves the robot. The robot holds still in Position; the person in control pushes the tool tip along
base +X, +Y, +Z (the 3D view shows the arrow). Each push is taken by itself when it is steady, or on Record. After
the third the fit runs; Save keeps the result in the settings in use (the active profile's, see worksets.py).
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np

from .. import ft_match as FM
from .. import transforms as tf
from ..bridge_api import wrench_fresh

FTMATCH_ADMIN = ("ftmatch_save", "ftmatch_clear")
TIP_TCP = "spatula_tip"
DIRS = {"+X": [1.0, 0.0, 0.0], "+Y": [0.0, 1.0, 0.0], "+Z": [0.0, 0.0, 1.0]}


class FtMatchCommands:
    def _ftmatch_init(self):
        self.ftm_clock = time.monotonic
        self.ftm = self._ftm_empty()

    @staticmethod
    def _ftm_empty() -> dict:
        return {"active": False, "policy": "", "ref": None, "axis": None, "caps": {}, "fit": None,
                "no_contact": None, "error": ""}

    # ------------------------------------------------------------------ helpers
    def _ftm_refs(self) -> list:
        """Every policy with its sim reference: [{"policy", "path", "found"}]."""
        out = []
        for name, m in sorted(self.core.policies.manifests.items()):
            p = FM.reference_path(m)
            out.append({"policy": name, "path": str(p) if p else "", "found": bool(p and p.is_file())})
        return out

    def _ftm_refusal(self) -> Optional[str]:
        c = self.core
        if c.mode != "position":
            return f"change to Position first (now '{c.mode}'): the robot must hold still"
        s = c.last_snap
        if s.freedrive:
            return "freedrive is on: the robot must hold still"
        if not c.idle():
            return "the robot is busy: wait until nothing runs"
        qd = s.joint_velocities
        if qd is not None and max((abs(v) for v in qd), default=0.0) > c.profile.stationary_joint_velocity:
            return "the robot is moving"
        if wrench_fresh(s) is None:
            return "no live F/T reading"
        return None

    def _ftm_tip(self) -> str:
        st = self.core.store
        if TIP_TCP in st.tcps:
            return TIP_TCP
        m = self.core.policies.manifests.get(self.ftm.get("policy") or "")
        if m is not None and m.required_tcp and m.required_tcp in st.tcps:
            return m.required_tcp
        return st.active_tcp

    def _ftm_tip_pose(self) -> Optional[dict]:
        name = self._ftm_tip()
        try:
            xyz, rpy = tf.T_to_mm_deg(self.core.geo.current_T_base_tcp(name))
        except Exception:
            return None
        return {"name": name, "xyz_mm": [round(v, 1) for v in xyz], "rpy_deg": [round(v, 2) for v in rpy]}

    def _ftm_next_axis(self) -> Optional[str]:
        return next((ax for ax in FM.AXES if not (self.ftm["caps"].get(ax) and
                                                  self.ftm["caps"][ax].state == "done")), None)

    def _ftm_release_from(self):
        """The newest no-contact baseline: a new push waits until the reading is back near it (hands off)."""
        nc = self.ftm.get("no_contact")
        return nc["published"] if nc else None

    def _ftm_arm(self, axis: Optional[str]):
        self.ftm["axis"] = axis
        if axis:
            self.ftm["caps"][axis] = FM.PushCapture(release_from=self._ftm_release_from())

    def _ftm_fit(self):
        f = self.ftm
        deltas = {ax: (c.delta.tolist() if c.delta is not None else None) for ax, c in f["caps"].items()}
        m = self.core.policies.manifests.get(f["policy"])
        obs = FM.read_obs(m.expanded_files().get("robot_config", "")) if m else {}
        old_axes, old_scale = obs.get("ft_obs_axes") or np.eye(3).tolist(), obs.get("ft_obs_scale")
        fit = FM.fit(deltas, f["ref"]["pushes"], old_scale=old_scale)
        if fit["M"] is not None:            # before / after for each push, what the policy would see
            fit["rows"] = {ax: {"sensor": [round(float(x), 2) for x in deltas[ax]],
                                "before": FM.apply_axes(old_axes, deltas[ax], old_scale),
                                "after": FM.apply_axes(fit["M"], deltas[ax], fit["scale"]),
                                "sim": [round(float(x), 2) for x in f["ref"]["pushes"][ax]]} for ax in FM.AXES}
            fit.update(explain=FM.explain(fit["M"], fit["scale"]), old_scale=old_scale, suggested_tip=self._ftm_suggest(fit["M"]))
        f["fit"] = fit

    # ------------------------------------------------------------------ tick (10 Hz)
    def _ftmatch_tick(self):
        f = self.ftm
        ax = f["axis"]
        if not f["active"] or not ax:
            return
        cap = f["caps"].get(ax)
        why = self._ftm_refusal()
        if why:
            f["caps"][ax] = FM.PushCapture(release_from=self._ftm_release_from())   # again once still
            f["error"] = why
            return
        f["error"] = ""
        s = self.core.last_snap
        cap.feed(self.ftm_clock(), wrench_fresh(s))
        if cap.state == "push" and f["no_contact"] is None:
            self._ftm_no_contact(cap.baseline)
        if cap.state == "done":
            self.core.bridge.log(f"F/T match: {ax} push recorded, {np.linalg.norm(cap.delta[:3]):.1f} N")
            self._ftm_after_push()

    def _ftm_after_push(self):
        nxt = self._ftm_next_axis()
        self._ftm_arm(nxt)
        if nxt is None:
            self._ftm_fit()

    def _ftm_no_contact(self, published):
        """The no-contact reading at this pose (raw sensor = published + bias), kept with the result."""
        b = self.core.ft_bias
        bias = list(b.live) if getattr(b, "live", None) is not None else [0.0] * 6
        pub = [round(float(v), 4) for v in published]
        tip = self._ftm_tip_pose()
        self.ftm["no_contact"] = {
            "raw": [round(p + q, 4) for p, q in zip(pub, bias)], "published": pub,
            "bias": [round(float(v), 4) for v in bias],
            "joints_deg": [round(float(np.degrees(q)), 3) for q in (self.core.last_snap.joints or [])],
            "tip": tip, "at": time.strftime("%Y-%m-%d %H:%M:%S")}

    # ------------------------------------------------------------------ commands
    def cmd_ftmatch_start(self, policy=""):
        refs = self._ftm_refs()
        pick = next((r for r in refs if r["policy"] == policy), None) if policy else \
            next((r for r in refs if r["found"]), refs[0] if refs else None)
        if pick is None:
            raise FM.FtMatchError("no policy: the sim reference comes from a policy's manifest (sim_ft_reference, "
                                  "or sim_ft_reference.json next to its robot_config)")
        if not pick["path"]:
            raise FM.FtMatchError(f"policy {pick['policy']} has no robot_config and no sim_ft_reference: there is "
                                  "nothing to match to")
        ref = FM.load_reference(pick["path"])            # FtMatchError: missing or bad, said in the message
        why = self._ftm_refusal()
        if why:
            raise FM.FtMatchError(why)
        self.ftm = {**self._ftm_empty(), "active": True, "policy": pick["policy"], "ref": ref}
        self._ftm_arm(FM.AXES[0])
        self.core.bridge.log(f"F/T match started (sim reference {ref['path']})")
        return self._ok("Hands off the tool, then push along the arrow", ftmatch=self.ftmatch_view())

    def cmd_ftmatch_push(self, axis):
        """Do this push again (or now): a fresh baseline first."""
        if not self.ftm["active"]:
            raise FM.FtMatchError("start the match first")
        if axis not in FM.AXES:
            raise FM.FtMatchError(f"no push {axis}")
        self.ftm["fit"] = None
        self._ftm_arm(axis)
        return self._ok(f"{axis}: hands off the tool, then push", ftmatch=self.ftmatch_view())

    def cmd_ftmatch_record(self):
        f = self.ftm
        ax = f["axis"]
        if not f["active"] or not ax:
            raise FM.FtMatchError("no push is waiting")
        why = self._ftm_refusal()
        if why:
            raise FM.FtMatchError(why)
        cap = f["caps"][ax]
        if not cap.record(self.ftm_clock()):
            raise FM.FtMatchError("no baseline yet: keep hands off the tool for a moment, then push")
        n = float(np.linalg.norm(cap.delta[:3]))
        self.core.bridge.log(f"F/T match: {ax} push recorded by hand, {n:.1f} N")
        if f["no_contact"] is None:
            self._ftm_no_contact(cap.baseline)
        self._ftm_after_push()
        return self._ok(f"{ax} recorded: {n:.1f} N", ftmatch=self.ftmatch_view())

    def cmd_ftmatch_cancel(self):
        self.ftm = self._ftm_empty()
        return self._ok("match stopped")

    def cmd_ftmatch_save(self):
        f = self.ftm
        fit = f["fit"]
        if not f["active"] or not fit or not fit["ok"]:
            raise FM.FtMatchError("nothing to save: the three pushes must match first")
        owner = self.worksets.active()
        res = {"ft_obs_axes": fit["M"], "ft_obs_scale": [float(x) for x in fit["scale"]],
               "explain": FM.explain(fit["M"], fit["scale"]), "torque": fit["torque"],
               "at": time.strftime("%Y-%m-%d %H:%M"), "profile": owner, "policy": f["policy"],
               "sim_reference": f["ref"]["path"], "rotation_deg": fit["rotation_deg"], "pushes": fit["pushes"],
               "deltas": {ax: f["caps"][ax].view()["delta"] for ax in FM.AXES},
               "no_contact": f["no_contact"], "suggested_tip": self._ftm_suggest(fit["M"])}
        self.core.ft_match.save(res)
        self.core.bridge.log(f"F/T match saved{' for ' + owner if owner else ''}: ft_obs_axes {fit['M']} "
                             f"({'; '.join(res['explain'])})")
        self.ftm = self._ftm_empty()
        return self._ok("Saved: policy runs use it from now on", ftmatch=self.ftmatch_view())

    def cmd_ftmatch_clear(self):
        if not self.core.ft_match.clear():
            raise FM.FtMatchError("no saved result")
        self.core.bridge.log("F/T match result removed: policy runs use the robot config as it is")
        return self._ok("Result removed", ftmatch=self.ftmatch_view())

    def cmd_ftmatch_get(self):
        return self._ok(ftmatch=self.ftmatch_view())
    cmd_ftmatch_get.read_only = True

    # ------------------------------------------------------------------ view
    def _ftm_suggest(self, M) -> dict:
        tip = self._ftm_tip()
        sensor = self.core.profile.ft_calibration.get("sensor_frame") or ""
        t = self.core.store.tcps.get(tip)
        parent = getattr(t, "parent", "") if t is not None and not getattr(t, "builtin", True) else ""
        R = None
        if parent and sensor:
            try:
                T = self.core.bridge.lookup_transform(parent, sensor)
                R = None if T is None else np.asarray(T)[:3, :3]
            except Exception:
                R = None
        out = FM.suggest_tip(M, R)
        out.update(tip=tip, sensor_frame=sensor, parent=parent if R is not None else "")
        if t is not None and not getattr(t, "builtin", True) and t.T_parent_tcp is not None:
            out["rpy_deg_now"] = [round(v, 1) for v in tf.T_to_mm_deg(t.T_parent_tcp)[1]]
        return out

    def ftmatch_view(self) -> dict:
        f = self.ftm
        saved = self.core.ft_match.load()
        out = {"active": f["active"], "policy": f["policy"], "axis": f["axis"], "error": f["error"],
               "refs": self._ftm_refs(), "saved": saved, "owner": self.worksets.active(),
               "refusal": self._ftm_refusal(), "dirs": DIRS, "min_n": FM.MIN_PUSH_N, "max_deg": FM.MAX_ANGLE_DEG}
        if not f["active"]:
            return out
        out["ref_path"] = f["ref"]["path"]
        out["tip"] = self._ftm_tip_pose()
        out["pushes"] = {ax: (f["caps"][ax].view() if ax in f["caps"] else None) for ax in FM.AXES}
        out["no_contact"] = f["no_contact"]
        out["fit"] = f["fit"]
        return out
