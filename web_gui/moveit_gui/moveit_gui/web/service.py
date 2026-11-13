"""The web front end's logic, independent of HTTP: state, commands, the control lease. Testable without a server.

Safety rules for a robot driven over a network:
  * One browser holds the CONTROL LEASE; every command that can move or reconfigure the robot needs it. Other
    browsers only watch. STOP is accepted from anyone, without a lease.
  * The lease holder sends a heartbeat. If it stops for LEASE_TIMEOUT_S while the robot is doing anything
    (motion, program, policy, freedrive, payload run), everything is stopped and the lease is dropped.
    If the robot is idle, the holder is only "away" (a tab in the background, a short network gap): it keeps control
    for IDLE_LEASE_S, and its next heartbeat or command brings it back. An away holder never stops the robot (someone
    may use the teach pendant meanwhile); anyone may take over from it, as always while idle.
  * Taking the lease from another browser is only possible while the robot is idle.
  * Motion rules (busy, UNKNOWN control state, MoveIt-capable mode) are RobotCore's, not this file's.

Long actions run as jobs in a worker thread; their outcome comes back as a "result" event.
"""
from __future__ import annotations

import contextlib
import base64
import itertools
import json
import math
import os
import secrets
import shutil
import threading
import tempfile
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Callable, Dict, List, Optional

import numpy as np
import yaml

from .. import bases as B
from .. import robots as R
from .. import cart_speed
from .. import transforms as tf
from ..bridge_api import wrench_fresh
from ..calibration import CalibrationError, FixtureDef, TcpDef, config_dir, pivot_quality, solve_pivot
from ..core import RobotCore
from ..ft_payload import PayloadCalibration, PayloadCalibrationError
from ..force import ForceError, WrenchTarget
from ..ft_bias import FtBiasError
from ..gains import GainsError
from ..geometry import ACTIVE_TCP, GeometryError, PoseSpec
from ..geometry_calibration import VialGeometryError, estimate_vial
from ..jog import JogCommand, JogError, JogVelocity
from ..keepout import make_box
from ..policies import (POLICY_UPLOAD_SUFFIXES, PolicyImportError, RecordingOptions, allowed_policy_roots,
                        list_policy_checkpoints,
                        policy_range_warnings, policy_path_allowed, split_range_notes)
from ..policy_action import RunPolicyResult
from ..activity import KINDS as ACTIVITY_KINDS, ActivityLog, summarize_args
from .users import Users, public_id
from .roles import Roles, RolesError, at_least
from ..program import CONTAINER_STEPS, STEP_TYPES, Program, ProgramError, ProgramWaypoint, RunState, Step, path_label, walk
from ..program_path import fk, simulate
from ..robot_model import model_or_error
from ..vials import VialLibrary, VialProfileError
from ..handeye import HandEyeError
from ..parts import PartError, merged_model as parts_merged
from ..setups import SetupError
from ..user_profiles import ProfileError
from ..services import ServiceError
from .workcell import WorkcellCommands
from .teaching import TeachCommands
from .batching import BatchCommands
from ..batch import BatchError, sweep_options
from ..teach import TeachError
from .flask_cmds import FLASK_ADMIN, FlaskCommands
from .ftmatch_cmds import FTMATCH_ADMIN, FtMatchCommands
from ..ft_match import FtMatchError
from ..flask_geometry import FlaskGeometryError
from ..flask_scrape import FlaskScrapeError
from ..flask_setup import FlaskSetupError
from ..scrape_pattern import ScrapeBaseline, ScrapePattern, ScrapePatternError, build, build_baseline

LEASE_TIMEOUT_S = 2.5
IDLE_LEASE_S = 120.0       # an away holder (robot idle) keeps control this long
LOG_EVERY_S = 30.0          # rate limit for repeated watchdog / tick errors
VIAL_POINTS = ("Rim 1", "Rim 2", "Rim 3", "Bottom")
KEEPOUT_TCP_CLEARANCE_MM = 5.0      # "Under TCP": the box top stays this far below the touching TCP
# commands that change the setup rather than run the robot: Admin only (roles.py)
ADMIN_COMMANDS = frozenset((
    "debug_profile",
    "tcp_save", "tcp_delete", "pivot_reset", "pivot_capture", "pivot_remove", "pivot_solve", "pivot_save",
    "gains_apply", "gains_apply_policy", "gains_forget", "gains_restore", "gains_preset_save", "gains_preset_delete",
    "ft_bias_set", "ft_bias_clear", "ft_bias_preset_save", "ft_bias_preset_delete",
    "keepout_save", "keepout_enable", "keepout_delete", "keepout_apply",
    "vial_capture", "vial_reset", "vial_save", "vial_use_example", "vial_profile_save", "vial_profile_delete",
    "payload_collect", "payload_evaluate", "payload_reset", "payload_save", "payload_apply", "record_configure",
    "drivers_save", "drivers_delete", "drivers_autostart", "parts_save", "parts_delete", "parts_mesh_upload",
    "parts_make_tcp", "base_switch", "robot_setup", "setup_save", "setup_delete", "setup_load", "setup_copy",
    "profile_delete",
    "policy_import", "policy_rename", "policy_sources", "policy_upload",
    "handeye_board", "handeye_start", "handeye_capture", "handeye_drop", "handeye_plan", "handeye_collect",
    "handeye_solve", "handeye_save", "handeye_apply", "handeye_delete", "roles_save", "ft_guard_save", *FLASK_ADMIN,
    *FTMATCH_ADMIN))
# commands allowed while the one in control does not have their own settings in use (worksets.py)
SWITCH_FREE = frozenset(("profile_activate", "stack_stop"))
# commands that leave Servo ready for the Jog tab; every other command with the lease gives the arm back first
JOG_KEEPS_READY = frozenset(("jog_arm", "jog_start", "jog_hold", "jog_stop", "hold_renew", "profile_prefs_save",
                             "gripper", "speed_override"))
# commands the activity log (activity.py) records, with their kind; every ADMIN_COMMAND is "setup" (roles_save "roles")
ACTIVITY_COMMANDS = {
    **{c: "program" for c in ("program_run", "program_pause", "program_resume", "program_stop", "program_scrape",
                              "program_scrape_baseline", "program_flask_scrape", "program_save", "program_open", "program_new",
                              "program_copy", "program_delete", "robot_play", "robot_pause", "robot_stop_program")},
    **{c: "policy" for c in ("policy_prepare", "policy_run", "policy_start", "policy_stop")},
    **{c: "record" for c in ("record_start", "record_stop")},
    **{c: "mode" for c in ("switch_mode", "freedrive", "recover")},
    **{c: "power" for c in ("power_on", "power_off", "brake_release")},
    **{c: "batch" for c in ("batch_run", "batch_stop", "batch_save")},
    **{c: "teach" for c in ("teach_start", "teach_insert", "teach_cancel")},
    "profile_activate": "setup",
}



# changes on every GUI start: an open page compares it to the one it loaded with and offers a reload
BOOT_ID = f"{time.time():.3f}"

class CommandError(Exception):
    pass


def _floats(n: int):
    def conv(v):
        out = [float(x) for x in v]
        if len(out) != n:
            raise ValueError(f"{n} numbers")
        return out
    return conv


def _choice(*allowed):
    def conv(v):
        if v not in allowed:
            raise ValueError(f"one of {', '.join(allowed)}")
        return v
    return conv


class Lease:
    """Who controls the robot. `token` is the secret the holder's page sends with every request (returned only by
    take_lease, never broadcast); `id` is public and only lets the holder's page recognise itself in the state."""

    def __init__(self):
        self.generation = 0
        self.token: Optional[str] = None
        self.id: Optional[str] = None
        self.name = ""
        self.seen = 0.0
        self.uid: Optional[str] = None               # the holder's public user id (users.py), if its page sent one
        self.key = ""                                # the holder's roles.py key (Tailscale login or IP): public
        self.role = ""
        self.away = False                            # went quiet while the robot was idle (see IDLE_LEASE_S)
        self.profile = ""                            # the holder's profile (user_profiles.py), "" if the name is none

    @property
    def client(self) -> Optional[str]:               # truthy while somebody holds the lease
        return self.token

    def holds(self, token: Optional[str]) -> bool:
        return bool(token) and bool(self.token) and secrets.compare_digest(str(token), self.token)

    def view(self) -> dict:
        return {"client": self.id, "name": self.name, "uid": self.uid, "away": self.away, "profile": self.profile}


def remote_power() -> bool:
    """Power on / brake release from any browser with control (default), or only from the robot PC
    (MOVEIT_GUI_LOCAL_POWER_ONLY=1)."""
    return os.environ.get("MOVEIT_GUI_LOCAL_POWER_ONLY", "").strip().lower() not in ("1", "true", "yes", "on")


class WebService(WorkcellCommands, TeachCommands, BatchCommands, FlaskCommands, FtMatchCommands):
    def __init__(self, core: RobotCore, clock: Callable[[], float] = time.monotonic):
        self.core = core
        self.clock = clock
        self._lease_generation = 0
        self.lease = self._new_lease()
        core.operator = lambda: self.lease.name or ""
        self.activity = ActivityLog(Path(os.environ["MOVEIT_GUI_ACTIVITY"]) if os.environ.get("MOVEIT_GUI_ACTIVITY") else None)
        self.activity.on_error = core.bridge.log
        core.log_mark = lambda: self._log_last
        core.log_since = self._log_since
        self._lease_lock = threading.Lock()               # lease state only, held for microseconds: heartbeats never wait
        self._command_context = threading.local()         # caller token while a command handler submits a job
        self.users = Users(clock)                         # who has the page open (one per browser)
        self.roles = Roles(Path(os.environ.get("MOVEIT_GUI_ROLES") or config_dir() / "roles.yaml"))
        self.asking: Optional[dict] = None                # someone asks the holder for control: {uid, name, at, kept}
        self.offer: Optional[dict] = None                 # the holder gave control to {uid, name, until}
        self._lock = threading.RLock()
        self._edit_lock = threading.RLock()               # program edits + undo/redo: snapshot, handler, compare
        self._last_err: Dict[str, float] = {}
        self._jobs = ThreadPoolExecutor(max_workers=4, thread_name_prefix="web-job")
        self._job_ids = itertools.count(1)
        self.results: deque = deque(maxlen=50)            # {id, cmd, ok, msg, t}
        self.logs: deque = deque(maxlen=2000)             # (seq, line)
        self._log_seq = itertools.count(1)
        self._log_last = 0
        self._log_lock = threading.Lock()
        self.program = Program(name="untitled", robot=core.profile.name)
        self.program_path: Optional[Path] = None
        self._undo: List[tuple] = []                      # program snapshots (yaml, path) before each edit
        self._redo: List[tuple] = []
        self._saved_yaml = self._program_yaml()           # what is on disk (or the empty new program)
        self.program_rev = 0                              # +1 on every change: browsers refetch, stale edits refused
        self._hold: Optional[dict] = None                 # hold-to-run motion: {"seen", "label"}
        self.prepared_policy: Optional[str] = None
        self._policy_step_cursor = time.time() - 0.2
        self._policy_last_step = None
        self._policy_was_busy = False
        self._policy_chart_policy = None
        self.vial_points: Dict[str, List[float]] = {}
        self._pivot_parent = ""
        self._pivot_samples: List[np.ndarray] = []
        self._pivot_result = None
        self.vials = VialLibrary(config_dir() / "vials" / f"{core.profile.name}.yaml", core.profile.example_fixtures,
                                 core.profile.base_link)
        self._plan_id = 0
        self._model_xml: Optional[str] = None
        self._model: dict = model_or_error(None)
        self.payload: Optional[PayloadCalibration] = None
        self.payload_progress = {"running": False, "done": 0, "total": 0, "error": ""}
        self.payload_applied = False
        try:
            if core.profile.ft_calibration.get("poses"):
                self.payload = PayloadCalibration(core.bridge, core.profile)
                self.payload.on_applied = core.gains.remember
        except PayloadCalibrationError as e:
            self.payload_progress["error"] = str(e)
        self._workcell_init()
        self._teach_init()
        self._flask_init()
        self._ftmatch_init()

    def _new_lease(self) -> Lease:
        self._lease_generation += 1
        lease = Lease()
        lease.generation = self._lease_generation
        return lease

    # ================================================================= polling
    def _log_error(self, what: str, e: BaseException):
        """Log a watchdog / tick failure, at most once per LOG_EVERY_S per kind (tick runs at 10 Hz)."""
        now = time.monotonic()
        if now - self._last_err.get(what, -1e9) >= LOG_EVERY_S:
            self._last_err[what] = now
            self.core.bridge.log(f"web: {what} failed: {type(e).__name__}: {e}")

    def tick(self):
        """Call at ~10 Hz from one thread: enforce the watchdogs, refresh the core, collect logs.
        The watchdogs come first and each has its own try: a failing core.update() must never disable them."""
        for name, fn in (("hold-to-run watchdog", self._check_hold), ("lease watchdog", self._check_lease)):
            try:
                fn()
            except Exception as e:
                self._log_error(name, e)
        try:
            self.core.update()
        except Exception as e:
            self._log_error("core.update", e)
        try:
            self._parts_tick()
        except Exception as e:
            self._log_error("parts", e)
        try:
            self.drivers.tick()
        except Exception as e:
            self._log_error("drivers", e)
        try:
            self._teach_tick()
        except Exception as e:
            self._log_error("teach", e)
        try:
            self._ftmatch_tick()
        except Exception as e:
            self._log_error("F/T match", e)
        self._drain_logs()

    def _drain_logs(self):
        with self._log_lock:
            for line in self.core.bridge.drain_logs():
                self._log_last = next(self._log_seq)
                self.logs.append((self._log_last, line))

    def _log_since(self, mark: int) -> List[str]:
        """The log lines after `mark` (a run's own lines), including what the bridge has not handed over yet."""
        self._drain_logs()
        return [line for seq, line in list(self.logs) if seq > mark]

    # ------------------------------------------------------------ activity log
    def _actor(self) -> dict:
        """The lease holder as the activity log names them (public things only: no token, no browser id)."""
        return self._actor_of(self.lease)

    @staticmethod
    def _actor_of(L: "Lease") -> dict:
        return {"name": (L.name or "").split(" @ ")[0], "key": L.key, "uid": L.uid or "", "role": L.role}

    def _act(self, kind: str, what: str, who: Optional[dict] = None, details: str = ""):
        try:
            self.activity.add(kind, what, who if who is not None else self._actor(), details)
        except Exception as e:                            # never let the log break an action
            self._log_error("activity log", e)

    def _check_hold(self):
        with self._lock:
            h = self._hold
            expired = h is not None and self.clock() - h["seen"] > HOLD_TIMEOUT_S
            if expired:
                self._hold = None
        if expired:                                       # act outside the lock
            self.core.stop_all()
            self.core.bridge.log(f"web: {h['label']} stopped: hold-to-run button released")

    def _check_lease(self):
        with self._lease_lock:
            lease = self.lease
            silent = self.clock() - lease.seen
            if not (lease.token and silent > LEASE_TIMEOUT_S):
                return
            if lease.away:
                if silent <= IDLE_LEASE_S:
                    return                                # away: never stops anything
                self.lease = self._new_lease()
        who = lease.name or lease.id
        if lease.away:
            self.core.bridge.log(f"web: {who} was away for {silent:.0f} s (limit {IDLE_LEASE_S:.0f} s): control released")
            self._act("control", "control released after being away", self._actor_of(lease),
                      f"away for {silent:.0f} s")
            return
        try:
            active = not self.core.idle() or self.core.last_snap.freedrive or self.payload_progress["running"]
        except Exception:
            active = True                                 # cannot tell: be safe
        if not active:
            with self._lease_lock:
                if self.lease is lease and self.clock() - lease.seen > LEASE_TIMEOUT_S:
                    lease.away = True
            self.core.bridge.log(f"web: {who} went quiet (no heartbeat for {silent:.1f} s) with the robot idle: "
                                 f"keeps control for {IDLE_LEASE_S:.0f} s unless someone takes over")
            self._act("control", "went away (robot idle, keeps control)", self._actor_of(lease))
            return
        with self._lease_lock:
            if self.lease is not lease or self.clock() - lease.seen <= LEASE_TIMEOUT_S:
                return                                    # taken over, or a heartbeat came meanwhile
            self.lease = self._new_lease()
        self.core.bridge.log(f"web: lost contact with {who} (no heartbeat for {silent:.1f} s, limit "
                             f"{LEASE_TIMEOUT_S:.1f} s): STOPPING")
        self.core.note_stop(f"lost contact with {lease.name.split(' @ ')[0] or lease.id}")
        self._act("control", "lost contact: control lost, robot stopped", self._actor_of(lease),
                  f"no heartbeat for {silent:.1f} s")
        for what, fn in (("stop_all", self.core.stop_all),
                         ("payload cancel", lambda: self.payload and self.payload.cancel()),
                         ("freedrive off", lambda: self.core.last_snap.freedrive and
                          self._submit("freedrive", lambda: self.core.freedrive(False)))):
            try:
                fn()
            except Exception as e:
                self._log_error(f"lease stop ({what})", e)

    # =================================================================== lease
    def take_lease(self, client: str = "", name: str = "", presented: str = "", bid: str = "",
                   role: str = "admin", key: str = "") -> dict:
        """Returns {client: public id, name, uid, token: secret}. The token is shown only here; keep it out of logs.
        client: the token to install (in-process callers and tests); empty = a random one, which is what the HTTP
        layer always uses. presented: the token the caller already holds, so the holder may take again.
        bid: the browser's own id (users.py), so the users list can say who has control."""
        if not at_least(role, "operator"):
            raise CommandError("you are a Viewer: an Admin can make you an Operator (SETUP › Roles)")
        uid = public_id(bid) if Users.valid(bid) else None
        with self._lease_lock:
            cur = self.lease
            took_from = ""
            again = cur.holds(client) or cur.holds(presented)
            off = self.offer if self.offer and self.clock() < self.offer["until"] else None
            if off and not again and off["uid"] != uid:
                raise CommandError(f"control is being handed to {off['name']} (for a few seconds)")
            if cur.token and not again:
                why = self.verified_standstill()
                if why:
                    raise CommandError(f"{cur.name or 'another browser'} is controlling a moving robot: {why}")
                self.core.bridge.log(f"web: {name or 'a browser'} took over control from {cur.name or cur.id}")
                took_from = cur.name.split(" @ ")[0] or "another browser"
            lease = self._new_lease()
            lease.token, lease.id = client or secrets.token_urlsafe(24), secrets.token_urlsafe(6)
            lease.name, lease.seen, lease.uid = name or "browser", self.clock(), uid
            lease.key, lease.role = key, role
            lease.profile = self._profile_of(lease.name)
            self.lease = lease
            self.offer = None
            if self.asking and (self.asking["uid"] == uid or not again):
                self.asking = None                        # answered by taking it (or a new holder: the question is gone)
            out = {**lease.view(), "token": lease.token}
        self._claim_settings(lease)
        owner = self.worksets.active()
        out["needs_switch"] = self._needs_switch(owner)
        out["settings_owner"] = owner
        if not again:                                     # a holder taking again (page reload) is no news
            gift = bool(off and off["uid"] == uid)
            self._act("control", "took over control" if took_from else "took control", self._actor_of(lease),
                      f"from {took_from}" if took_from else ("given to them" if gift else ""))
        return out

    # ------------------------------------------------------- asking for control
    ASK_S = 60.0      # a question for control is open this long
    KEPT_S = 8.0      # "kept" is shown to the one who asked this long
    OFFER_S = 15.0    # control given to someone waits this long for them to take it

    def ask_control(self, bid: str, name: str = "", role: str = "admin", key: str = "") -> dict:
        """A browser without control asks the holder for it (the holder's page shows a pop-up: Give / Keep)."""
        if not at_least(role, "operator"):
            raise CommandError("you are a Viewer: an Admin can make you an Operator (SETUP › Roles)")
        if not Users.valid(bid):
            raise CommandError("this page has no browser id: reload it")
        uid = public_id(bid)
        with self._lease_lock:
            if not self.lease.token:
                raise CommandError("nobody has control: take it")
            if self.lease.uid == uid:
                raise CommandError("you already have control")
            a, now = self.asking, self.clock()
            if a and a["uid"] == uid and not a["kept"] and now - a["at"] < self.ASK_S:
                return {"ok": True, "msg": "already asked: waiting for an answer"}
            if a and a["uid"] == uid and a["kept"] and now - a["at"] < 20.0:
                raise CommandError(f"{self.lease.name.split(' @ ')[0]} keeps control: ask again in {20 - (now - a['at']):.0f} s")
            if a and a["uid"] != uid and not a["kept"] and now - a["at"] < self.ASK_S:
                raise CommandError(f"{a['name']} is asking already: wait for the answer")
            self.asking = {"uid": uid, "name": name or self.users.label(bid), "at": self.clock(), "kept": False}
        self.core.bridge.log(f"web: {self.asking['name']} asks {self.lease.name} for control")
        self._act("control", "asked for control", {"name": self.asking["name"], "uid": uid, "key": key, "role": role},
                  f"from {self.lease.name.split(' @ ')[0]}")
        return {"ok": True, "msg": f"asked {self.lease.name.split(' @ ')[0]} for control"}

    def cmd_control_answer(self, give=False):
        """The holder answers a question for control. Give: control goes to the one who asked (robot idle only)."""
        with self._lease_lock:
            a = self.asking
            if not a or a["kept"] or self.clock() - a["at"] > self.ASK_S:
                raise CommandError("nobody is asking for control now")
            who = self._actor()
            if not give:
                a["kept"], a["at"] = True, self.clock()
                holder = self.lease.name
            else:
                why = self.verified_standstill()
                if why:
                    raise CommandError(f"the robot is moving or busy: {why}; stop it first, then give control")
                holder, self.lease = self.lease.name, self._new_lease()
                self.offer = {"uid": a["uid"], "name": a["name"], "until": self.clock() + self.OFFER_S}
                self.asking = None
        self.core.bridge.log(f"web: {holder} {'gave control to' if give else 'keeps control; asked by'} {a['name']}")
        self._act("control", "gave control" if give else "kept control", who, details=(f"to {a['name']}" if give else f"asked by {a['name']}"))
        return self._ok(f"control given to {a['name']}" if give else "you keep control")

    def _lease_state(self) -> dict:
        now = self.clock()
        a, o = self.asking, self.offer
        if a and now - a["at"] > (self.KEPT_S if a["kept"] else self.ASK_S):
            a = self.asking = None
        if o and now >= o["until"]:
            o = self.offer = None
        if a and not a["kept"] and not self.lease.token and not o:
            a = self.asking = None                        # the holder left: just take control
        owner = self.worksets.active()
        return {**self.lease.view(), "needs_switch": self._needs_switch(owner), "settings_owner": owner,
                "ask": {"id": a["uid"], "name": a["name"], "kept": a["kept"], "age_s": round(now - a["at"], 1)} if a else None,
                "offer": {"id": o["uid"], "name": o["name"]} if o else None}

    def heartbeat(self, token: str) -> bool:
        with self._lease_lock:
            if self.lease.holds(token):
                self.lease.seen = self.clock()
                self.lease.away = False
                return True
            return False

    def verified_standstill(self) -> Optional[str]:
        """Return why the robot is not proven still, or None when the check is fail-closed and clear."""
        if (not self.core.idle() or self.core.jogging or self.core.force_active or
                getattr(self, "payload_progress", {}).get("running") or
                getattr(self, "he", {}).get("progress", {}).get("running", False)):
            return "the robot is busy (motion, program, policy or force mode is running)"
        snap = self.core.bridge.snapshot()
        age = snap.joint_state_age
        if age is None or age > self.core.profile.joint_state_timeout_s:
            return "joint states are not fresh: cannot verify the robot is still"
        qd = snap.joint_velocities
        if qd is None:
            return "joint speeds are unavailable: cannot verify the robot is still"
        if any(abs(float(v)) > self.core.profile.stationary_joint_velocity for v in qd):
            return "the robot is moving"
        if snap.program_state == "PAUSED" or (snap.program_state == "PLAYING" and snap.program_running is not True):
            return "a pendant program is running"
        return None

    def release(self, token: str, confirm: bool = False):
        with self._lease_lock:
            if not self.lease.holds(token):
                return
            if not confirm:
                why = self.verified_standstill()
                if why:
                    raise CommandError(f"{why}: release anyway? (the robot keeps running)")
            who, self.lease = self._actor(), self._new_lease()
        self._act("control", "released control while busy" if confirm else "released control", who)

    # ------------------------------------------------------------------ roles
    def cmd_roles_get(self):
        return {"ok": True, **self.roles.view()}
    cmd_roles_get.read_only = True

    def cmd_roles_save(self, default="admin", people=None):
        try:
            self.roles.save(str(default), dict(people or {}))
        except RolesError as e:
            raise CommandError(str(e)) from None
        self.core.bridge.log(f"web: roles saved (default {default}, {len(people or {})} people)")
        self._act("roles", "saved roles", details=f"default {default}, " + ", ".join(
            f"{k}={v}" for k, v in sorted((people or {}).items()))[:150])
        return {"ok": True, "msg": "roles saved", **self.roles.view()}

    def stop(self, who: str = "", bid: str = "", key: str = "", role: str = "") -> dict:
        """Anyone may stop. Synchronous. bid: the pressing browser's own id (users.py), to name it in the activity log."""
        name = self.users.label(bid) if Users.valid(bid) else (who or "a browser")
        self._act("stop", "pressed STOP", {"name": name, "key": key, "role": role,
                                           "uid": public_id(bid) if Users.valid(bid) else ""})
        self.core.note_stop(name)
        self.core.stop_all()
        if self.payload:
            self.payload.cancel()
        self.core.bridge.log(f"web: STOP from {who or 'a browser'}")
        return {"ok": True, "msg": "stopped"}

    # ================================================================ commands
    def _log_refusal(self, cmd: str, reason: str):
        self.core.bridge.log(f"web: REFUSED {cmd}: {reason}")
        kind = "roles" if cmd == "roles_save" else "setup" if cmd in ADMIN_COMMANDS else ACTIVITY_COMMANDS.get(cmd, "setup")
        self._act(kind, f"refused {cmd.replace('_', ' ')}", details=reason)

    def command(self, client: str, cmd: str, args: dict, local: bool = False, role: str = "admin") -> dict:
        try:
            return self._command(client, cmd, args, local=local, role=role)
        except CommandError as e:
            self._log_refusal(cmd, str(e))
            raise

    def _command(self, client: str, cmd: str, args: dict, local: bool = False, role: str = "admin") -> dict:
        """local: the browser runs on the robot PC (decided by the server from the connection, never by the page).
        role: the caller's role (roles.py, decided by the server); in-process callers and tests are admin."""
        handler = getattr(self, f"cmd_{cmd}", None)
        if handler is None:
            raise CommandError(f"unknown command '{cmd}'")
        if cmd in ADMIN_COMMANDS and not at_least(role, "admin"):
            raise CommandError(f"only an Admin can change the setup (you are {role.capitalize()})")
        local_only = getattr(handler, "local_only", False)
        if local_only and not local and not remote_power():
            raise CommandError("only from a browser on the robot PC, where you can see the robot "
                               "(or on the teach pendant): MOVEIT_GUI_LOCAL_POWER_ONLY is set")
        read_only = getattr(handler, "read_only", False)
        holds = self.heartbeat(client)                    # any request with the valid token is also a heartbeat
        if not read_only and not holds:
            raise CommandError("take control first: another browser or nobody holds the control lease")
        if not read_only and cmd not in SWITCH_FREE and self._needs_switch():
            raise CommandError(self._switch_refusal())
        if not read_only and cmd not in JOG_KEEPS_READY:
            self.core.jog_disarm("another command")
        args = dict(args or {})
        args.pop("local", None)                           # the server says where the browser is, never the page
        if cmd == "record_delete":
            args["_role"] = role                          # server-derived role; a browser cannot grant itself admin
        if local_only:
            args["local"] = bool(local)
        rev = args.pop("rev", None)
        edits = cmd in PROGRAM_EDITS or cmd in ("program_undo", "program_redo")
        with (self._edit_lock if edits else contextlib.nullcontext()):    # snapshot, handler, compare: one at a time
            if cmd.startswith("program_") and rev is not None and not read_only and int(rev) != self.program_rev:
                raise CommandError("the program was changed (another browser or an undo): it has been reloaded, "
                                   "check your selection and try again")
            before = self._snapshot() if edits else None
            previous_token = getattr(self._command_context, "token", None)
            self._command_context.token = client
            try:
                out = handler(**args)
            except (GeometryError, ProgramError, CalibrationError, PayloadCalibrationError, VialGeometryError,
                    GainsError, FtBiasError, PartError, HandEyeError, SetupError, ServiceError, B.BaseError, R.RobotError,
                    TeachError, BatchError, ProfileError, FlaskGeometryError, FlaskScrapeError, FlaskSetupError,
                    FtMatchError) as e:
                raise CommandError(str(e)) from None
            finally:
                if previous_token is None:
                    self._command_context.__dict__.pop("token", None)
                else:
                    self._command_context.token = previous_token
            if before is not None and self._record_edit(before, undoable=cmd in PROGRAM_EDITS):
                if isinstance(out, dict) and "program" in out:
                    out["program"].update(self._history_view(), rev=self.program_rev)
        kind = "roles" if cmd == "roles_save" else "setup" if cmd in ADMIN_COMMANDS else ACTIVITY_COMMANDS.get(cmd)
        if kind and cmd != "roles_save":              # roles_save logs itself with the list
            self._act(kind, cmd.replace("_", " "),
                      None if holds else {"name": "someone without control", "role": role}, summarize_args(args))
        return out

    def _record_edit(self, before: tuple, undoable: bool = True) -> bool:
        """After an edit (edit lock held): a changed program gets a new revision and, if undoable, an undo entry."""
        if before[0] == self._program_yaml() and before[1] == self.program_path:
            return False
        self.program_rev += 1
        if undoable:
            self._undo.append(before)
            del self._undo[:-UNDO_DEPTH]
            self._redo.clear()
        return True

    def _submit(self, cmd: str, fn: Callable[[], tuple]) -> dict:
        jid = next(self._job_ids)
        epoch = getattr(self.core.bridge, "stop_epoch", 0)
        actor = self._actor()
        caller_token = getattr(self._command_context, "token", None)
        with self._lease_lock:
            submitted_lease = self.lease
            submitted_generation = submitted_lease.generation
            submitted_token = caller_token if caller_token is not None else submitted_lease.token

        def run():
            try:
                if getattr(self.core.bridge, "stop_epoch", 0) != epoch:
                    # A job that waited in the queue (all job threads busy) must not start after a STOP.
                    ok, msg = False, "not started: STOP was pressed before it began"
                elif not self._job_lease_still_matches(submitted_lease, submitted_generation, submitted_token):
                    ok, msg = False, "not started: control changed"
                else:
                    ok, msg = fn()
            except (GeometryError, ProgramError, CalibrationError, PayloadCalibrationError, ValueError) as e:
                ok, msg = False, str(e)                      # PartError, HandEyeError, SetupError are ValueErrors
            except Exception as e:                       # a job must always report back
                ok, msg = False, f"{type(e).__name__}: {e}"
            if not ok:
                self.core.last_refusal = msg
                self._act(ACTIVITY_COMMANDS.get(cmd, "setup"), f"failed {cmd.lower()}", actor, msg)
            self.core.bridge.log(("" if ok else "FAILED: ") + f"{cmd}: {msg}")
            self.results.append({"id": jid, "cmd": cmd, "ok": bool(ok), "msg": msg, "t": time.time()})
            notify = getattr(self, "on_result", None)
            if notify:
                notify()
        self._jobs.submit(run)
        return {"ok": True, "job": jid, "msg": "started"}

    def _job_lease_still_matches(self, lease: Lease, generation: int, token: Optional[str]) -> bool:
        with self._lease_lock:
            return (self.lease is lease and self.lease.generation == generation and
                    (not token or self.lease.holds(token)))

    @staticmethod
    def _ok(msg="ok", **kw):
        return {"ok": True, "msg": msg, **kw}

    @staticmethod
    def _err(msg) -> CommandError:
        return CommandError(msg)

    # ---------------------------------------------------------------- manual
    def cmd_move_joints(self, q_deg, vel=0.1, acc=0.1):
        q = [math.radians(float(v)) for v in q_deg]
        if len(q) != len(self.core.profile.joint_names):
            raise CommandError(f"need {len(self.core.profile.joint_names)} joint values")
        v, a = _frac(vel), _frac(acc)
        why = self.core.motion_refusal()
        if why:
            raise CommandError(why)
        return self._submit("MoveJ", lambda: self.core.move_joints(q, v, a))

    def cmd_move_waypoint(self, name, vel=0.1, acc=0.1):
        w = self.core.profile.waypoints.get(name)
        if w is None or w.joints is None:
            raise CommandError(f"unknown joint waypoint '{name}'")
        return self.cmd_move_joints([math.degrees(v) for v in w.joints], vel, acc)

    def _mm_s(self, speed_mm_s):
        """OPERATE's optional mm/s of a linear move: empty / 0 = none, else in (0, profile max]."""
        try:
            return cart_speed.optional_mm_s(speed_mm_s, self.core.profile.max_cart_speed_mm_s)
        except ValueError as e:
            raise CommandError(str(e)) from None

    def cmd_move_pose(self, frame, xyz_mm, rpy_deg, linear=False, vel=0.1, acc=0.1, speed_mm_s=None):
        pose = PoseSpec.from_T(frame, self.core.store.active_tcp, tf.mm_deg_to_T(xyz_mm, rpy_deg))
        v, a = _frac(vel), _frac(acc)
        mm_s = self._mm_s(speed_mm_s)
        why = self.core.motion_refusal()
        if why:
            raise CommandError(why)
        self.core.geo.planning_target(pose)          # fail now, not in the job, if the frame/TCP is unknown
        return self._submit("MovePose", lambda: self.core.move_pose(pose, bool(linear), v, a, mm_s))

    # ------------------------------------------------------- plan / preview / execute
    # Like RViz MotionPlanning: Plan computes a trajectory and nothing moves; the page previews it; Execute runs
    # exactly that trajectory if the arm is still at its start.
    def _planned(self, ok_msg):
        ok, msg = ok_msg
        if ok:
            self._plan_id += 1
        return ok, msg

    def cmd_plan_joints(self, q_deg, vel=0.1, acc=0.1):
        q = [math.radians(float(v)) for v in q_deg]
        if len(q) != len(self.core.profile.joint_names):
            raise CommandError(f"need {len(self.core.profile.joint_names)} joint values")
        v, a = _frac(vel), _frac(acc)
        return self._submit("plan", lambda: self._planned(self.core.plan_joints(q, v, a)))

    def cmd_plan_pose(self, frame, xyz_mm, rpy_deg, linear=False, vel=0.1, acc=0.1, speed_mm_s=None):
        pose = PoseSpec.from_T(frame, self.core.store.active_tcp, tf.mm_deg_to_T(xyz_mm, rpy_deg))
        v, a = _frac(vel), _frac(acc)
        mm_s = self._mm_s(speed_mm_s)
        self.core.geo.planning_target(pose)          # fail now, not in the job, if the frame/TCP is unknown
        return self._submit("plan", lambda: self._planned(self.core.plan_pose(pose, bool(linear), v, a, mm_s)))

    def cmd_plan_get(self):
        return self._ok(plan=self.core.plan_view(), id=self._plan_id)
    cmd_plan_get.read_only = True

    def cmd_plan_execute(self):
        why = self.core.motion_refusal()
        if why:
            raise CommandError(why)
        self._plan_id += 1                     # the plan is used once: browsers drop its preview now
        return self._submit("execute plan", self.core.execute_plan)

    def cmd_plan_clear(self):
        self.core.clear_plan()
        self._plan_id += 1
        return self._ok("plan cleared")

    # ------------------------------------------------------------------ force
    def cmd_force_apply(self, wrench, frame="tool"):
        """Apply, or change while applying. The page renews with force_hold (else zero in 0.5 s)."""
        target = self._wrench(wrench, frame)
        ok, msg = self.core.force_start(target)
        if not ok:
            raise CommandError(msg)
        return self._ok(msg)

    def _wrench(self, wrench, frame):
        try:
            return WrenchTarget.parse(wrench, frame, self.core.force_cfg)
        except ForceError as e:
            raise CommandError(str(e)) from None

    def cmd_force_hold(self):
        return {"ok": True, "active": self.core.force_hold()}

    def cmd_force_release(self):
        self.core.force_release()
        return self._ok("releasing: ramping to zero")
    cmd_force_release.read_only = True     # lowering the force never needs the lease

    def cmd_force_stop(self):
        self.core.force_stop("stopped: zeroed at once")
        return self._ok("force zeroed")
    cmd_force_stop.read_only = True

    def robot_model(self) -> dict:
        """The robot model plus the live parts (SETUP > Parts), for the 3D views and the path preview."""
        m = self._base_model()
        return parts_merged(self.parts, m) if m.get("ok") else m

    def cmd_program_preview_path(self, assume_missing=True):
        """PROGRAMS 3D preview: the TCP path of the open program from the current joints (kinematics only)."""
        m = self.robot_model()
        if not m.get("ok"):
            raise CommandError(f"no robot model for the preview: {m.get('msg')}")
        q = self.core.last_snap.joints
        if not q:
            raise CommandError("no joint state: the preview starts from where the robot is")
        walls = {n: (float(fx.meta["radius_m"]), float(fx.meta["height_m"]) if fx.meta.get("height_m") else None)
                 for n, fx in self.core.store.fixtures.items() if fx.meta.get("radius_m")}   # vials of known size
        out = simulate(self.program, self.core.profile, self.core.geo, m, list(q), self.core.store.active_tcp,
                       assume_missing=bool(assume_missing), vials=walls)
        return self._ok(out.get("note") or f"{len(out['points']) - 1} moves", **out)
    cmd_program_preview_path.read_only = True

    def cmd_history(self, n=50, name=""):
        from ..history import KEEP_RUNS
        h = self.core.history
        return self._ok(runs=h.recent(min(int(n), KEEP_RUNS), str(name or "")), file=str(h.path), keep=KEEP_RUNS,
                        names=sorted({e.get("name", "") for e in h.entries}))
    cmd_history.read_only = True

    def cmd_history_get(self, id):
        e = self.core.history.get(str(id))
        if e is None:
            raise CommandError("that run is no longer in the history")
        return self._ok(run=e)
    cmd_history_get.read_only = True

    def cmd_cpu_profile(self, seconds=5):
        """DIAGNOSTICS: which threads of the GUI use the CPU, and where (cpu_sample.py). Changes nothing."""
        from ..cpu_sample import sample
        return self._ok(threads=sample(float(seconds)))
    cmd_cpu_profile.read_only = True

    def cmd_activity(self, n=300, who="", kind=""):
        a = self.activity
        return self._ok(events=a.recent(min(int(n), 2000), str(who or ""), str(kind or "")), seq=a.seq,
                        people=a.people(), kinds=list(ACTIVITY_KINDS),
                        file=str(a.path))
    cmd_activity.read_only = True

    # ------------------------------------------------------------ recording (rosbag2 / MCAP)
    # Nothing here moves the robot. Starting needs control (one operator decides what fills the disk);
    # stopping, like STOP, never does. The big STOP does not end a recording: the moment after a STOP is often
    # exactly what you want on record.
    def cmd_record_catalog(self, n=30):
        rec = self.core.recorder
        return self._ok(**rec.catalog(), recent=rec.recent(int(n)), dir=str(rec.root))
    cmd_record_catalog.read_only = True

    def cmd_record_configure(self, presets=None, topics=None, exclude=None, auto=None, max_s=None, max_mb=None):
        ok, msg = self.core.recorder.configure(presets, topics, exclude, auto, max_s, max_mb)
        if not ok:
            raise CommandError(msg)
        return self._ok(msg, settings=self.core.recorder.settings)

    def cmd_record_start(self, name="", note="", preset=""):
        """preset "" = the topics ticked in DIAGNOSTICS; "all" = the header's Record button (everything for a run)."""
        name, note = str(name or ""), str(note or "")
        if len(name) > 60:
            raise CommandError("the recording name is limited to 60 characters")
        if len(note) > 200:
            raise CommandError("the note is limited to 200 characters")
        ok, msg = self.core.record_start(name=name, note=note, presets=[str(preset)] if preset else None)
        if not ok:
            raise CommandError(msg)
        return self._ok(msg)

    def cmd_record_stop(self):
        ok, msg = self.core.recorder.stop("stopped by the operator")
        return self._ok(msg)
    cmd_record_stop.read_only = True

    def record_mcap(self, name=""):
        return self.core.recorder.mcap_path(str(name or ""))

    def cmd_record_delete(self, name="", _role="viewer"):
        name = str(name or "")
        rec = self.core.recorder
        info = rec.record_info(name)
        if info is None:
            raise CommandError("recording not found")
        owner = str(info.get("operator") or "").strip()
        mine = str(self.lease.profile or "").strip()
        if _role != "admin" and (not owner or owner.lower() != mine.lower()):
            raise CommandError("only the recording owner or an Admin can delete it")
        ok, msg = rec.delete_recording(name)
        if not ok:
            raise CommandError(msg)
        self._act("record", "deleted recording", details=name)
        return self._ok(msg)

    def cmd_speed_override(self, percent):
        ok, msg = self.core.set_speed_override(float(percent) / 100.0)
        if not ok:
            raise CommandError(msg)
        return self._ok(msg)

    def cmd_switch_mode(self, mode):
        if mode not in self.core.profile.modes:
            raise CommandError(f"unknown mode '{mode}'")
        why = self.base_lacks(mode=mode)
        if why:
            raise CommandError(why)
        return self._submit(f"switch to {mode}", lambda: self.core.switch_mode(mode))

    def cmd_freedrive(self, on):
        return self._submit("freedrive", lambda: self.core.freedrive(bool(on)))

    def cmd_recover(self):
        return self._submit("re-verify", self.core.modes.verify_recovery)

    # ------------------------------------------------------------ robot power (UR Dashboard)
    # Power off may come from any browser that has control: it only makes the robot safer.
    # Power on and brake release wake the arm (releasing the brakes moves the joints slightly), so they need
    # control and an explicit "I can see the robot", from any browser (logged with who and where). With
    # MOVEIT_GUI_LOCAL_POWER_ONLY=1 only a browser on the robot PC (or the pendant) may.
    # The pendant must be in Remote Control mode; in Local mode the robot refuses and says so.
    def _who(self) -> str:
        return self.lease.name or "a browser"

    def cmd_power_off(self):
        s = self.core.last_snap
        if s.robot_mode == "POWER_OFF":
            raise CommandError("the robot is already powered off")
        if not self.core.idle() or s.freedrive:
            self.stop(f"{self._who()} (power off)")       # stop GUI motion first, then the brakes engage
        self.core.bridge.log(f"web: power off from {self._who()}")
        return self._submit("power off", lambda: self.core.bridge.dashboard("power_off"))

    def _require_seen(self, seen, what):
        if seen is not True:
            raise CommandError(f"{what}: confirm that you can see the robot and its workspace is clear")
        if not self.core.idle():
            raise CommandError(f"{what}: the GUI is commanding the robot: stop it first")

    def _where(self, local) -> str:
        return "on the robot PC" if local else "from a remote browser"

    def cmd_power_on(self, seen=False, local=False):
        self._require_seen(seen, "power on")
        if self.core.last_snap.robot_mode in ("POWER_ON", "IDLE", "RUNNING"):
            raise CommandError("the robot is already powered on")
        self.core.bridge.log(f"web: power on from {self._who()} {self._where(local)}")
        return self._submit("power on", lambda: self.core.bridge.dashboard("power_on"))
    cmd_power_on.local_only = True

    def cmd_brake_release(self, seen=False, local=False):
        self._require_seen(seen, "brake release")
        m = self.core.last_snap.robot_mode
        if m == "RUNNING":
            raise CommandError("the brakes are already released (robot mode RUNNING)")
        if m not in ("IDLE", "?"):
            raise CommandError(f"robot mode is {m}: power on first and wait for IDLE")
        self.core.bridge.log(f"web: brake release from {self._who()} {self._where(local)}")
        return self._submit("brake release", lambda: self.core.bridge.dashboard("brake_release"))
    cmd_brake_release.local_only = True

    def cmd_robot_loaded_program(self):
        """Which program the pendant has loaded (read-only), so the operator can confirm it before Play."""
        ok, name = self.core.bridge.loaded_program()
        if not ok:
            raise CommandError(f"cannot read which program is loaded on the pendant: {name}")
        return self._ok(name, program=name)
    cmd_robot_loaded_program.read_only = True

    def cmd_robot_play(self, program):
        """Start the program loaded on the pendant through the Dashboard (in Remote Control the pendant's own Play
        is disabled). Dashboard play starts WHATEVER is loaded, so the operator confirms the program by name and it
        is only started if that is still the one loaded. Any name is fine (External Control saved under any name).
        Starting it does not move the arm, but the driver would execute a pending trajectory goal: cancel those."""
        c, s = self.core, self.core.last_snap
        if s.robot_mode != "RUNNING":
            raise CommandError(f"robot mode is {s.robot_mode}: power on and release the brakes first")
        if s.program_running is True:
            raise CommandError("the program is already running")
        # a GUI program PAUSED (by the robot stop or by its Pause) may wait: nothing else may command the arm
        if c.busy or c.policies.busy or c.jogging or c.force_active or c.runner.state.name not in ("IDLE", "PAUSED"):
            raise CommandError("the GUI is commanding the robot: stop it first")
        ok, name = self.core.bridge.loaded_program()
        if not ok:
            raise CommandError(f"cannot read which program is loaded on the pendant ({name}): start it there")
        if name != program:
            raise CommandError(f"the pendant now has '{name}' loaded, not '{program}' that you confirmed: "
                               "press Play program again")
        self.core.bridge.cancel_motion()                  # no stale goal may run when the program starts
        resume = s.program_state == "PAUSED"
        # the GUI run that the robot stop paused carries on once the robot program plays again
        c._resume_after_robot = c.runner.state.name == "PAUSED" and bool(c.runner.pause_reason)
        self.core.bridge.log(f"web: {'resume' if resume else 'play'} {name} from {self._who()}")
        return self._submit("resume program" if resume else "play program", lambda: self.core.bridge.dashboard("play"))

    def _robot_program_hold(self, what: str):
        """Before the pendant program pauses or stops: a GUI program run pauses (it can carry on afterwards),
        everything else the GUI commands stops. Then the robot halts with the brakes off and power on."""
        c = self.core
        c._resume_after_robot = False
        if c.runner.state.name == "RUNNING":
            c.runner.pause(f"the robot program was {what} from the GUI")
        c.force_stop(f"robot program {what}: zeroed at once")
        c.jog_stop(f"robot program {what}")
        if c.policies.busy:
            c.policies.cancel()
        if c.busy:
            c.bridge.cancel_motion()

    def cmd_robot_pause(self):
        """Pause the program on the robot (UR Dashboard): the arm halts, power and brakes stay as they are.
        ▶ Resume program continues it. Allowed from any browser with control: it only stops motion."""
        s = self.core.last_snap
        if s.program_running is not True and s.program_state != "PLAYING":
            raise CommandError("the robot program is not running" +
                               (" (already paused)" if s.program_state == "PAUSED" else ""))
        self._robot_program_hold("paused")
        self.core.bridge.log(f"web: pause robot program from {self._who()}")
        return self._submit("pause program", lambda: self.core.bridge.dashboard("pause"))

    def cmd_robot_stop_program(self):
        """Stop the program on the robot (UR Dashboard), power and brakes untouched. ▶ Play starts it again
        from the beginning (External Control: the GUI can command the arm again)."""
        s = self.core.last_snap
        if s.program_running is not True and s.program_state not in ("PLAYING", "PAUSED"):
            raise CommandError("the robot program is not running")
        self._robot_program_hold("stopped")
        self.core.bridge.log(f"web: stop robot program from {self._who()}")
        return self._submit("stop program", lambda: self.core.bridge.dashboard("stop"))

    def _manual_refusal(self) -> Optional[str]:
        """Why a manual (non-motion) robot command such as the gripper is not allowed now, or None."""
        c = self.core
        if c.runner.state.name != "IDLE" or c.batch.active:
            return "a program is running: pause or stop it first"
        if c.policies.busy:
            return "a policy is running: stop it first"
        if self.payload_progress["running"]:
            return "payload calibration is moving the robot: wait or stop it first"
        return None

    def _require_manual(self):
        why = self._manual_refusal()
        if why:
            self.core.last_refusal = why
            raise CommandError(why)

    def cmd_gripper(self, position=None, effort=None, percent=None):
        """position in joint units, or percent closed (0 = open, 100 = closed). effort in N."""
        self._require_manual()
        g = self.core.profile.gripper
        if not g.get("enabled"):
            raise CommandError("no gripper in this profile")
        why = self.base_lacks(gripper=True)
        if why:
            raise CommandError(why)
        lo, hi = float(g.get("open_position", 0.0)), float(g.get("closed_position", 0.8))
        if percent is not None:
            pos = lo + (hi - lo) * min(100.0, max(0.0, float(percent))) / 100.0
        elif position is not None:
            pos = float(position)
        else:
            raise CommandError("give a position or a percent")
        limit = float(g.get("effort_limit", g.get("max_effort", 40.0)))
        eff = float(effort if effort is not None else g.get("max_effort", 40.0))
        if not 0.0 < eff <= limit:
            raise CommandError(f"force must be in (0, {limit:g}] N")
        bridge = self.core.bridge

        def run():
            ok = bridge.gripper_move(pos, eff)
            gs = bridge.snapshot().gripper_status
            return ok, gs["object"] if gs else ("done" if ok else "failed")
        return self._submit("gripper", run)

    def cmd_gripper_reactivate(self):
        self._require_manual()
        if not self.core.profile.gripper.get("reactivate_service"):
            raise CommandError("no reactivate service in this profile")
        return self._submit("gripper reactivate", self.core.bridge.gripper_reactivate)

    def cmd_gripper_reconnect(self):
        """Bring the gripper up again after the stack started while the robot was off (no tool power)."""
        g = self.core.profile.gripper
        if not g.get("enabled") or not g.get("controller_manager"):
            raise CommandError("no gripper controller manager in this profile")
        m = self.core.last_snap.robot_mode
        if m not in ("?", "IDLE", "RUNNING"):
            raise CommandError(f"robot mode is {m}: the gripper gets power from the robot. Power the robot on first "
                               "(OPERATE, Robot), then Reconnect gripper")
        return self._submit("gripper reconnect", self.core.bridge.gripper_reconnect)

    def cmd_set_tcp(self, name):
        if not self.core.idle():
            raise CommandError("the robot is busy: change the TCP while idle")
        ok, msg = self.core.set_active_tcp(name)
        if not ok:
            raise CommandError(msg)
        return self._ok(msg)

    def cmd_tcp_save(self, name, parent, xyz_mm, rpy_deg, replace=False):
        """Create (or edit) a virtual TCP: a fixed offset from a parent TCP or link (mm / deg).
        Pose semantics only: MoveIt's collision model is unchanged."""
        if not self.core.idle():
            raise CommandError("the robot is busy: change TCPs while idle")
        try:
            xyz, rpy = [float(v) for v in xyz_mm], [float(v) for v in rpy_deg]
        except (TypeError, ValueError):
            raise CommandError("offsets must be numbers") from None
        if len(xyz) != 3 or len(rpy) != 3:
            raise CommandError("give x, y, z (mm) and roll, pitch, yaw (deg)")
        if max(abs(v) for v in xyz) > 2000:
            raise CommandError("an offset over 2 m is surely a typo (units are mm)")
        if parent not in self._tcp_parents():
            raise CommandError(f"unknown parent '{parent}'")
        if str(name).strip() == ACTIVE_TCP:
            raise CommandError(f"'{ACTIVE_TCP}' is reserved: in a program pose it means whichever TCP is active")
        tcp = TcpDef(str(name).strip(), "virtual", parent=parent, T_parent_tcp=tf.mm_deg_to_T(xyz, rpy),
                     meta={"method": "manual offset (web)"})
        self.core.store.put_tcp(tcp, replace=bool(replace))     # CalibrationError -> CommandError
        self.core.bridge.log(f"TCP {tcp.name}: {xyz} mm, {rpy} deg from {parent} ({'edited' if replace else 'created'})")
        return self._ok(f"TCP {tcp.name} saved")

    def cmd_tcp_delete(self, name):
        if not self.core.idle():
            raise CommandError("the robot is busy: change TCPs while idle")
        self.core.store.delete_tcp(str(name))
        self.core.bridge.log(f"TCP {name} deleted")
        return self._ok(f"TCP {name} deleted")

    def cmd_pivot_reset(self, parent):
        if parent not in self._tcp_parents():
            raise CommandError(f"unknown parent '{parent}'")
        self._pivot_parent = parent
        self._pivot_samples = []
        self._pivot_result = None
        return self._ok(pivot=self._pivot_view())

    def cmd_pivot_capture(self, parent):
        if parent not in self._tcp_parents():
            raise CommandError(f"unknown parent '{parent}'")
        if self._pivot_parent != parent:
            self.cmd_pivot_reset(parent)
        if len(self._pivot_samples) >= 100:
            raise CommandError("maximum 100 pivot samples")
        # Reads TF only. The operator moves the robot in freedrive; never command motion here.
        T = self.core.geo.current_T_base_tcp(parent)
        if not np.all(np.isfinite(T)):
            raise CommandError("invalid parent pose")
        self._pivot_samples.append(T)
        self._pivot_result = None
        return self._ok(pivot=self._pivot_view())

    def cmd_pivot_remove(self, index):
        i = int(index)
        if not 0 <= i < len(self._pivot_samples):
            raise CommandError("no sample at that index")
        del self._pivot_samples[i]
        self._pivot_result = None
        return self._ok(pivot=self._pivot_view())

    def cmd_pivot_solve(self):
        self._pivot_result = solve_pivot(self._pivot_samples)
        return self._ok(pivot=self._pivot_view())

    def cmd_pivot_save(self, name, rpy_deg, replace=False):
        if not self.core.idle():
            raise CommandError("the robot is busy: save TCPs while idle")
        res = self._pivot_result
        if res is None:
            raise CommandError("solve the current samples first")
        if self._pivot_parent not in self._tcp_parents():
            raise CommandError("pivot parent is no longer available")
        rpy = [float(v) for v in rpy_deg]
        if len(rpy) != 3 or not all(math.isfinite(v) and abs(v) <= 180 for v in rpy):
            raise CommandError("orientation must be three finite angles in [-180, 180] degrees")
        name = str(name).strip()
        if name == ACTIVE_TCP:
            raise CommandError(f"'{ACTIVE_TCP}' is reserved")
        tcp = TcpDef(name, "virtual", parent=self._pivot_parent,
                     T_parent_tcp=tf.mm_deg_to_T(res.p_parent_tcp * 1000, rpy),
                     meta={"method": "pivot", "sample_count": res.sample_count,
                           "residual_rms_m": round(res.rms_m, 7),
                           "residual_max_m": round(res.max_m, 7),
                           "orientation": "manual RPY (not calibrated)" if any(rpy)
                           else "inherited from parent"})
        self.core.store.put_tcp(tcp, replace=bool(replace))
        self.core.bridge.log(f"Pivot TCP {name} saved from {res.sample_count} samples, RMS {res.rms_m * 1000:.3f} mm")
        return self._ok(f"TCP {name} saved")

    def _pivot_view(self):
        quality, reason, span = pivot_quality(self._pivot_samples)
        samples = []
        for T in self._pivot_samples:
            xyz, rpy = tf.T_to_mm_deg(T)
            samples.append({"xyz_mm": [round(float(x), 2) for x in xyz],
                            "rpy_deg": [round(float(x), 2) for x in rpy]})
        out = {"parent": self._pivot_parent, "samples": samples,
               "quality": quality, "quality_reason": reason, "span_deg": round(span, 1), "result": None}
        if self._pivot_result is not None:
            r = self._pivot_result
            out["result"] = {"xyz_mm": [round(float(x * 1000), 3) for x in r.p_parent_tcp],
                             "pivot_base_mm": [round(float(x * 1000), 2) for x in r.p_base_pivot],
                             "rms_mm": round(r.rms_m * 1000, 3), "max_mm": round(r.max_m * 1000, 3),
                             "worst_sample": int(np.argmax(r.residuals_m)) + 1}
        return out

    def cmd_pivot_get(self):
        return self._ok(pivot=self._pivot_view())
    cmd_pivot_get.read_only = True

    def _tcp_parents(self) -> List[str]:
        p = self.core.profile
        out = list(self.core.store.tcps) + [p.planning_link] + list(p.tf_frames)
        return list(dict.fromkeys(out))

    def cmd_current_pose(self, frame):
        """Active TCP in `frame`, mm / deg: what "Copy current" fills in."""
        xyz, rpy = tf.T_to_mm_deg(self.core.geo.current_pose(frame, self.core.store.active_tcp).T())
        return self._ok(xyz_mm=[round(v, 2) for v in xyz], rpy_deg=[round(v, 2) for v in rpy])
    cmd_current_pose.read_only = True

    def cmd_tf_frames(self):
        """Every known frame with its pose in the base frame (mm / deg), for the 3D view's Frames overlay."""
        c, p = self.core, self.core.profile
        kinds: Dict[str, str] = {}

        def add(name, kind):
            if name and name not in kinds:
                kinds[name] = kind
        add(p.base_link, "base")
        for f in p.tf_frames:
            add(f, "tf")
        for f in c.store.fixtures:
            add(f, "fixture")
        for f in c.store.vision:
            add(f, "vision")
        add(p.ft_calibration.get("sensor_frame"), "ft")
        for cam in p.cameras:
            add(cam.get("frame"), "camera")
        out = []
        for name, kind in kinds.items():
            row = {"name": name, "kind": kind}
            try:
                xyz, rpy = tf.T_to_mm_deg(c.geo.T_base_frame(name))
                row.update(xyz_mm=[round(float(v), 2) for v in xyz], rpy_deg=[round(float(v), 2) for v in rpy])
            except Exception as e:  # noqa: BLE001 - an unresolvable frame is reported, not fatal
                row["error"] = str(e) or type(e).__name__
            out.append(row)
        for name in c.store.tcps:                    # a TCP named like a TF frame replaces that row
            out = [r for r in out if r["name"] != name]
            row = {"name": name, "kind": "tcp", "active": name == c.store.active_tcp}
            try:
                xyz, rpy = tf.T_to_mm_deg(c.geo.current_T_base_tcp(name))
                row.update(xyz_mm=[round(float(v), 2) for v in xyz], rpy_deg=[round(float(v), 2) for v in rpy])
            except Exception as e:  # noqa: BLE001
                row["error"] = str(e) or type(e).__name__
            out.append(row)
        return self._ok(frames=out, base=p.base_link)
    cmd_tf_frames.read_only = True

    # -------------------------------------------------------------------- jog
    def _jog_cmd(self, kind, axis, sign, frame="base") -> JogCommand:
        try:
            return JogCommand.parse(kind, axis, sign, frame, len(self.core.profile.joint_names))
        except JogError as e:
            raise CommandError(str(e)) from None

    def cmd_jog_step(self, kind, axis, sign, step, frame="base", vel=0.1, acc=0.1):
        """One press = one short MoveIt move: `step` mm (or deg for rotations and joints)."""
        c = self._jog_cmd(kind, axis, sign, frame)
        v, a = _frac(vel), _frac(acc)
        why = self.core.motion_refusal()
        if why:
            raise CommandError(why)
        st = float(step)
        return self._submit(f"jog {c.describe()}", lambda: self.core.jog_step(c, st, v, a))

    def cmd_jog_start(self, kind, axis=None, sign=None, frame="base", speed=0.1, values=None):
        """Hold-to-jog: six normalized slider values, or legacy one-axis buttons."""
        if values is not None:
            try:
                v = JogVelocity.parse(kind, values, frame, len(self.core.profile.joint_names))
            except JogError as e:
                raise CommandError(str(e)) from None
            # The original MoveIt-GUI mapped its six sliders linearly to Servo velocity. Keep that interaction;
            # `speed` remains the operator's overall cap from OPERATE > Velocity %.
            ok, msg = self.core.jog_start_velocity(v, _frac(speed), shape_values=False)
        else:
            ok, msg = self.core.jog_start(self._jog_cmd(kind, axis, sign, frame), _frac(speed))
        if not ok:
            raise CommandError(msg)
        return self._ok(msg)

    def cmd_jog_hold(self, values=None):
        try:
            return {"ok": True, "jogging": self.core.jog_hold(values)}
        except JogError as e:
            raise CommandError(str(e)) from None

    def cmd_jog_stop(self):
        self.core.jog_stop()
        return self._ok("jog stopped")
    cmd_jog_stop.read_only = True          # stopping never needs the lease

    def cmd_debug_profile(self, seconds=10):
        """Where the GUI's time goes: samples every thread for a few seconds, then logs the busiest places."""
        from . import perf
        sec = min(30.0, max(1.0, float(seconds)))

        def run():
            for line in perf.sample(sec):
                self.core.bridge.log(line)
        threading.Thread(target=run, daemon=True, name="web-profile").start()
        return self._ok(f"profiling for {sec:.0f} s: the result goes to the log")
    cmd_debug_profile.read_only = True     # it only reads; admin only (ADMIN_COMMANDS)

    def cmd_jog_arm(self):
        """Jog tab open: keep Servo ready so a press moves at once. Renew about every second."""
        ok, msg = self.core.jog_arm()
        return {"ok": ok, "msg": msg}

    def cmd_jog_disarm(self):
        self.core.jog_disarm("Jog tab closed")
        return self._ok("Servo released")
    cmd_jog_disarm.read_only = True        # giving the arm back never needs the lease

    # ---------------------------------------------------------------- program
    def _programs_dir(self, owner="", create=False) -> Path:
        """Where `owner`'s programs are: <programs_dir>/<profile name>/; "" = Shared, the programs dir itself."""
        if not str(owner or "").strip():
            return self.core.programs_dir
        d = self.core.programs_dir / self.profiles.find(owner)
        if create:
            d.mkdir(parents=True, exist_ok=True)
        return d

    def _program_owner(self) -> str:
        """Whose folder the open program is in ("" = Shared or not saved yet)."""
        p = self.program_path
        return p.parent.name if p and p.parent != self.core.programs_dir.resolve() else ""

    def _program_files(self, owner="") -> dict:
        d = self._programs_dir(owner)
        return dict(owner=self.profiles.find(owner) if str(owner or "").strip() else "",
                    profiles=[x["name"] for x in self.profiles.list()],
                    files=sorted(p.name for p in d.glob("*.y*ml") if not p.name.endswith(".batch.yaml")))   # PROGRAMS > Batch files

    def cmd_program_list(self, owner=""):
        return self._ok(**self._program_files(owner))

    def cmd_program_copy(self, source_owner, name, owner="", new_name="", replace=False):
        """Copy a program file of another profile (or Shared) into profile `owner`."""
        src = self._safe_path(name, source_owner)
        if not src.is_file():
            raise CommandError(f"no program '{name}'")
        new = str(new_name or "").strip() or src.name
        if not new.endswith((".yaml", ".yml")):
            new += ".yaml"
        dst = self._safe_path(new, self._own(owner), create=True)        # into your own profile (or Shared)
        if dst.exists() and not replace:
            raise CommandError(f"{dst.name} already exists there")
        if dst == src:
            raise CommandError("pick another profile to copy into")
        shutil.copyfile(src, dst)
        who = self.profiles.find(owner) if str(owner or "").strip() else "Shared"
        self.core.bridge.log(f"program: copied {src} to {dst}")
        return self._ok(f"copied {src.name} to {who}" + (f" as {dst.name}" if dst.name != src.name else ""),
                        **self._program_files(owner))
    cmd_program_list.read_only = True

    def cmd_program_get(self):
        return self._ok(**self.program_view())
    cmd_program_get.read_only = True

    def _editable(self):
        if self.core.runner.state != RunState.IDLE or self.core.batch.active:
            raise CommandError("stop the program before editing")

    def cmd_program_new(self, name=None):
        self._editable()
        self.program, self.program_path = Program(name=str(name or "untitled"), robot=self.core.profile.name), None
        self._mark_saved()
        return self._ok("new program", **self.program_view())

    # ------------------------------------------------- UR-style tree editing
    def _path(self, path, index=None) -> tuple:
        if path is None and index is not None:
            path = [index]
        if not isinstance(path, (list, tuple)) or not path:
            raise CommandError("select a step first")
        try:
            path = tuple(int(i) for i in path)
            self.program.step_at(path)
        except (TypeError, ValueError, ProgramError) as e:
            raise CommandError(f"no such step: {e}") from None
        return path

    # --------------------------------------------- unsaved changes, undo/redo
    def _program_yaml(self) -> str:
        return yaml.safe_dump(self.program.to_dict(), sort_keys=False, default_flow_style=None)

    def _snapshot(self) -> tuple:
        return self._program_yaml(), self.program_path

    def _restore(self, snap: tuple):
        text, path = snap
        self.program = Program.from_dict(yaml.safe_load(text) or {}, self.core.profile)
        self.program.notes = []
        self.program_path = path

    def _history_view(self) -> dict:
        return {"dirty": self._program_yaml() != self._saved_yaml,
                "can_undo": bool(self._undo), "can_redo": bool(self._redo)}

    def _mark_saved(self):
        self._saved_yaml = self._program_yaml()

    def cmd_program_undo(self):
        self._editable()
        if not self._undo:
            raise CommandError("nothing to undo")
        self._redo.append(self._snapshot())
        self._restore(self._undo.pop())
        return self._reply("undone")

    def cmd_program_redo(self):
        self._editable()
        if not self._redo:
            raise CommandError("nothing to redo")
        self._undo.append(self._snapshot())
        self._restore(self._redo.pop())
        return self._reply("redone")

    def _reply(self, msg, selected=None):
        return self._ok(msg, selected=list(selected) if selected is not None else None, **self.program_view())

    def _record_waypoint(self, kind: str, frame=None, name=None) -> str:
        """A waypoint at the robot's current position: 'joints' or 'pose' (active TCP in `frame`)."""
        s = self.core.last_snap
        if kind == "joints":
            if not s.joints or s.joint_state_age is None or s.joint_state_age > self.core.profile.joint_state_timeout_s:
                raise CommandError("no fresh joint state to record")
            name = name or self._new_wp("wp")
            self.program.waypoints[name] = ProgramWaypoint(name, joints=list(s.joints))
        else:
            frame = frame or self.core.profile.base_link
            pose = self.core.geo.current_pose(frame, self.core.store.active_tcp)
            name = name or self._new_wp("pose")
            self.program.waypoints[name] = ProgramWaypoint(name, pose=pose)
        return name

    def _default_step(self, type_, params) -> Step:
        prof = self.core.profile
        if type_ == "movej":
            return Step("movej", {"target": self._record_waypoint("joints"), "velocity": 0.1})
        if type_ == "move_pose":
            name = self._record_waypoint("pose", params.get("frame"))
            return Step("move_pose", {"target": name, "cartesian": bool(params.get("cartesian", False)),
                                      "velocity": 0.1})
        defaults = {
            "wait": {"seconds": 1.0},
            "gripper": {"command": "open"},
            "set_io": {"pin": 0, "state": True},
            "speed": {"fraction": 0.5},
            "controller": {"mode": self.core.mode if self.core.mode in prof.modes
                           and not prof.modes[self.core.mode].hidden else prof.default_mode},
            "policy": {"policy": next(iter(self.core.policies.manifests), ""), "duration": 8.0},
            "loop": {"times": 2},
            "folder": {"name": "Folder"},
            "comment": {"text": ""},
            # force work: tool axes; +Z of the TCP is "forward" for most tools
            "move_relative": {"offset_mm": [0.0, 0.0, 10.0], "rotate_deg": [0.0, 0.0, 0.0], "frame": "tool",
                              "cartesian": True, "velocity": 0.05},
            "move_until_contact": {"direction": [0.0, 0.0, 1.0], "frame": "tool", "distance_mm": 20.0,
                                   "force_n": 3.0, "velocity": 0.02, "on_no_contact": "fail"},
            "force": {"wrench": [0.0, 0.0, 5.0, 0.0, 0.0, 0.0], "frame": "tool"},
            "force_off": {},
            "zero_ft": {},
            "photo": {"topic": self._default_camera(), "label": ""},
            "record_start": {"preset": "all"},
            "record_stop": {},
        }
        if type_ not in defaults:
            raise CommandError(f"cannot insert a '{type_}' step")
        st = Step(type_, dict(defaults[type_]))
        keep_frame = type_ in ("move_relative", "move_until_contact", "force")
        self._apply_params(st, {k: v for k, v in params.items() if k != "frame" or keep_frame})
        return st

    # editable parameters per type and how to read them from the browser
    _PARAMS = {
        "movej": {"target": str, "velocity": float, "acceleration": float, "on_fail": _choice("stop", "continue")},
        "move_pose": {"target": str, "cartesian": bool, "velocity": float, "acceleration": float,
                      "speed_mm_s": float, "on_fail": _choice("stop", "continue")},
        "gripper": {"command": str, "position": float, "effort": float},
        "wait": {"seconds": float},
        "set_io": {"pin": int, "state": bool},
        "speed": {"fraction": float},
        "controller": {"mode": str},
        "policy": {"policy": str, "duration": float},
        "loop": {"times": int},
        "folder": {"name": str},
        "comment": {"text": str},
        "move_relative": {"offset_mm": _floats(3), "rotate_deg": _floats(3), "frame": str, "cartesian": bool,
                          "velocity": float, "acceleration": float, "pivot": _choice("tcp", "frame"),
                          "speed_mm_s": float, "on_fail": _choice("stop", "continue")},
        "move_until_contact": {"direction": _floats(3), "frame": str, "distance_mm": float, "force_n": float,
                               "velocity": float, "on_no_contact": _choice("fail", "continue"),
                               "speed_mm_s": float, "on_fail": _choice("stop", "continue")},
        "force": {"wrench": _floats(6), "frame": _choice("tool", "base"), "wait": bool},
        "force_off": {"timeout_s": float},
        "zero_ft": {},
        "photo": {"topic": str, "label": str},
        "record_start": {"preset": str, "note": str},
        "record_stop": {},
        "replay": {"name": str, "speed": float, "velocity": float, "on_fail": _choice("stop", "continue")},
    }

    def _default_camera(self) -> str:
        cams = [c["topic"] for c in self.core.profile.cameras]
        return next((t for t in cams if "color" in t), cams[0] if cams else "/camera/camera/color/image_rect_raw")

    def _apply_params(self, st: Step, params: dict):
        allowed = self._PARAMS.get(st.type, {})
        new = dict(st.params)
        for k, v in (params or {}).items():
            if k not in allowed:
                raise CommandError(f"{st.title} has no parameter '{k}'")
            if v is None or v == "":
                new.pop(k, None)
                continue
            try:
                new[k] = allowed[k](v)
            except (TypeError, ValueError):
                raise CommandError(f"{k}: bad value {v!r}") from None
            if k == "speed_mm_s" and new[k] == 0:
                new.pop(k)                           # 0 mm/s = no limit, as on OPERATE: not saved
        if st.type in ("movej", "move_pose") and "target" in new:
            w = self.program.waypoints.get(new["target"])
            if w is None and not (st.type == "movej" and new["target"] in self.core.profile.waypoints):
                raise CommandError(f"unknown waypoint '{new['target']}'")
            if w is not None and (w.joints is None) == (st.type == "movej"):
                raise CommandError(f"waypoint '{new['target']}' is a {'pose' if w.joints is None else 'joint'} waypoint")
        if st.type == "gripper" and new.get("command") == "position":
            new.pop("command")
        st.params = new
        problems = Program(steps=[st], waypoints=self.program.waypoints).validate(self.core.profile)
        hard = [x for x in problems if "empty loop" not in x]
        if hard:
            raise CommandError("; ".join(x.split(": ", 1)[-1] for x in hard))

    def cmd_program_insert(self, type, path=None, where="after", params=None):
        self._editable()
        if type not in STEP_TYPES:
            raise CommandError(f"unknown step type '{type}'")
        st = self._default_step(type, params or {})
        at = self._insert_step(st, None if path is None or path == [] else self._path(path), where)
        return self._reply(f"added {st.title}", at)

    def _insert_step(self, st: Step, path, where="after") -> tuple:
        """Put st after / before the step at path, or into it (a container); path None = at the end. Its path."""
        if path is None:
            self.program.steps.append(st)
            return (len(self.program.steps) - 1,)
        path = tuple(path)
        if where == "into":
            parent = self.program.step_at(path)
            if not parent.container:
                raise CommandError(f"{parent.title} cannot contain steps: insert after it instead")
            parent.children.append(st)
            return path + (len(parent.children) - 1,)
        lst = self.program.siblings(path)
        at = path[-1] + (1 if where != "before" else 0)
        lst.insert(at, st)
        return path[:-1] + (at,)

    def cmd_program_update(self, path, params):
        self._editable()
        path = self._path(path)
        st = self.program.step_at(path)
        old = dict(st.params)
        try:
            self._apply_params(st, params or {})
        except CommandError:
            st.params = old
            raise
        return self._reply(f"step {path_label(path)} updated", path)

    def cmd_program_type_waypoint(self, name, frame=None, xyz_mm=None, rpy_deg=None, joints_deg=None, tcp=None):
        """Type a waypoint instead of teaching it: a pose (frame, XYZ mm, RPY deg of its TCP, e.g. in a vial
        fixture) or joint angles (deg). Every step using it moves there. tcp: which TCP goes there, a named one or
        "active" (whichever TCP is active when it runs); the numbers stay."""
        self._editable()
        w = self.program.waypoints.get(name)
        if w is None:
            raise CommandError(f"unknown waypoint '{name}'")
        try:
            if w.joints is not None:
                q = [math.radians(float(v)) for v in (joints_deg or [])]
                if len(q) != len(self.core.profile.joint_names):
                    raise CommandError(f"{len(self.core.profile.joint_names)} joint angles needed")
                self.program.waypoints[name] = ProgramWaypoint(name, joints=q)
            else:
                xyz = [float(v) for v in (xyz_mm or [])]
                rpy = [float(v) for v in (rpy_deg or [])]
                if len(xyz) != 3 or len(rpy) != 3 or not all(math.isfinite(v) for v in xyz + rpy):
                    raise CommandError("a pose needs X Y Z in mm and R P Y in degrees")
                fr = frame or w.pose.frame
                tc = str(tcp) if tcp else w.pose.tcp
                bad = self.core.geo.static_problems(fr, tc)
                if bad:
                    raise CommandError("; ".join(bad))
                T = tf.mm_deg_to_T(xyz, rpy)
                self.program.waypoints[name] = ProgramWaypoint(name, pose=PoseSpec.from_T(fr, tc, T))
        except (TypeError, ValueError):
            raise CommandError("bad number in the waypoint") from None
        return self._reply(f"waypoint {name} typed")

    def cmd_program_set_waypoint(self, path, frame=None):
        """Re-teach the move step's waypoint from where the robot is now (UR 'Set waypoint')."""
        self._editable()
        path = self._path(path)
        st = self.program.step_at(path)
        if st.type not in ("movej", "move_pose"):
            raise CommandError("only move steps have a waypoint")
        name = st.params.get("target")
        if name not in self.program.waypoints:          # inline or profile target: make it a program waypoint
            name = None
        if st.type == "movej":
            name = self._record_waypoint("joints", name=name)
        else:
            old = self.program.waypoints.get(name) if name else None
            name = self._record_waypoint("pose", frame or (old.pose.frame if old and old.pose else None), name=name)
        st.params.pop("joints", None)
        st.params.pop("pose", None)
        st.params["target"] = name
        users = sum(1 for x in self.program.all_steps() if x.params.get("target") == name)
        return self._reply(f"waypoint {name} set to the current position"
                           + (f" ({users} steps use it)" if users > 1 else ""), path)

    def cmd_hold_renew(self):
        """Hold-to-run: the browser renews every ~150 ms while the button is held (see tick())."""
        with self._lock:
            if self._hold is None:
                return self._ok("not holding", holding=False)
            self._hold["seen"] = self.clock()
            return self._ok("holding", holding=True)

    def _held(self, label: str, fn: Callable[[], tuple]) -> dict:
        """Run a motion job that stops unless the browser keeps renewing (hold-to-run)."""
        with self._lock:
            self._hold = {"seen": self.clock(), "label": label}
        hold = self._hold

        def run():
            try:
                return fn()
            finally:
                with self._lock:
                    if self._hold is hold:
                        self._hold = None
        return self._submit(label, run)

    def cmd_program_move_here(self, path, vel=0.1, acc=0.1, hold=False):
        """Move the robot to the move step's target (UR 'Move here'). hold=True: hold-to-run (renewals)."""
        path = self._path(path)
        st = self.program.step_at(path)
        v, a = _frac(vel), _frac(acc)
        why = self.core.motion_refusal()
        if why:
            raise CommandError(why)
        try:
            if st.type == "movej":
                q = list(self.program.joint_target(st, self.core.profile))
                return (self._held if hold else self._submit)(f"move here (step {path_label(path)})",
                                                              lambda: self.core.move_joints(q, v, a))
            if st.type == "move_pose":
                pose = self.program.pose_target(st)
                self.core.geo.planning_target(pose)
                lin = bool(st.params.get("cartesian"))
                mm_s = st.params.get("speed_mm_s") if lin else None
                return (self._held if hold else self._submit)(f"move here (step {path_label(path)})",
                                                              lambda: self.core.move_pose(pose, lin, v, a, mm_s))
        except ProgramError as e:
            raise CommandError(str(e)) from None
        raise CommandError("only move steps have a position to move to")

    def cmd_program_rename_waypoint(self, old, new):
        """Rename a program waypoint; every step that uses it follows."""
        self._editable()
        new = str(new).strip()
        if old not in self.program.waypoints:
            raise CommandError(f"no waypoint '{old}' in this program")
        if not new or not all(ch.isalnum() or ch in "_-" for ch in new):
            raise CommandError("waypoint names use letters, digits, _ and - only")
        if new == old:
            return self._reply("unchanged")
        if new in self.program.waypoints:
            raise CommandError(f"there is already a waypoint '{new}'")
        self.program.waypoints = {(new if k == old else k): v for k, v in self.program.waypoints.items()}
        self.program.waypoints[new].name = new
        n = 0
        for s in self.program.all_steps():
            if s.params.get("target") == old:
                s.params["target"] = new
                n += 1
        return self._reply(f"waypoint {old} renamed to {new} ({n} step{'s' if n != 1 else ''})")

    def cmd_program_duplicate(self, path):
        self._editable()
        path = self._path(path)
        lst = self.program.siblings(path)
        lst.insert(path[-1] + 1, Step.from_dict(lst[path[-1]].to_dict()))
        return self._reply("step duplicated", path[:-1] + (path[-1] + 1,))

    def cmd_program_rename(self, name):
        self._editable()
        if not str(name).strip():
            raise CommandError("give the program a name")
        self.program.name = str(name).strip()
        return self._reply(f"renamed to {self.program.name}")

    def cmd_program_open(self, name, owner=""):
        self._editable()
        path = self._safe_path(name, owner)
        self.program = Program.load(path, self.core.profile)
        self.program_path = path
        self._mark_saved()
        self.core.bridge.log(f"program: opened {path.name}")
        for note in self.program.notes:
            self.core.bridge.log(f"program: {note}")
        return self._ok(f"opened {path.name}", **self.program_view())

    def cmd_program_set_yaml(self, text):
        """Replace the open program with edited YAML (validated by the same parser as files)."""
        self._editable()
        try:
            d = yaml.safe_load(text) or {}
        except yaml.YAMLError as e:
            raise CommandError(f"YAML: {e}")
        self.program = Program.from_dict(d, self.core.profile)
        return self._ok("program updated", **self.program_view())

    def cmd_program_save(self, name=None, owner=None):
        """owner: whose folder to save into ("" = Shared); default: where the open program is."""
        if owner is None:
            owner = self._program_owner()
        owner = self._own(owner)                    # nobody saves into another person's programs
        name = name or (self.program_path.name if self.program_path else None)
        if not name:
            raise CommandError("name the program")
        if "." in name and not name.endswith((".yaml", ".yml")):
            raise CommandError("program names are plain .yaml file names")
        path = self._safe_path(name if name.endswith((".yaml", ".yml")) else name + ".yaml", owner, create=True)
        self.program.name, self.program.robot = path.stem, self.core.profile.name
        self.program.save(path)
        self.program_path = path
        self._mark_saved()
        self.core.bridge.log(f"saved {path}")
        return self._ok(f"saved {path.name}", **self.program_view())

    def cmd_program_check(self):
        problems = self.core.runner.preflight(self.program)
        return self._ok("program OK" if not problems else "problems found", problems=problems)
    cmd_program_check.read_only = True

    def cmd_program_record(self, kind="pose", frame=None):
        """Teach: add a MovePose (current TCP pose in `frame`) or MoveJ (current joints) step at the end."""
        self._editable()
        s = self.core.last_snap
        if kind == "joints":
            if not s.joints or s.joint_state_age is None or s.joint_state_age > self.core.profile.joint_state_timeout_s:
                raise CommandError("no fresh joint state to record")
            name = self._new_wp("wp")
            self.program.waypoints[name] = ProgramWaypoint(name, joints=list(s.joints))
            self.program.steps.append(Step("movej", {"target": name, "velocity": 0.1}))
        else:
            frame = frame or self.core.profile.base_link
            pose = self.core.geo.current_pose(frame, self.core.store.active_tcp)
            name = self._new_wp("pose")
            self.program.waypoints[name] = ProgramWaypoint(name, pose=pose)
            self.program.steps.append(Step("move_pose", {"target": name, "velocity": 0.1}))
        return self._ok(f"recorded {name}", **self.program_view())

    def cmd_program_delete_step(self, path=None, index=None):
        self._editable()
        path = self._path(path, index)
        lst = self.program.siblings(path)
        del lst[path[-1]]
        sel = path if path[-1] < len(lst) else (path[:-1] + (path[-1] - 1,) if path[-1] > 0 else path[:-1])
        return self._reply("step deleted", sel or None)

    def cmd_program_move_step(self, path=None, delta=1, index=None):
        self._editable()
        path = self._path(path, index)
        lst = self.program.siblings(path)
        i, j = path[-1], path[-1] + int(delta)
        if not 0 <= j < len(lst):
            raise CommandError("cannot move further")
        lst[i], lst[j] = lst[j], lst[i]
        return self._reply("step moved", path[:-1] + (j,))

    def cmd_program_run(self, start=0, step_mode=False, loop=False):
        if self.teach is not None:
            raise CommandError("finish teaching first (Teach & more)")
        if not self.core.idle():
            raise CommandError("robot is busy")
        if self.core.modes.control_state_unknown:
            raise CommandError("control state is UNKNOWN: recover first")
        start = self._path(start) if isinstance(start, (list, tuple)) else int(start or 0)
        problems = self.core.runner.preflight(self.program, start)
        if problems:
            raise CommandError("preflight failed: " + "; ".join(problems))
        self.core.program_result = None
        if not self.core.runner.start(self.program, start, bool(loop), bool(step_mode)):
            if self.core.program_result is not None:            # preflight failed inside start()
                raise CommandError(self.core.program_result[1])
            raise CommandError("the previous run is still ending: try again")
        return self._ok("program started")

    def cmd_program_pause(self):
        self.core.runner.pause()
        return self._ok("paused")

    def cmd_program_resume(self, step_mode=False):
        why = self.core.robot_refusal()
        if why:
            raise CommandError(f"cannot resume: {why}")
        self.core._resume_after_robot = False
        self.core.runner.resume(bool(step_mode))
        return self._ok("resumed")

    def cmd_program_stop(self):
        self.core.note_stop(self._actor()["name"] or "the operator")
        self.core.batch.stop("stopped from PROGRAMS")       # a batch ends with its run
        self.core.runner.stop()
        return self._ok("stopping")

    def cmd_program_scrape(self, frame, policy, duration=8.0, step_deg=10.0, count=37, back_and_forth=False,
                           approach_mm=50.0):
        """New scraping program from the CURRENT TCP pose as the taught contact pose (see scrape_pattern)."""
        self._editable()
        m = self.core.policies.manifests.get(policy)
        if m is None:
            raise CommandError(f"unknown policy '{policy}'")
        modes = self.core.profile.modes
        pm = m.required_mode
        motion = pm if pm in modes and modes[pm].supports_moveit else (
            "position" if "position" in modes else self.core.profile.default_mode)
        start = self.core.geo.current_pose(frame, self.core.store.active_tcp)
        try:
            self.program = build(ScrapePattern(start=start, policy=policy, policy_mode=pm, motion_mode=motion,
                                               duration=float(duration), step_deg=float(step_deg), count=int(count),
                                               back_and_forth=bool(back_and_forth),
                                               approach_m=float(approach_mm) / 1000.0),
                                 f"scrape_{frame}", self.core.profile.name)
        except ScrapePatternError as e:
            raise CommandError(str(e))
        self.program_path = None
        return self._ok(f"generated {len(self.program.steps)} steps: check, then save", **self.program_view())

    def cmd_program_scrape_baseline(self, frame, press_axis="+z", force_n=5.0, contact_n=2.0, stroke_mm=15.0,
                                    strokes=2, step_deg=10.0, count=37, back_and_forth=False, approach_mm=50.0,
                                    standoff_mm=3.0, search_mm=10.0):
        """New scripted (no policy) scraping program from the CURRENT TCP pose as the taught contact pose."""
        self._editable()
        modes = self.core.profile.modes
        force_mode = next((n for n, m in modes.items() if m.wrench.get("topic") and m.supports_moveit), None)
        if force_mode is None:
            raise CommandError("no control mode of this robot takes both a force and MoveIt moves (Admittance)")
        motion = "position" if "position" in modes else self.core.profile.default_mode
        start = self.core.geo.current_pose(frame, self.core.store.active_tcp)
        try:
            self.program = build_baseline(ScrapeBaseline(
                start=start, force_mode=force_mode, motion_mode=motion, press_axis=str(press_axis),
                force_n=float(force_n), contact_n=float(contact_n), stroke_mm=float(stroke_mm), strokes=int(strokes),
                step_deg=float(step_deg), count=int(count), back_and_forth=bool(back_and_forth),
                approach_m=float(approach_mm) / 1000.0, standoff_mm=float(standoff_mm), search_mm=float(search_mm)),
                f"scrape_baseline_{frame}", self.core.profile.name, max_force_n=float(self.core.force_cfg["max_force_n"]))
        except ScrapePatternError as e:
            raise CommandError(str(e))
        self.program_path = None
        return self._ok(f"generated {len(self.program.steps)} steps: check, then save", **self.program_view())

    def _new_wp(self, prefix):
        k = 1
        while f"{prefix}{k}" in self.program.waypoints or f"{prefix}{k}" in self.core.profile.waypoints:
            k += 1
        return f"{prefix}{k}"

    def _safe_path(self, name: str, owner="", create=False) -> Path:
        if not name or "/" in name or "\\" in name or name.startswith("."):
            raise CommandError("program names are plain .yaml file names")
        d = self._programs_dir(owner, create).resolve()
        p = (d / name).resolve()                     # file names only: no paths from the browser
        if p.parent != d or p.suffix not in (".yaml", ".yml"):
            raise CommandError("program names are plain .yaml file names")
        return p

    def _rows(self) -> list:
        prof, wps = self.core.profile, self.program.waypoints
        rows = []
        for path, s in walk(self.program.steps):
            p = s.params
            wp = None
            if s.type in ("movej", "move_pose") and p.get("target"):
                w = wps.get(p["target"])
                if w is not None:
                    wp = {"name": w.name, "kind": "joints" if w.joints is not None else "pose",
                          "users": sum(1 for x in self.program.all_steps() if x.params.get("target") == w.name)}
                    if w.pose is not None:
                        wp.update(frame=w.pose.frame, tcp=w.pose.tcp)
                        try:
                            xyz, rpy = tf.T_to_mm_deg(w.pose.T())
                            wp.update(xyz_mm=[round(v, 1) for v in xyz], rpy_deg=[round(v, 1) for v in rpy])
                            bx, br = tf.T_to_mm_deg(self.core.geo.T_base_frame(w.pose.frame) @ w.pose.T())
                            wp.update(base_xyz_mm=[round(v, 1) for v in bx], base_rpy_deg=[round(v, 1) for v in br])
                        except Exception:                 # malformed pose: the row's problems say so
                            pass
                    else:
                        wp["joints_deg"] = [round(math.degrees(v), 2) for v in w.joints]
                elif p["target"] in prof.waypoints:
                    wp = {"name": p["target"], "kind": "joints", "profile": True}
            detail = s.summary()
            if s.type in ("movej", "move_pose"):
                v = f"  v={round(100 * float(p['velocity']))} %" if "velocity" in p else ""
                where = f" · {wp['frame']} / {wp['tcp']}" if wp and wp.get("frame") else ""
                detail = f"{p.get('target') or ('joints' if s.type == 'movej' else 'pose')}{where}{v}{s.speed_text()}"
            elif s.type == "controller":
                m = prof.modes.get(p.get("mode"))
                detail = m.label if m else detail
            try:
                problems = Program(steps=[Step(s.type, s.params, [] if s.container else s.children)],
                                   waypoints=wps).validate(prof)
                problems = [x.split(": ", 1)[-1] for x in problems if not (s.container and "empty" in x)]
                if s.container and not s.children:
                    problems.append(f"empty {s.type}: add steps into it")
            except Exception as e:
                problems = [str(e)]
            rows.append({"path": list(path), "label": path_label(path), "depth": len(path) - 1, "type": s.type,
                         "title": s.title, "detail": detail, "container": s.container,
                         "params": {k: v for k, v in p.items() if k != "points"}, "waypoint": wp, "problems": problems,
                         "sweep": sweep_options(s)})
        return rows

    def program_view(self) -> dict:
        steps = []
        for s in self.program.steps:
            detail = s.summary()
            if s.type == "move_pose":
                try:
                    pose = self.program.pose_target(s)
                    detail = (f"{s.params.get('target', '') + ': ' if 'target' in s.params else ''}"
                              f"{pose.frame} / {pose.tcp}{'  linear' if s.params.get('cartesian') else ''}"
                              f"{s.speed_text()}")
                except Exception:
                    pass
            elif s.type == "controller":
                m = self.core.profile.modes.get(s.params.get("mode"))
                detail = f"{m.label} · {m.command_interface.upper()}" if m else detail
            steps.append({"type": s.type, "detail": detail})
        return {"program": {"name": self.program.name, "file": self.program_path.name if self.program_path else None,
                            "owner": self._program_owner(),
                            "steps": steps, "rows": self._rows(), **self._history_view(), "rev": self.program_rev,
                            "yaml": yaml.safe_dump(self.program.to_dict(), sort_keys=False, default_flow_style=None)}}


    # ------------------------------------------------------ controller gains
    # SETUP > Controller gains. Reading is free; applying needs the lease and is checked / ramped by gains.py.
    def cmd_gains_get(self, mode=None, refresh=False):
        g = self.core.gains
        modes = [mode] if mode else list(g.sets)
        if refresh:
            for m in modes:
                g.read(m)
        out = g.view()
        out["yaml"] = {m: g.yaml_snippet(m) for m in modes if g.live.get(m)}
        out["current_mode"] = self.core.mode
        return self._ok(**out)
    cmd_gains_get.read_only = True

    def cmd_gains_check(self, mode, values):
        """What an apply would change (no robot effect): {old, new, warnings} or the refusal."""
        old, new, warn = self.core.gains.plan(mode, values)
        return self._ok(old=old, new=new, warnings=warn, yaml=self.core.gains.yaml_snippet(
            mode, {**(self.core.gains.live.get(mode) or {}), **new}))
    cmd_gains_check.read_only = True

    def cmd_gains_apply(self, mode, values):
        self.core.gains.plan(mode, values)              # refuse at once what the job would refuse
        epoch = getattr(self.core.bridge, "stop_epoch", 0)
        return self._submit(f"gains {mode}", lambda: self.core.gains.apply(
            mode, values, stopped=lambda: getattr(self.core.bridge, "stop_epoch", 0) != epoch))

    def cmd_gains_apply_policy(self, name):
        m = self.core.policies.manifests.get(name)
        if m is None or not m.controller_gains:
            raise CommandError(f"policy '{name}' declares no controller gains")
        values = self.core.gains.values_from_params(m.required_mode, m.controller_gains)
        return self.cmd_gains_apply(m.required_mode, values)

    def cmd_gains_forget(self, mode):
        return self._ok(self.core.gains.forget(mode)[1])

    def cmd_gains_restore(self, mode):
        """Put the saved values back: inactive at once, active ramped (same checks as any live apply)."""
        g = self.core.gains
        if not g.saved.get(mode):
            raise CommandError(f"nothing saved for {mode}")
        if self.core.mode != mode:
            return self._submit(f"restore {mode} gains", lambda: g.restore(mode, "by the operator"))
        g.read(mode)
        values = g.values_from_params(mode, g.saved[mode])
        extra = [k for k in g.saved[mode] if k not in {n for r in g.gain_set(mode).rows for n in r.names}
                 and k not in g.gain_set(mode).enable]
        if extra:
            raise CommandError(f"{', '.join(extra[:2])} can only be restored outside {mode} (e.g. the payload "
                               "compensation): switch mode first, it is restored on the way back")
        return self.cmd_gains_apply(mode, values)

    def cmd_gains_preset_save(self, name, mode, values):
        return self._ok(self.core.gains.save_preset(name, mode, values)[1], presets=self.core.gains.presets)

    def cmd_gains_preset_delete(self, name):
        return self._ok(self.core.gains.delete_preset(name)[1], presets=self.core.gains.presets)

    # ---------------------------------------------------------------- F/T bias
    # SETUP > F/T + Payload: software bias on the broadcaster (ft_bias.py). Reading is free; changing it needs
    # the lease, nothing using the reading, and is read back.
    def _ft_bias_ready(self):
        why = self.core.ft_bias.why_not()
        if why:
            raise CommandError(why)
        return self.core.ft_bias

    # F/T soft / hard limit (ft_guard.py)
    def cmd_ft_guard_get(self):
        return {"ok": True, **self.core.ft_guard.view()}
    cmd_ft_guard_get.read_only = True

    def cmd_ft_guard_save(self, enabled=False, soft_f=0, hard_f=0, soft_t=0, hard_t=0):
        from ..ft_guard import FtGuardError
        try:
            cfg = self.core.ft_guard.save({"enabled": enabled, "soft_f": soft_f, "hard_f": hard_f,
                                           "soft_t": soft_t, "hard_t": hard_t})
        except FtGuardError as e:
            raise CommandError(str(e)) from None
        lim = ", ".join(f"{k} {cfg[k]:g}" for k in ("soft_f", "hard_f", "soft_t", "hard_t") if cfg[k])
        self.core.bridge.log(f"F/T limit {'on' if cfg['enabled'] else 'off'}" + (f": {lim}" if lim else ""))
        return {"ok": True, "msg": "F/T limit saved", **self.core.ft_guard.view()}

    def cmd_ft_guard_reset_peak(self):
        self.core.ft_guard.reset_peak()
        return {"ok": True}

    def cmd_ft_bias_get(self, refresh=False):
        if refresh:
            self.core.ft_bias.read()
        return self._ok(ft_bias=self.core.ft_bias.view())
    cmd_ft_bias_get.read_only = True

    def cmd_ft_bias_zero(self, plain=False):
        b = self._ft_bias_ready()
        c = self.core
        still_v = c.profile.stationary_joint_velocity
        epoch = getattr(c.bridge, "stop_epoch", 0)

        def sample():
            s = c.bridge.snapshot()
            qd = s.joint_velocities
            still = qd is None or max((abs(v) for v in qd), default=0.0) <= still_v
            w = wrench_fresh(s)                      # a frozen sensor must not become the bias
            return (list(w) if w else None), still and not s.freedrive
        return self._submit("zero F/T", lambda: b.zero(
            sample, stopped=lambda: getattr(c.bridge, "stop_epoch", 0) != epoch, plain=bool(plain)))

    def cmd_ft_bias_set(self, bias):
        v = self.core.ft_bias.check(bias)
        b = self._ft_bias_ready()
        return self._submit("set F/T bias", lambda: b.set(v, "typed"))

    def cmd_ft_bias_clear(self):
        b = self._ft_bias_ready()
        return self._submit("clear F/T bias", b.clear)

    def cmd_ft_bias_use(self, name):
        p = self.core.ft_bias.presets.get(name)
        if p is None:
            raise CommandError(f"no F/T bias set named '{name}'")
        b = self._ft_bias_ready()
        return self._submit(f"F/T bias '{name}'", lambda: b.set(p["bias"], f"set '{name}'"))

    def cmd_ft_bias_preset_save(self, name, bias):
        return self._ok(self.core.ft_bias.save_preset(name, bias)[1], ft_bias=self.core.ft_bias.view())

    def cmd_ft_bias_preset_delete(self, name):
        return self._ok(self.core.ft_bias.delete_preset(name)[1], ft_bias=self.core.ft_bias.view())

    # ----------------------------------------------------------------- policy
    def cmd_policy_list(self):
        return self._ok(policies=[self._policy_info(m) for m in self.core.policies.manifests.values()],
                        errors=dict(self.core.policies.errors))

    @staticmethod
    def _policy_info(m):
        card = m.card() or {}
        return {"name": m.name, "description": m.description, "algorithm": m.algorithm or card.get("algorithm", ""),
                "backend": m.backend, "mode": m.required_mode, "tcp": m.required_tcp, "hz": m.control_hz,
                "duration": m.duration, "policy_card": bool(card)}
    cmd_policy_list.read_only = True

    def cmd_policy_sources(self, folder=""):
        lib = self.core.policies.library
        roots = [str(x) for x in lib.allowed_roots]
        if not folder:
            return self._ok(roots=roots, folder="", checkpoints=[])
        p = Path(folder).expanduser().resolve()
        if not policy_path_allowed(p, lib.allowed_roots):
            raise CommandError("that folder is outside the allowed policy roots")
        try:
            checkpoints = [str(x) for x in list_policy_checkpoints(p, lib.card_command)]
            info = []
            for checkpoint in checkpoints:
                q = Path(checkpoint)
                try:
                    st = q.stat()
                    try:
                        rel = str(q.relative_to(p))
                    except ValueError:
                        rel = q.name
                    info.append({"path": checkpoint, "name": q.name, "rel": rel, "size": st.st_size, "mtime": st.st_mtime})
                except OSError:
                    continue
            return self._ok(roots=roots, folder=str(p), checkpoints=checkpoints, checkpoint_info=info)
        except PolicyImportError as e:
            return self._ok(roots=roots, folder=str(p), checkpoints=[], error=str(e))

    cmd_policy_sources.read_only = True               # lists files only; still admin-only

    def cmd_policy_import(self, folder, checkpoint, name, description="", backend="", required_mode="",
                          required_tcp="", overwrite=False, no_normalization=False):
        lib = self.core.policies.library
        folder = str(Path(folder).expanduser().resolve())
        checkpoint = str(Path(checkpoint).expanduser())
        def job():
            try:
                out = lib.import_policy(folder, checkpoint, name, description, backend, required_mode, required_tcp,
                                        overwrite=bool(overwrite), no_normalization=bool(no_normalization))
            except PolicyImportError as e:
                return False, str(e)
            return True, out["msg"]
        return self._submit("policy import", job)

    def cmd_policy_upload(self, folder, files):
        lib = self.core.policies.library

        def job():
            decoded = []
            for item in files or []:
                if not isinstance(item, dict):
                    return False, "uploaded file entry is invalid"
                try:
                    data = base64.b64decode(str(item.get("data_b64", "")), validate=True)
                except Exception:
                    return False, "uploaded file is not base64"
                decoded.append((str(item.get("path", "")), data))
            try:
                saved = lib.save_uploaded_folder(str(folder), decoded)
            except PolicyImportError as e:
                return False, str(e)
            return True, f"uploaded policy folder {saved.name}"

        return self._submit("policy upload", job)

    def cmd_policy_rename(self, name, new_name=None, description=None, overwrite=False):
        lib = self.core.policies.library
        if name in (self.prepared_policy, self.core.policies.current):
            raise CommandError(f"'{name}' is prepared or running: stop it first")
        try:
            out = lib.edit_manifest(name, new_name, description, overwrite=bool(overwrite))
        except PolicyImportError as e:
            raise CommandError(str(e)) from None
        return self._ok(out["msg"], **out)

    def cmd_policy_requirements(self, name):
        return self._ok(requirements=[{"label": r.label, "state": r.state, "detail": r.detail,
                                       "fix": self._requirement_fix(name, r) if r.state in ("fail", "warn") else None}
                                      for r in self.core.policies.requirements(name)])

    def _requirement_fix(self, name, r) -> Optional[dict]:
        """How the browser can fix a failed readiness check: a command to send, or a page to open."""
        m = self.core.policies.manifests.get(name)
        if m is None:
            return None
        if m.required_tcp and r.label == f"TCP {m.required_tcp}" and m.required_tcp in self.core.store.tcps:
            return {"label": f"Use {m.required_tcp}", "cmd": "set_tcp", "args": {"name": m.required_tcp}}
        mode = self.core.profile.modes.get(m.required_mode)
        if mode and r.label == mode.label:
            return {"label": f"Switch to {mode.label}", "cmd": "switch_mode", "args": {"mode": m.required_mode},
                    "confirm": f"Switch the robot to {mode.label}?" +
                               (" This commands DIRECT TORQUE." if mode.direct_torque else "")}
        if r.label.startswith("Frame "):
            return {"label": "Open SETUP", "page": "SETUP"}
        if r.label == "F/T axes":
            return {"label": "Open SETUP", "page": "SETUP", "hint": "SETUP › F/T + payload › Match simulation"}
        if r.label == "Controller gains" and m.controller_gains:
            return {"label": "Apply trained gains", "cmd": "gains_apply_policy", "args": {"name": name},
                    "confirm": f"Set the {m.required_mode} controller to the gains {name} was trained with? "
                               "Ramped if the mode is active."}
        if r.label in ("ATI Gamma", "MoveIt Servo"):
            return {"label": "Open DIAGNOSTICS", "page": "DIAGNOSTICS",
                    "hint": "MoveIt Servo starts with ~/ur7e_gui.sh -- servo:=true" if r.label == "MoveIt Servo" else ""}
        return None
    cmd_policy_requirements.read_only = True

    def cmd_policy_prepare(self, name, recording=None):
        if name not in self.core.policies.manifests:
            raise CommandError(f"unknown policy '{name}'")
        self.prepared_policy = None
        goal = self.core.policies.goal(name, recording=_recording(recording))
        done = threading.Event()
        out = {}

        def finished(res):
            out["res"] = res
            done.set()

        def job():
            self.core.policies.prepare(goal, finished)
            done.wait(120)
            res = out.get("res")
            if res is None:
                return False, "prepare timed out"
            self.prepared_policy = name if res.success else None
            return res.success, "ready" if res.success else res.stop_reason
        return self._submit(f"prepare {name}", job)

    def cmd_policy_start(self, name, duration=None, recording=None):
        """Prepare and start through PolicyExecutor for the OPERATE policy window."""
        if name not in self.core.policies.manifests:
            raise CommandError(f"unknown policy '{name}'")
        if not self.core.idle():
            raise CommandError("robot is busy")
        goal = self.core.policies.goal(name, duration=float(duration) if duration not in (None, "") else None,
                                       recording=_recording(recording))
        launch_epoch = getattr(self.core.bridge, "stop_epoch", 0)
        launch_generation = self.core.policies.cancel_generation
        caller_token = getattr(self._command_context, "token", None)
        with self._lease_lock:
            launch_lease = self.lease
            launch_lease_generation = launch_lease.generation
            launch_token = caller_token if caller_token is not None else launch_lease.token
        self._policy_chart_policy = name
        self._policy_last_step = None

        def job():
            done, out = threading.Event(), {}
            if not self.core.policies.prepare(goal, lambda res: (out.setdefault("prepare", res), done.set())):
                return False, out.get("prepare", RunPolicyResult(False, "preflight failed")).stop_reason
            if not done.wait(120):
                return False, "prepare timed out"
            prepared = out["prepare"]
            if not prepared.success:
                return False, prepared.stop_reason
            if getattr(self.core.bridge, "stop_epoch", 0) != launch_epoch:
                return False, "not started: STOP was pressed during prepare"
            if not self._job_lease_still_matches(launch_lease, launch_lease_generation, launch_token):
                return False, "not started: control changed during prepare"
            if self._needs_switch():
                return False, "not started: settings changed during prepare; switch to your settings first"
            self.core.policy_result, self.core.policy_feedback = None, None
            self.core.command_source = f"policy {name}"
            if not self.core.policies.start(goal, expected_generation=launch_generation):
                self.core.command_source = "idle"
                return False, self.core.policy_result.stop_reason if self.core.policy_result else "not started"
            return True, f"{name} started"
        return self._submit(f"start policy {name}", job)

    def cmd_policy_run(self, name, recording=None):
        if self.prepared_policy != name:
            raise CommandError("prepare this policy first")
        if not self.core.idle():
            raise CommandError("robot is busy")
        self._policy_chart_policy = name
        self._policy_last_step = None
        self.core.policy_result, self.core.policy_feedback = None, None
        self.core.command_source = f"policy {name}"
        if not self.core.policies.start(self.core.policies.goal(name, recording=_recording(recording)),
                                        lambda _r: setattr(self, "prepared_policy", None)):
            self.core.command_source = "idle"
            raise CommandError(self.core.policy_result.stop_reason if self.core.policy_result else "not started")
        return self._ok(f"{name} started")

    def cmd_policy_stop(self):
        self.core.note_stop(self._actor()["name"] or "the operator")
        self.core.policies.cancel()
        return self._ok("stopping policy")
    cmd_policy_stop.read_only = True

    # ------------------------------------------------------ keep-out zones
    # Boxes MoveIt must plan around (planning-scene collision objects `gui_keepout/<name>`). Editing changes what
    # MoveIt plans, never moves the robot; it is refused while anything runs, and a box around the current TCP,
    # tool flange or base is refused (MoveIt would start in collision and every plan would fail).
    def _keepout_editable(self):
        if not self.core.idle():
            raise CommandError("the robot is busy: change keep-out zones while idle")

    def _keepout_apply_job(self, why: str) -> dict:
        return self._submit("keep-out zones", lambda: self.core.keepout.apply(why))

    def cmd_keepout_save(self, name, frame, size_mm, xyz_mm, rpy_deg, enabled=True, replace=None):
        """Create a box, or (replace=<old name>) change / rename one. Then apply the scene in the background."""
        self._keepout_editable()
        box = make_box(name, frame, size_mm, xyz_mm, rpy_deg, bool(enabled))
        warn = self.core.keepout.check_edit(box)          # KeepoutError (a CalibrationError) -> CommandError
        old = self.core.keepout_store.boxes.get(replace) if replace else None
        self.core.keepout_store.put(box, replace=replace or None)
        what = "edited" + (f" (was {replace})" if replace and replace != box.name else "") if old else "created"
        self.core.bridge.log(f"keep-out box {box.name} {what}: {box.size_mm} mm at {box.xyz_mm} mm, {box.rpy_deg}° "
                             f"in {box.frame}{'' if box.enabled else ' (disabled)'}")
        self._keepout_apply_job(f"{box.name} {what.split(' ')[0]}")
        return self._ok(f"keep-out box {box.name} saved" + (": " + "; ".join(warn) if warn else ""), warnings=warn)

    def cmd_keepout_enable(self, name, enabled):
        self._keepout_editable()
        st = self.core.keepout_store
        if name not in st.boxes:
            raise CommandError(f"no keep-out box '{name}'")
        b = st.boxes[name]
        new = make_box(b.name, b.frame, b.size_mm, b.xyz_mm, b.rpy_deg, bool(enabled), b.meta)
        warn = self.core.keepout.check_edit(new)
        st.put(new, replace=name)
        self.core.bridge.log(f"keep-out box {name} {'enabled' if new.enabled else 'disabled'}")
        self._keepout_apply_job(f"{name} {'enabled' if new.enabled else 'disabled'}")
        return self._ok(f"keep-out box {name} {'enabled' if new.enabled else 'disabled'}", warnings=warn)

    def cmd_keepout_delete(self, name):
        self._keepout_editable()
        self.core.keepout_store.delete(str(name))
        self.core.bridge.log(f"keep-out box {name} deleted")
        self._keepout_apply_job(f"{name} deleted")
        return self._ok(f"keep-out box {name} deleted")

    def cmd_keepout_apply(self):
        """Send every enabled box to MoveIt again (e.g. after restarting move_group by hand)."""
        self._keepout_editable()
        return self._keepout_apply_job("re-apply")

    def cmd_keepout_from_tcp(self, frame, size_mm):
        """Form helper: a box centred under the TCP, its top face KEEPOUT_TCP_CLEARANCE_MM below it, axes of
        `frame`. Touch a surface (table, holder top) with the TCP, then use this. Nothing is saved."""
        try:
            size = [float(v) for v in size_mm]
        except (TypeError, ValueError):
            raise CommandError("size must be three numbers (mm)") from None
        if len(size) != 3 or min(size) <= 0:
            raise CommandError("size must be three positive numbers (mm)")
        if not (frame == self.core.profile.base_link or frame in self.core.store.fixtures):
            raise CommandError(f"'{frame}' is not the base frame or a fixture")
        T = self.core.geo.current_pose(frame, self.core.store.active_tcp).T()
        c = T[:3, 3] * 1000.0 - np.array([0.0, 0.0, size[2] / 2 + KEEPOUT_TCP_CLEARANCE_MM])
        return self._ok(xyz_mm=[round(float(v), 1) for v in c], rpy_deg=[0.0, 0.0, 0.0])
    cmd_keepout_from_tcp.read_only = True

    def keepout_view(self) -> dict:
        p = self.core.profile
        return {**self.core.keepout.view(), "frames": [p.base_link] + list(self.core.store.fixtures)}

    # ------------------------------------------------------------ vial setup
    def cmd_vial_capture(self, point):
        if point not in VIAL_POINTS:
            raise CommandError(f"point must be one of {', '.join(VIAL_POINTS)}")
        T = self.core.geo.current_pose(self.core.profile.base_link, self.core.store.active_tcp).T()
        self.vial_points[point] = [float(v) for v in T[:3, 3]]
        return self._ok(f"captured {point}", vial=self.vial_view())

    def cmd_vial_reset(self):
        self.vial_points.clear()
        return self._ok("cleared", vial=self.vial_view())

    def cmd_vial_save(self, name="vial"):
        g = self._vial_fit()
        if g is None:
            raise CommandError("capture three rim points first")
        T = np.eye(4)
        T[:3, 3] = [g.center_xy[0], g.center_xy[1], g.bottom_z]
        store = self.core.store
        store.put_fixture(FixtureDef(name, self.core.profile.base_link, T, g.to_meta()), replace=name in store.fixtures)
        self.core.bridge.log(f"saved fixture {name}")
        return self._ok(f"saved fixture {name}")

    def cmd_vial_use_example(self, name="vial"):
        """Save the profile's example vial (scraping.py's target_xyz) as the real fixture, on request."""
        e = (self.core.profile.example_fixtures or {}).get(name)
        if not e:
            raise CommandError(f"no example '{name}' in this profile")
        T = tf.mm_deg_to_T(list(e.get("xyz_mm", [0, 0, 0])), list(e.get("rpy_deg", [0, 0, 0])))
        meta = {"source": f"example: {e.get('source', 'profile')}"}
        if e.get("radius_mm"):
            meta["radius_m"] = float(e["radius_mm"]) / 1000.0
        if e.get("height_mm"):
            meta["height_m"] = float(e["height_mm"]) / 1000.0
        store = self.core.store
        store.put_fixture(FixtureDef(name, e.get("parent", self.core.profile.base_link), T, meta),
                          replace=name in store.fixtures)
        self.core.bridge.log(f"saved fixture {name} from the profile example ({e.get('source', '')}): "
                             f"{e.get('xyz_mm')} mm in {e.get('parent', 'base_link')}")
        return self._ok(f"saved {name} at the example position {e.get('xyz_mm')} mm: check it matches your vial")

    # ------------------------------------------------------------ vial profiles
    def _vial_put(self, fixture: str, name: str, e: dict) -> None:
        fixture = str(fixture or "vial").strip()
        if not fixture:
            raise CommandError("give the fixture name to use it as (e.g. vial)")
        T = tf.mm_deg_to_T(list(e["xyz_mm"]), [0, 0, 0])
        meta = {"vial_profile": name, "source": f"vial profile {name}: {e.get('source', '')}"}
        if e.get("radius_mm"):
            meta["radius_m"] = float(e["radius_mm"]) / 1000.0
        if e.get("height_mm"):
            meta["height_m"] = float(e["height_mm"]) / 1000.0
        store = self.core.store
        try:
            store.put_fixture(FixtureDef(fixture, self.core.profile.base_link, T, meta), replace=fixture in store.fixtures)
        except CalibrationError as ex:
            raise CommandError(str(ex)) from None
        self.core.bridge.log(f"fixture {fixture} = vial profile {name}: {e['xyz_mm']} mm in {self.core.profile.base_link}")

    def cmd_vial_profiles(self):
        return self._ok(vials=self.vials.view(self.core.store.fixtures))
    cmd_vial_profiles.read_only = True

    def cmd_vial_profile_use(self, name, fixture="vial"):
        """Make a vial profile the fixture programs use (default "vial"). Refused while a program runs."""
        if not self.core.idle() or self.core.runner.state.name != "IDLE":
            raise CommandError("the robot is busy: change the vial while idle")
        try:
            e = self.vials.get(name)
        except VialProfileError as ex:
            raise CommandError(str(ex)) from None
        self._vial_put(fixture, name, e)
        return self._ok(f"{fixture} is now vial profile {name} ({', '.join(f'{v:g}' for v in e['xyz_mm'])} mm)",
                        vials=self.vials.view(self.core.store.fixtures))

    def cmd_vial_profile_save(self, name, xyz_mm=None, radius_mm=None, height_mm=None, source="touch",
                              fixture="vial", use=False):
        """Save a vial profile: from the touched points (source touch), from a fixture (source fixture) or typed
        numbers (xyz_mm, radius_mm, height_mm). use=True also makes it `fixture` at once."""
        try:
            if source == "touch":
                g = self._vial_fit()
                if g is None:
                    raise CommandError("capture three rim points first")
                rim_z = min(self.vial_points[n][2] for n in VIAL_POINTS[:3] if n in self.vial_points)
                h = (rim_z - g.bottom_z) * 1000.0 if g.bottom_source == "touched" else height_mm
                e = self.vials.save(name, [g.center_xy[0] * 1000.0, g.center_xy[1] * 1000.0, g.bottom_z * 1000.0],
                                    g.radius * 1000.0, h or None, "touched rim" + (" + bottom" if g.bottom_source == "touched" else ""))
            elif source == "fixture":
                fx = self.core.store.fixtures.get(fixture)
                if fx is None or fx.parent != self.core.profile.base_link:
                    raise CommandError(f"no fixture '{fixture}' in {self.core.profile.base_link}")
                r = fx.meta.get("radius_m")
                hh = fx.meta.get("height_m")
                e = self.vials.save(name, [float(v) * 1000.0 for v in fx.T_parent_fixture[:3, 3]],
                                    r * 1000.0 if r else radius_mm, hh * 1000.0 if hh else height_mm,
                                    f"from fixture {fixture}")
            else:
                if xyz_mm is None:
                    raise CommandError("give the centre (x, y, z mm)")
                e = self.vials.save(name, xyz_mm, radius_mm, height_mm, "typed")
        except VialProfileError as ex:
            raise CommandError(str(ex)) from None
        except (TypeError, ValueError):
            raise CommandError("bad number in the vial profile") from None
        if use:
            self._vial_put(fixture, str(name).strip(), e)
        return self._ok(f"vial profile {str(name).strip()} saved" + (f" and used as {fixture}" if use else ""),
                        vials=self.vials.view(self.core.store.fixtures))

    def cmd_vial_profile_delete(self, name):
        try:
            self.vials.delete(name)
        except VialProfileError as ex:
            raise CommandError(str(ex)) from None
        return self._ok(f"vial profile {name} deleted (fixtures made from it stay)",
                        vials=self.vials.view(self.core.store.fixtures))

    def _vial_fit(self):
        rim = [self.vial_points[n] for n in VIAL_POINTS[:3] if n in self.vial_points]
        if len(rim) < 3:
            return None
        return estimate_vial(rim, self.vial_points.get("Bottom"))

    def vial_view(self) -> dict:
        out = {"points": self.vial_points, "result": None, "error": "",
               "profiles": self.vials.view(self.core.store.fixtures)}
        try:
            g = self._vial_fit()
            if g:
                out["result"] = {"cx": g.center_xy[0], "cy": g.center_xy[1], "r": g.radius, "z": g.bottom_z,
                                 "spread": g.rim_z_spread}
        except VialGeometryError as e:
            out["error"] = str(e)
        return out

    def cmd_vial_get(self):
        return self._ok(vial=self.vial_view())
    cmd_vial_get.read_only = True

    # ---------------------------------------------------------------- payload
    def cmd_payload_collect(self, index=None):
        """Move to one calibration pose and sample it (index), or every pose in turn (no index). A single pose
        does not fit anything: Evaluate does, from the poses collected so far."""
        cal = self.payload
        if cal is None:
            raise CommandError(self.payload_progress["error"] or "this profile has no F/T calibration poses")
        if self.payload_progress["running"] or not self.core.idle():
            raise CommandError("robot is busy")
        if not self.core.modes.mode_supports_moveit(self.core.mode):
            raise CommandError(f"MoveIt motion is not available in '{self.core.mode}'. Change to Position first.")
        idx = list(range(len(cal.poses))) if index is None else [int(index)]
        if any(not 0 <= k < len(cal.poses) for k in idx):
            raise CommandError(f"no calibration pose {index}")
        self.core.ft_bias.read()
        live = list(self.core.ft_bias.live) if self.core.ft_bias.live is not None else None
        fresh = index is None or not any(cal.samples)
        if not fresh and getattr(self, "payload_bias", None) is not None and live is not None and any(
                abs(a - b) > 1e-6 for a, b in zip(self.payload_bias, live)):
            raise CommandError("the F/T bias changed since the first pose was collected: the poses would not fit "
                               "together. Press Start over and collect them again")
        why = self.core.begin_motion("F/T + payload")
        if why:
            raise CommandError(why)
        if fresh:
            cal.reset()
            self.payload_bias = live
        if not hasattr(cal, "base_velocity"):
            cal.base_velocity = cal.velocity
        cal.velocity = max(0.01, cal.base_velocity * self.core.speed_override)   # GUI SPEED applies here too
        cal.result = None
        self.payload_applied = False
        self.payload_progress.update(running=True, done=0, total=len(idx), error="", pose=idx[0])

        def job():
            try:
                for n, k in enumerate(idx):
                    self.payload_progress["pose"] = k
                    cal.run_pose(k)
                    self.payload_progress["done"] = n + 1
                if index is not None:
                    have = sum(1 for x in cal.samples if x is not None)
                    return True, (f"{cal.poses[idx[0]]['name']} collected ({have} of {len(cal.poses)}). " +
                                  ("Evaluate when you have the poses you want." if have >= 3 else
                                   "Evaluate needs at least 3."))
                r = cal.evaluate()
                return True, f"{'PASS' if r.passed else 'FAIL'}: {r.mass:.3f} kg"
            except PayloadCalibrationError as e:
                self.payload_progress["error"] = str(e)
                return False, str(e)
            finally:
                self.payload_progress["running"] = False
                self.core.end_motion()
        return self._submit("F/T + payload", job)

    def cmd_payload_evaluate(self):
        cal = self.payload
        if cal is None or self.payload_progress["running"]:
            raise CommandError("nothing to evaluate" if cal is None else "a pose is being collected")
        try:
            r = cal.evaluate()
        except PayloadCalibrationError as e:
            raise CommandError(str(e)) from None
        self.payload_applied = False
        return self._ok(f"{'PASS' if r.passed else 'FAIL'}: {r.mass:.3f} kg from "
                        f"{len(r.samples)} pose(s)")

    def cmd_payload_reset(self):
        if self.payload is None or self.payload_progress["running"]:
            raise CommandError("a pose is being collected" if self.payload else "no F/T calibration in this profile")
        self.payload.reset()
        self.payload_bias = None
        self.payload_applied = False
        self.payload_progress.update(done=0, total=0, error="")
        return self._ok("calibration samples cleared")

    def _payload_accept(self, accept_failed: bool):
        r = self.payload.result
        if not r.passed and accept_failed and not r.accepted:
            r.accepted = True
            issues = "; ".join(str(x) for x in r.quality.get("issues", []))
            self.core.bridge.log(f"payload: result used although checks failed: {issues}")

    def cmd_payload_save(self, accept_failed: bool = False):
        if not self.payload or not self.payload.result:
            raise CommandError("nothing to save")
        self._payload_accept(bool(accept_failed))
        return self._ok(f"saved {self.payload.save()}")

    def cmd_payload_apply(self, accept_failed: bool = False):
        cal = self.payload
        if not cal or not cal.result:
            raise CommandError("nothing to apply")
        if not cal.result.usable and not accept_failed:
            raise CommandError("the fit did not pass its checks: confirm to use it anyway")
        self._payload_accept(bool(accept_failed))
        if accept_failed and cal.saved_path is not None:
            cal.save()                                    # the file says it was accepted
        if (self.payload_view() or {}).get("result", {}).get("bias_changed"):
            raise CommandError("the F/T bias changed since these poses were collected: collect them again "
                               "(or set the bias back) before applying")
        if cal.saved_path is None:
            cal.save()
        def job():
            ok, msg = cal.apply_to_admittance(self.core.mode)
            self.payload_applied = self.payload_applied or ok
            return ok, msg
        return self._submit("apply to admittance", job)

    def _payload_saved(self, cal) -> dict:
        """What is kept across restarts: the payload the GUI restores on the controller (gains store), the last
        calibration file, the last F/T bias. Shown in SETUP > F/T + Payload even when nothing was evaluated now."""
        g, mode = self.core.gains, cal.apply_mode
        sv = (g.saved.get(mode) or {}) if mode else {}
        ctrl = None
        if "gravity_compensation.CoG.force" in sv:
            force = float(sv["gravity_compensation.CoG.force"])
            ctrl = {"mass": force / 9.80665, "force": force, "cog": [float(v) for v in sv.get("gravity_compensation.CoG.pos", [0, 0, 0])],
                    "frame": sv.get("gravity_compensation.frame.id", ""), "at": g.saved_at.get(mode, ""),
                    "node": cal.apply_node, "differs": g.unsynced.get(mode, "")}
        d = Path(cal_dir) if (cal_dir := getattr(cal, "save_dir", None)) else config_dir() / "ft_payload"
        try:
            files = sorted(d.glob(f"{self.core.profile.name}_*.json"))
            key = (str(files[-1]), files[-1].stat().st_mtime, len(files)) if files else None
        except OSError:
            files, key = [], None
        if key != getattr(self, "_pfile_key", "x"):
            self._pfile_key, self._pfile = key, None
            if files:
                try:
                    j = json.loads(files[-1].read_text())
                    stamp = files[-1].stem.rsplit("_", 2)
                    at = f"{stamp[-2][:4]}-{stamp[-2][4:6]}-{stamp[-2][6:]} {stamp[-1][:2]}:{stamp[-1][2:4]}" \
                        if len(stamp) == 3 and len(stamp[-2]) == 8 else ""
                    self._pfile = {"name": files[-1].name, "path": str(files[-1]), "count": len(files), "at": at,
                                   "mass": j.get("mass_kg"), "cog": j.get("cog_m"), "passed": j.get("passed"),
                                   "accepted": bool(j.get("accepted_despite_failed_checks")),
                                   "poses": len(j.get("samples") or []),
                                   "applied": bool(j.get("applied_to_controller"))}
                except Exception as e:
                    self._pfile = {"name": files[-1].name, "path": str(files[-1]), "count": len(files),
                                   "error": f"unreadable: {e}"}
        b = self.core.ft_bias
        bias = None
        if b.available and b.last:
            bias = {**{k: b.last.get(k) for k in ("bias", "how", "at", "restored")},
                    "in_use": b.live is not None and not b.lost()}
        return {"controller": ctrl, "file": self._pfile, "bias": bias}

    def payload_view(self) -> Optional[dict]:
        cal = self.payload
        if cal is None:
            return None
        r = cal.result
        out = {"poses": [p["name"] for p in cal.poses], "joints": [list(p["joints"]) for p in cal.poses],
               **self.payload_progress,
               "samples": [s.mean_wrench[:3] if s else None for s in cal.samples], "result": None,
               "saved": self._payload_saved(cal)}
        if r:
            bias = r.info.get("residual_bias")
            out["result"] = {"passed": r.passed, "accepted": r.accepted, "mass": r.mass, "cog": r.cog,
                             "issues": [str(x) for x in list(r.quality.get("issues", [])) +
                                        list(r.quality.get("warnings", []))],
                             "yaml": cal.admittance_yaml(), "applied": self.payload_applied,
                             "offset_n": float(np.linalg.norm(np.asarray(bias[:3], float)))
                             if bias is not None and len(bias) >= 3 else None,
                             "can_apply": bool(cal.apply_node),           # a failed fit asks first
                             # the fit holds for the bias it was collected with: a later zero / clear breaks it
                             "bias_changed": bool(getattr(self, "payload_bias", None) is not None
                                                  and self.core.ft_bias.live is not None
                                                  and any(abs(a - b) > 1e-3 for a, b in
                                                          zip(self.payload_bias, self.core.ft_bias.live)))}
        return out

    # ---------------------------------------------------------------- cameras
    def cmd_camera_status(self):
        b = self.core.bridge
        try:
            topics = b.image_topics()
        except Exception as e:
            topics, err = [], str(e)
        else:
            err = ""
        cams = []
        for r in self.drivers.view()["services"]:              # the image outputs of SETUP > Drivers & cameras
            info = next((o["topic"] for o in r["outputs"] if o["topic"].endswith("camera_info")), "")
            cams += [{"name": f"{r['label']} {o['label']}", "topic": o["topic"], "info": info, "driver": r["name"]}
                     for o in r["outputs"] if o["image"]]
        cams += [c for c in self.core.profile.cameras if all(c["topic"] != d["topic"] for d in cams)]   # the profile's
        # the same picture as JPEG (image_transport's <topic>/compressed), where the driver offers it: the GUI passes
        # it to the browser as it is, no decoding or re-encoding, and far less data than the raw picture
        have = {t for t, _ in topics}
        ordered = []
        for c in cams:
            if c["topic"] + "/compressed" in have and "depth" not in c["topic"]:
                ordered.append({**c, "name": c["name"] + " (compressed)",
                                "topic": c["topic"] + "/compressed", "compressed": True})
            ordered.append(c)
        cams = ordered
        return self._ok(error=err, topics=[t for t, _ in topics],
                        cameras=[{**c, "publishers": b.topic_publishers(c["topic"])} for c in cams])
    cmd_camera_status.read_only = True

    # =================================================================== views
    def info(self) -> dict:
        c, p = self.core, self.core.profile
        base = self.running_base()
        no_mode = {n: self.base_lacks(mode=n) for n in p.modes}
        return {
            "base": {"name": base, "label": B.BASES[base][0]},
            "robot": p.display_name, "profile": p.name, "ur_type": p.ur_type, "robot_ip": p.robot_ip,
            "fake": c.fake, "remote_power": remote_power(), "base_link": p.base_link, "joint_names": p.joint_names,
            "max_cart_speed_mm_s": p.max_cart_speed_mm_s,
            "modes": [{"name": n, "label": m.label, "interface": m.command_interface, "direct_torque": m.direct_torque,
                       "experimental": m.experimental, "moveit": m.supports_moveit,
                       "allowed": m.allow_activation and not no_mode[n],
                       "hidden": getattr(m, "hidden", False), "requires_ft": m.requires_ft,
                       "disabled_reason": no_mode[n] or getattr(m, "disabled_reason", "")}
                      for n, m in p.modes.items()],
            "waypoints": [{"name": n, "joints_deg": [round(math.degrees(v), 3) for v in w.joints]}
                          for n, w in p.waypoints.items() if w.joints is not None],
            "gripper": {**{k: p.gripper.get(k) for k in ("enabled", "model", "open_position", "closed_position",
                                                           "max_effort", "effort_limit")},
                        **({"enabled": False} if self.base_lacks(gripper=True) else {}),
                        "can_reactivate": bool(p.gripper.get("reactivate_service")),
                        "can_reconnect": bool(p.gripper.get("controller_manager"))},
            "cameras": p.cameras,
            "policies": [self._policy_info(m) for m in c.policies.manifests.values()],
            "vial_points": list(VIAL_POINTS),
            "jog": {"step_mm": c.jog_cfg["step_mm"], "step_deg": c.jog_cfg["step_deg"]},
            "force": {k: c.force_cfg[k] for k in ("max_force_n", "max_torque_nm", "ramp_force_nps",
                                                  "ramp_torque_nmps", "max_duration_s")},
            "force_modes": [n for n, m in p.modes.items() if m.wrench.get("topic")],
            "ft_frame": p.ft_calibration.get("sensor_frame"),
            "joint_limits_deg": self.joint_limits_deg(), "example_fixtures": p.example_fixtures,
        }

    def joint_limits_deg(self) -> list:
        """[[lower, upper] | None] per arm joint, in degrees, from the URDF (None = unknown)."""
        m = self.robot_model()
        by_name = {j["name"]: j for j in m.get("joints", [])} if m.get("ok") else {}
        out = []
        for n in self.core.profile.joint_names:
            lim = (by_name.get(n) or {}).get("limit")
            out.append([round(math.degrees(lim["lower"]), 2), round(math.degrees(lim["upper"]), 2)] if lim else None)
        return out

    def calibration_view(self) -> dict:
        st = self.core.store
        def tcp_view(n, t):
            d = {"name": n, "builtin": t.builtin}
            if not t.builtin and t.T_parent_tcp is not None:
                xyz, rpy = tf.T_to_mm_deg(t.T_parent_tcp)
                d.update(parent=t.parent, xyz_mm=[round(v, 2) for v in xyz], rpy_deg=[round(v, 2) for v in rpy],
                         method=t.meta.get("method", ""))
            return d
        return {"tcps": [tcp_view(n, t) for n, t in st.tcps.items()], "tcp_parents": self._tcp_parents(),
                "active_tcp": st.active_tcp, "frames": st.frame_names(),
                "fixtures": [n for n in st.fixtures]}

    TCP_HOLD_S = 2.0                     # the TCP pose shown while TF is late, at most this long
    JOINTS_FRESH_S = 0.5                 # joint states older than this are not used for the TCP pose

    def _tcp_from_joints(self):
        """T_base_tcp from the latest joint states and the robot model (display only), None if it can't."""
        c = self.core
        snap = c.last_snap
        q = snap.joints
        if not q or snap.joint_state_age is None or snap.joint_state_age > self.JOINTS_FRESH_S:
            return None
        try:
            link, _, T_link_tcp = c.geo.tcp_anchor(c.store.active_tcp)
            m = self._base_model()
            if not m.get("ok"):
                return None
            T = fk(m, dict(zip(c.profile.joint_names, q)), c.profile.base_link, link)
            if T is None:
                T = fk(self.robot_model(), dict(zip(c.profile.joint_names, q)), c.profile.base_link, link)
            return None if T is None else T @ T_link_tcp
        except Exception:
            return None


    def state(self) -> dict:
        c = self.core
        s = c.last_snap
        tcp = None
        try:
            T = c.geo.current_pose(c.profile.base_link, c.store.active_tcp).T()
            xyz, rpy = tf.T_to_mm_deg(T)
            tcp = {"xyz_mm": [round(v, 2) for v in xyz], "rpy_deg": [round(v, 2) for v in rpy]}
            self._tcp_last = (time.monotonic(), c.store.active_tcp, tcp)
        except Exception:
            # TF a moment late (a loaded PC): show the last pose, marked stale, rather than blanking the view.
            # Display only: moves and Record pose still need fresh TF.
            # First the joints through the robot model (the URDF robot_state_publisher uses too: the same pose,
            # without waiting for /tf, which camera drivers can crowd), else the last pose, marked stale.
            T = self._tcp_from_joints()
            if T is not None:
                xyz, rpy = tf.T_to_mm_deg(T)
                tcp = {"xyz_mm": [round(v, 2) for v in xyz], "rpy_deg": [round(v, 2) for v in rpy]}
                self._tcp_last = (time.monotonic(), c.store.active_tcp, tcp)
            else:
                t0, name, last = getattr(self, "_tcp_last", (0.0, None, None))
                if last and name == c.store.active_tcp and time.monotonic() - t0 < self.TCP_HOLD_S:
                    tcp = {**last, "stale": True}
        tcp_off = None
        try:                                   # changes when the active TCP or its definition does (preview cache key)
            link, _, T_lt = c.geo.tcp_anchor(c.store.active_tcp)
            tcp_sig = f"{c.store.active_tcp}|{link}|" + ",".join(f"{v:.5f}" for v in np.asarray(T_lt)[:3].ravel())
            oxyz, orpy = tf.T_to_mm_deg(T_lt)  # the 3D view draws paths at link * this (a virtual TCP is not in the URDF)
            tcp_off = {"link": link, "xyz_mm": [round(v, 3) for v in oxyz], "rpy_deg": [round(v, 4) for v in orpy]}
        except Exception:
            tcp_sig = str(c.store.active_tcp)
        fb = c.policy_feedback
        pr = c.policy_result
        policy_busy = c.policies.busy
        if policy_busy:
            if not self._policy_was_busy:
                self._policy_last_step = None
            self._policy_was_busy = True
        elif self._policy_was_busy:
            self._policy_last_step = None
            self._policy_was_busy = False
        if c.policies.current and c.policies.current != self._policy_chart_policy:
            self._policy_chart_policy = c.policies.current
            self._policy_last_step = None
        policy_steps = self._policy_steps()
        if not policy_busy:
            self._policy_last_step = None
        step_age = None
        if self._policy_last_step:
            received = self._policy_last_step.get("_received")
            if received is not None:
                step_age = max(0.0, time.time() - float(received))
        chart = self._policy_chart_info(self._policy_chart_policy)
        range_warnings = []
        if c.policies.current:
            m = c.policies.manifests.get(c.policies.current)
            if m and c.policies.range_check:
                try:
                    range_warnings = split_range_notes(c.policies.range_check(m))[0]
                except Exception as e:
                    range_warnings = [f"range check unavailable: {e}"]
        return {
            "t": time.time(), "boot": BOOT_ID,
            "robot_mode": s.robot_mode, "safety_mode": s.safety_mode, "program_running": s.program_running,
            "program_state": s.program_state,
            "remote_control": s.remote_control,
            "speed": s.speed_scaling, "speed_override": c.speed_override, "connected": s.connected,
            "joints_deg": [round(math.degrees(v), 3) for v in s.joints] if s.joints else None,
            "joint_state_age": s.joint_state_age,
            "joint_vel_dps": [round(math.degrees(v), 2) for v in s.joint_velocities] if s.joint_velocities else None,
            "tcp": tcp, "tcp_sig": tcp_sig, "tcp_off": tcp_off, "wrench": [round(v, 3) for v in s.wrench] if s.wrench else None,
            "ft_samples": self._ft_samples(), "ft_guard": c.ft_guard.view(),
            "gripper_position": s.gripper_position, "gripper_status": s.gripper_status,
            "gripper_controller": s.gripper_controllers.get(c.profile.gripper.get("controller") or ""),
            "freedrive": s.freedrive,
            "mode": c.shown_mode, "mode_label": c.mode_label(c.shown_mode), "command_source": c.command_source,
            "busy": c.busy and not c.jog_ready_only(),
            "idle": c.idle() or c.jog_ready_only(), "motion_refusal": c.motion_refusal(), "manual_refusal": self._manual_refusal(),
            "unknown": c.modes.control_state_unknown, "unknown_reason": c.modes.unknown_reason,
            "last_refusal": c.last_refusal, "controllers": s.controllers,
            "runner": {"state": c.runner.state.name, "step": c.program_step,
                       "pause_reason": c.runner.pause_reason if c.runner.state.name == "PAUSED" else "",
                       "resume_with_robot": c._resume_after_robot,
                       "loops": [list(x) for x in c.runner.loop_progress],
                       "skipped": len(getattr(c.runner, "skipped", [])),
                       "path": list(c.runner.current_path) if c.runner.current_path else None,
                       "result": c.program_result, "program": self.program.name,
                       "dirty": self._program_yaml() != self._saved_yaml, "rev": self.program_rev},
            "policy": {"busy": policy_busy, "current": c.policies.current, "prepared": self.prepared_policy,
                       "feedback": vars(fb) if fb else None,
                       "result": {"success": pr.success, "reason": pr.stop_reason, "log_dir": pr.log_dir}
                       if pr else None,
                       "supports_feedback": c.policies.supports_feedback,
                       "range_warnings": range_warnings,
                       "step_warnings": ((self._policy_last_step or {}).get("out_of_range", [])
                                         if policy_busy else []),
                       "step_age_s": step_age,
                       **chart},
            "policy_steps": policy_steps,
            "payload": self.payload_view(),
            "ftmatch": self.ftmatch_view(),
            "ft_bias": self.core.ft_bias.view(),
            "calibration": self.calibration_view(),
            "keepout": self.keepout_view(),
            "workcell": self.workcell_state(),
            "record": self.core.recorder.view(),
            "gains_applying": self.core.gains.applying,
            "lease": self._lease_state(),
            "activity_seq": self.activity.seq, "history_last": (self.core.history.entries[-1].get("id") if self.core.history.entries else ""),
            "users": self.users.view(self.lease.uid),
            "jog": {"active": c.jogging, "ready": c._jog_arm is not None, "hold_refusal": c.jog_hold_refusal(),
                    "servo_warning": c.servo_warning() if c.jogging else ""},
            "force": {**c.force_view(), "note": c.force_note()},
            "plan": self._plan_summary(),
            "gripper_joint": c.profile.gripper.get("joint_name"),
            "teach": self.teach_view(),
            "batch": c.batch.view(),
        }

    def _ft_samples(self) -> list:
        """F/T samples since the previous state (100 Hz) for the live chart; [] if the bridge has none."""
        fn = getattr(self.core.bridge, "wrench_samples", None)
        if fn is None:
            return []
        out = fn(getattr(self, "_ft_t", time.time() - 0.2))
        if out:
            self._ft_t = out[-1][0]
        return out

    def _policy_steps(self) -> list:
        fn = getattr(self.core.bridge, "policy_step_samples", None)
        if fn is None:
            return []
        try:
            rows = fn(self._policy_step_cursor)
        except Exception as e:
            self.core.bridge.log(f"web: policy step stream failed: {e}")
            return []
        if not rows:
            return []
        self._policy_step_cursor = max(float(r.get("_received", self._policy_step_cursor)) for r in rows)
        clean = []
        for row in rows:
            self._policy_last_step = row
            clean.append({k: v for k, v in row.items() if k != "_received"})
        return clean

    @staticmethod
    def _policy_chart_info_for_manifest(m):
        card = m.card() or {}
        raw_names = card.get("action_names")
        names, units = [], []
        for item in raw_names if isinstance(raw_names, list) else []:
            if isinstance(item, dict):
                names.append(str(item.get("name") or ""))
                units.append(str(item.get("unit") or ""))
            else:
                names.append(str(item))
                units.append("")
        raw_units = card.get("action_units")
        if isinstance(raw_units, list):
            units = [str(raw_units[i] or "") if i < len(raw_units) else "" for i in range(len(names))]
        elif isinstance(raw_units, dict):
            units = [str(raw_units.get(name) or "") for name in names]
        return {"action_names": names, "action_units": units}

    def _policy_chart_info(self, name):
        m = self.core.policies.manifests.get(name) if name else None
        return self._policy_chart_info_for_manifest(m) if m else {"action_names": [], "action_units": []}

    def _plan_summary(self):
        p = self.core.plan_view()
        return {"id": self._plan_id, "what": p["what"], "duration_s": p["duration_s"], "age_s": p["age_s"]} \
            if p else {"id": self._plan_id}

    def shutdown(self):
        self._teach_end("GUI closed")
        self._jobs.shutdown(wait=False, cancel_futures=True)
        self.drivers_stop_all()
        self.core.recorder.stop_and_wait("GUI closed")      # never leave a `ros2 bag record` running


# commands that change the open program: undoable (new/open are guarded in the browser instead)
PROGRAM_EDITS = {"program_insert", "program_rename_waypoint", "program_update", "program_set_waypoint", "program_type_waypoint", "program_duplicate", "program_rename",
                 "program_set_yaml", "program_record", "program_delete_step", "program_move_step", "program_scrape", "program_scrape_baseline",
                 "program_flask_scrape",
                 "program_new", "program_open", "teach_mark", "teach_insert"}
UNDO_DEPTH = 50
HOLD_TIMEOUT_S = 0.4          # hold-to-run: no renewal for this long -> STOP


def _frac(v) -> float:
    v = float(v)
    if not 0.0 < v <= 1.0:
        raise CommandError("velocity / acceleration scaling must be in (0, 1]")
    return v


def _recording(d: Optional[dict]) -> RecordingOptions:
    d = d or {}
    return RecordingOptions(record_obs=d.get("record_obs"), record_video=d.get("record_video"),
                            video_fps=d.get("video_fps"), output_root=d.get("output_root") or None)
