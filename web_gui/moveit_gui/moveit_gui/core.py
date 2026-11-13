"""RobotCore: everything the operator front ends share, without Qt.

The Qt window and the web server are two views of one core. The core owns the services (bridge, profile,
calibration store, geometry, control modes, program runner, policy executor) and the rules that keep motion
safe: when manual motion is allowed, what "busy" means, freedrive in and out, STOP. A front end calls these
and shows the result; it does not re-implement them.

Thread model: update() is polled (10 Hz) from the front end's thread. Blocking actions (motion, mode switches,
freedrive) run in the caller's worker thread and return (ok, message).
"""
from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Callable, List, Optional, Tuple

import numpy as np

from . import cart_speed, force, jog
from .bridge_api import Snapshot, wrench_fresh
from .calibration import CalibrationStore
from .control_modes import CONTROL_STATE_UNKNOWN, ControlModeManager, detect_mode
from .geometry import Geometry, PoseSpec
from .history import RunHistory
from .jog import JogCommand, JogVelocity
from .keepout import KeepoutScene, KeepoutStore
from .policies import PolicyLibrary
from .policy_action import MockPolicyRunner, RunPolicyFeedback, RunPolicyResult, SubprocessPolicyRunner
from .policy_executor import PolicyExecutor
from .profile import RobotProfile, reorder_joints
from .batch import BatchRunner
from .program import ProgramHooks, ProgramRunner, RunState
from .recorder import NAME_RE, FakeBagProcess, Recorder
from .ft_bias import FtBias
from .ft_guard import FtGuard
from .gains import Gains
from .calibration import config_dir as _config_dir

Result = Tuple[bool, str]


class RobotCore:
    def __init__(self, bridge, profile: RobotProfile, profiles_dir: Path, programs_dir: Path, policies_dir: Path,
                 fake: bool = False):
        self.bridge, self.profile, self.fake = bridge, profile, fake
        self.profiles_dir, self.programs_dir, self.policies_dir = Path(profiles_dir), Path(programs_dir), \
            Path(policies_dir)
        self.store = CalibrationStore(profile)
        self.history = RunHistory(profile.name)
        # Session recording (rosbag2/MCAP), next to the programs: /data/recordings in the robot container
        self.recorder = Recorder(self.programs_dir.parent / "recordings", profile.name,
                                 getattr(bridge, "topic_types", lambda: []), bridge.log,
                                 spawn=FakeBagProcess if fake else None)
        self._auto_rec: Optional[str] = None         # recording directory started by auto-record
        self.operator: Callable[[], str] = lambda: ""    # who is in control (the web service sets it)
        self._step_rec: Optional[str] = None         # recording directory a program's record_start step began
        for note in self.store.notes:
            bridge.log(f"calibration: {note}")
        self.geo = Geometry(self.store, bridge.lookup_transform, bridge.read_pose_source)
        # Keep-out zones: MoveIt collision boxes, applied once MoveIt is up and on every change
        self.keepout_store = KeepoutStore(self.store.path, profile.name,
                                          lambda f: f == profile.base_link or f in self.store.fixtures)
        for note in self.keepout_store.notes:
            bridge.log(note)
        self.keepout = KeepoutScene(self.keepout_store, self.geo, bridge, lambda: self.store.active_tcp, bridge.log)
        self.runner: Optional[ProgramRunner] = None
        self.modes = ControlModeManager(
            bridge, profile, log=bridge.log,
            policy_running=lambda: self.policies.busy,
            program_running=lambda: self.runner is not None and (self.runner.state != RunState.IDLE or self.batch.active))
        # One policy runtime path: AUTOMATION and the program `policy` step both go through this.
        library = PolicyLibrary(policies_dir)
        self.policies = PolicyExecutor(library, self.modes, bridge,
                                       MockPolicyRunner() if fake else SubprocessPolicyRunner(library, self.modes),
                                       active_tcp=lambda: self.store.active_tcp,
                                       frame_exists=lambda f: f in self.store.frame_names())
        # SETUP > Controller gains: admittance / impedance parameters on the running controller nodes
        self.gains = Gains.from_profile(profile.gains, bridge, bridge.log, lambda: self.mode, self._gains_refusal,
                                        settings_path=_config_dir() / "gains" / f"{profile.name}.json")
        self.policies.gains_check = self._policy_gains_check
        # SETUP > F/T + Payload: software bias on the F/T broadcaster (never the ATI hardware bias)
        self.ft_bias = FtBias(profile.ft_bias, profile.topics.get("wrench", ""), bridge, bridge.log,
                              self._ft_bias_refusal, settings_path=_config_dir() / "ft_bias" / f"{profile.name}.json")
        self.ft_bias.payload = self._ft_payload_expected
        self.modes.before_activate = self._before_activate
        # Listeners are called from the runner's worker thread; front ends marshal to their own thread.
        self.step_listeners: List[Callable[[int], None]] = []
        self.state_listeners: List[Callable[[RunState], None]] = []
        self.done_listeners: List[Callable[[Tuple[bool, str]], None]] = []
        self.runner = ProgramRunner(bridge, profile, self.modes, self.geo, self.policies,
                                    on_step=self._on_step, on_state=self._on_state, on_log=bridge.log,
                                    on_done=lambda ok, msg: self._on_done(ok, msg))
        self.runner.speed_override = lambda: self.speed_override
        self.runner.robot_stopped = self._robot_stopped_during_run
        self.runner.hooks = _CoreProgramHooks(self)
        # PROGRAMS > Batch: one program run several times; the whole batch counts as one program run (idle())
        self.batch = BatchRunner(self)
        self.last_run_recording = ""                 # the recording of the last program run ("" = none)
        self.last_snap = Snapshot()
        self._watchdog_why = None
        self._watchdog_stop = None
        self._resume_after_robot = False       # ▶ Resume program: resume the paused GUI run once it plays
        self.mode = "unknown"
        self.command_source = "idle"          # idle | manual (MoveIt) | program | policy <name> | freedrive | ...
        self.busy = False
        self.last_refusal = ""
        self.mode_before_freedrive = profile.default_mode
        self.program_step = -1
        self._runs: dict = {}                  # kind -> facts about the run going on now (see _touch_run)
        self.log_mark: Callable[[], int] = lambda: 0                 # the web service sets these two: the log lines
        self.log_since: Callable[[int], list] = lambda mark: []      # a run produced
        self.program_result: Optional[Tuple[bool, str]] = None
        self.policy_feedback: Optional[RunPolicyFeedback] = None
        self.policy_result: Optional[RunPolicyResult] = None
        self._lock = threading.Lock()
        self._jog: Optional[jog.JogStream] = None
        self._jog_velocity: Optional[JogVelocity] = None
        self._jog_speed = 0.0
        self.speed_override = 1.0              # GUI speed override: scales every GUI motion (not policies)
        self._jog_mode = ""
        self._jog_route = ""
        self._jog_restore_mode = ""
        self._jog_shape = True
        self.jog_cfg = jog.jog_config(profile)
        self._force: Optional[force.ForceStream] = None
        self.force_cfg = force.force_config(profile)
        self.force_last_end = ""
        self._selected_axes: dict = {}               # mode -> admittance selected_axes (read once per mode entry)
        self._mode_seen = "unknown"
        self._plan: Optional[dict] = None            # the last plan: trajectory, preview, start, time
        # F/T soft / hard limit (ft_guard.py): RosBridge feeds every sample from its F/T callback; other bridges
        # (fake) get the 10 Hz snapshot reading from update()
        self.ft_guard = FtGuard(_config_dir() / "ft_limit.yaml", on_hard=self._ft_hard_stop, log=bridge.log)
        self.ft_guard.freedrive = lambda: bool(self.last_snap.freedrive)
        self.ft_guard.commanding = self.commanding
        self._ft_pushed = hasattr(bridge, "on_wrench")
        if self._ft_pushed:
            bridge.on_wrench = self.ft_guard.feed
        self.policies.feedback_listeners.append(self._on_policy_feedback)
        self.policies.done_listeners.append(self._on_policy_done)
        for f, e in self.policies.errors.items():
            bridge.log(f"policy manifest {f}: {e}")

    # ------------------------------------------------------------------ state
    def update(self) -> Snapshot:
        """Poll once: snapshot, detected mode, command source. Call at the UI rate."""
        s = self.bridge.snapshot()
        self.last_snap = s
        if not self._ft_pushed:
            self.ft_guard.feed(s.wrench)
        if self.modes.control_state_unknown:
            self.mode = CONTROL_STATE_UNKNOWN
        else:
            self.mode = detect_mode(self.profile, s.controllers) if s.controllers else "unknown"
        if self.mode != self._mode_seen:
            self._mode_seen = self.mode
            self._selected_axes.clear()                  # re-read on the next entry: parameters may change
            if self.jogging and self.mode != self._jog_mode:
                self.jog_stop("control mode changed")
            if self.force_active:
                # e.g. the driver stopped the controller (pendant program stopped): never carry a force over
                threading.Thread(target=self.force_stop, args=("control mode changed: zeroed at once",),
                                 daemon=True).start()
        self._robot_watchdog()
        p = self._plan
        if p is not None and s.joints and \
                max(abs(a - b) for a, b in zip(s.joints, p["points"][0][1])) > self.PLAN_START_TOL_RAD:
            self._plan = None                        # the arm left the plan's start (program, jog, pendant, ...)
            self._plan_dropped = "the arm moved since planning: plan again"
        if s.freedrive:
            self.command_source = "freedrive"
        elif self.command_source == "freedrive":
            self.command_source = "idle"
        # Startup / move_group restart / fixture re-taught: re-apply the keep-out boxes (never while moving)
        self.keepout.poll(bool(s.connected.get("MoveIt")), self.idle() and not s.freedrive)
        self._auto_record()
        self._track_runs(s)
        self.recorder.poll()
        self.gains.poll(self.idle() and not s.freedrive)
        self.ft_bias.poll(not self.ft_bias.busy)
        return s

    def _run_label(self) -> Optional[Tuple[str, str]]:
        """(kind, name) of the program or policy running now, else None."""
        if self.runner is not None and self.runner.state != RunState.IDLE:
            return "program", getattr(self.runner, "program_name", "") or "program"
        if self.policies.busy:
            name = (getattr(self.policies, "started", None) or ("policy",))[0]
            return "policy", name or "policy"
        return None

    def _auto_record(self):
        """Auto-record: one recording per program / policy run, started and stopped with it."""
        run = self._run_label()
        rec = self.recorder
        if run and self._auto_rec is None and rec.settings.get("auto") and not rec.recording:
            ok, msg = rec.start(name=f"{run[0]}_{run[1]}", source="auto",
                                run={"kind": run[0], "name": run[1], "start": time.time()})
            self._auto_rec = rec.view()["recording"]["dir"] if ok else ""
            if not ok:
                self.bridge.log(f"auto-record: {msg}")
        elif not run and self._auto_rec is not None:
            live = rec.view()["recording"]
            if self._auto_rec and live and live["dir"] == self._auto_rec:
                rec.stop("the run ended")
            self._auto_rec = None

    def recording_dir(self) -> str:
        live = self.recorder.view()["recording"]
        return live["dir"] if live else ""

    def record_start(self, name: str = "", note: str = "", presets=None, source: str = "") -> Tuple[bool, str]:
        """Start a recording with the who / what / why in meta.json. presets None = the saved selection."""
        who = self.operator() or ""
        run = self._run_label()
        info = {"operator": who, "note": note or "", "mode": self.mode,
                "program": run[1] if run and run[0] == "program" else ""}
        return self.recorder.start(name=name or (f"program_{run[1]}" if run else ""),
                                   source=source or f"manual ({who or 'operator'})",
                                   run={"kind": run[0], "name": run[1], "start": time.time()} if run else None,
                                   presets=presets, info=info)

    def take_photo(self, topic: str, label: str = "") -> Tuple[bool, str]:
        """One still from an image topic: into the running recording's photos/ folder, else <recordings>/../photos.
        File name: <date>_<time>_<label>.jpg"""
        grab = getattr(self.bridge, "grab_photo", None)
        if grab is None:
            return False, "this bridge cannot take photos"
        jpeg, why = grab(topic)
        if jpeg is None:
            return False, why or f"{topic}: no image"
        live = self.recording_dir()
        folder = Path(live) / "photos" if live else self.recorder.root.parent / "photos"
        stamp = time.strftime("%Y%m%d_%H%M%S")
        tag = NAME_RE.sub("_", (label or "photo").strip())[:60].strip("_") or "photo"
        try:
            folder.mkdir(parents=True, exist_ok=True)
            path, n = folder / f"{stamp}_{tag}.jpg", 1
            while path.exists():
                n += 1
                path = folder / f"{stamp}_{tag}~{n}.jpg"
            path.write_bytes(jpeg)
        except OSError as e:
            return False, f"cannot save the photo in {folder}: {e}"
        return True, f"photo saved: {path}"

    def mode_label(self) -> str:
        m = self.profile.modes.get(self.mode)
        if self.mode == CONTROL_STATE_UNKNOWN:
            return "CONTROL STATE UNKNOWN"
        if m:
            if m.direct_torque:
                return f"{m.label} · Direct torque"
            ci = (m.command_interface or "").strip().lower()
            return m.label if ci in ("", "none") else f"{m.label} · {ci.capitalize()}"
        return self.mode.capitalize()

    def _gains_refusal(self) -> Optional[str]:
        """Why gains may not change under an active controller right now (None: they may)."""
        if self.force_active:
            return "a force is applied"
        if self.policies.busy:
            return "a policy is running"
        if self.runner is not None and (self.runner.state != RunState.IDLE or self.batch.active):
            return "a program is running"
        if self.busy or self.jogging:
            return "the robot is moving"
        return None

    def _ft_bias_refusal(self) -> Optional[str]:
        """Why the F/T bias may not change now (None: it may). Anything using the reading must not see it jump."""
        m = self.profile.modes.get(self.mode)
        if m is not None and m.requires_ft:
            return f"{m.label} uses the F/T reading (switch to another mode first)"
        return self._gains_refusal()

    def _policy_gains_check(self, mode: str, expected: dict):
        ok, msg = self.gains.compare(mode, expected)
        return ok, msg

    def idle(self) -> bool:
        return not self.busy and self.runner.state == RunState.IDLE and not self.policies.busy and not self.batch.active

    def refuse(self, msg: str) -> Result:
        self.last_refusal = msg
        self.bridge.log(msg)
        return False, msg

    # ----------------------------------------------------------------- motion
    def motion_refusal(self) -> Optional[str]:
        """Why manual MoveIt motion is not allowed right now, or None."""
        if self.busy or self.runner.state != RunState.IDLE or self.policies.busy or self.batch.active:
            return "robot is busy"
        if self.modes.control_state_unknown:
            return ("control state is UNKNOWN: GUI motion is disabled until it is recovered "
                    "(OPERATE, Arm control)")
        robot = self.robot_refusal()
        if robot:
            return robot
        if not self.modes.mode_supports_moveit(self.mode):
            m = self.profile.modes.get(self.mode)
            why = ("a direct-torque mode: MoveIt trajectories are not available" if m and m.direct_torque
                   else "a mode without a trajectory controller")
            return f"{m.label if m else self.mode} is {why}. Change to Position or Admittance."
        return None

    STATE_STALE_S = 1.0                  # joint states older than this: the robot / driver is gone
    CONTROLLERS_STALE_S = 3.0            # controller list older than this: controller_manager is gone

    def robot_refusal(self) -> Optional[str]:
        """The robot itself cannot take commands. Unknown values ("?", None) do not refuse, with one exception:
        the UR io_and_status_controller publishes robot_program_running only when it CHANGES from its initial
        false. So while the robot reports its mode but that topic never said anything, the External Control
        program has not run since the driver started. A goal sent then is accepted but held at speed scaling 0:
        the GUI waits, and the arm would start moving later when someone presses Play."""
        s = self.bridge.snapshot()
        # robot_mode / safety_mode / program_running are published on change only, so their own age says nothing.
        # The liveness signals are the joint states (500 Hz while the driver and the robot network live) and the
        # controller list (polled twice a second): when either is stale, every status above may be a dead value.
        if s.joint_state_age is None or s.joint_state_age > self.STATE_STALE_S:
            return ("no fresh joint states" if s.joint_state_age is None else
                    f"joint states are {s.joint_state_age:.1f} s old") + ": the UR driver or the robot network is down"
        if getattr(s, "controllers_age", None) is not None and s.controllers_age > self.CONTROLLERS_STALE_S:
            return f"the controller list is {s.controllers_age:.0f} s old: controller_manager does not answer"
        if s.safety_mode not in ("?", "NORMAL", "REDUCED"):
            return f"safety mode is {s.safety_mode}: clear it on the pendant"
        if s.robot_mode not in ("?", "RUNNING"):
            return f"robot mode is {s.robot_mode}: power on and release the brakes (OPERATE or the pendant)"
        if s.program_running is not True and getattr(s, "program_state", None) == "PAUSED":
            return "the robot program is paused: ▶ Resume program (OPERATE, Robot) or Play on the pendant"
        if s.program_running is not True and getattr(s, "program_state", None) == "PLAYING":
            return ("a program plays on the pendant, but it is not External Control connected to this PC: in the "
                    "program's External Control node set Host IP to this PC, then press Play again")
        if s.program_running is False or (s.program_running is None and s.robot_mode == "RUNNING"
                                          and self.profile.topics.get("program_running")):
            return ("the External Control program is not running: start it with ▶ Play program (OPERATE, Arm "
                    "control) or Play on the pendant. The UR driver holds the arm until it runs")
        return None

    def _robot_stopped_during_run(self) -> Optional[str]:
        """A program motion failed: was it because the robot stopped taking motion (pendant Pause / Stop, a
        protective stop, power)? The driver's status topics lag the failed goal a little, so look for 0.5 s."""
        t_end = time.monotonic() + 0.5
        while True:
            why = self.robot_refusal()
            if why or time.monotonic() > t_end:
                return why
            time.sleep(0.05)

    def begin_motion(self, source: str = "manual (MoveIt)",
                     refusal: Optional[Callable[[], Optional[str]]] = None) -> Optional[str]:
        """Claim the arm for one manual action. Returns the refusal, or None (then call end_motion).
        refusal: the rule to check instead of motion_refusal (e.g. force, which needs no MoveIt)."""
        with self._lock:
            why = (refusal or self.motion_refusal)()
            if why is None:
                self.busy, self.command_source = True, source
                if self._plan is not None:           # any other motion makes a kept plan meaningless
                    self._plan, self._plan_dropped = None, "the arm moved since planning (another motion ran): plan again"
            return why

    def end_motion(self):
        with self._lock:
            self.busy, self.command_source = False, "idle"

    def run_motion(self, fn: Callable[[], bool], source: str = "manual (MoveIt)") -> Result:
        """Blocking: claim, run fn (a bridge motion), release."""
        why = self.begin_motion(source)
        if why:
            return False, why
        self._watchdog_stop = None
        try:
            ok = bool(fn())
        finally:
            self.end_motion()
        if not ok and self._watchdog_stop:                # say why, not just "cancelled"
            return False, f"motion stopped: {self._watchdog_stop}"
        return ok, "motion done" if ok else "motion failed / cancelled" + self.keepout_hint()

    def keepout_hint(self) -> str:
        """Why MoveIt refuses every plan when the robot starts inside a keep-out box (start state in collision)."""
        inside = getattr(getattr(self, "keepout", None), "inside", None)
        return (f": the robot is inside keep-out zone {', '.join(inside)}, so MoveIt refuses to plan "
                "(jog or freedrive out, or untick the box)") if inside else ""

    # ------------------------------------------------------- speed override
    SPEED_OVERRIDE_MIN = 0.05

    def set_speed_override(self, f: float) -> Result:
        f = float(f)
        if not self.SPEED_OVERRIDE_MIN <= f <= 1.0:
            return False, f"speed override must be {self.SPEED_OVERRIDE_MIN * 100:.0f}..100 %"
        self.speed_override = f
        self.bridge.log(f"GUI speed override {f * 100:.0f} %")
        return True, f"speed {f * 100:.0f} %"

    def scaled(self, vel: float, acc: float):
        """MoveIt velocity / acceleration scaling after the GUI speed override (never below 1 %)."""
        f = self.speed_override
        return max(0.01, float(vel) * f), max(0.01, float(acc) * f)

    def move_joints(self, q, vel: float, acc: float) -> Result:
        vel, acc = self.scaled(vel, acc)
        return self.run_motion(lambda: self.bridge.move_joints(list(q), vel, acc))

    def linear_speed(self, speed_mm_s, linear: bool) -> float:
        """A linear move's TCP speed limit for the bridge: m/s after the GUI speed override, 0.0 = none (no value,
        0, or not a linear move). Raises ValueError outside (0, profile max]."""
        v = cart_speed.optional_mm_s(speed_mm_s, self.profile.max_cart_speed_mm_s)
        return v / 1000.0 * self.speed_override if (v > 0 and linear) else 0.0

    def move_pose(self, pose, linear: bool, vel: float, acc: float, speed_mm_s=None) -> Result:
        """pose: geometry.PoseSpec (frame + TCP). Resolved to the MoveIt link goal here. speed_mm_s: TCP speed
        limit of a linear move (None / 0 = only the velocity scaling)."""
        target = self.geo.planning_target(pose)          # GeometryError propagates to the caller
        try:
            cs = self.linear_speed(speed_mm_s, linear)
        except ValueError as e:
            return False, str(e)
        vel, acc = self.scaled(vel, acc)
        return self.run_motion(lambda: self.bridge.move_pose(target.link, target.T_base_link, linear, vel, acc,
                                                             cart_speed=cs, speed_point=target.T_link_tcp[:3, 3]))

    # ------------------------------------------------------------------- jog
    def jog_step(self, cmd: JogCommand, step: float, vel: float, acc: float) -> Result:
        """Blocking. One step: mm or deg (by axis), as a short MoveIt move. Cartesian steps are linear and
        turn about the active TCP."""
        if not 0 < step <= max(self.jog_cfg["step_mm"] + self.jog_cfg["step_deg"]):
            return False, "step must be > 0 and at most the largest configured step"
        vel, acc = self.scaled(vel, acc)
        if cmd.kind == "joint":
            q = self.last_snap.joints
            if not q:
                return False, "no joint state"
            target = jog.joint_step(q, cmd, math.radians(step))
            return self.run_motion(lambda: self.bridge.move_joints(target, vel, acc), f"jog {cmd.describe()}")
        tcp = self.store.active_tcp
        T = self.geo.current_T_base_tcp(tcp)                      # GeometryError propagates to the caller
        T_goal = jog.cartesian_step(T, cmd, step / 1000.0, math.radians(step))
        target = self.geo.planning_target(PoseSpec.from_T(self.geo.base, tcp, T_goal))
        return self.run_motion(lambda: self.bridge.move_pose(target.link, target.T_base_link, True, vel, acc),
                               f"jog {cmd.describe()}")

    def jog_hold_refusal(self) -> Optional[str]:
        """Why hold-to-jog is not available now, or None."""
        robot = self.robot_refusal()
        if robot:
            return robot
        route, _ = jog.servo_route(self.profile, self.mode)
        if not route:
            return f"MoveIt Servo has no configured route for {self.mode}. Change to Position or Admittance."
        if not self.bridge.servo_ready(route):
            return ("Continuous jog is off (MoveIt Servo is not running): use the step jog, or turn on Continuous "
                    "jog in SETUP › Setups › Hardware…. If Servo is running, one step jog wakes it up.")
        return None

    def jog_start(self, cmd: JogCommand, speed: float) -> Result:
        """Start a hold-to-jog at `speed` (0..1 of the configured top speed). The front end must call
        jog_hold() every < jog.HOLD_TIMEOUT_S while the button is held, and jog_stop() on release."""
        if not 0 < speed <= 1:
            return False, "speed must be between 0 and 100 %"
        values = JogVelocity.from_command(cmd, len(self.profile.joint_names))
        return self.jog_start_velocity(values, speed, shape_values=False)

    def jog_start_velocity(self, velocity: JogVelocity, speed: float, shape_values: bool = True) -> Result:
        """Begin a Servo hold with six simultaneous normalized axes; every renewal can change them."""
        why = self.jog_hold_refusal()
        if why:
            return False, why
        if not 0 < speed <= 1:
            return False, "speed must be between 0 and 100 %"
        if not any(velocity.values):
            return False, "move at least one jog slider away from zero"
        speed = max(0.01, speed * self.speed_override)
        why = self.begin_motion(f"jog {velocity.describe()} (Servo)")
        if why:
            return False, why
        start_mode = self.mode
        route_name, route_cfg = jog.servo_route(self.profile, start_mode)
        restore_mode = ""
        try:
            ok, msg = self.bridge.servo_command_type(
                velocity.kind if velocity.kind == "joint" else "twist", route_name)
            if not ok:
                self.end_motion()
                return False, f"Servo: {msg}"

            # Position Servo streams to its own joint_trajectory_controller (servo_position mode), switched in
            # with the same fail-closed transactional mode switching as every other controller transition.
            stream_mode = str(route_cfg.get("control_mode") or start_mode)
            if stream_mode != start_mode:
                ok, msg = self.modes.switch_to(stream_mode)
                if not ok:
                    self.update()
                    self.end_motion()
                    return False, f"Servo controller switch: {msg}"
                restore_mode = start_mode
                self.update()
                if self.mode != stream_mode:
                    self.modes.switch_to(restore_mode)
                    self.update()
                    self.end_motion()
                    return False, f"Servo controller switch succeeded but mode is {self.mode}"

            self.bridge.log(
                f"Servo jog start: {velocity.describe()} · route {route_name} · {self.mode} · "
                f"direct Servo output · {jog.STREAM_HZ:.0f} Hz")
            self._jog_velocity = velocity
            self._jog_speed = speed
            self._jog_mode = self.mode
            self._jog_route = route_name or ""
            self._jog_restore_mode = restore_mode
            self._jog_shape = shape_values
            send, zero = self._jog_senders(velocity, route_name or "", route_cfg)
            send = self._jog_diagnosed(send, route_name or "", route_cfg)
            stream = jog.JogStream(send, zero, self._jog_ended)
            with self._lock:
                self._jog = stream
            stream.start()
        except Exception:
            if restore_mode:
                ok, msg = self.modes.switch_to(restore_mode)
                if not ok:
                    self.bridge.log(f"Servo cleanup could not restore {restore_mode}: {msg}")
                self.update()
            self.end_motion()
            raise
        return True, f"jogging {velocity.describe()}"

    def _jog_senders(self, velocity: JogVelocity, route: str, route_cfg: dict):
        c, b = self.jog_cfg, self.bridge
        def values():
            with self._lock:
                v = list(self._jog_velocity.values)
                speed = self._jog_speed
                shaped = self._jog_shape
            return [(jog.shape(x) if shaped else x) * speed for x in v]

        if velocity.kind == "joint":
            def send_joint():
                b.servo_joint_jog([x * math.radians(float(c["max_joint_dps"])) for x in values()], route)
            return send_joint, (lambda: b.servo_joint_jog([0.0] * len(velocity.values), route))
        frame = route_cfg.get("frame", self.geo.base)
        tip = route_cfg.get("tip_link", "tool0")
        tcp = self.store.active_tcp
        lin = float(c["max_linear_mps"])
        ang = math.radians(float(c["max_angular_dps"]))

        def send():
            # Recomputed every tick from the measured poses: the axes of a tool-frame jog and the lever arm of
            # a rotation follow the TCP as it moves. Expressed in Servo's frame.
            T_tcp = self.geo.current_T_base_tcp(tcp)
            T_tip = self.geo.T_base_frame(tip)
            v, w = jog.servo_twist_vec(JogVelocity(velocity.kind, values(), velocity.frame), T_tcp, T_tip, lin, ang)
            if frame != self.geo.base:
                R = self.geo.T_base_frame(frame)[:3, :3].T
                v, w = (R @ np.asarray(v)).tolist(), (R @ np.asarray(w)).tolist()
            b.servo_twist(v, w, frame, route)
        return send, (lambda: b.servo_twist([0.0] * 3, [0.0] * 3, frame, route))

    def _jog_diagnosed(self, send, route: str, route_cfg: dict):
        """Wrap send() to collect what the jog log line reports when it ends: did Servo pass commands on to its
        controller, what did it say, and did the arm actually move."""
        b = self.bridge
        diag = {"t0": time.monotonic(), "route": route, "out_topic": route_cfg.get("output_topic", ""),
                "out0": b.servo_output_count(route), "q0": list(self.last_snap.joints or []), "status": {}}
        try:
            diag["p0"] = self.geo.current_T_base_tcp(self.store.active_tcp)[:3, 3].copy()
        except Exception:
            diag["p0"] = None
        self._jog_diag = diag

        def wrapped():
            send()
            code, text = b.servo_status(route)
            if code is not None:
                diag["status"][code] = text
        return wrapped

    def _jog_summary(self) -> str:
        d = getattr(self, "_jog_diag", None)
        if not d:
            return ""
        self._jog_diag = None
        b = self.bridge
        out1 = b.servo_output_count(d["route"])
        sent = "not watched" if out1 is None or d["out0"] is None else f"{out1 - d['out0']}"
        q1 = list(b.snapshot().joints or [])
        dq = max((abs(a - c) for a, c in zip(q1, d["q0"])), default=0.0)
        moved = f"max joint {math.degrees(dq):.2f}°"
        if d["p0"] is not None:
            try:
                p1 = self.geo.current_T_base_tcp(self.store.active_tcp)[:3, 3]
                moved += f", TCP {1000 * float(np.linalg.norm(p1 - d['p0'])):.1f} mm"
            except Exception:
                pass
        status = "; ".join(f"{c} {t}" for c, t in sorted(d["status"].items())) or "no status received"
        return (f"Servo jog summary ({d['route']}, {time.monotonic() - d['t0']:.1f} s): Servo commands to "
                f"{d['out_topic'] or 'its controller'}: {sent}; Servo status: {status}; arm moved: {moved}")

    def jog_hold(self, values=None) -> bool:
        """Renew the hold. False = no jog is streaming (it ended), the front end should stop renewing."""
        s = self._jog
        if s is None or not s.alive:
            return False
        if self.mode != self._jog_mode:
            self.jog_stop("control mode changed")
            return False
        if values is not None:
            with self._lock:
                current = self._jog_velocity
            if current is None:
                return False
            try:
                v = jog.parse_values(values, len(current.values))
            except jog.JogError:
                self.jog_stop("invalid jog values")
                raise
            with self._lock:
                if self._jog_velocity is None:
                    return False
                self._jog_velocity.values = v
        s.hold()
        return True

    def jog_stop(self, reason: str = "released"):
        s = self._jog
        if s is not None:
            s.stop(reason)
            s.join()

    def _jog_ended(self, reason: str):
        with self._lock:
            restore_mode = self._jog_restore_mode
            self._jog = None
            self._jog_velocity = None
            self._jog_route = ""
            self._jog_restore_mode = ""
        if restore_mode:
            # Servo decelerates after the zero command and then publishes a few halt messages: let them reach
            # the controller before taking it away, so the arm stops on Servo's ramp rather than abruptly.
            time.sleep(jog.RESTORE_DELAY_S)
            ok, msg = self.modes.switch_to(restore_mode)
            if not ok:
                self.bridge.log(f"Servo jog stopped but could not restore {restore_mode}: {msg}")
            self.update()
        self.end_motion()
        if reason != "released":
            self.bridge.log(f"jog ended: {reason}")
        summary = self._jog_summary()
        if summary:
            self.bridge.log(summary)

    def servo_warning(self) -> str:
        """Servo's own warning while it moves (singularity, collision, joint bound), '' when all is well."""
        code, text = self.bridge.servo_status(self._jog_route or None)
        return text if code not in (None, 0) else ""

    @property
    def jogging(self) -> bool:
        s = self._jog
        return s is not None and s.alive

    # ---------------------------------------------------------- plan / execute
    PLAN_MAX_AGE_S = 300.0
    PLAN_START_TOL_RAD = 0.01          # the arm must still be where the plan starts (MoveIt checks it too)

    def plan_joints(self, q, vel: float, acc: float) -> Result:
        """Blocking. Plan only: nothing moves until execute_plan()."""
        if self.busy:
            return False, "robot is busy"
        vel, acc = self.scaled(vel, acc)
        return self._keep_plan(self.bridge.plan_joints(list(q), vel, acc), "joint target")

    def plan_pose(self, pose, linear: bool, vel: float, acc: float, speed_mm_s=None) -> Result:
        if self.busy:
            return False, "robot is busy"
        target = self.geo.planning_target(pose)          # GeometryError propagates to the caller
        try:
            cs = self.linear_speed(speed_mm_s, linear)
        except ValueError as e:
            return False, str(e)
        vel, acc = self.scaled(vel, acc)
        return self._keep_plan(self.bridge.plan_pose(target.link, target.T_base_link, linear, vel, acc,
                                                     cart_speed=cs, speed_point=target.T_link_tcp[:3, 3]),
                               f"{'linear ' if linear else ''}{pose.tcp} in {pose.frame}"
                               + (f" ≤ {cs * 1000:.0f} mm/s" if cs else ""))

    def _keep_plan(self, traj, what: str) -> Result:
        if traj is None or not getattr(traj, "points", None):
            self._plan = None
            return False, "planning failed" + (self.keepout_hint() or " (see the log)")
        names = list(traj.joint_names)
        pts = []
        for p in traj.points:
            q = reorder_joints(names, p.positions, self.profile.joint_names)
            if q is None:
                self._plan = None
                return False, "the plan does not cover every arm joint"
            pts.append((p.time_from_start.sec + p.time_from_start.nanosec * 1e-9, q))
        self._plan = {"traj": traj, "points": pts, "what": what, "t": time.monotonic()}
        self._plan_dropped = ""
        return True, f"planned {what}: {pts[-1][0]:.1f} s, {len(pts)} points. Preview, then Execute"

    def plan_view(self) -> Optional[dict]:
        p = self._plan
        if p is None:
            return None
        return {"what": p["what"], "age_s": round(time.monotonic() - p["t"], 1), "duration_s": p["points"][-1][0],
                "points": [{"t": round(t, 3), "q": [round(v, 5) for v in q]} for t, q in p["points"]]}

    def clear_plan(self):
        self._plan = None

    def execute_plan(self) -> Result:
        """Blocking. Execute the kept plan through the active mode's controller, if the arm is still at its
        start and it is fresh. The plan is used once."""
        p, self._plan = self._plan, None
        if p is None:
            return False, getattr(self, "_plan_dropped", "") or "no plan: Plan first"
        if time.monotonic() - p["t"] > self.PLAN_MAX_AGE_S:
            return False, "the plan is too old: plan again"
        q = self.bridge.snapshot().joints
        if not q:
            return False, "no joint state"
        dev = max(abs(a - b) for a, b in zip(q, p["points"][0][1]))
        if dev > self.PLAN_START_TOL_RAD:
            return False, f"the arm moved since planning ({math.degrees(dev):.1f}° off the start): plan again"
        return self.run_motion(lambda: self.bridge.execute_trajectory(p["traj"]), f"execute plan ({p['what']})")

    # ----------------------------------------------------------------- force
    def force_refusal(self) -> Optional[str]:
        m = self.profile.modes.get(self.mode)
        if not (m and m.wrench.get("topic")):
            able = [x.label for x in self.profile.modes.values() if x.wrench.get("topic")]
            return f"applying a force needs {' or '.join(able) or 'a mode with a wrench target'} " \
                   f"(now: {m.label if m else self.mode})"
        if self.modes.control_state_unknown:
            return "control state is UNKNOWN"
        return self.robot_refusal()

    def _force_claim_refusal(self) -> Optional[str]:
        if self.busy or self.runner.state != RunState.IDLE or self.policies.busy or self.batch.active:
            return "robot is busy"
        return self.force_refusal()

    def force_note(self) -> str:
        """What the active mode does with a force (admittance: which axes respond)."""
        m = self.profile.modes.get(self.mode)
        if not (m and m.wrench.get("params_node")):
            return ""
        mode = self.mode
        if mode not in self._selected_axes:
            self._selected_axes[mode] = None             # fetched once, in the background (a service call)

            def fetch():
                self._selected_axes[mode] = self.bridge.get_node_parameters(
                    m.wrench["params_node"], ["admittance.selected_axes", "control.frame.id"])
            threading.Thread(target=fetch, daemon=True).start()
        got = self._selected_axes.get(mode) or {}
        if not got:
            return ""
        return force.selected_axes_note(got.get("admittance.selected_axes"), got.get("control.frame.id") or "?")

    @property
    def force_active(self) -> bool:
        f = self._force
        return f is not None and f.alive

    def force_view(self) -> dict:
        f = self._force
        alive = f is not None and f.alive
        return {"active": alive, "releasing": alive and f.releasing,
                "current": [round(v, 3) for v in f.current] if alive else None,
                "target": f.target.wrench if alive else None, "frame": f.target.frame if alive else None,
                "refusal": self.force_refusal(), "last_end": self.force_last_end}

    def force_start(self, target: "force.WrenchTarget") -> Result:
        """Apply (or change) the force. The front end must call force_hold() every < force.HOLD_TIMEOUT_S."""
        f = self._force
        if f is not None and f.alive and getattr(f, "owner", None) == "program":
            return False, "a running program owns the force: stop the program first"
        if f is not None and f.alive and not f.releasing:
            if f.target.frame == target.frame:
                f.retarget(target)
                f.hold()
                return True, "force target changed"
            self.force_stop("frame changed")
        why = self.begin_motion(f"force ({self.mode_label()})", self._force_claim_refusal)
        if why:
            return False, why
        try:
            stream = force.ForceStream(target, self._force_sender(target.frame), self._force_ended, self.force_cfg)
            with self._lock:
                self._force = stream
            self.force_last_end = ""
            stream.start()
        except Exception:
            self.end_motion()
            raise
        return True, "applying force"

    def _force_sender(self, frame: str):
        w = self.profile.modes[self.mode].wrench
        topic, out_frame = w["topic"], w.get("frame", self.geo.base)
        point = w.get("point", out_frame)
        tcp = self.store.active_tcp

        def send(w6):
            if not any(w6):                          # zero needs no TF: it must go out even when TF fails
                self.bridge.publish_wrench(topic, out_frame, [0.0] * 6)
                return
            T_tcp = self.geo.current_T_base_tcp(tcp)
            out = force.wrench_for_controller(force.WrenchTarget(list(w6), frame), T_tcp,
                                              self.geo.T_base_frame(out_frame), self.geo.T_base_frame(point))
            self.bridge.publish_wrench(topic, out_frame, out)
        return send

    def force_hold(self) -> bool:
        f = self._force
        if f is None or not f.alive:
            return False
        f.hold()
        return True

    def force_release(self):
        """Ramp down to zero, then end."""
        f = self._force
        if f is not None:
            f.release()

    def force_stop(self, reason: str = "stopped"):
        """Zero at once."""
        f = self._force
        if f is not None:
            f.stop(reason)
            f.join()

    def _force_ended(self, reason: str):
        with self._lock:
            self._force = None
        self.end_motion()
        self.force_last_end = reason
        self.bridge.log(f"force ended: {reason}")

    # ------------------------------------------------- program force (runner)
    def program_force_apply(self, wrench, frame: str) -> Result:
        """A force owned by the running program: stays applied across its steps, renewed while the program
        runs, ramped out by force_off / the end of the run, zeroed at once by STOP or a failed step."""
        try:
            target = force.WrenchTarget.parse(wrench, frame, self.force_cfg)
        except force.ForceError as e:
            return False, str(e)
        self._sync_mode()
        why = self.force_refusal()
        if why:
            return False, why
        f = self._force
        if f is not None and f.alive and not f.releasing:
            if getattr(f, "owner", "") != "program":
                return False, "a force applied from OPERATE is active: release it first"
            if f.target.frame == target.frame:
                f.retarget(target)
                return True, f"force target changed to {target.wrench} ({frame})"
            self.force_stop("frame changed")
        stream = force.ForceStream(target, self._force_sender(target.frame), self._program_force_ended, self.force_cfg)
        stream.owner = "program"
        with self._lock:
            self._force = stream
        self.force_last_end = ""
        stream.start()
        threading.Thread(target=self._program_force_hold, args=(stream,), daemon=True, name="program-force").start()
        return True, f"applying {target.wrench} ({frame})"

    def _sync_mode(self):
        """A program step right after a controller step: the mode it switched to, not the last tick's."""
        m = self.modes.current_mode()
        if m and m in self.profile.modes and m != self.mode:
            self.mode = self._mode_seen = m            # not a "mode changed" for the next update(): it is ours
            self._selected_axes.clear()

    def _program_force_hold(self, stream):
        while stream.alive and self.runner.state != RunState.IDLE:
            stream.hold()
            time.sleep(0.1)
        if stream.alive and not stream.releasing:
            stream.stop("the program is no longer running: zeroed at once")

    def _program_force_ended(self, reason: str):
        with self._lock:
            if self._force is not None and not self._force.alive:
                self._force = None
        self.force_last_end = reason
        self.bridge.log(f"force ended: {reason}")

    def program_force_release(self, wait_s: float) -> Result:
        f = self._force
        if f is None or not f.alive:
            return True, "no force applied"
        f.release()
        t_end = time.monotonic() + wait_s
        while f.alive and time.monotonic() < t_end:
            time.sleep(0.02)
        if f.alive:
            f.stop("release took too long: zeroed at once")
            f.join()
            return False, f"the force did not ramp out within {wait_s:g} s: zeroed at once"
        return True, "force ramped out"

    def _before_activate(self, target: str):
        """Before switching to `target`: an F/T mode with a payload needs a matching bias; then the saved gains."""
        m = self.profile.modes.get(target)
        if m is not None and m.requires_ft:
            w = (self.gains.saved.get(target) or {}).get("gravity_compensation.CoG.force")
            why = self.ft_bias.admittance_refusal(bool(w) and abs(float(w)) > 1e-6)
            if why:
                return False, f"{m.label} refused: {why}"
        ok, msg = self.gains.restore(target, f"before switching to {target}")
        return ok, (msg if ok else f"saved controller gains could not be restored: {msg}")

    def _ft_payload_expected(self):
        """The weight of the payload loaded on the admittance controller, as the sensor sees it now (FtBias.payload)."""
        fc = self.profile.ft_calibration or {}
        node = (fc.get("apply_to") or {}).get("node")
        if not node:
            return None, ""
        keys = ["gravity_compensation.CoG.force", "gravity_compensation.CoG.pos", "gravity_compensation.frame.id"]
        p = self.bridge.get_node_parameters(node, keys)
        if p is None:
            return None, (f"cannot read the payload loaded on {node} (not running?): zeroing now would also zero "
                          "the tool weight the controller compensates. Start the stack, or use Zero all")
        w, pos, frame = p.get(keys[0]), p.get(keys[1]), p.get(keys[2])
        if not w or abs(float(w)) < 1e-6 or pos is None:
            return None, ""                       # no payload compensated: zero everything
        sensor = fc.get("sensor_frame") or frame
        if frame and sensor and frame != sensor:
            return None, f"{node} compensates in {frame}, the F/T reading is in {sensor}"
        T = self.bridge.lookup_transform(fc.get("reference_frame", "base_link"), sensor)
        if T is None:
            return None, f"no transform {fc.get('reference_frame', 'base_link')} -> {sensor} for the payload weight"
        from .ft_bias import G, gravity_wrench
        return gravity_wrench(T, float(w), pos), f"Kept the {abs(float(w)) / G:.3f} kg payload loaded on {node}"

    def program_zero_ft(self, stopped) -> Result:
        """zero_ft step: the program's own run does not block it; a mode using the reading or a force does."""
        self._sync_mode()
        m = self.profile.modes.get(self.mode)
        if m is not None and m.requires_ft:
            return False, f"zero F/T refused: {m.label} uses the F/T reading. Zero before switching to it"
        if self.force_active:
            return False, "zero F/T refused: a force is applied"
        s = self.bridge.snapshot

        def sample():
            snap = s()
            qd = snap.joint_velocities
            still = qd is None or max((abs(v) for v in qd), default=0.0) <= self.profile.stationary_joint_velocity
            return wrench_fresh(snap), still       # a frozen sensor reads as "no F/T data": the zero fails
        return self.ft_bias.zero(sample, stopped=stopped, refusal=lambda: None)

    # ---------------------------------------------------------- run history facts
    def _touch_run(self, kind: str) -> dict:
        """Facts about the run going on now (who started it, F/T peak, log start, who stopped it)."""
        r = self._runs.get(kind)
        if r is None:
            r = self._runs[kind] = {"operator": self.operator() or "", "peak_f": 0.0, "peak_t": 0.0, "stopped_by": "",
                                    "log": self.log_mark(), "step": -1}
        return r

    def _track_runs(self, snap):
        if self.runner.state != RunState.IDLE:
            self._touch_run("program")
        if getattr(self.policies, "current", None):       # cleared just before the policy's done callbacks
            self._touch_run("policy")
        w = snap.wrench if self._runs else None
        if w and len(w) >= 6 and snap.wrench_age is not None and snap.wrench_age < 1.0:
            f, t = sum(x * x for x in w[:3]) ** 0.5, sum(x * x for x in w[3:6]) ** 0.5
            for r in self._runs.values():
                r["peak_f"], r["peak_t"] = max(r["peak_f"], f), max(r["peak_t"], t)

    def note_stop(self, who: str):
        """Remember who ended the runs going on now (the first one counts)."""
        for r in self._runs.values():
            r["stopped_by"] = r["stopped_by"] or who

    def _run_extras(self, kind: str) -> dict:
        r = self._runs.pop(kind, None) or {"operator": self.operator() or "", "peak_f": 0.0, "peak_t": 0.0,
                                           "stopped_by": "", "log": self.log_mark(), "step": -1}
        return {"operator": r["operator"], "stopped_by": r["stopped_by"], "peak_force_n": round(r["peak_f"], 2),
                "peak_torque_nm": round(r["peak_t"], 3), "_step": r["step"], "_log": self.log_since(r["log"])}

    def stop_all(self, reason: str = ""):
        if reason:
            self.note_stop(reason)
        # each part on its own: one that raises must not keep the others (or the cancel) from running
        for part in (lambda: self.batch.stop("stopped by STOP"),                   # first: no next run of a batch starts
                     lambda: self.force_stop("STOP: zeroed at once"),    # joins the stream for at most 2 s
                     lambda: self.jog_stop("STOP"), self.runner.stop, self.policies.cancel, self.bridge.cancel_motion):
            try:
                part()
            except Exception as e:
                self.bridge.log(f"STOP: {type(e).__name__}: {e}")
        self.command_source = "idle"
        self.bridge.log(f"STOPPED: {reason}" if reason else "STOP GUI MOTION pressed")

    def _ft_hard_stop(self, why: str):
        """The F/T hard limit (ft_guard.py, its own thread): stop everything the GUI commands, motion first."""
        self.bridge.log(f"F/T HARD LIMIT: {why}: stopping the GUI's motion")
        for part in (self.bridge.cancel_motion, self.runner.stop, self.policies.cancel,
                     lambda: self.jog_stop("F/T hard limit"), lambda: self.force_stop("F/T hard limit: zeroed")):
            try:
                part()
            except Exception as e:
                self.bridge.log(f"F/T limit stop: {type(e).__name__}: {e}")
        self.last_refusal = f"F/T hard limit: {why}"
        self.command_source = "idle"

    def commanding(self) -> bool:
        """The GUI is commanding the arm: a move, a program or policy run, a jog or a force."""
        return bool(self.busy or self.runner.state != RunState.IDLE or self.policies.busy or self.jogging
                    or self.force_active)

    def _robot_watchdog(self):
        """While the GUI commands the arm, the robot must stay able to execute: if the program stops, the arm
        powers off or a protective stop comes, a goal would just wait (held at speed 0) and the GUI would show
        MOVING until someone presses STOP, and resume later on Play. End everything at once instead."""
        why = self.robot_refusal() if self.commanding() else None
        if why and why != self._watchdog_why:
            self.last_refusal = self._watchdog_stop = why
            if self.runner.state in (RunState.RUNNING, RunState.PAUSED):
                # a program run is PAUSED, not ended: Resume (or ▶ Play / Resume program) carries on with the
                # step it was on. Everything else the GUI commands stops. A program force is zeroed at once
                # (nothing may push while the robot does not move) and the run re-applies it on resume.
                self.force_stop("the robot stopped: zeroed at once")
                self.jog_stop("the robot stopped")
                if self.policies.busy:
                    self.policies.cancel()
                self.runner.pause(why)
                self.bridge.cancel_motion()
                self.bridge.log(f"PROGRAM PAUSED: the robot cannot execute: {why}")
            else:
                self.stop_all(f"the robot cannot execute: {why}")
        self._watchdog_why = why
        if not why and self._resume_after_robot and self.robot_refusal() is None:
            self._resume_after_robot = False
            if self.runner.state == RunState.PAUSED:
                self.bridge.log("the robot program runs again: the GUI program resumes")
                self.runner.resume()

    # -------------------------------------------------------------- freedrive
    def freedrive(self, on: bool) -> Result:
        """Blocking. Off returns to the mode that was active before freedrive."""
        if not on:
            if not self.last_snap.freedrive:
                return True, "freedrive is off"
            self.bridge.set_freedrive_keepalive(False)
            return self.modes.switch_to(self.mode_before_freedrive)
        if self.last_snap.freedrive:
            return True, "freedrive is on"
        if "freedrive" not in self.profile.modes:
            return False, "profile has no freedrive mode"
        if not self.idle():
            return False, "robot is busy"
        self.mode_before_freedrive = self.mode if self.mode in self.profile.modes else self.profile.default_mode
        ok, msg = self.modes.switch_to("freedrive")
        if ok:
            self.bridge.set_freedrive_keepalive(True)
        return ok, msg

    def switch_mode(self, target: str) -> Result:
        if self.busy:
            return False, "robot is busy"
        return self.modes.switch_to(target)

    # ------------------------------------------------------------ calibration
    def set_active_tcp(self, name: str) -> Result:
        if name not in self.store.tcps:
            return False, f"unknown TCP '{name}'"
        self.store.set_active_tcp(name)
        self.bridge.log(f"active TCP: {name}")
        return True, f"active TCP: {name}"

    # ---------------------------------------------------------------- runner
    def _on_step(self, i: int):
        self.program_step = i
        if i >= 0:
            r = self._touch_run("program")
            r["step"] = max(r["step"], i)
        for fn in list(self.step_listeners):
            fn(i)

    def _on_state(self, s: RunState):
        self.command_source = "program" if s != RunState.IDLE else "idle"
        for fn in list(self.state_listeners):
            fn(s)

    def _on_done(self, ok: bool, msg: str):
        self.program_result = (ok, msg)
        self.bridge.log(f"program: {msg}")
        x = self._run_extras("program")
        self.last_run_recording = self.recording_dir() or self._step_rec or ""
        total = int(getattr(self.runner, "steps_total", 0) or 0)
        self.history.record("program", getattr(self.runner, "program_name", "?"),
                            getattr(self.runner, "started_at", None), ok, msg.splitlines()[0] if msg else "",
                            log=x.pop("_log"), speed_override=self.speed_override,
                            steps_done=total if ok else max(0, x.pop("_step")), steps_total=total,
                            recording=self.last_run_recording, **{k: v for k, v in x.items() if k != "_step"})
        self._step_rec = None
        for fn in list(self.done_listeners):
            fn((ok, msg))

    # ---------------------------------------------------------------- policy
    def _on_policy_feedback(self, fb: RunPolicyFeedback):
        self.policy_feedback = fb

    def _on_policy_done(self, res: RunPolicyResult):
        self.policy_result = res
        name, t0 = getattr(self.policies, "started", (None, None))
        if name:
            x = self._run_extras("policy")
            x.pop("_step")
            self.history.record("policy", name, t0, res.success, res.stop_reason, res.log_dir or "",
                                log=x.pop("_log"), recording=self.recording_dir(), **x)
        if self.command_source.startswith("policy"):
            self.command_source = "idle"
        self.bridge.log(f"policy: {res.stop_reason}" + (f" (log {res.log_dir})" if res.log_dir else ""))


class _CoreProgramHooks(ProgramHooks):
    """ProgramRunner access to the core: the force a program applies, F/T, the active TCP."""

    def __init__(self, core: RobotCore):
        self.core = core

    def active_tcp(self) -> str:
        return self.core.store.active_tcp

    def wrench(self):
        return wrench_fresh(self.core.bridge.snapshot())     # None = none or stale (older than 0.2 s)

    def force_apply(self, wrench, frame):
        return self.core.program_force_apply(wrench, frame)

    def force_progress(self):
        f = self.core._force
        if f is None or not f.alive:
            return None
        return list(f.current), list(f.target.wrench)

    def force_release(self, wait_s):
        return self.core.program_force_release(wait_s)

    def force_stop(self, reason):
        self.core.force_stop(reason)

    def zero_ft(self, stopped):
        return self.core.program_zero_ft(stopped)

    def record_start(self, presets, name, note):
        ok, msg = self.core.record_start(name=name, note=note, presets=presets or None)
        live = self.core.recorder.view()["recording"]
        if ok and live:
            self.core._step_rec = live["dir"]
        return ok, msg

    def record_stop(self, reason, wait=False):
        if wait:
            self.core.recorder.stop_and_wait(reason)
            return True, "recording finished"
        return self.core.recorder.stop(reason)

    def recording(self):
        return self.core.recording_dir() or None

    def photo(self, topic, label):
        return self.core.take_photo(topic, label)
