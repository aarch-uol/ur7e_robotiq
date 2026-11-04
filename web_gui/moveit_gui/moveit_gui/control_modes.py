"""Live control-mode switching (Position <-> Admittance <-> Impedance ...).

A mode is a set of ros2_control controllers. Switching is one STRICT
controller_manager request: deactivate every *managed* controller that is
active but not part of the target mode, activate the target set together
(chained controllers such as admittance must go in the same request).

A switch is a transaction:
  1. record the known-good previous mode
  2. preflight, cancel motion, stop freedrive, wait for a VERIFIED standstill
  3. one STRICT switch
  4. verify exactly the target mode's controllers are active
  5. verify command-interface ownership
  6. target-specific initialisation (MoveIt action server up; direct torque:
     the controller's reference equals the measured pose)
Any failure after step 3 deactivates the target and rolls back to the previous mode,
which is verified the same way. If the rollback cannot be verified the manager latches
CONTROL_STATE_UNKNOWN: motion stays disabled until someone recovers the stack by hand
and re-verifies it (verify_recovery()).

MoveIt routing has no state of its own: moveit_controller_for() derives the controller
from what is active, and the bridge names it in every ExecuteTrajectory goal.

ROS-free: talks to the robot only through the `bridge` object.
"""
from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

from .profile import ControlMode, RobotProfile

UNKNOWN = "unknown"   # managed controllers active in a combination no mode matches
NONE = "none"         # no managed motion controller active
CONTROL_STATE_UNKNOWN = "control_state_unknown"   # failed rollback: latched, motion disabled


@dataclass
class SwitchPlan:
    activate: List[str]
    deactivate: List[str]

    @property
    def empty(self) -> bool:
        return not self.activate and not self.deactivate


class ModeSwitchError(RuntimeError):
    pass


def active_set(states: Dict[str, str]) -> set:
    return {n for n, s in states.items() if s == "active"}


def detect_mode(profile: RobotProfile, states: Dict[str, str]) -> str:
    """Return the mode whose controllers are exactly the active managed ones."""
    active_managed = active_set(states) & set(profile.managed_controllers)
    if not active_managed:
        return NONE
    for mode in profile.modes.values():
        if set(mode.activate) == active_managed:
            return mode.name
    return UNKNOWN


def moveit_controller_for(profile: RobotProfile, states: Optional[Dict[str, str]]) -> Optional[str]:
    """The FollowJointTrajectory controller MoveIt must execute through right now, or None."""
    if not states:
        return None
    m = profile.modes.get(detect_mode(profile, states))
    return m.moveit_controller if m and m.supports_moveit and m.moveit_controller else None


def plan_switch(profile: RobotProfile, states: Dict[str, str], target: str) -> SwitchPlan:
    if target not in profile.modes:
        raise ModeSwitchError(f"unknown control mode '{target}'")
    wanted = profile.modes[target].activate
    missing = [c for c in wanted if c not in states]
    if missing:
        raise ModeSwitchError(
            "controller(s) not loaded: " + ", ".join(missing)
            + ". The bringup must load every mode's controllers (inactive) at startup.")
    active = active_set(states)
    deactivate = [c for c in profile.managed_controllers if c in active and c not in wanted]
    activate = [c for c in wanted if c not in active]
    return SwitchPlan(activate=activate, deactivate=deactivate)


def crosses_torque_boundary(profile: RobotProfile, current: str, target: str) -> bool:
    a, b = profile.modes.get(current), profile.modes.get(target)
    if a is None or b is None:
        return True          # unknown current state: treat as the more dangerous transition
    return a.direct_torque != b.direct_torque


def transition_steps(profile: RobotProfile, current: str, target: str) -> List[str]:
    """What a switch current -> target does, in order, as shown to the operator.

    Position <-> Admittance stays inside the position domain (same hardware interface).
    Anything <-> a direct-torque mode crosses the command-interface boundary and adds
    interface-release / ownership checks and reference initialisation.
    """
    t = profile.modes[target]
    cur = profile.modes.get(current)
    steps = ["Stop the active command source (program, policy, Servo, freedrive)",
             "Cancel the active trajectory",
             "Verify joint state is fresh and has velocities",
             "Verify the arm is stationary"]
    if t.requires_ft:
        steps.append("Verify the F/T sensor is publishing")
    old = ", ".join(cur.activate) if cur else "active arm controllers"
    if crosses_torque_boundary(profile, current, target):
        steps += [f"Deactivate {old}",
                  f"Verify the {cur.command_interface if cur else 'old'} interfaces are released",
                  f"Activate {', '.join(t.activate)} (STRICT)",
                  f"Verify {t.command_interface.upper()} interface ownership",
                  "Verify the controller's reference at the current pose" if t.direct_torque
                  else "Verify the " + (t.moveit_controller or "MoveIt") + " action server is up"]
    else:
        steps += [f"Switch {old} → {', '.join(t.activate)} in one STRICT request",
                  f"Verify active controllers and {t.command_interface.upper()} claims"]
        if t.supports_moveit and t.moveit_controller:
            steps.append(f"Verify the {t.moveit_controller} action server is up")
    steps.append("On any failure: deactivate the target and roll back to the previous mode")
    return steps


def claim_problems(profile: RobotProfile, target: str, claims: Dict[str, List[str]]) -> List[str]:
    """Check the arm joints' hardware interfaces are claimed with the target mode's interface only."""
    m = profile.modes[target]
    if m.command_interface == "none":
        return []
    joint_claims = {}
    for ctrl, ifaces in claims.items():
        if ctrl not in profile.managed_controllers:
            continue
        for iface in ifaces:
            joint, _, kind = iface.rpartition("/")
            if joint in profile.joint_names:
                joint_claims.setdefault(joint, set()).add(kind)
    out = []
    for j in profile.joint_names:
        kinds = joint_claims.get(j, set())
        if kinds != {m.command_interface}:
            out.append(f"{j}: claimed {sorted(kinds) or 'nothing'}, expected {m.command_interface}")
    return out


class ControlModeManager:
    """Fail-closed mode switching.

    Every refusal returns a sentence the GUI can show as-is, e.g.
    "Cannot enter Impedance: joint velocity state is stale".
    Unknown / stale robot state is never treated as safe.
    """

    def __init__(self, bridge, profile: RobotProfile, log=print,
                 stationary_timeout_s: float = 3.0, switch_timeout_s: float = 5.0,
                 policy_running: Callable[[], bool] = lambda: False,
                 program_running: Callable[[], bool] = lambda: False):
        self.bridge = bridge
        self.profile = profile
        self.log = log
        self.stationary_timeout_s = stationary_timeout_s
        self.switch_timeout_s = switch_timeout_s
        self.policy_running = policy_running
        self.program_running = program_running
        self._lock = threading.Lock()
        # Called with the target mode after the preflight, before anything is switched: (ok, msg). Not ok
        # refuses the switch (RobotCore: put the saved controller gains back on the still inactive controller).
        self.before_activate: Optional[Callable[[str], Tuple[bool, str]]] = None
        self.unknown_reason = ""             # non-empty = CONTROL_STATE_UNKNOWN is latched

    @property
    def control_state_unknown(self) -> bool:
        return bool(self.unknown_reason)

    def current_mode(self) -> str:
        if self.control_state_unknown:
            return CONTROL_STATE_UNKNOWN
        states = self.bridge.list_controllers()
        if states is None:
            return UNKNOWN
        return detect_mode(self.profile, states)

    def label(self, mode: str) -> str:
        m = self.profile.modes.get(mode)
        return m.label if m else mode

    # ------------------------------------------------------------ preflight
    def preflight(self, target: str, states: Optional[Dict[str, str]] = None,
                  from_program: bool = False) -> List[str]:
        """Everything that blocks a switch to `target`, as user-facing sentences. [] = OK."""
        m = self.profile.modes.get(target)
        if m is None:
            return [f"unknown control mode '{target}'"]
        out: List[str] = []
        if self.control_state_unknown:
            out.append(f"control state is unknown ({self.unknown_reason}); recover manually first")
        if not m.allow_activation:
            out.append(m.disabled_reason or "mode is disabled in the robot profile")
        if self.policy_running():
            out.append("a policy is running; stop it first")
        if self.program_running() and not from_program:
            out.append("a program is running; stop it first")
        age = self.bridge.joint_state_age()
        if age is None:
            out.append("no joint state received")
        elif age > self.profile.joint_state_timeout_s:
            out.append(f"joint state is stale ({age:.1f} s old)")
        elif not self.bridge.joint_velocities_available():
            out.append("joint velocity state is unavailable, cannot verify the arm is stationary")
        if states is None:
            states = self.bridge.list_controllers()
        if states is None:
            out.append("controller_manager not reachable")
        else:
            missing = [c for c in m.activate if c not in states]
            if missing:
                out.append("controller(s) not loaded: " + ", ".join(missing))
        snap = self.bridge.snapshot()
        if m.requires_ft and not snap.connected.get("F/T"):
            out.append("F/T sensor topic is not receiving data")
        if m.experimental:
            if snap.robot_mode != "RUNNING":
                out.append(f"robot mode is {snap.robot_mode}, needs RUNNING")
            if snap.safety_mode != "NORMAL":
                out.append(f"safety mode is {snap.safety_mode}, needs NORMAL")
            if snap.program_running is not True:
                out.append("External Control program is not running")
        return out

    def switch_to(self, target: str, from_program: bool = False) -> Tuple[bool, str]:
        """Blocking. Call from a worker thread, never the Qt thread."""
        if not self._lock.acquire(blocking=False):
            return False, "another mode switch is in progress"
        try:
            return self._switch(target, from_program)
        except ModeSwitchError as e:
            return False, f"Cannot enter {self.label(target)}: {e}"
        finally:
            self._lock.release()

    def _switch(self, target: str, from_program: bool) -> Tuple[bool, str]:
        states = self.bridge.list_controllers()
        if self.control_state_unknown:
            raise ModeSwitchError(f"control state is unknown ({self.unknown_reason}); recover manually first")
        if states is not None and target in self.profile.modes:
            if detect_mode(self.profile, states) == target:
                return True, f"already in {self.label(target)}"
        problems = self.preflight(target, states, from_program)
        if problems:
            raise ModeSwitchError("; ".join(problems))
        if self.before_activate is not None:
            ok, msg = self.before_activate(target)
            if not ok:
                raise ModeSwitchError(msg)
        previous = detect_mode(self.profile, states)
        plan = plan_switch(self.profile, states, target)

        # 1. stop anything that is commanding the robot through the GUI
        self.bridge.cancel_motion()
        if target != "freedrive":
            self.bridge.set_freedrive_keepalive(False)
        # 2. wait for verified standstill (None = cannot verify = refuse)
        deadline = time.monotonic() + self.stationary_timeout_s
        while True:
            still = self.bridge.is_stationary()
            if still is True:
                break
            if still is None:
                raise ModeSwitchError("cannot verify the arm is stationary (joint velocities stale or missing)")
            if time.monotonic() > deadline:
                raise ModeSwitchError("arm did not come to rest")
            time.sleep(0.05)
        # 2b. hardware the target's controllers read from (still before anything is switched)
        self._ensure_hardware(self.profile.modes[target])
        # 3. one strict request
        self.log(f"switch -> {target}: deactivate {plan.deactivate}, activate {plan.activate}")
        if not self.bridge.switch_controllers(plan.activate, plan.deactivate, strict=True,
                                              timeout_s=self.switch_timeout_s):
            now = self.bridge.list_controllers()
            if now is not None and detect_mode(self.profile, now) == previous:
                raise ModeSwitchError("controller_manager rejected the STRICT switch; still in "
                                      + self.label(previous))
            return self._rollback(target, previous, "controller_manager rejected the STRICT switch")
        # 4.-6. verify; any failure from here on must not leave the target running
        failure = self._verify_mode(target)
        if failure:
            return self._rollback(target, previous, failure)
        m = self.profile.modes[target]
        return True, f"now in {self.label(target)} ({m.command_interface.upper()} interface verified)"

    def _ensure_hardware(self, m: ControlMode, settle_s: float = 0.3):
        """Bring back a hardware component that ros2_control took down after a read error (e.g. the ATI topic
        bridge after a > 0.1 s gap). Only called after preflight, so a requires_ft mode has live F/T data now.
        The component must still be active after settle_s: a bridge that stops again is not used."""
        if not m.hardware:
            return
        states = self.bridge.hardware_states()
        if states is None:
            raise ModeSwitchError("cannot read the ros2_control hardware components")
        down = []
        for name in m.hardware:
            st = states.get(name)
            if st is None:
                raise ModeSwitchError(f"hardware component {name} is not loaded")
            if st != "active":
                self.log(f"hardware {name} is {st}: re-activating it")
                ok, msg = self.bridge.activate_hardware(name)
                if not ok:
                    raise ModeSwitchError(f"could not re-activate hardware {name}: {msg}")
                down.append(name)
        if not down:
            return
        time.sleep(settle_s)
        states = self.bridge.hardware_states() or {}
        for name in down:
            if states.get(name) != "active":
                raise ModeSwitchError(f"hardware {name} stopped again after re-activation "
                                      f"({states.get(name, 'unknown')}): check its data source")
            self.log(f"hardware {name} active again")

    def _verify_mode(self, mode: str) -> str:
        """Everything that must hold once `mode` is active. '' = verified, else the reason."""
        states = self.bridge.list_controllers()
        if states is None:
            return "controller_manager did not answer after the switch"
        now = detect_mode(self.profile, states)
        if now != mode:
            return f"switch reported success but the active controllers match '{now}'"
        m = self.profile.modes[mode]
        if m.command_interface != "none":
            claims = self.bridge.controller_claims()
            if claims is None:
                return f"{m.command_interface} interface ownership could not be verified"
            bad = claim_problems(self.profile, mode, claims)
            if bad:
                return "interface claims are wrong: " + "; ".join(bad)
        if m.direct_torque:
            ok, why = self.bridge.verify_reference_at_current_pose(m.reference_check)
            if not ok:
                return f"reference initialisation failed: {why}"
        elif m.supports_moveit and m.moveit_controller:
            if not self.bridge.trajectory_action_ready(m.moveit_controller):
                return f"{m.moveit_controller} action server is not available to MoveIt"
        return ""

    def _rollback(self, target: str, previous: str, failure: str) -> Tuple[bool, str]:
        """Leave `target` and return to `previous`. Latches CONTROL_STATE_UNKNOWN if that fails."""
        tl = self.label(target)
        self.log(f"switch to {target} failed ({failure}); rolling back to {previous}")
        self.bridge.cancel_motion()
        states = self.bridge.list_controllers()
        reason = ""
        if states is None:
            reason = "controller_manager not reachable during rollback"
        elif previous == UNKNOWN:
            # There is no known-good mode to return to, so only take the target back out.
            wanted = set(self.profile.modes[target].activate)
            off = [c for c in active_set(states) if c in wanted]
            self.bridge.switch_controllers([], off, strict=True, timeout_s=self.switch_timeout_s)
            reason = "no known-good mode to return to"
        else:
            wanted = self.profile.modes[previous].activate if previous in self.profile.modes else []
            active = active_set(states)
            off = [c for c in self.profile.managed_controllers if c in active and c not in wanted]
            on = [c for c in wanted if c not in active]
            if not self.bridge.switch_controllers(on, off, strict=True, timeout_s=self.switch_timeout_s):
                reason = "rollback switch was rejected"
            elif previous == NONE:
                now = self.bridge.list_controllers()
                if now is None or detect_mode(self.profile, now) != NONE:
                    reason = "target controllers are still active"
            else:
                reason = self._verify_mode(previous)
        if reason:
            self.unknown_reason = f"{failure}; rollback failed: {reason}"
            self.bridge.cancel_motion()
            self.log(f"CONTROL_STATE_UNKNOWN: {self.unknown_reason}")
            raise ModeSwitchError(f"{failure}. Rollback failed: {reason}. Control state is UNKNOWN: GUI motion "
                                  "is disabled until the stack is recovered manually")
        raise ModeSwitchError(f"{failure}. Rolled back to {self.label(previous)} (verified)")

    def verify_recovery(self) -> Tuple[bool, str]:
        """Operator recovered the stack by hand: clear the latch only if a real mode verifies."""
        if not self.control_state_unknown:
            return True, "control state is known"
        states = self.bridge.list_controllers()
        mode = detect_mode(self.profile, states) if states is not None else UNKNOWN
        if mode not in self.profile.modes:
            return False, f"active controllers match '{mode}', not a mode of this robot"
        problem = self._verify_mode(mode)
        if problem:
            return False, problem
        self.unknown_reason = ""
        self.log(f"control state recovered: {mode}")
        return True, f"control state recovered: {self.label(mode)} verified"

    def mode_supports_moveit(self, mode: Optional[str] = None) -> bool:
        if self.control_state_unknown:
            return False
        mode = mode or self.current_mode()
        m = self.profile.modes.get(mode)
        return bool(m and m.supports_moveit)


class ReferenceShiftError(ValueError):
    pass


def shift_to_reference(points: List[List[float]], names: List[str], reference: Dict[str, float],
                       max_shift_rad: float) -> Tuple[List[List[float]], List[float]]:
    """Shift a planned joint trajectory so it starts at the trajectory controller's reference.

    In a chained admittance mode the arm stands at reference + the admittance offset (a force pushes it). MoveIt
    plans from the measured joints, so its first point is the pushed pose; the trajectory controller jumps its
    reference there and the admittance controller adds the offset again: the push doubles at the start of every
    move (reproduced on the real controller chain: 20 mm under 5 N becomes 40 mm). Shifting every point by
    (reference - first point) keeps the reference continuous: the arm follows the planned path offset by the push
    it already has, so the press stays as it is. Returns (shifted points, shift per joint)."""
    if not points:
        raise ReferenceShiftError("empty trajectory")
    missing = [n for n in names if n not in reference]
    if missing:
        raise ReferenceShiftError(f"no reference for {', '.join(missing)}")
    d = [float(reference[n]) - float(points[0][i]) for i, n in enumerate(names)]
    worst = max(abs(v) for v in d)
    if worst > max_shift_rad:
        k = max(range(len(d)), key=lambda i: abs(d[i]))
        raise ReferenceShiftError(
            f"the arm is {math.degrees(worst):.1f} deg ({names[k]}) off its reference (pushed by a force or "
            f"blocked); limit {math.degrees(max_shift_rad):.1f} deg: release the force before moving")
    return [[float(p[i]) + d[i] for i in range(len(names))] for p in points], d
