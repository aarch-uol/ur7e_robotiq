"""'Change control mode' dialog: two domains separated by the direct-torque boundary.

Trajectory / position domain (Position, Admittance) shares the POSITION hardware interface.
The direct-torque domain (Impedance) claims EFFORT interfaces and takes a Cartesian reference,
not MoveIt trajectories. Selecting a target shows the ordered transition and the live preflight;
Apply runs the switch through ControlModeManager (which re-checks everything).
"""
from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from ..control_modes import crosses_torque_boundary, transition_steps
from .theme import SPACE, T
from .widgets import Collapsible, button, hline, label, muted, repolish, section_label

SOURCE_TEXT = {"trajectory": "MoveIt trajectory (FollowJointTrajectory)",
               "cartesian_reference": "Cartesian reference pose (+ optional wrench)",
               "manual": "hand guiding"}


def mode_flow(m) -> str:
    """Human-readable command path of a mode, e.g. 'MoveIt → a → b → POSITION'."""
    src = {"trajectory": "MoveIt", "cartesian_reference": "Cartesian reference", "manual": "hand"}.get(
        m.command_source, m.command_source)
    chain = m.chain or m.activate
    iface = "joint " + m.command_interface.upper() if m.command_interface != "none" else "robot freedrive"
    return " → ".join([src, *chain, iface])


def mode_nodes(m):
    """A mode's behaviour as the operator should read it: no ROS names (those are in tooltips / DIAGNOSTICS)."""
    if m.direct_torque:
        return ["Cartesian reference", "Impedance controller", "UR effort"]
    if m.command_source == "manual":
        return ["Hand guiding", "UR freedrive"]
    if m.chain:
        return ["MoveIt", "Trajectory", m.label, f"UR {m.command_interface}"]
    return ["MoveIt", "Scaled trajectory", f"UR {m.command_interface}"]


class ModeOption(QtWidgets.QFrame):
    clicked = QtCore.pyqtSignal(str)

    def __init__(self, mode, current: bool, parent=None):
        super().__init__(parent)
        self.mode = mode
        self.setCursor(QtCore.Qt.PointingHandCursor)
        self.setObjectName("modeopt")
        self.setToolTip(mode_flow(mode))
        self.set_selected(False)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(SPACE["m"], SPACE["s"], SPACE["m"], SPACE["s"])
        lay.setSpacing(3)
        head = QtWidgets.QHBoxLayout()
        head.addWidget(label(mode.label, "title"))
        if mode.experimental:
            head.addWidget(label("EXPERIMENTAL", "badge_warn"), 0, QtCore.Qt.AlignVCenter)
        head.addStretch(1)
        if current:
            head.addWidget(label("ACTIVE", "badge"), 0, QtCore.Qt.AlignVCenter)
        lay.addLayout(head)
        lay.addWidget(label(" → ".join(mode_nodes(mode)), "flow", wrap=True))
        if mode.requires_ft:
            lay.addWidget(muted("ATI Gamma required"))
        if mode.note:
            lay.addWidget(muted(mode.note))
        if not mode.allow_activation:
            lay.addWidget(label(f"Disabled: {mode.disabled_reason}", "warn", wrap=True))

    def set_selected(self, on: bool):
        self.setStyleSheet(f"#modeopt {{ background:{T['selected'] if on else T['raised']}; border-radius:3px; }}")

    def mousePressEvent(self, e):
        self.clicked.emit(self.mode.name)


class ControlStackDialog(QtWidgets.QDialog):
    def __init__(self, win, parent=None):
        super().__init__(parent or win)
        self.win = win
        self.setWindowTitle("Arm control stack")
        self.resize(720, 560)
        prof, current = win.profile, win.mode
        outer = QtWidgets.QVBoxLayout(self)
        sa = QtWidgets.QScrollArea()
        sa.setWidgetResizable(True)
        sa.setFrameShape(QtWidgets.QFrame.NoFrame)
        inner = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(inner)
        sa.setWidget(inner)
        outer.addWidget(sa, 1)
        lay.setSpacing(SPACE["s"])
        self.options = {}
        pos_modes = [m for m in prof.visible_modes() if not m.direct_torque]
        dt_modes = [m for m in prof.visible_modes() if m.direct_torque]
        lay.addWidget(section_label("Position domain"))
        grid = QtWidgets.QHBoxLayout()
        for m in pos_modes:
            o = ModeOption(m, m.name == current)
            o.clicked.connect(self._select)
            self.options[m.name] = o
            grid.addWidget(o, 1)
        lay.addLayout(grid)
        if dt_modes:
            lay.addSpacing(SPACE["xs"])
            sep = label("DIRECT TORQUE BOUNDARY", "warn")
            sep.setToolTip("Crossing releases the joint POSITION interfaces and claims EFFORT. MoveIt trajectories "
                           "are not available on the other side.")
            bl = QtWidgets.QHBoxLayout()
            bl.addWidget(hline(), 1)
            bl.addWidget(sep)
            bl.addWidget(hline(), 1)
            lay.addLayout(bl)
            for m in dt_modes:
                o = ModeOption(m, m.name == current)
                o.clicked.connect(self._select)
                self.options[m.name] = o
                lay.addWidget(o)
        lay.addSpacing(SPACE["xs"])
        lay.addWidget(section_label("Transition"))
        self.plan_lbl = label("Select a mode.", "secondary", wrap=True)
        lay.addWidget(self.plan_lbl)
        self.preflight_lbl = label("", "", wrap=True)
        lay.addWidget(self.preflight_lbl)
        self.steps = Collapsible("Steps")
        self.steps_lbl = label("", "muted", wrap=True)
        self.steps.content_layout.addWidget(self.steps_lbl)
        lay.addWidget(self.steps)
        lay.addStretch(1)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(muted("The gripper is not affected.", wrap=False))
        row.addStretch(1)
        cancel = button("Close", "quiet")
        cancel.clicked.connect(self.reject)
        row.addWidget(cancel)
        self.apply_btn = button("Apply", "primary")
        self.apply_btn.setEnabled(False)
        self.apply_btn.clicked.connect(self._apply)
        row.addWidget(self.apply_btn)
        outer.addLayout(row)
        self.target = None

    def _select(self, name: str):
        self.target = name
        for n, o in self.options.items():
            o.set_selected(n == name)
        prof, cur = self.win.profile, self.win.mode
        m = prof.modes[name]
        if name == cur:
            self.plan_lbl.setText(f"{m.label} is already active.")
            self.preflight_lbl.setText("")
            self.apply_btn.setEnabled(False)
            return
        cross = crosses_torque_boundary(prof, cur, name)
        steps = transition_steps(prof, cur, name)
        self.plan_lbl.setText(f"<b style='color:{T['warn']}'>Crosses the direct-torque boundary</b>" if cross
                              else "Stays in the position domain")
        self.steps_lbl.setText("<br>".join(f"{i}. {s}" for i, s in enumerate(steps, 1)))
        self.preflight_lbl.setText("Checking preflight…")
        self.apply_btn.setEnabled(False)
        self.apply_btn.setText(f"Enter {m.label} (direct torque)" if m.direct_torque else f"Switch to {m.label}")
        self.apply_btn.setProperty("role", "experimental" if cross else "primary")
        repolish(self.apply_btn)
        self.win.bg(lambda: self.win.modes.preflight(name), lambda probs, n=name: self._preflight_done(n, probs))

    def _preflight_done(self, name, problems):
        if name != self.target:
            return
        if problems:
            self.preflight_lbl.setText(f"<b style='color:{T['danger']}'>Cannot switch now</b><br>"
                                       + "<br>".join(f"✗ {p}" for p in problems))
            self.apply_btn.setEnabled(False)
        else:
            self.preflight_lbl.setText(f"<span style='color:{T['ok']}'>✓ Preflight passed</span> "
                                       "(checked again when you apply)")
            self.apply_btn.setEnabled(True)

    def _apply(self):
        name = self.target
        m = self.win.profile.modes[name]
        if m.direct_torque and not self.win.confirm(
                f"{m.label} — EXPERIMENTAL",
                f"Enter direct-torque control ({', '.join(m.activate)})?\nKeep the e-stop within reach."):
            return
        self.apply_btn.setEnabled(False)
        self.preflight_lbl.setText("Switching…")

        def done(res):
            ok, msg = res
            if ok:
                self.win.report(res)
                self.accept()
            else:
                self.win.show_refusal(msg)
                self.preflight_lbl.setText(f"<b style='color:{T['danger']}'>{msg}</b>")
        self.win.bg(lambda: self.win.modes.switch_to(name), done)
