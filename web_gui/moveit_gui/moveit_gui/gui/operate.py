"""OPERATE: the daily screen. Manual motion | robot state | control + gripper, with the F/T strip below."""
from __future__ import annotations

import math

from PyQt5 import QtCore, QtWidgets

from .. import transforms as tf
from ..geometry import GeometryError, PoseSpec
from ..jog import CART_AXES, JogCommand
from .control_stack import ControlStackDialog, mode_nodes
from .theme import SPACE, mono_font
from .widgets import (Card, CenterBar, Collapsible, Dot, FlowView, ReadoutGrid, button, hline, label, muted,
                      page_widget, section_label, set_kind)

JOINT_LABELS = ("J1", "J2", "J3", "J4", "J5", "J6")


def _compact_spin(lo, hi, suffix, width=86):
    sp = QtWidgets.QDoubleSpinBox()
    sp.setRange(lo, hi)
    sp.setDecimals(1)
    sp.setSuffix(suffix)
    sp.setFont(mono_font())
    sp.setAlignment(QtCore.Qt.AlignRight)
    sp.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
    sp.setProperty("compact", True)
    sp.setFixedWidth(width)
    return sp


class OperatePage(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self._seeded = {"joints": False, "pose": False}
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        sa = QtWidgets.QScrollArea()          # only scrolls when the window is really small
        sa.setWidgetResizable(True)
        sa.setFrameShape(QtWidgets.QFrame.NoFrame)
        page = page_widget()
        sa.setWidget(page)
        outer.addWidget(sa)
        lay = QtWidgets.QVBoxLayout(page)
        lay.setContentsMargins(SPACE["s"], SPACE["s"], SPACE["s"], SPACE["s"])
        lay.setSpacing(SPACE["s"])
        cols = QtWidgets.QHBoxLayout()
        cols.setSpacing(SPACE["s"])
        cols.addWidget(self._manual_card(), 5)
        cols.addWidget(self._state_card(), 3)
        right = QtWidgets.QVBoxLayout()
        right.setSpacing(SPACE["s"])
        right.addWidget(self._stack_card(), 1)
        right.addWidget(self._gripper_card())
        cols.addLayout(right, 4)
        lay.addLayout(cols, 1)
        lay.addWidget(self._ft_strip())

    # ============================================================ robot menu
    def _robot_menu(self):
        w = self.win
        more = button("Robot ▾", "quiet", "Power, PolyScope and Dashboard commands")
        menu = QtWidgets.QMenu(more)
        menu.addAction("Power on", lambda: w.dashboard("power_on"))
        menu.addAction("Brake release", lambda: w.dashboard("brake_release"))
        menu.addSeparator()
        menu.addAction("PolyScope play (External Control)", lambda: w.dashboard("play"))
        menu.addAction("PolyScope pause", lambda: w.dashboard("pause"))
        menu.addAction("PolyScope stop…", lambda: w.dashboard("stop", confirm=True))
        menu.addSeparator()
        menu.addAction("Unlock protective stop", lambda: w.dashboard("unlock_protective_stop"))
        menu.addAction("Restart safety…", lambda: w.dashboard("restart_safety", confirm=True))
        menu.addAction("Close popup", lambda: w.dashboard("close_popup"))
        menu.addAction("Close safety popup", lambda: w.dashboard("close_safety_popup"))
        menu.addSeparator()
        menu.addAction("Resend robot program (External Control)",
                       lambda: w.bg(w.bridge.resend_robot_program, w.report))
        menu.addAction("Load PolyScope program…", self._load_program)
        menu.addAction("Speed slider…", self._speed_dialog)
        menu.addAction("Digital outputs…", self._io_dialog)
        menu.addSeparator()
        menu.addAction("Power off…", lambda: w.dashboard("power_off", confirm=True))
        menu.addAction("Zero UR built-in F/T (not the ATI)…",
                       lambda: w.confirm("Zero UR F/T", "Zero the UR's built-in wrist sensor? The ATI is not "
                                         "touched.") and w.bg(w.bridge.zero_ft_sensor, w.report))
        more.setMenu(menu)
        return more

    def _load_program(self):
        name, ok = QtWidgets.QInputDialog.getText(self, "Load PolyScope program", "File on the robot",
                                                  text="external_control.urp")
        if ok:
            self.win.bg(lambda: self.win.bridge.load_program(name), self.win.report)

    def _speed_dialog(self):
        v, ok = QtWidgets.QInputDialog.getInt(self, "Speed slider", "Speed slider [%]", 30, 1, 100)
        if ok:
            self.win.bg(lambda: self.win.bridge.set_speed_slider(v / 100),
                        lambda r: self.win.report((r, f"speed slider {v}%" if r else "speed slider failed")))

    def _io_dialog(self):
        d = QtWidgets.QDialog(self)
        d.setWindowTitle("Digital outputs")
        lay = QtWidgets.QHBoxLayout(d)
        for pin in range(8):
            b = QtWidgets.QPushButton(f"DO{pin}")
            b.setCheckable(True)
            b.toggled.connect(lambda on, p=pin: self.win.bg(lambda: self.win.bridge.set_digital_out(p, on)))
            lay.addWidget(b)
        d.exec_()

    # =============================================================== manual
    def _manual_card(self):
        w = self.win
        c = Card()
        tabs = QtWidgets.QTabWidget()
        tabs.setCornerWidget(self._robot_menu())
        c.body.addWidget(tabs, 1)

        joint = QtWidgets.QWidget()
        jl = QtWidgets.QVBoxLayout(joint)
        jl.setContentsMargins(0, SPACE["s"], 0, 0)
        jl.addWidget(section_label("Joint target"))
        gl = QtWidgets.QGridLayout()
        gl.setVerticalSpacing(2)
        gl.setHorizontalSpacing(SPACE["m"])
        gl.setColumnStretch(1, 1)
        self.joint_spins = []
        for i, name in enumerate(w.profile.joint_names):
            gl.addWidget(label(JOINT_LABELS[i] if i < 6 else f"J{i + 1}", "field"), i, 0)
            gl.addWidget(label(name.replace("_joint", ""), "secondary"), i, 1)
            sp = _compact_spin(-360.0, 360.0, " °")
            self.joint_spins.append(sp)
            gl.addWidget(sp, i, 2)
        jl.addLayout(gl)
        row = QtWidgets.QHBoxLayout()
        wp = QtWidgets.QComboBox()
        wp.addItem("Waypoint…")
        for name in w.profile.waypoints:
            wp.addItem(name)
        wp.activated.connect(lambda i: self._load_waypoint(wp.itemText(i)) if i > 0 else None)
        row.addWidget(wp)
        b1 = button("Copy current", "quiet")
        b1.clicked.connect(self.copy_joints)
        row.addWidget(b1)
        row.addStretch(1)
        jl.addLayout(row)
        jl.addStretch(1)
        self.movej_btn = button("MoveJ", "primary")
        self.movej_btn.clicked.connect(self._movej)
        jl.addWidget(self.movej_btn)
        tabs.addTab(joint, "Joint")

        cart = QtWidgets.QWidget()
        cl = QtWidgets.QVBoxLayout(cart)
        cl.setContentsMargins(0, SPACE["s"], 0, 0)
        head = QtWidgets.QHBoxLayout()
        self.tcp_lbl = section_label("")
        head.addWidget(self.tcp_lbl)
        head.addStretch(1)
        head.addWidget(label("in", "field"))
        self.frame = QtWidgets.QComboBox()
        self.frame.currentTextChanged.connect(lambda _: self._reseed_pose())
        head.addWidget(self.frame)
        cl.addLayout(head)
        gl = QtWidgets.QGridLayout()
        gl.setVerticalSpacing(2)
        gl.setColumnStretch(1, 1)
        self.pose_spins = []
        for i, (name, rng, suf) in enumerate((("X", 2000, " mm"), ("Y", 2000, " mm"), ("Z", 2000, " mm"),
                                              ("Roll", 180, " °"), ("Pitch", 180, " °"), ("Yaw", 180, " °"))):
            gl.addWidget(label(name, "field"), i, 0)
            sp = _compact_spin(-rng, rng, suf, 100)
            sp.setDecimals(2)
            self.pose_spins.append(sp)
            gl.addWidget(sp, i, 2)
        cl.addLayout(gl)
        row = QtWidgets.QHBoxLayout()
        self.linear = QtWidgets.QCheckBox("Linear")
        row.addWidget(self.linear)
        b1 = button("Copy current", "quiet")
        b1.clicked.connect(lambda: self.copy_pose(quiet=False))
        row.addWidget(b1)
        row.addStretch(1)
        cl.addLayout(row)
        cl.addStretch(1)
        self.movep_btn = button("MovePose", "primary")
        self.movep_btn.clicked.connect(self._movep)
        cl.addWidget(self.movep_btn)
        tabs.addTab(cart, "Cartesian")
        tabs.addTab(self._jog_tab(), "Jog")
        tabs.currentChanged.connect(lambda _i: self._jog_release())

        settings = QtWidgets.QHBoxLayout()
        settings.setSpacing(SPACE["s"])
        settings.addWidget(label("Velocity", "field"))
        self.vel = self._scale_spin(float(w.profile.moveit.get("default_velocity_scaling", 0.1)))
        settings.addWidget(self.vel)
        self.acc = self._scale_spin(float(w.profile.moveit.get("default_acceleration_scaling", 0.1)))
        self.acc.setToolTip("Acceleration scaling")
        settings.addWidget(self.acc)
        self.acc_lbl = label("Accel", "field")
        settings.insertWidget(settings.count() - 1, self.acc_lbl)
        settings.addStretch(1)
        self.confirm = QtWidgets.QCheckBox("Confirm moves")
        self.confirm.setChecked(not w.fake)
        settings.addWidget(self.confirm)
        c.body.addLayout(settings)
        return c

    # ================================================================== jog
    def _jog_tab(self):
        """Step: one press = one short MoveIt move. Hold: MoveIt Servo while the button is down (the core stops
        it 0.3 s after the last renewal, so a frozen window stops the arm too)."""
        w = self.win
        page = QtWidgets.QWidget()
        lay = QtWidgets.QVBoxLayout(page)
        lay.setContentsMargins(0, SPACE["s"], 0, 0)
        row = QtWidgets.QHBoxLayout()
        self.jog_type = QtWidgets.QComboBox()
        for text, data in (("Cartesian · base", "base"), ("Cartesian · tool", "tool"), ("Joint", "joint")):
            self.jog_type.addItem(text, data)
        self.jog_type.currentIndexChanged.connect(lambda _i: self._jog_labels())
        row.addWidget(self.jog_type)
        self.jog_step = QtWidgets.QComboBox()
        self.jog_step.addItem("Hold (Servo)", None)
        cfg = w.core.jog_cfg
        for mm, deg in zip(cfg["step_mm"], cfg["step_deg"]):
            self.jog_step.addItem(f"{mm:g} mm · {deg:g} °", (float(mm), float(deg)))
        self.jog_step.setCurrentIndex(0)       # continuous Servo, matching the original MoveIt-GUI interaction
        row.addWidget(self.jog_step)
        row.addStretch(1)
        lay.addLayout(row)
        gl = QtWidgets.QGridLayout()
        gl.setVerticalSpacing(4)
        gl.setColumnStretch(3, 1)
        self.jog_labels, self.jog_buttons = [], []
        for i in range(max(6, len(w.profile.joint_names))):
            lab = label("", "field")
            gl.addWidget(lab, i, 0)
            self.jog_labels.append(lab)
            for col, sign in ((1, -1), (2, +1)):
                b = button("−" if sign < 0 else "+", "secondary")
                b.setFixedWidth(56)
                b.setAutoRepeat(False)
                b.pressed.connect(lambda i=i, sign=sign: self._jog_press(i, sign))
                b.released.connect(self._jog_release)
                gl.addWidget(b, i, col)
                self.jog_buttons.append(b)
        lay.addLayout(gl)
        self.jog_note = muted("")
        self.jog_note.setWordWrap(True)
        lay.addWidget(self.jog_note)
        lay.addStretch(1)
        self._jog_down = False
        self._jog_timer = QtCore.QTimer(self)
        self._jog_timer.setInterval(100)
        self._jog_timer.timeout.connect(self._jog_renew)
        self._jog_labels()
        return page

    def _jog_labels(self):
        joint = self.jog_type.currentData() == "joint"
        names = self.win.profile.joint_names
        for i, lab in enumerate(self.jog_labels):
            name = (names[i] if i < len(names) else None) if joint else (CART_AXES[i].upper() if i < 6 else None)
            lab.setText(f"J{i + 1} {name.replace('_joint', '')}" if joint and name else (name or ""))
            for b in self.jog_buttons[2 * i:2 * i + 2]:
                b.setVisible(bool(name))
            lab.setVisible(bool(name))

    def _jog_cmd(self, axis, sign) -> JogCommand:
        t = self.jog_type.currentData()
        return JogCommand.parse("joint" if t == "joint" else "cartesian", axis, sign,
                                "tool" if t == "tool" else "base", len(self.win.profile.joint_names))

    def _jog_press(self, axis, sign):
        w = self.win
        cmd = self._jog_cmd(axis, sign)
        v, a = self.vel.value() / 100.0, self.acc.value() / 100.0
        step = self.jog_step.currentData()
        if step is not None:
            size = step[0] if cmd.kind == "cartesian" and cmd.axis < 3 else step[1]
            w.bg(lambda: w.core.jog_step(cmd, size, v, a), lambda r: None if r[0] else w.report(r))
            return
        self._jog_down = True

        def started(res):
            if not res[0]:
                self._jog_down = False
                w.report(res)
            elif not self._jog_down:             # released before Servo answered
                w.bg(w.core.jog_stop)
            else:
                self._jog_timer.start()
        w.bg(lambda: w.core.jog_start(cmd, v), started)

    def _jog_renew(self):
        if not self._jog_down or not any(b.isDown() for b in self.jog_buttons) or not self.win.core.jog_hold():
            self._jog_release()

    def _jog_release(self):
        if not getattr(self, "_jog_down", False) and not self._jog_timer.isActive():
            return
        self._jog_down = False
        self._jog_timer.stop()
        self.win.bg(self.win.core.jog_stop)

    def _refresh_jog(self, s):
        core = self.win.core
        hold = self.jog_step.currentData() is None
        why = (core.jog_hold_refusal() or (None if core.jogging else core.motion_refusal())) if hold \
            else core.motion_refusal()
        if not self._jog_down:
            for b in self.jog_buttons:
                b.setEnabled(not why)
        warning = core.servo_warning() if core.jogging else ""
        self.jog_note.setText(f"Servo: {warning}" if warning else why if why and core.idle() else
                              "Hold a button to move, release to stop. Speed: Velocity." if hold else
                              "Each press moves one step (Cartesian: straight line, rotations about the TCP).")

    @staticmethod
    def _scale_spin(value):
        sp = QtWidgets.QSpinBox()
        sp.setRange(1, 100)
        sp.setSuffix(" %")
        sp.setValue(int(round(value * 100)))
        sp.setAlignment(QtCore.Qt.AlignRight)
        sp.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        sp.setFixedWidth(52)
        return sp

    def _load_waypoint(self, name):
        wpt = self.win.profile.waypoints.get(name)
        if wpt and wpt.joints:
            for sp, q in zip(self.joint_spins, wpt.joints):
                sp.setValue(math.degrees(q))

    def copy_joints(self):
        q = self.win.last_snap.joints
        if q:
            for sp, v in zip(self.joint_spins, q):
                sp.setValue(math.degrees(v))

    def copy_pose(self, quiet=True) -> bool:
        try:
            pose = self.win.geo.current_pose(self.frame.currentText(), self.win.store.active_tcp)
        except GeometryError as e:
            if not quiet:
                self.win.report((False, str(e)))
            return False
        xyz, rpy = tf.T_to_mm_deg(pose.T())
        for sp, v in zip(self.pose_spins, xyz + rpy):
            sp.setValue(v)
        return True

    def _reseed_pose(self):
        self._seeded["pose"] = self.copy_pose()

    def _movej(self):
        q = [math.radians(sp.value()) for sp in self.joint_spins]
        v, a = self.vel.value() / 100.0, self.acc.value() / 100.0
        self.win.start_motion(lambda: self.win.bridge.move_joints(q, v, a),
                              "MoveJ to " + ", ".join(f"{sp.value():.1f}°" for sp in self.joint_spins),
                              self.confirm.isChecked())

    def _movep(self):
        vals = [sp.value() for sp in self.pose_spins]
        pose = PoseSpec.from_T(self.frame.currentText(), self.win.store.active_tcp, tf.mm_deg_to_T(vals[:3], vals[3:]))
        try:
            target = self.win.geo.planning_target(pose)
        except GeometryError as e:
            self.win.report((False, f"cannot move: {e}"))
            return
        lin, v, a = self.linear.isChecked(), self.vel.value() / 100.0, self.acc.value() / 100.0
        self.win.start_motion(lambda: self.win.bridge.move_pose(target.link, target.T_base_link, lin, v, a),
                              f"{'Linear move' if lin else 'Move'}: TCP {pose.tcp} to "
                              f"{[round(x, 1) for x in vals[:3]]} mm in {pose.frame}\n(MoveIt goal for {target.link})",
                              self.confirm.isChecked())

    # ================================================================ state
    def _state_card(self):
        c = Card()
        head = QtWidgets.QHBoxLayout()
        self.state_tcp = section_label("TCP")
        head.addWidget(self.state_tcp)
        head.addStretch(1)
        self.state_frame = QtWidgets.QComboBox()
        self.state_frame.setToolTip("Reference frame of the displayed pose")
        head.addWidget(self.state_frame)
        c.body.addLayout(head)
        self.pose_grid = ReadoutGrid([(k, k) for k in ("X", "Y", "Z")])
        c.body.addWidget(self.pose_grid)
        c.body.addSpacing(SPACE["xs"])
        self.rot_grid = ReadoutGrid([("R", "Roll"), ("P", "Pitch"), ("Yw", "Yaw")])
        c.body.addWidget(self.rot_grid)
        c.body.addSpacing(SPACE["s"])
        joints = Collapsible("Joints")
        self.joint_grid = ReadoutGrid()
        for i, name in enumerate(self.win.profile.joint_names):
            self.joint_grid.add(i, JOINT_LABELS[i] if i < 6 else f"J{i + 1}", name)
        joints.content_layout.addWidget(self.joint_grid)
        c.body.addWidget(joints)
        c.body.addStretch(1)
        return c

    # ======================================================= control stack
    def _stack_card(self):
        c = Card()
        head = QtWidgets.QHBoxLayout()
        self.mode_lbl = label("—", "title")
        head.addWidget(self.mode_lbl)
        self.exp_badge = label("EXPERIMENTAL", "badge_warn")
        head.addWidget(self.exp_badge)
        head.addStretch(1)
        self.iface_lbl = label("", "section")
        head.addWidget(self.iface_lbl)
        c.body.addLayout(head)
        self.flow = FlowView()
        c.body.addWidget(self.flow)
        self.unknown_banner = label("", "banner_bad", wrap=True)
        c.body.addWidget(self.unknown_banner)
        c.body.addSpacing(SPACE["xs"])
        self.ati_row = QtWidgets.QWidget()
        al = QtWidgets.QHBoxLayout(self.ati_row)
        al.setContentsMargins(0, 0, 0, 0)
        al.setSpacing(5)
        al.addWidget(label("ATI Gamma", "field"))
        self.ati_dot = Dot(7)
        al.addWidget(self.ati_dot)
        self.ati_lbl = label("—", "secondary")
        al.addWidget(self.ati_lbl)
        al.addStretch(1)
        c.body.addWidget(self.ati_row)
        self.source_lbl = label("", "secondary")
        c.body.addWidget(self.source_lbl)
        c.body.addStretch(1)
        row = QtWidgets.QHBoxLayout()
        self.change_btn = button("Change…", "secondary")
        self.change_btn.clicked.connect(lambda: ControlStackDialog(self.win).exec_())
        row.addWidget(self.change_btn)
        self.recover_btn = button("Re-verify", "secondary", "After recovering the controllers by hand, check whether a "
                                                            "known mode is really active")
        self.recover_btn.clicked.connect(self._recover)
        row.addWidget(self.recover_btn)
        row.addWidget(self.win.freedrive_button())
        row.addStretch(1)
        c.body.addLayout(row)
        return c

    def _recover(self):
        w = self.win
        w.bg(w.modes.verify_recovery, lambda r: w.report(r) if r[0] else w.show_refusal(r[1]))

    # ============================================================== gripper
    def _gripper_card(self):
        w = self.win
        g = w.profile.gripper
        c = Card()
        if not g.get("enabled"):
            c.body.addWidget(section_label("Gripper"))
            c.body.addWidget(muted("No gripper in this robot profile."))
            self.grip_status = None
            return c
        head = QtWidgets.QHBoxLayout()
        title = section_label(g.get("model", "gripper"))
        title.setToolTip("Runs on its own controller manager, independent of the arm control mode.")
        head.addWidget(title)
        head.addStretch(1)
        self.grip_dot = Dot(7)
        head.addWidget(self.grip_dot)
        self.grip_status = label("", "secondary")
        head.addWidget(self.grip_status)
        c.body.addLayout(head)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(label("Closed", "field"))
        row.addStretch(1)
        self.grip_val = label("—", "value")
        self.grip_val.setFont(mono_font())
        row.addWidget(self.grip_val)
        c.body.addLayout(row)
        self.grip_bar = QtWidgets.QProgressBar()
        self.grip_bar.setRange(0, 100)
        self.grip_bar.setProperty("tone", "neutral")
        self.grip_bar.setTextVisible(False)
        c.body.addWidget(self.grip_bar)
        lo, hi = float(g.get("open_position", 0.0)), float(g.get("closed_position", 0.8))
        self._grip_range = (lo, hi)
        row = QtWidgets.QHBoxLayout()
        ob = button("Open", "secondary")
        ob.clicked.connect(lambda: self._gripper(lo))
        cb = button("Close", "secondary")
        cb.clicked.connect(lambda: self._gripper(hi))
        row.addWidget(ob)
        row.addWidget(cb)
        c.body.addLayout(row)
        adv = Collapsible("Advanced")
        row = QtWidgets.QHBoxLayout()
        row.addWidget(label("Target", "field"))
        self.grip_target = QtWidgets.QSpinBox()
        self.grip_target.setRange(0, 100)
        self.grip_target.setSuffix(" %")
        self.grip_target.setAlignment(QtCore.Qt.AlignRight)
        self.grip_target.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.grip_target.setFixedWidth(52)
        row.addWidget(self.grip_target)
        row.addWidget(label("Force", "field"))
        self.grip_effort = QtWidgets.QDoubleSpinBox()
        self.grip_effort.setRange(0, 1000)
        self.grip_effort.setDecimals(0)
        self.grip_effort.setValue(float(g.get("max_effort", 40.0)))
        self.grip_effort.setSuffix(" N")
        self.grip_effort.setAlignment(QtCore.Qt.AlignRight)
        self.grip_effort.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.grip_effort.setFixedWidth(52)
        row.addWidget(self.grip_effort)
        go = button("Go", "quiet")
        go.clicked.connect(lambda: self._gripper(lo + (hi - lo) * self.grip_target.value() / 100))
        row.addWidget(go)
        row.addStretch(1)
        adv.content_layout.addLayout(row)
        self.grip_adv = label("", "muted", wrap=True, selectable=True)
        adv.content_layout.addWidget(self.grip_adv)
        if g.get("reactivate_service"):
            rb = button("Reactivate gripper", "secondary", str(g["reactivate_service"]))
            rb.clicked.connect(lambda: w.bg(w.bridge.gripper_reactivate, w.report))
            adv.content_layout.addWidget(rb)
        c.body.addWidget(adv)
        return c

    def _gripper(self, pos: float):
        eff = self.grip_effort.value()
        self.win.bg(lambda: self.win.bridge.gripper_move(pos, eff),
                    lambda ok: self.win.report((ok, "gripper done" if ok else "gripper failed")))

    # ================================================================== F/T
    def _ft_strip(self):
        c = Card()
        row = QtWidgets.QHBoxLayout()
        row.setSpacing(SPACE["m"])
        c.body.addLayout(row)
        col = QtWidgets.QVBoxLayout()
        col.setSpacing(0)
        t = section_label("ATI F/T")
        t.setToolTip("The OPERATE strip is read-only. The bias is software only (web SETUP › F/T + Payload); the ATI box is never biased.")
        col.addWidget(t)
        self.ft_state = label("", "muted")
        col.addWidget(self.ft_state)
        row.addLayout(col)
        self.ft_cells = []
        for name in ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz"):
            box = QtWidgets.QVBoxLayout()
            box.setSpacing(3)
            head = QtWidgets.QHBoxLayout()
            head.addWidget(label(name, "field"))
            head.addStretch(1)
            lab = label("—", "value")
            lab.setFont(mono_font())
            head.addWidget(lab)
            box.addLayout(head)
            bar = CenterBar()
            box.addWidget(bar)
            row.addLayout(box, 1)
            self.ft_cells.append((name, lab, bar))
        return c

    # ============================================================== refresh
    def calibration_changed(self):
        w = self.win
        for cb in (self.frame, self.state_frame):
            cur = cb.currentText()
            cb.blockSignals(True)
            cb.clear()
            cb.addItems(w.store.frame_names())
            i = cb.findText(cur)
            cb.setCurrentIndex(i if i >= 0 else 0)
            cb.blockSignals(False)
        self.tcp_lbl.setText(f"TARGET · {w.store.active_tcp.upper()}")
        self.state_tcp.setText(f"TCP · {w.store.active_tcp.upper()}")
        self._reseed_pose()

    def refresh(self, s):
        w = self.win
        if s.joints and not self._seeded["joints"]:
            self.copy_joints()
            self._seeded["joints"] = True
        if not self._seeded["pose"]:
            self._seeded["pose"] = self.copy_pose()
        try:
            pose = w.geo.current_pose(self.state_frame.currentText(), w.store.active_tcp)
            xyz, rpy = tf.T_to_mm_deg(pose.T())
            for k, v in zip(("X", "Y", "Z"), xyz):
                self.pose_grid.set(k, f"{v:9.1f} mm")
            for k, v in zip(("R", "P", "Yw"), rpy):
                self.rot_grid.set(k, f"{v:9.1f} °")
        except GeometryError:
            for k in ("X", "Y", "Z"):
                self.pose_grid.set(k, "—")
            for k in ("R", "P", "Yw"):
                self.rot_grid.set(k, "—")
        for i, q in enumerate(s.joints or []):
            self.joint_grid.set(i, f"{math.degrees(q):9.1f} °")
        self._refresh_stack(s)
        self._refresh_jog(s)
        if self.grip_status is not None:
            self._refresh_gripper(s)
        wr = s.wrench
        for i, (_name, lab, bar) in enumerate(self.ft_cells):
            if wr is None:
                lab.setText("—")
                bar.set_fraction(0)
            else:
                lab.setText(f"{wr[i]:+.2f} {'N' if i < 3 else 'Nm'}")
                bar.set_fraction(wr[i] / (20.0 if i < 3 else 2.0))
        self.ft_state.setText("live" if s.connected.get("F/T") else "no data")

    def _refresh_stack(self, s):
        w = self.win
        unknown = w.modes.control_state_unknown
        m = w.profile.modes.get(w.mode)
        self.unknown_banner.setVisible(unknown)
        self.recover_btn.setVisible(unknown)
        self.change_btn.setEnabled(not unknown)
        for b in (self.movej_btn, self.movep_btn):
            b.setEnabled(not unknown and bool(m and m.supports_moveit))
        if unknown:
            self.mode_lbl.setText("Control state unknown")
            self.iface_lbl.setText("MOTION DISABLED")
            self.exp_badge.setVisible(False)
            self.unknown_banner.setText(w.modes.unknown_reason)
            self.flow.set_nodes([])
            self.ati_row.setVisible(False)
        else:
            self.mode_lbl.setText(m.label if m else w.mode.capitalize())
            self.iface_lbl.setText(("DIRECT TORQUE" if m.direct_torque else m.command_interface.upper()) if m else "")
            self.exp_badge.setVisible(bool(m and m.experimental))
            self.flow.set_nodes(mode_nodes(m) if m else [])
            self.flow.setToolTip(" → ".join(m.chain or m.activate) if m else "See DIAGNOSTICS.")
            self.ati_row.setVisible(bool(m and m.requires_ft))
            if m and m.requires_ft:
                live = bool(s.connected.get("F/T"))
                self.ati_dot.set_state(live)
                self.ati_lbl.setText("Live" if live else "Silent")
        self.source_lbl.setText(f"Commanding: {w.command_source}")
        self.source_lbl.setVisible(w.command_source != "idle")

    def _refresh_gripper(self, s):
        w = self.win
        g = w.profile.gripper
        ctrl = g.get("controller")
        cstate = s.gripper_controllers.get(ctrl) if ctrl else None
        if not s.connected.get("Gripper"):
            st, dot = "Unavailable", None
        elif cstate is not None and cstate != "active":
            st, dot = f"Fault · {cstate}", False
        elif s.gripper_status and s.gripper_status["fault_code"] not in (0, 255):
            st, dot = f"Fault · {s.gripper_status['fault']}", False
        elif s.gripper_status:
            st, dot = s.gripper_status["object"].capitalize(), True
        else:
            st, dot = "Ready", True
        self.grip_status.setText(st)
        self.grip_dot.set_state(dot)
        set_kind(self.grip_status, "error" if dot is False else "secondary")
        if s.gripper_position is not None:
            lo, hi = self._grip_range
            pct = int(round(100 * (s.gripper_position - lo) / (hi - lo))) if hi != lo else 0
            self.grip_bar.setValue(pct)
            self.grip_val.setText(f"{pct} %")
        gj = s.gripper_joint or [float("nan")] * 3
        ctrls = ", ".join(f"{k}: {v}" for k, v in sorted(s.gripper_controllers.items())) or "unknown"
        gs = s.gripper_status
        status = (f"\nmotor current {gs['current_a']:.2f} A · fault {gs['fault']} ({gs['severity']})" if gs else "")
        self.grip_adv.setText(f"{g.get('joint_name')}\npos {gj[0]:.4f}  vel {gj[1]:.4f}  effort {gj[2]:.3f}"
                              f"{status}\n{g.get('controller_manager', '—')}\n{ctrls}")
