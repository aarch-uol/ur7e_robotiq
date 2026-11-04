"""PROGRAMS: step table + selected-step inspector + run bar.

Secondary actions (duplicate, move, delete, run one step) live in the inspector footer,
the right-click menu and keyboard shortcuts (Ctrl+D, Alt+Up/Down, Del, Ctrl+Enter),
not as a column of equal buttons.
"""
from __future__ import annotations

import copy
import math
from pathlib import Path
from typing import Optional

import yaml
from PyQt5 import QtCore, QtGui, QtWidgets

from .. import transforms as tf
from ..geometry import GeometryError, PoseSpec
from ..program import STEP_TYPES, Program, ProgramWaypoint, RunState, Step
from .theme import SPACE, T, mono_font
from .widgets import Card, button, hline, label, muted, page_widget, repolish, section_label

STEP_TEMPLATES = {
    "movej": {"target": "home", "velocity": 0.1},
    "move_pose": {"pose": {"frame": "base_link", "tcp": "tool0", "position": [0.4, 0.0, 0.3],
                           "quat_xyzw": [1.0, 0.0, 0.0, 0.0]}, "cartesian": False, "velocity": 0.1},
    "gripper": {"command": "close", "effort": 40},
    "wait": {"seconds": 1.0},
    "controller": {"mode": "position"},
    "policy": {"policy": "name_of_policy", "duration": 8.0},
    "set_io": {"pin": 0, "state": True},
    "speed": {"fraction": 0.5},
    "comment": {"text": ""},
}
BADGE = {"movej": "MOVE J", "move_pose": "MOVE POSE", "gripper": "GRIPPER", "wait": "WAIT", "controller": "CONTROL",
         "policy": "POLICY", "set_io": "SET I/O", "speed": "SPEED",
         "loop": "LOOP", "folder": "FOLDER", "comment": "COMMENT"}
TYPE_LABEL = {"movej": "MoveJ", "move_pose": "MovePose", "gripper": "Gripper", "wait": "Wait",
              "controller": "Mode", "policy": "Policy", "set_io": "Set I/O", "speed": "Speed",
              "loop": "Loop", "folder": "Folder", "comment": "Comment"}


class BadgeDelegate(QtWidgets.QStyledItemDelegate):
    """Step type as a small neutral chip: scannable without colouring rows."""
    def paint(self, painter, option, index):
        selected = option.state & QtWidgets.QStyle.State_Selected
        bg = index.data(QtCore.Qt.BackgroundRole)
        if selected:
            painter.fillRect(option.rect, QtGui.QColor(T["selected"]))
        elif bg is not None and bg.style() != QtCore.Qt.NoBrush:
            painter.fillRect(option.rect, bg)
        painter.fillRect(option.rect.left(), option.rect.bottom(), option.rect.width(), 1, QtGui.QColor(T["app"]))
        text = index.data() or ""
        font = QtGui.QFont(option.font)
        font.setPixelSize(10)
        font.setBold(True)
        fm = QtGui.QFontMetrics(font)
        r = QtCore.QRect(option.rect.left() + 6, option.rect.center().y() - 8, fm.horizontalAdvance(text) + 10, 16)
        painter.save()
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QColor(T["sunken"] if not selected else T["raised"]))
        painter.drawRoundedRect(r, 2, 2)
        painter.setFont(font)
        painter.setPen(QtGui.QColor(T["text2"]))
        painter.drawText(r, QtCore.Qt.AlignCenter, text)
        painter.restore()

    def sizeHint(self, option, index):
        return QtCore.QSize(92, 26)


class ProgramsPage(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.program = Program(name="untitled", robot=win.profile.name)
        self.path: Optional[Path] = None
        self._loading = False
        page = page_widget()
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)
        lay = QtWidgets.QVBoxLayout(page)
        lay.setContentsMargins(SPACE["s"], SPACE["s"], SPACE["s"], SPACE["s"])
        lay.setSpacing(SPACE["s"])

        top = QtWidgets.QHBoxLayout()
        self.title = QtWidgets.QLabel()
        self.title.setProperty("kind", "title")
        top.addWidget(self.title)
        top.addStretch(1)
        file_btn = button("File ▾", "quiet")
        fm = QtWidgets.QMenu(file_btn)
        fm.addAction("New", self.new_program)
        fm.addAction("New scraping program…", self.new_scrape_program)
        fm.addAction("Open…", self.open_program)
        fm.addAction("Save as…", lambda: self.save_program(ask=True))
        fm.addSeparator()
        fm.addAction("Check program", self.check_program)
        file_btn.setMenu(fm)
        top.addWidget(file_btn)
        self.save_btn = button("Save", "secondary", "Ctrl+S")
        self.save_btn.clicked.connect(self.save_program)
        top.addWidget(self.save_btn)
        self._saved = None
        lay.addLayout(top)
        self.notes = QtWidgets.QLabel()
        self.notes.setWordWrap(True)
        self.notes.setProperty("kind", "banner_warn")
        self.notes.hide()
        lay.addWidget(self.notes)

        split = QtWidgets.QSplitter()
        lay.addWidget(split, 1)
        left = Card()
        self.table = QtWidgets.QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["#", "STEP", "DETAILS"])
        self.table.setShowGrid(False)
        self.table.setItemDelegateForColumn(1, BadgeDelegate(self.table))
        self.table.horizontalHeader().setDefaultAlignment(QtCore.Qt.AlignLeft)
        self.table.verticalHeader().setDefaultSectionSize(28)
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.table.customContextMenuRequested.connect(self._context_menu)
        self.table.itemSelectionChanged.connect(self._selection_changed)
        left.body.addWidget(self.table, 1)
        row = QtWidgets.QHBoxLayout()
        add = button("+ Add step ▾", "secondary")
        menu = QtWidgets.QMenu(add)
        menu.addAction("MoveJ — current joints", self.record_joints)
        menu.addAction("MovePose — current pose", self.record_pose)
        menu.addSeparator()
        for t in STEP_TYPES:
            if t not in STEP_TEMPLATES:          # loops and folders: edited in the web GUI (nested steps)
                continue
            menu.addAction(TYPE_LABEL[t], lambda t=t: self.add_step(self._template(t)))
        add.setMenu(menu)
        row.addWidget(add)
        row.addWidget(win.freedrive_button())
        rec = button("Record pose", "secondary", "Adds a MovePose step to where the TCP is now")
        rec.clicked.connect(self.record_pose)
        row.addWidget(rec)
        row.addStretch(1)
        row.addWidget(muted("in", wrap=False))
        self.record_frame = QtWidgets.QComboBox()
        self.record_frame.currentTextChanged.connect(lambda _: self._update_record_label())
        row.addWidget(self.record_frame)
        self.record_lbl = muted("", wrap=False)
        row.addWidget(self.record_lbl)
        left.body.addLayout(row)
        split.addWidget(left)

        self.inspector = Card()
        self.insp_title = label("No step selected", "title")
        self.inspector.body.addWidget(self.insp_title)
        self.form_host = QtWidgets.QWidget()
        self.form = QtWidgets.QFormLayout(self.form_host)
        self.form.setContentsMargins(0, 0, 6, 0)
        form_scroll = QtWidgets.QScrollArea()          # long forms (MovePose) must not grow the window
        form_scroll.setWidgetResizable(True)
        form_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        form_scroll.setWidget(self.form_host)
        self.inspector.body.addWidget(form_scroll, 1)
        foot = QtWidgets.QHBoxLayout()
        foot.setSpacing(2)
        self.run_one = button("Run step", "secondary", "Run only this step (Ctrl+Enter)")
        self.run_one.clicked.connect(self.run_selected_step)
        foot.addWidget(self.run_one)
        foot.addStretch(1)
        dup = button("Duplicate", "quiet", "Ctrl+D")
        dup.clicked.connect(self.duplicate_step)
        foot.addWidget(dup)
        dele = button("Delete", "quiet", "Del")
        dele.clicked.connect(self.delete_step)
        foot.addWidget(dele)
        more = button("⋯", "quiet", "More")
        mm = QtWidgets.QMenu(more)
        mm.addAction("Move up  (Alt+Up)", lambda: self.move_step(-1))
        mm.addAction("Move down  (Alt+Down)", lambda: self.move_step(1))
        mm.addAction("Edit YAML…", self.edit_yaml)
        more.setMenu(mm)
        foot.addWidget(more)
        self.inspector.body.addWidget(hline())
        self.inspector.body.addLayout(foot)
        split.addWidget(self.inspector)
        split.setSizes([620, 420])

        bar = QtWidgets.QFrame()
        bar.setProperty("surface", "band")
        runbar = QtWidgets.QHBoxLayout(bar)
        runbar.setContentsMargins(SPACE["m"], SPACE["s"], SPACE["m"], SPACE["s"])
        runbar.setSpacing(SPACE["s"])
        self.run_state = label("READY", "title")
        self.run_state.setMinimumWidth(90)
        self.run_btn = button("▶ RUN", "primary")
        self.run_btn.setMinimumWidth(110)
        self.run_btn.clicked.connect(lambda: self.run_program(step_mode=False))
        self.step_btn = button("Step", "secondary", "Run one step, then pause")
        self.step_btn.clicked.connect(lambda: self.run_program(step_mode=True))
        self.pause_btn = button("Pause", "secondary")
        self.pause_btn.clicked.connect(win.runner.pause)
        self.stop_btn = button("■ STOP", "danger", "Stop the GUI program (not PolyScope)")
        self.stop_btn.clicked.connect(win.runner.stop)
        self.from_sel = QtWidgets.QCheckBox("From selected")
        self.loop = QtWidgets.QCheckBox("Loop")
        for wdg in (self.run_state, self.run_btn, self.step_btn, self.pause_btn, self.stop_btn):
            runbar.addWidget(wdg)
        runbar.addStretch(1)
        runbar.addWidget(self.from_sel)
        runbar.addWidget(self.loop)
        lay.addWidget(bar)

        for seq, fn in (("Ctrl+D", self.duplicate_step), ("Alt+Up", lambda: self.move_step(-1)),
                        ("Alt+Down", lambda: self.move_step(1)), ("Del", self.delete_step),
                        ("Ctrl+Return", self.run_selected_step), ("Ctrl+S", self.save_program)):
            sc = QtWidgets.QShortcut(QtGui.QKeySequence(seq), self.table)
            sc.setContext(QtCore.Qt.WidgetWithChildrenShortcut)
            sc.activated.connect(fn)
        self.runner_state_changed(RunState.IDLE)
        self.refresh_table()

    # ================================================================ table
    def refresh_table(self, select: Optional[int] = None):
        name = self.path.name if self.path else "unsaved"
        self._mark_saved_state()
        self.title.setText(f"{self.program.name}  <span style='color:{T['muted']};font-size:12px;font-weight:400'>{name} · "
                           f"{len(self.program.steps)} steps · robot {self.program.robot or '—'}</span>")
        if self.program.notes:
            self.notes.setText("Interpreted with defaults (saved as v2 on next Save):\n  " +
                               "\n  ".join(self.program.notes))
        self.notes.setVisible(bool(self.program.notes))
        self.table.setRowCount(len(self.program.steps))
        for i, s in enumerate(self.program.steps):
            for c, text in enumerate((str(i + 1), BADGE.get(s.type, s.type.upper()), self._details(s))):
                self.table.setItem(i, c, QtWidgets.QTableWidgetItem(text))
        self.table.resizeColumnsToContents()
        self.table.horizontalHeader().setSectionResizeMode(2, QtWidgets.QHeaderView.Stretch)
        if select is not None and 0 <= select < len(self.program.steps):
            before = self.selected()
            self.table.selectRow(select)
            if self.selected() == before:        # same row, new program or step: rebuild the inspector anyway
                self._selection_changed()
        else:
            self._selection_changed()

    def _details(self, s: Step) -> str:
        if s.type == "move_pose":
            try:
                pose = self.program.pose_target(s)
                lin = "  linear" if s.params.get("cartesian") else ""
                tgt = f"{s.params['target']}: " if "target" in s.params else ""
                return f"{tgt}{pose.frame} / {pose.tcp}{lin}"
            except Exception:
                return s.summary()
        if s.type == "controller":
            m = self.win.profile.modes.get(s.params.get("mode"))
            return f"{m.label} · {m.command_interface.upper()}" if m else s.summary()
        return s.summary()

    def selected(self) -> int:
        rows = self.table.selectionModel().selectedRows()
        return rows[0].row() if rows else -1

    def highlight_step(self, i: int):
        for r in range(self.table.rowCount()):
            for c in range(3):
                it = self.table.item(r, c)
                if it:
                    it.setBackground(QtGui.QBrush(QtGui.QColor(T["warn_bg"])) if r == i else QtGui.QBrush())
        if i >= 0 and self.table.item(i, 0):
            self.table.scrollToItem(self.table.item(i, 0))

    def _context_menu(self, pos):
        if self.selected() < 0:
            return
        m = QtWidgets.QMenu(self)
        m.addAction("Run this step", self.run_selected_step)
        m.addAction("Duplicate", self.duplicate_step)
        m.addAction("Move up", lambda: self.move_step(-1))
        m.addAction("Move down", lambda: self.move_step(1))
        m.addAction("Edit YAML…", self.edit_yaml)
        m.addSeparator()
        m.addAction("Delete", self.delete_step)
        m.exec_(self.table.viewport().mapToGlobal(pos))

    # ============================================================ inspector
    def _clear_form(self):
        while self.form.rowCount():
            self.form.removeRow(0)

    def _selection_changed(self):
        self._clear_form()
        i = self.selected()
        has = i >= 0
        self.run_one.setEnabled(has and self.win.runner.state == RunState.IDLE)
        if not has:
            self.insp_title.setText("No step selected")
            return
        s = self.program.steps[i]
        self.insp_title.setText(f"{i + 1} · {TYPE_LABEL.get(s.type, s.type)}")
        getattr(self, f"_form_{s.type}", self._form_generic)(i, s)

    def _set(self, i: int, key: str, value, refresh=True):
        if self._loading:
            return
        self.program.steps[i].params[key] = value
        if refresh:
            self._refresh_row(i)

    def _refresh_row(self, i):
        s = self.program.steps[i]
        self.table.item(i, 2).setText(self._details(s))
        self._mark_saved_state()

    def _dump(self) -> str:
        return yaml.safe_dump(self.program.to_dict(), sort_keys=True)

    def _mark_saved_state(self):
        """Save is only emphasised when there is something to save. Run stays the screen's main action."""
        if self._saved is None:
            self._saved = self._dump()
        role = "primary" if self._dump() != self._saved else "secondary"
        if self.save_btn.property("role") != role:
            self.save_btn.setProperty("role", role)
            repolish(self.save_btn)

    def _spin(self, value, lo, hi, dec=2, suffix="", step=None):
        sp = QtWidgets.QDoubleSpinBox()
        sp.setRange(lo, hi)
        sp.setDecimals(dec)
        sp.setValue(float(value))
        if suffix:
            sp.setSuffix(suffix)
        if step:
            sp.setSingleStep(step)
        return sp

    def _speed_rows(self, i, s):
        v = QtWidgets.QSpinBox()
        v.setRange(1, 100)
        v.setSuffix(" %")
        v.setValue(int(round(float(s.params.get("velocity", 0.1)) * 100)))
        v.setFixedWidth(64)
        v.valueChanged.connect(lambda x: self._set(i, "velocity", round(x / 100, 3)))
        self.form.addRow("Velocity", v)

    def _form_movej(self, i, s):
        cb = QtWidgets.QComboBox()
        names = [n for n, w in self.program.waypoints.items() if w.joints is not None] + list(self.win.profile.waypoints)
        cb.addItems(names)
        if "target" in s.params:
            cb.setCurrentText(s.params["target"])
        cb.currentTextChanged.connect(lambda t: (s.params.pop("joints", None), self._set(i, "target", t)))
        self.form.addRow("Waypoint", cb)
        try:
            q = self.program.joint_target(s, self.win.profile)
            self.form.addRow("Joints", muted("  ".join(f"{math.degrees(v):.1f}°" for v in q)))
        except Exception as e:
            self.form.addRow("Joints", muted(f"✗ {e}"))
        rec = button("Record current joints", "secondary")
        rec.clicked.connect(lambda: self._rerecord_joints(i))
        self.form.addRow("", rec)
        self._speed_rows(i, s)

    def _form_move_pose(self, i, s):
        w = self.win
        try:
            pose = self.program.pose_target(s)
        except Exception as e:
            self.form.addRow(muted(f"✗ {e}"))
            return
        if "target" in s.params:
            users = sum(1 for st in self.program.steps if st.params.get("target") == s.params["target"])
            self.form.addRow("Waypoint", muted(f"{s.params['target']}" + (f"  (used by {users} steps)" if users > 1
                                                                          else "")))
        frame = QtWidgets.QComboBox()
        frame.addItems(w.store.frame_names())
        frame.setCurrentText(pose.frame)
        tcp = QtWidgets.QComboBox()
        tcp.addItems(list(w.store.tcps))
        tcp.setCurrentText(pose.tcp)
        self.form.addRow(section_label("Target"))
        self.form.addRow("Frame", frame)
        self.form.addRow("TCP", tcp)
        xyz, rpy = tf.T_to_mm_deg(pose.T()) if self._quat_ok(pose) else ([0, 0, 0], [0, 0, 0])
        spins = []

        def triple(title, names, values, rng, suffix):
            self.form.addRow(section_label(title))
            row = QtWidgets.QHBoxLayout()
            row.setSpacing(SPACE["s"])
            for name, v in zip(names, values):
                row.addWidget(label(name, "field"))
                sp = self._spin(v, -rng, rng, 1, suffix)
                sp.setFont(mono_font())
                sp.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
                spins.append(sp)
                row.addWidget(sp, 1)
            self.form.addRow(row)
        triple("Position", ("X", "Y", "Z"), xyz, 2000, " mm")
        triple("Orientation", ("Roll", "Pitch", "Yaw"), rpy, 180, " °")

        def apply(*_):
            if self._loading:
                return
            vals = [sp.value() for sp in spins]
            new = PoseSpec.from_T(frame.currentText(), tcp.currentText(), tf.mm_deg_to_T(vals[:3], vals[3:]))
            self._store_pose(i, new)
        for wdg in spins:
            wdg.valueChanged.connect(apply)
        frame.currentTextChanged.connect(apply)
        tcp.currentTextChanged.connect(apply)
        self.form.addRow(section_label("Motion"))
        planned = QtWidgets.QRadioButton("Planned")
        lin = QtWidgets.QRadioButton("Linear")
        (lin if s.params.get("cartesian") else planned).setChecked(True)
        lin.toggled.connect(lambda on: self._set(i, "cartesian", on))
        row = QtWidgets.QHBoxLayout()
        row.addWidget(planned)
        row.addWidget(lin)
        row.addStretch(1)
        self.form.addRow(row)
        self._speed_rows(i, s)
        rec = button("Record current pose", "secondary", "Pose of this step's TCP in this step's frame, now")
        rec.clicked.connect(lambda: self._rerecord_pose(i, frame.currentText(), tcp.currentText()))
        self.form.addRow(rec)

    @staticmethod
    def _quat_ok(pose):
        try:
            pose.T()
            return True
        except Exception:
            return False

    def _store_pose(self, i, pose: PoseSpec):
        s = self.program.steps[i]
        if "target" in s.params and s.params["target"] in self.program.waypoints:
            self.program.waypoints[s.params["target"]].pose = pose
        else:
            s.params["pose"] = pose.to_dict()
        self._refresh_row(i)

    def _form_gripper(self, i, s):
        cmd = QtWidgets.QComboBox()
        cmd.addItems(["open", "close", "position"])
        cmd.setCurrentText(s.params.get("command", "position") if "position" not in s.params else "position")
        pos = self._spin(s.params.get("position", 0.0), 0, 1, 3)
        pos.setEnabled(cmd.currentText() == "position")

        def cmd_changed(t):
            pos.setEnabled(t == "position")
            if t == "position":
                s.params.pop("command", None)
                self._set(i, "position", pos.value())
            else:
                s.params.pop("position", None)
                self._set(i, "command", t)
        cmd.currentTextChanged.connect(cmd_changed)
        pos.valueChanged.connect(lambda v: self._set(i, "position", v) if cmd.currentText() == "position" else None)
        eff = self._spin(s.params.get("effort", self.win.profile.gripper.get("max_effort", 40)), 0, 1000, 1, " N")
        eff.valueChanged.connect(lambda v: self._set(i, "effort", v))
        self.form.addRow("Command", cmd)
        self.form.addRow("Position", pos)
        self.form.addRow("Effort", eff)

    def _form_wait(self, i, s):
        sp = self._spin(s.params.get("seconds", 1.0), 0, 3600, 2, " s")
        sp.valueChanged.connect(lambda v: self._set(i, "seconds", v))
        self.form.addRow("Duration", sp)

    def _form_controller(self, i, s):
        cb = QtWidgets.QComboBox()
        for m in self.win.profile.visible_modes():
            cb.addItem(f"{m.label} · {m.command_interface.upper()}", m.name)
        cb.setCurrentIndex(max(0, cb.findData(s.params.get("mode"))))
        info = muted("")

        def changed(_):
            name = cb.currentData()
            m = self.win.profile.modes[name]
            info.setText(("Crosses into DIRECT TORQUE: no MoveIt steps until the program returns to a "
                          "position-domain mode." if m.direct_torque else
                          f"MoveIt steps go to {m.moveit_controller}.") +
                         ("" if m.allow_activation else f"  Disabled: {m.disabled_reason}"))
            self._set(i, "mode", name)
        cb.currentIndexChanged.connect(changed)
        self.form.addRow("Mode", cb)
        self.form.addRow(info)
        self._loading = True
        changed(0)
        self._loading = False

    def _form_policy(self, i, s):
        cb = QtWidgets.QComboBox()
        cb.setEditable(True)
        cb.addItems(list(self.win.policies.manifests))
        cb.setCurrentText(s.params.get("policy", ""))
        cb.currentTextChanged.connect(lambda t: self._set(i, "policy", t))
        sp = self._spin(s.params.get("duration", 8.0), 0.1, 3600, 1, " s")
        sp.valueChanged.connect(lambda v: self._set(i, "duration", v))
        self.form.addRow("Policy", cb)
        self.form.addRow("Duration", sp)
        m = self.win.policies.manifests.get(s.params.get("policy"))
        if m:
            self.form.addRow(muted(f"Requires mode {m.required_mode} (a controller step must come first)."))

    def _form_generic(self, i, s):
        self.form.addRow(muted(yaml.safe_dump(s.params, sort_keys=False).strip()))
        self.form.addRow(muted("Edit with YAML…"))

    # ============================================================== editing
    def _editable(self) -> bool:
        if self.win.runner.state != RunState.IDLE:
            self.win.report((False, "stop the program before editing"))
            return False
        return True

    def _template(self, t: str) -> Step:
        params = copy.deepcopy(STEP_TEMPLATES[t])
        if t == "move_pose":
            params["pose"]["frame"] = self.win.profile.base_link
            params["pose"]["tcp"] = self.win.store.active_tcp
        return Step(t, params)

    def add_step(self, step: Step):
        if not self._editable():
            return
        i = self.selected()
        at = i + 1 if i >= 0 else len(self.program.steps)
        self.program.steps.insert(at, step)
        self.refresh_table(at)

    def _new_wp_name(self, prefix: str) -> str:
        k = 1
        while f"{prefix}{k}" in self.program.waypoints or f"{prefix}{k}" in self.win.profile.waypoints:
            k += 1
        return f"{prefix}{k}"

    def _fresh_joints(self):
        s = self.win.last_snap
        if not s.joints or s.joint_state_age is None or s.joint_state_age > self.win.profile.joint_state_timeout_s:
            self.win.report((False, "no fresh joint state to record"))
            return None
        return list(s.joints)

    def record_joints(self):
        q = self._fresh_joints()
        if q is None:
            return
        name = self._new_wp_name("wp")
        self.program.waypoints[name] = ProgramWaypoint(name=name, joints=q)
        self.add_step(Step("movej", {"target": name, "velocity": 0.1}))
        self.win.bridge.log(f"recorded {name}: {[round(math.degrees(v), 2) for v in q]} °")

    def record_pose(self):
        frame, tcp = self.record_frame.currentText(), self.win.store.active_tcp
        try:
            pose = self.win.geo.current_pose(frame, tcp)
        except GeometryError as e:
            self.win.report((False, f"cannot record pose: {e}"))
            return
        name = self._new_wp_name("pose")
        self.program.waypoints[name] = ProgramWaypoint(name=name, pose=pose)
        self.add_step(Step("move_pose", {"target": name, "velocity": 0.1}))
        xyz, _ = tf.T_to_mm_deg(pose.T())
        self.win.bridge.log(f"recorded {name}: TCP {tcp} in {frame} {[round(v, 2) for v in xyz]} mm")

    def _rerecord_joints(self, i):
        q = self._fresh_joints()
        if q is None:
            return
        s = self.program.steps[i]
        name = s.params.get("target")
        if name in self.program.waypoints:
            self.program.waypoints[name].joints = q
        else:
            name = self._new_wp_name("wp")
            self.program.waypoints[name] = ProgramWaypoint(name=name, joints=q)
            s.params.pop("joints", None)
            s.params["target"] = name
        self.refresh_table(i)

    def _rerecord_pose(self, i, frame, tcp):
        try:
            pose = self.win.geo.current_pose(frame, tcp)
        except GeometryError as e:
            self.win.report((False, f"cannot record pose: {e}"))
            return
        self._store_pose(i, pose)
        self.refresh_table(i)

    def duplicate_step(self):
        i = self.selected()
        if i >= 0 and self._editable():
            s = self.program.steps[i]
            self.program.steps.insert(i + 1, Step(s.type, copy.deepcopy(s.params)))
            self.refresh_table(i + 1)

    def move_step(self, d: int):
        i = self.selected()
        j = i + d
        if i < 0 or not (0 <= j < len(self.program.steps)) or not self._editable():
            return
        st = self.program.steps
        st[i], st[j] = st[j], st[i]
        self.refresh_table(j)

    def delete_step(self):
        i = self.selected()
        if i >= 0 and self._editable():
            del self.program.steps[i]
            self.refresh_table(min(i, len(self.program.steps) - 1))

    def edit_yaml(self):
        i = self.selected()
        if i < 0 or not self._editable():
            return
        step = self.program.steps[i]
        dlg = QtWidgets.QDialog(self)
        dlg.setWindowTitle(f"Edit step {i + 1} (YAML)")
        lay = QtWidgets.QVBoxLayout(dlg)
        lay.addWidget(muted("Poses need frame, tcp, position [m], quat_xyzw. TCPs: " + ", ".join(self.win.store.tcps)
                            + ". Frames: " + ", ".join(self.win.store.frame_names())))
        ed = QtWidgets.QPlainTextEdit(yaml.safe_dump(step.to_dict(), sort_keys=False))
        ed.setFont(QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont))
        lay.addWidget(ed)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        dlg.resize(520, 340)
        while dlg.exec_() == QtWidgets.QDialog.Accepted:
            try:
                self.program.steps[i] = Step.from_dict(yaml.safe_load(ed.toPlainText()) or {})
                break
            except Exception as e:
                QtWidgets.QMessageBox.warning(self, "Invalid step", str(e))
        self.refresh_table(i)

    # ================================================================ files
    def new_program(self):
        if self._editable():
            self.program = Program(name="untitled", robot=self.win.profile.name)
            self.path = None
            self._saved = None
            self.refresh_table()

    def new_scrape_program(self):
        if not self._editable():
            return
        if self._saved is not None and self._dump() != self._saved and not self.win.confirm(
                "Unsaved changes", "Replace the open program? Its changes are not saved."):
            return
        from .scrape_dialog import ScrapeDialog
        d = ScrapeDialog(self.win, self)
        if d.exec_() and d.program is not None:
            self.program = d.program
            self.path = None
            self._saved = ""                     # never saved: Save is the next thing to do
            self.refresh_table(0)
            self.win.bridge.log(f"generated {self.program.name}: {len(self.program.steps)} steps. Check it, then save.")

    def open_program(self):
        if not self._editable():
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Open program", str(self.win.programs_dir),
                                                        "Programs (*.yaml *.yml)")
        if path:
            self.load_file(Path(path))

    def load_file(self, path: Path):
        try:
            self.program = Program.load(path, self.win.profile)
        except Exception as e:
            QtWidgets.QMessageBox.warning(self, "Cannot open program", str(e))
            return
        self.path = path
        self._saved = None
        for note in self.program.notes:
            self.win.bridge.log(f"program: {note}")
        self.refresh_table(0)

    def save_program(self, ask: bool = False):
        if ask or self.path is None:
            path, _ = QtWidgets.QFileDialog.getSaveFileName(
                self, "Save program", str(self.win.programs_dir / f"{self.program.name}.yaml"), "Programs (*.yaml)")
            if not path:
                return
            self.path = Path(path)
            self.program.name = self.path.stem
        prof = self.win.profile
        if self.program.robot and self.program.robot != prof.name and not self.win.confirm(
                "Different robot", f"This program was written for '{self.program.robot}'. Save it for '{prof.name}'?"):
            return
        self.program.robot = prof.name
        self.program.save(self.path)
        self._saved = self._dump()
        self.win.bridge.log(f"saved {self.path}")
        self.refresh_table(self.selected())

    # ================================================================== run
    def _show_preflight(self, problems):
        self.win.bridge.log("preflight failed:\n  " + "\n  ".join(problems))
        QtWidgets.QMessageBox.warning(self, "Program preflight failed",
                                      "Nothing was executed.\n\n" + "\n".join(f"• {p}" for p in problems))

    def check_program(self):
        problems = self.win.runner.preflight(self.program)
        if problems:
            self._show_preflight(problems)
        else:
            self.win.report((True, "program preflight OK"))

    def run_program(self, step_mode: bool):
        r = self.win.runner
        if r.state == RunState.PAUSED:
            r.resume(step_mode=step_mode)
            return
        if r.state != RunState.IDLE or self.win.busy or not self.program.steps:
            return
        start = max(0, self.selected()) if self.from_sel.isChecked() else 0
        problems = r.preflight(self.program, start)
        if problems:
            self._show_preflight(problems)
            return
        if not self.win.fake and not self.win.confirm(
                "Run program", f"Run '{self.program.name}' from step {start + 1} on the robot?"):
            return
        r.start(self.program, start_index=start, loop=self.loop.isChecked(), step_mode=step_mode)

    def run_selected_step(self):
        i = self.selected()
        r = self.win.runner
        if i < 0 or r.state != RunState.IDLE:
            return
        one = Program(self.program.name, self.program.robot, [self.program.steps[i]], self.program.waypoints)
        problems = r.preflight(one)
        if problems:
            self._show_preflight(problems)
            return
        if not self.win.fake and not self.win.confirm("Run step", f"Run step {i + 1}?"):
            return
        r.start(one)

    def runner_state_changed(self, s: RunState):
        self.run_state.setText("READY" if s == RunState.IDLE else s.value.upper())
        idle, running, paused = s == RunState.IDLE, s == RunState.RUNNING, s == RunState.PAUSED
        self.run_btn.setEnabled(idle or paused)
        self.run_btn.setText("▶ RESUME" if paused else "▶ RUN")
        self.step_btn.setEnabled(idle or paused)
        self.pause_btn.setEnabled(running)
        self.stop_btn.setEnabled(not idle)
        self.run_one.setEnabled(idle and self.selected() >= 0)
        if idle:
            self.highlight_step(-1)

    # ========================================================= calibration
    def calibration_changed(self):
        cur = self.record_frame.currentText()
        self.record_frame.blockSignals(True)
        self.record_frame.clear()
        self.record_frame.addItems(self.win.store.frame_names())
        i = self.record_frame.findText(cur)
        self.record_frame.setCurrentIndex(i if i >= 0 else 0)
        self.record_frame.blockSignals(False)
        self._update_record_label()

    def _update_record_label(self):
        self.record_lbl.setText(f"with TCP {self.win.store.active_tcp}")

    def refresh(self, s):
        pass
