"""SETUP pages with their own workflow: vial geometry and F/T + payload. The maths lives outside the GUI."""
from __future__ import annotations

import numpy as np
from PyQt5 import QtCore, QtWidgets

from ..calibration import CalibrationError, FixtureDef
from ..ft_payload import PayloadCalibration, PayloadCalibrationError
from ..geometry import GeometryError
from ..geometry_calibration import VialGeometryError, estimate_vial
from .theme import SPACE
from .widgets import (Card, Dot, ReadoutGrid, button, label, mono_label, muted, page_widget, section_label,
                      set_kind)

POINTS = ("Rim 1", "Rim 2", "Rim 3", "Bottom")


class VialPage(QtWidgets.QWidget):
    def __init__(self, win, emit_changed):
        super().__init__()
        self.win, self.emit_changed = win, emit_changed
        self.points = {}
        self.result = None
        page = page_widget()
        QtWidgets.QVBoxLayout(self).addWidget(page)
        self.layout().setContentsMargins(0, 0, 0, 0)
        lay = QtWidgets.QHBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(SPACE["s"])

        cap = Card("Touch points")
        top = QtWidgets.QHBoxLayout()
        self.tcp_lbl = label("", "secondary")
        top.addWidget(self.tcp_lbl, 1)
        top.addWidget(win.freedrive_button())
        cap.body.addLayout(top)
        cap.body.addWidget(muted("In freedrive, touch the TCP to the rim at three points spread around it and "
                                 "capture each. Optionally touch the bottom."))
        grid = QtWidgets.QGridLayout()
        grid.setVerticalSpacing(SPACE["xs"])
        grid.setColumnStretch(2, 1)
        self.marks, self.vals = {}, {}
        for r, name in enumerate(POINTS):
            d = Dot(8)
            self.marks[name] = d
            grid.addWidget(d, r, 0)
            grid.addWidget(label(name + (" (optional)" if name == "Bottom" else "")), r, 1)
            v = label("—", "muted")
            self.vals[name] = v
            grid.addWidget(v, r, 2)
            b = button("Capture", "secondary")
            b.clicked.connect(lambda _=False, n=name: self.capture(n))
            grid.addWidget(b, r, 3)
        cap.body.addLayout(grid)
        cap.body.addStretch(1)
        reset = button("Reset", "quiet")
        reset.clicked.connect(self.reset)
        cap.body.addWidget(reset, 0, QtCore.Qt.AlignLeft)
        lay.addWidget(cap, 3)

        res = Card("Vial")
        self.grid = ReadoutGrid([("cx", "Centre X"), ("cy", "Centre Y"), ("r", "Radius"), ("z", "Bottom Z"),
                                 ("spread", "Rim height spread")])
        res.body.addWidget(self.grid)
        self.err = label("", "error", wrap=True)
        res.body.addWidget(self.err)
        res.body.addWidget(muted("XY circle in the base frame, assuming a level rim and a vertical vial. This is not "
                                 "a full 3D fit. Without a bottom touch, Z is the lowest rim point."))
        res.body.addStretch(1)
        row = QtWidgets.QHBoxLayout()
        self.name = QtWidgets.QLineEdit("vial")
        row.addWidget(self.name, 1)
        self.save_btn = button("Save as fixture", "primary")
        self.save_btn.setEnabled(False)
        self.save_btn.clicked.connect(self.save)
        row.addWidget(self.save_btn)
        res.body.addLayout(row)
        lay.addWidget(res, 2)

    def refresh_labels(self):
        self.tcp_lbl.setText(f"With TCP {self.win.store.active_tcp}, in {self.win.profile.base_link}")

    def capture(self, name):
        w = self.win
        try:
            T = w.geo.current_pose(w.profile.base_link, w.store.active_tcp).T()
        except GeometryError as e:
            w.report((False, f"cannot capture: {e}"))
            return
        self.points[name] = T[:3, 3].copy()
        self.marks[name].set_state(True)
        self.vals[name].setText("  ".join(f"{v * 1000:.1f}" for v in self.points[name]) + " mm")
        self._update()

    def reset(self):
        self.points.clear()
        for n in POINTS:
            self.marks[n].set_state(None)
            self.vals[n].setText("—")
        self._update()

    def _update(self):
        self.result = None
        self.err.setText("")
        rim = [self.points[n] for n in POINTS[:3] if n in self.points]
        if len(rim) == 3:
            try:
                self.result = estimate_vial(rim, self.points.get("Bottom"))
            except VialGeometryError as e:
                self.err.setText(str(e))
        g = self.result
        vals = {"cx": g.center_xy[0], "cy": g.center_xy[1], "r": g.radius, "z": g.bottom_z,
                "spread": g.rim_z_spread} if g else {}
        for k in ("cx", "cy", "r", "z", "spread"):
            self.grid.set(k, f"{vals[k] * 1000:9.2f} mm" if g else "—")
        self.save_btn.setEnabled(g is not None)

    def save(self):
        g, name = self.result, self.name.text().strip()
        T = np.eye(4)
        T[:3, 3] = [g.center_xy[0], g.center_xy[1], g.bottom_z]
        store = self.win.store
        try:
            store.put_fixture(FixtureDef(name, self.win.profile.base_link, T, g.to_meta()),
                              replace=name in store.fixtures)
        except CalibrationError as e:
            self.win.report((False, str(e)))
            return
        self.win.report((True, f"saved fixture {name}"))
        self.emit_changed()


class PayloadPage(QtWidgets.QWidget):
    """The GUI owns moves, ATI data and progress; calibration_math owns the fit. Nothing touches controller YAML."""
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.cal = None
        self._running = False
        page = page_widget()
        QtWidgets.QVBoxLayout(self).addWidget(page)
        self.layout().setContentsMargins(0, 0, 0, 0)
        lay = QtWidgets.QHBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(SPACE["s"])
        left = Card("Calibration poses")
        try:
            if win.profile.ft_calibration.get("poses"):
                self.cal = PayloadCalibration(win.bridge, win.profile)
        except PayloadCalibrationError as e:
            left.body.addWidget(label(str(e), "error", wrap=True))
        n = len(self.cal.poses) if self.cal else 0
        self.table = QtWidgets.QTableWidget(n, 4)
        self.table.setHorizontalHeaderLabels(["POSE", "FX N", "FY N", "FZ N"])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setShowGrid(False)
        self.table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        for r in range(n):
            self.table.setItem(r, 0, QtWidgets.QTableWidgetItem(self.cal.poses[r]["name"]))
        left.body.addWidget(self.table, 1)
        if not self.cal:
            left.body.addWidget(muted("This robot profile has no ft_calibration poses."))
        self.bar = QtWidgets.QProgressBar()
        self.bar.setTextVisible(False)
        self.bar.setRange(0, max(n, 1))
        left.body.addWidget(self.bar)
        row = QtWidgets.QHBoxLayout()
        self.run_btn = button("▶ Collect all poses", "primary", "Moves the arm through every pose above")
        self.run_btn.clicked.connect(self.collect)
        self.stop_btn = button("■ Stop", "danger")
        self.stop_btn.clicked.connect(lambda: self.cal and self.cal.cancel())
        self.stop_btn.setEnabled(False)
        self.status = label("", "secondary")
        row.addWidget(self.run_btn)
        row.addWidget(self.stop_btn)
        row.addWidget(self.status, 1)
        left.body.addLayout(row)
        self.run_btn.setEnabled(self.cal is not None)
        lay.addWidget(left, 3)

        res = Card("Result")
        self.verdict = label("Not evaluated", "title")
        res.body.addWidget(self.verdict)
        self.grid = ReadoutGrid([("m", "Mass"), ("x", "CoG X"), ("y", "CoG Y"), ("z", "CoG Z")])
        res.body.addWidget(self.grid)
        self.issues = label("", "secondary", wrap=True)
        res.body.addWidget(self.issues)
        res.body.addSpacing(SPACE["s"])
        res.body.addWidget(section_label("Admittance gravity compensation"))
        self.yaml = mono_label()
        self.yaml.setTextFormat(QtCore.Qt.PlainText)          # keep the YAML indentation
        self.yaml.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.yaml.setText("—")
        res.body.addWidget(self.yaml)
        res.body.addWidget(muted("Apply sets the running admittance controller and reads it back; do it outside "
                                 "Admittance. It lasts until the controller restarts: paste the YAML into the "
                                 "controller config to keep it. Controller YAML files are never changed."))
        res.body.addStretch(1)
        row = QtWidgets.QHBoxLayout()
        self.save_btn = button("Save result", "secondary")
        self.save_btn.setEnabled(False)
        self.save_btn.clicked.connect(self.save)
        self.copy_btn = button("Copy YAML", "quiet")
        self.copy_btn.setEnabled(False)
        self.copy_btn.clicked.connect(lambda: QtWidgets.QApplication.clipboard().setText(self.cal.admittance_yaml()))
        self.apply_btn = button("Apply to admittance", "primary")
        self.apply_btn.setEnabled(False)
        self.apply_btn.clicked.connect(self.apply)
        row.addWidget(self.save_btn)
        row.addWidget(self.copy_btn)
        row.addStretch(1)
        row.addWidget(self.apply_btn)
        res.body.addLayout(row)
        lay.addWidget(res, 2)

    def collect(self):
        w = self.win
        if w.busy or self._running:
            w.report((False, "robot is busy"))
            return
        if not w.modes.mode_supports_moveit(w.mode):
            w.report((False, f"MoveIt motion is not available in '{w.mode}'. Change to Position first."))
            return
        if not w.fake and not w.confirm("F/T + payload", f"Move through {len(self.cal.poses)} poses at "
                                        f"{self.cal.velocity * 100:.0f}% speed? Check that the workspace is clear."):
            return
        self._running = True
        w.busy, w.command_source = True, "F/T + payload"
        self.run_btn.setEnabled(False)
        self.stop_btn.setEnabled(True)
        self.save_btn.setEnabled(False)
        self.cal.reset()
        self.bar.setValue(0)

        def work():
            for i in range(len(self.cal.poses)):
                w.dispatch.later(self.status.setText, f"Pose {i + 1} of {len(self.cal.poses)}")
                self.cal.run_pose(i)
                w.dispatch.later(self._pose_done, i)
            return self.cal.evaluate()

        def guarded():
            try:
                return work()
            except Exception as e:           # a refused pose or bad fit must end the run visibly
                return e
        w.bg(guarded, self._finished)

    def _pose_done(self, i):
        s = self.cal.samples[i]
        for c in range(3):
            it = QtWidgets.QTableWidgetItem(f"{s.mean_wrench[c]:+.2f}")
            it.setTextAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
            self.table.setItem(i, c + 1, it)
        self.bar.setValue(i + 1)

    def _finished(self, res):
        w = self.win
        self._running = False
        w.busy, w.command_source = False, "idle"
        self.run_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        if isinstance(res, Exception):
            self.status.setText("")
            self.verdict.setText("Failed")
            set_kind(self.issues, "error")
            self.issues.setText(str(res))
            return
        self.show_result(res)

    def show_result(self, r):
        self.status.setText("Done")
        self.verdict.setText("PASS" if r.passed else "FAIL")
        self.grid.set("m", f"{r.mass:9.3f} kg")
        for k, v in zip("xyz", r.cog):
            self.grid.set(k, f"{v * 1000:9.1f} mm")
        notes = list(r.quality.get("issues", [])) + list(r.quality.get("warnings", []))
        set_kind(self.issues, "secondary" if r.passed else "error")
        self.issues.setText("\n".join(str(n) for n in notes))
        self.save_btn.setEnabled(True)
        try:
            self.yaml.setText(self.cal.admittance_yaml().rstrip())
            self.copy_btn.setEnabled(True)
            self.apply_btn.setEnabled(r.passed and bool(self.cal.apply_node))
        except PayloadCalibrationError as e:
            self.yaml.setText(str(e))

    def _bias_note(self) -> str:
        """The controller has no offset term: the sensor offset the fit found stays in the compensated wrench."""
        b = self.cal.result.info.get("residual_bias")
        if b is None or len(b) < 3:
            return ""
        return (f"\n\nThe fit found a sensor offset of {np.linalg.norm(np.asarray(b[:3], float)):.2f} N, which the "
                "controller cannot remove. Keep ATI hardware biasing off, and keep the F/T software bias as it is now.")

    def apply(self):
        w, cal = self.win, self.cal
        p = cal.admittance_params()
        if not w.confirm("Apply to admittance", f"Set {cal.apply_node} gravity compensation to "
                                                 f"{cal.result.mass:.3f} kg at {[round(v * 1000, 1) for v in p['gravity_compensation.CoG.pos']]} mm "
                                                 f"in {cal.sensor_frame}?" + self._bias_note()):
            return
        if cal.saved_path is None:
            self.save()

        def done(r):
            w.report(r) if r[0] else w.show_refusal(r[1])
        w.bg(lambda: cal.apply_to_admittance(w.mode), done)

    def save(self):
        try:
            self.win.report((True, f"saved {self.cal.save()}"))
        except (OSError, PayloadCalibrationError) as e:
            self.win.report((False, str(e)))
