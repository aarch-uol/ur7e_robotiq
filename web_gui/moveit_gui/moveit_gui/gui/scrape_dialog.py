"""PROGRAMS > File > New scraping program: teach one contact pose, get the whole sweep as a program."""
from __future__ import annotations

from typing import Optional

from PyQt5 import QtWidgets

from .. import transforms as tf
from ..geometry import GeometryError, PoseSpec
from ..scrape_pattern import ScrapePattern, ScrapePatternError, build
from .theme import SPACE
from .widgets import button, label, muted


class ScrapeDialog(QtWidgets.QDialog):
    def __init__(self, win, parent=None):
        super().__init__(parent)
        self.win = win
        self.start: Optional[PoseSpec] = None
        self.program = None
        self.setWindowTitle("New scraping program")
        self.setMinimumWidth(460)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setSpacing(SPACE["m"])
        lay.addWidget(muted("Drag the tool onto the vial rim where scraping should start, in freedrive, and "
                            "record it. Every other angle is that pose turned about the vial axis."))
        form = QtWidgets.QFormLayout()
        form.setVerticalSpacing(SPACE["s"])
        self.frame = QtWidgets.QComboBox()
        frames = win.store.frame_names()
        self.frame.addItems(frames)
        vials = [n for n, f in win.store.fixtures.items() if f.meta.get("method") == "vial_xy_circle"] or \
                [n for n in frames if "vial" in n]
        if vials:
            self.frame.setCurrentText(vials[0])
        self.frame.currentTextChanged.connect(lambda _: self._clear_start())
        form.addRow("Vial frame", self.frame)

        row = QtWidgets.QHBoxLayout()
        row.addWidget(win.freedrive_button())
        rec = button("Record contact pose", "secondary")
        rec.clicked.connect(self.record)
        row.addWidget(rec)
        row.addStretch(1)
        form.addRow("Start", row)
        self.start_lbl = label("not recorded", "muted")
        form.addRow("", self.start_lbl)

        self.policy = QtWidgets.QComboBox()
        self.policy.addItems(list(win.policies.manifests))
        self.policy.currentTextChanged.connect(self._policy_changed)
        form.addRow("Policy", self.policy)
        self.mode_lbl = label("", "secondary")
        form.addRow("", self.mode_lbl)

        def spin(v, lo, hi, dec, suffix):
            s = QtWidgets.QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(dec)
            s.setValue(v)
            s.setSuffix(suffix)
            s.valueChanged.connect(self._summary)
            return s
        self.duration = spin(8.0, 0.5, 600, 1, " s")
        form.addRow("Per angle", self.duration)
        self.step = spin(10.0, -180, 180, 1, " °")
        form.addRow("Angle step", self.step)
        self.count = QtWidgets.QSpinBox()
        self.count.setRange(1, 361)
        self.count.setValue(37)
        self.count.valueChanged.connect(self._summary)
        form.addRow("Angles", self.count)
        self.back = QtWidgets.QCheckBox("Then sweep back")
        self.back.toggled.connect(self._summary)
        form.addRow("", self.back)
        self.approach = spin(50.0, 5, 300, 0, " mm")
        form.addRow("Approach height", self.approach)
        lay.addLayout(form)
        self.summary = label("", "secondary")
        lay.addWidget(self.summary)
        self.err = label("", "error", wrap=True)
        lay.addWidget(self.err)

        btns = QtWidgets.QHBoxLayout()
        btns.addStretch(1)
        cancel = button("Cancel", "quiet")
        cancel.clicked.connect(self.reject)
        self.ok = button("Create program", "primary")
        self.ok.clicked.connect(self.create)
        btns.addWidget(cancel)
        btns.addWidget(self.ok)
        lay.addLayout(btns)
        self._policy_changed(self.policy.currentText())

    # ------------------------------------------------------------------
    def _modes(self):
        """(policy mode, motion mode). Moves run in the policy's mode when MoveIt works there (admittance)."""
        m = self.win.policies.manifests.get(self.policy.currentText())
        if m is None:
            return None, None
        modes = self.win.profile.modes
        pm = m.required_mode
        if pm in modes and modes[pm].supports_moveit:
            return pm, pm
        motion = "position" if "position" in modes else self.win.profile.default_mode
        return pm, motion

    def _policy_changed(self, name):
        m = self.win.policies.manifests.get(name)
        if m is not None and m.duration:
            self.duration.setValue(float(m.duration))
        pm, motion = self._modes()
        if pm is None:
            self.mode_lbl.setText("No policies found in the policy folder.")
        elif pm == motion:
            self.mode_lbl.setText(f"Moves and policy both run in {pm}.")
        else:
            self.mode_lbl.setText(f"Moves in {motion}, switches to {pm} for each policy run.")
        self._summary()

    def _clear_start(self):
        self.start = None
        self.start_lbl.setText("not recorded")
        self._summary()

    def record(self):
        try:
            self.start = self.win.geo.current_pose(self.frame.currentText(), self.win.store.active_tcp)
        except GeometryError as e:
            self.err.setText(f"Cannot record: {e}")
            return
        xyz, rpy = tf.T_to_mm_deg(self.start.T())
        self.start_lbl.setText(f"{self.start.tcp} at " + "  ".join(f"{v:.1f}" for v in xyz) + " mm")
        self.err.setText("")
        self._summary()

    def pattern(self) -> Optional[ScrapePattern]:
        pm, motion = self._modes()
        if self.start is None or pm is None:
            return None
        return ScrapePattern(start=self.start, policy=self.policy.currentText(), policy_mode=pm, motion_mode=motion,
                             duration=self.duration.value(), step_deg=self.step.value(), count=self.count.value(),
                             back_and_forth=self.back.isChecked(), approach_m=self.approach.value() / 1000.0)

    def _summary(self, *_):
        p = self.pattern()
        self.ok.setEnabled(p is not None)
        if p is None:
            self.summary.setText("Record the contact pose to continue." if self._modes()[0] else "")
            return
        n = len(p.angles())
        span = p.step_deg * (p.count - 1)
        self.summary.setText(f"{n} policy runs, {span:.0f}° sweep{' and back' if p.back_and_forth else ''}, "
                             f"about {n * (p.duration + 6) / 60:.0f} min")

    def create(self):
        try:
            self.program = build(self.pattern(), f"scrape_{self.frame.currentText()}", self.win.profile.name)
        except ScrapePatternError as e:
            self.err.setText(str(e))
            return
        self.accept()
