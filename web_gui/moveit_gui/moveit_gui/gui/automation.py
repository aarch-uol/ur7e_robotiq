"""AUTOMATION: pick a policy, see what it needs, run it. File paths are behind Details.

Everything goes through win.policies (PolicyExecutor), the same path as a PROGRAM policy step. The GUI
never loads models or computes observations. Only feedback the backend really reports is shown.
"""
from __future__ import annotations

import os
import time
from pathlib import Path

from PyQt5 import QtCore, QtWidgets

from ..policies import RecordingOptions
from ..policy_executor import FAIL, OK, RUNNER, WARN  # noqa: F401
from .control_stack import mode_nodes
from .theme import SPACE, mono_font
from .widgets import (Card, Collapsible, Dot, FlowView, ReadoutGrid, button, label, muted, page_widget,
                      scrolled, section_label, set_kind)

BACKENDS = {"impedance": "Impedance backend", "servo_admittance": "Servo + Admittance backend"}
SERVO_FLOW = ["Policy action", "MoveIt Servo", "Admittance", "UR position"]
AXES = ("Fx", "Fy", "Fz", "Tx", "Ty", "Tz")


def _clear(layout):
    while layout.count():
        it = layout.takeAt(0)
        w = it.widget()
        if w:
            w.setParent(None)
            w.deleteLater()


class RequirementRow(QtWidgets.QWidget):
    def __init__(self, r):
        super().__init__()
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 1, 0, 1)
        lay.setSpacing(6)
        dot = Dot(8)
        dot.set_state({OK: True, FAIL: False}.get(r.state))
        lay.addWidget(dot, 0, QtCore.Qt.AlignVCenter)
        lay.addWidget(label(r.label), 0)
        detail = label(r.detail if r.state in (FAIL, WARN) else "", "muted", wrap=True)
        detail.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        lay.addWidget(detail, 1)
        if r.detail:
            self.setToolTip(r.detail)


class AutomationPage(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.prepared = None
        self._reqs = []
        self._checking = False
        self._last_check = 0.0
        self._duration = None
        page = page_widget()
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)
        lay = QtWidgets.QHBoxLayout(page)
        lay.setContentsMargins(SPACE["s"], SPACE["s"], SPACE["s"], SPACE["s"])
        lay.setSpacing(SPACE["s"])

        left = Card("Policies")
        left.setFixedWidth(190)
        self.list = QtWidgets.QListWidget()
        self.list.currentTextChanged.connect(self._show)
        left.body.addWidget(self.list, 1)
        rl = button("Reload", "quiet", f"Manifests in {win.policies.directory}")
        rl.clicked.connect(self.reload)
        left.body.addWidget(rl, 0, QtCore.Qt.AlignLeft)
        lay.addWidget(left)

        lay.addWidget(scrolled(self._summary()), 1)
        lay.addWidget(self._execution())
        win.policies.feedback_listeners.append(lambda fb: win.dispatch.later(self._feedback, fb))
        win.policies.done_listeners.append(lambda r: win.dispatch.later(self._finished, r))
        self.reload()

    # ============================================================= summary
    def _summary(self):
        host = page_widget()
        c = QtWidgets.QVBoxLayout(host)
        c.setContentsMargins(SPACE["l"], SPACE["m"], SPACE["l"], SPACE["m"])
        c.setSpacing(SPACE["xs"])
        self.name_lbl = label("", "title")
        self.name_lbl.setStyleSheet("font-size:20px; font-weight:700;")
        c.addWidget(self.name_lbl)
        self.desc_lbl = label("", "secondary", wrap=True)
        c.addWidget(self.desc_lbl)
        self.facts_lbl = label("", "value")
        c.addWidget(self.facts_lbl)
        c.addSpacing(SPACE["l"])
        row = QtWidgets.QHBoxLayout()
        self.backend_lbl = label("", "value")
        self.backend_lbl.setStyleSheet("font-weight:600;")
        row.addWidget(self.backend_lbl)
        self.torque_badge = label("DIRECT TORQUE", "badge_warn")
        row.addWidget(self.torque_badge)
        row.addStretch(1)
        c.addLayout(row)
        self.flow = FlowView()
        c.addWidget(self.flow)
        c.addSpacing(SPACE["s"])
        self.tcp_lbl = label("", "secondary")
        c.addWidget(self.tcp_lbl)
        self.mode_note = label("", "banner_warn", wrap=True)
        c.addWidget(self.mode_note)
        c.addSpacing(SPACE["l"])
        self.details = Collapsible("Recording and files")
        d = self.details.content_layout
        d.setSpacing(SPACE["s"])
        rec = QtWidgets.QHBoxLayout()
        rec.setSpacing(SPACE["m"])
        self.rec_obs = QtWidgets.QCheckBox("Observations")
        self.rec_obs.setToolTip("Partly checked keeps the robot config's setting. --record-obs / --no-record-obs")
        self.rec_obs.setTristate(True)
        self.rec_obs.setCheckState(QtCore.Qt.PartiallyChecked)
        self.rec_video = QtWidgets.QCheckBox("Video")
        self.rec_video.setToolTip("Partly checked keeps the robot config's setting. --record-video / --no-record-video")
        self.rec_video.setTristate(True)
        self.rec_video.setCheckState(QtCore.Qt.PartiallyChecked)
        self.rec_fps = QtWidgets.QSpinBox()
        self.rec_fps.setRange(1, 120)
        self.rec_fps.setValue(30)
        self.rec_fps.setSuffix(" fps")
        self.rec_fps.setButtonSymbols(QtWidgets.QAbstractSpinBox.NoButtons)
        self.rec_fps.setFixedWidth(60)
        self.rec_fps.setEnabled(False)
        self.rec_video.stateChanged.connect(lambda st: self.rec_fps.setEnabled(st == QtCore.Qt.Checked))
        for b in (self.rec_obs, self.rec_video, self.rec_fps):
            rec.addWidget(b)
        rec.addStretch(1)
        d.addLayout(rec)
        out = QtWidgets.QHBoxLayout()
        out.addWidget(label("Output", "field"))
        self.rec_root = QtWidgets.QLineEdit()
        self.rec_root.setPlaceholderText("runner default")
        self.rec_root.setToolTip("--output-root")
        out.addWidget(self.rec_root, 1)
        d.addLayout(out)
        self.details_lbl = label("", "muted", wrap=True, selectable=True)
        self.details_lbl.setFont(mono_font(11))
        d.addWidget(self.details_lbl)
        c.addWidget(self.details)
        c.addStretch(1)
        return host

    # =========================================================== execution
    def _execution(self):
        c = Card()
        c.setFixedWidth(320)
        head = QtWidgets.QHBoxLayout()
        self.state_dot = Dot(10)
        head.addWidget(self.state_dot)
        self.state_lbl = label("—", "title")
        head.addWidget(self.state_lbl)
        head.addStretch(1)
        c.body.addLayout(head)
        c.body.addSpacing(SPACE["xs"])
        self.stack = QtWidgets.QStackedWidget()
        c.body.addWidget(self.stack, 1)

        ready = QtWidgets.QWidget()
        rl = QtWidgets.QVBoxLayout(ready)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.addWidget(section_label("Readiness"))
        self.req_box = QtWidgets.QVBoxLayout()
        self.req_box.setSpacing(2)
        rl.addLayout(self.req_box)
        rl.addSpacing(SPACE["s"])
        self.result_lbl = label("", "secondary", wrap=True)
        rl.addWidget(self.result_lbl)
        rl.addStretch(1)
        self.stack.addWidget(scrolled(ready))

        prep = QtWidgets.QWidget()
        pl = QtWidgets.QVBoxLayout(prep)
        pl.setContentsMargins(0, 0, 0, 0)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(section_label("Force startup baseline"))
        row.addStretch(1)
        self.base_count = label("", "value")
        self.base_count.setFont(mono_font())
        row.addWidget(self.base_count)
        pl.addLayout(row)
        self.base_bar = QtWidgets.QProgressBar()
        self.base_bar.setTextVisible(False)
        pl.addWidget(self.base_bar)
        pl.addSpacing(SPACE["s"])
        self.base_grid = ReadoutGrid([(k, k) for k in AXES[:3]])
        pl.addWidget(self.base_grid)
        pl.addSpacing(SPACE["s"])
        self.prep_checks = {}
        for key, text in (("pose", "Pose"), ("obs", "Observation"), ("norm", "Normalizer")):
            r = QtWidgets.QHBoxLayout()
            r.setSpacing(6)
            d = Dot(8)
            r.addWidget(d)
            r.addWidget(label(text))
            r.addStretch(1)
            pl.addLayout(r)
            self.prep_checks[key] = d
        pl.addStretch(1)
        self.stack.addWidget(prep)

        run = QtWidgets.QWidget()
        ul = QtWidgets.QVBoxLayout(run)
        ul.setContentsMargins(0, 0, 0, 0)
        self.elapsed_lbl = label("", "value")
        self.elapsed_lbl.setFont(mono_font(18))
        ul.addWidget(self.elapsed_lbl)
        self.elapsed_bar = QtWidgets.QProgressBar()
        self.elapsed_bar.setTextVisible(False)
        self.elapsed_bar.setRange(0, 1000)
        ul.addWidget(self.elapsed_bar)
        ul.addSpacing(SPACE["s"])
        ul.addWidget(section_label("Policy command"))
        self.action_lbl = label("—", "value", wrap=True)
        self.action_lbl.setFont(mono_font())
        self.action_lbl.setToolTip("The runner's action vector, in its order. RunPolicy feedback does not name the "
                                   "components.")
        ul.addWidget(self.action_lbl)
        ul.addSpacing(SPACE["s"])
        ul.addWidget(section_label("Measured F/T"))
        self.run_grid = ReadoutGrid([(k, k) for k in AXES])
        ul.addWidget(self.run_grid)
        self.fb_note = muted("")
        ul.addWidget(self.fb_note)
        ul.addStretch(1)
        self.stack.addWidget(run)

        row = QtWidgets.QHBoxLayout()
        self.prep_btn = button("Prepare", "secondary", "Preflight, then the runner's force startup baseline where the "
                                                       "backend supports it")
        self.prep_btn.clicked.connect(self.prepare)
        self.run_btn = button("▶ Run", "primary")
        self.run_btn.clicked.connect(self.run)
        self.stop_btn = button("■ STOP POLICY", "danger")
        self.stop_btn.clicked.connect(self.stop)
        row.addWidget(self.prep_btn, 1)
        row.addWidget(self.run_btn, 1)
        row.addWidget(self.stop_btn, 1)
        c.body.addLayout(row)
        return c

    # ================================================================ data
    def reload(self):
        pol = self.win.policies
        if pol.busy:
            return
        pol.reload()
        cur = self.list.currentItem().text() if self.list.currentItem() else None
        self.list.clear()
        for n in pol.manifests:
            self.list.addItem(n)
        for f in pol.errors:
            self.list.addItem(f"⚠ {f}")
        items = self.list.findItems(cur or "", QtCore.Qt.MatchExactly) if cur else []
        self.list.setCurrentRow(self.list.row(items[0]) if items else 0)
        if not pol.manifests and not pol.errors:
            self.name_lbl.setText("No policies")
            self.desc_lbl.setText("Copy policies/_template.yaml.example to the manifest folder.")
        self._update_state()

    def _manifest(self):
        it = self.list.currentItem()
        return self.win.policies.manifests.get(it.text()) if it else None

    def _show(self, name: str):
        self.prepared = None
        self.result_lbl.setText("")
        self._set_requirements([])
        m = self.win.policies.manifests.get(name)
        if m is None:
            if name.startswith("⚠ "):
                self.name_lbl.setText(name[2:])
                self.desc_lbl.setText(f"Manifest error: {self.win.policies.errors.get(name[2:], '')}")
            self._update_state()
            return
        self.name_lbl.setText(m.name.replace("_", " ").upper())
        self.desc_lbl.setText(m.description)
        self.desc_lbl.setVisible(bool(m.description))
        parts = [m.algorithm, f"{m.control_hz:g} Hz" if m.control_hz else "",
                 f"{m.duration:g} s" if m.duration else "until stopped"]
        self.facts_lbl.setText(" · ".join(p for p in parts if p))
        mode = self.win.profile.modes.get(m.required_mode)
        self.backend_lbl.setText(BACKENDS.get(m.backend, m.backend or (mode.label if mode else m.required_mode)))
        self.torque_badge.setVisible(bool(mode and mode.direct_torque))
        self.flow.set_nodes(SERVO_FLOW if m.backend == "servo_admittance" else
                            (["Policy action"] + mode_nodes(mode) if mode else []))
        self.tcp_lbl.setText(f"Required TCP: {m.required_tcp}" if m.required_tcp else "Any TCP")
        self._details(m)
        self._last_check = 0.0
        self._check(m)

    def _recording(self) -> RecordingOptions:
        def tri(cb):
            return None if cb.checkState() == QtCore.Qt.PartiallyChecked else cb.isChecked()
        return RecordingOptions(record_obs=tri(self.rec_obs), record_video=tri(self.rec_video),
                                video_fps=float(self.rec_fps.value()) if self.rec_video.checkState() == QtCore.Qt.Checked
                                else None,
                                output_root=self.rec_root.text().strip() or None)

    def _details(self, m):
        names = {"model": "Model file", "normalization": "Normalizer", "training_config": "Training config",
                 "robot_config": "Robot deployment config"}
        lines = [f"{names[k]}\n  {os.path.expanduser(m.files[k])}" for k in names if k in m.files]
        self.details_lbl.setText("\n".join(lines) or "Custom command manifest")

    def _check(self, m):
        if self._checking:
            return
        self._checking = True
        name = m.name

        def done(reqs):
            self._checking = False
            self._last_check = time.monotonic()
            if self._manifest() is m:
                self._set_requirements(reqs)
        self.win.bg(lambda: self.win.policies.requirements(name), done)

    def _set_requirements(self, reqs):
        key = [(r.label, r.state, r.detail) for r in reqs]
        if key != [(r.label, r.state, r.detail) for r in self._reqs]:
            _clear(self.req_box)
            for r in reqs:
                self.req_box.addWidget(RequirementRow(r))
        self._reqs = reqs
        if any(r.state == FAIL for r in reqs):
            self.prepared = None
        m = self._manifest()
        mode = self.win.profile.modes.get(m.required_mode) if m else None
        off = bool(mode and self.win.mode != m.required_mode)
        self.mode_note.setVisible(off)
        if off:
            self.mode_note.setText(f"Switch to {mode.label} first (OPERATE, Arm control). The GUI never changes "
                                   "control mode for a policy on its own.")
        self._update_state()

    def _update_state(self):
        busy = self.win.policies.busy
        ready = bool(self._reqs) and not any(r.state == FAIL for r in self._reqs)
        name = self.list.currentItem().text() if self.list.currentItem() else None
        self.prep_btn.setVisible(not busy)
        self.run_btn.setVisible(not busy)
        self.stop_btn.setVisible(busy)
        self.prep_btn.setEnabled(ready)
        self.run_btn.setEnabled(ready and self.prepared == name and name is not None)
        if busy:
            return
        self.stack.setCurrentIndex(0)
        if self.prepared == name and ready:
            self._state("READY", True)
        elif ready:
            self._state("NOT PREPARED", None)
        elif self._reqs:
            self._state("NOT READY", False)
        else:
            self._state("—", None)

    def _state(self, text, dot):
        self.state_lbl.setText(text)
        self.state_dot.set_state(dot)

    # ============================================================== actions
    def prepare(self):
        m = self._manifest()
        if m is None:
            return
        self._state("PREPARING", "warn")
        self.stack.setCurrentIndex(1)
        self.prep_btn.setEnabled(False)
        self.win.policies.prepare(self.win.policies.goal(m.name, recording=self._recording()),
                                  lambda r, n=m.name: self.win.dispatch.later(self._prepared, (n, r)))

    def _prepared(self, arg):
        name, res = arg
        self.prepared = name if res.success else None
        self.result_lbl.setText(("Force startup baseline locked" if res.stop_reason == "ready" else res.stop_reason)
                                if res.success else f"Failed: {res.stop_reason}")
        set_kind(self.result_lbl, "secondary" if res.success else "error")
        self._update_state()

    def run(self):
        m = self._manifest()
        if m is None:
            return
        mode = self.win.profile.modes.get(m.required_mode)
        if not self.win.fake and not self.win.confirm("Run policy", f"Run {m.name} on the robot?" + (
                "\n\nThis commands DIRECT TORQUE." if mode and mode.direct_torque else "")):
            return
        self._duration = m.duration
        self.win.command_source = f"policy {m.name}"
        self._state("RUNNING", True)
        self.stack.setCurrentIndex(2)
        self.elapsed_lbl.setText("")
        self.fb_note.setText("" if self.win.policies.supports_feedback else
                             "No live feedback from the subprocess runner until RunPolicy exists.")
        self.win.policies.start(self.win.policies.goal(m.name, recording=self._recording()))
        self._update_state()

    def stop(self):
        self.win.policies.cancel()

    def _finished(self, res):
        self.win.command_source = "idle"
        self.prepared = None
        self.result_lbl.setText(("Finished: " if res.success else "Stopped: ") + res.stop_reason +
                                (f"\nLog: {res.log_dir}" if res.log_dir else ""))
        set_kind(self.result_lbl, "secondary" if res.success else "error")
        self._update_state()

    def _feedback(self, fb):
        if fb.state == "baseline":
            self.stack.setCurrentIndex(1)
            self.base_bar.setMaximum(fb.baseline_total or 1)
            self.base_bar.setValue(fb.baseline_count)
            self.base_count.setText(f"{fb.baseline_count} / {fb.baseline_total}")
            for i, k in enumerate(AXES[:3]):
                self.base_grid.set(k, f"{fb.wrench[i]:+7.2f} N" if fb.wrench else "—")
            for key, v in (("pose", fb.pose_fresh), ("obs", fb.observation_ok), ("norm", fb.normalizer_ok)):
                self.prep_checks[key].set_state(v)
        elif fb.state == "running":
            self.stack.setCurrentIndex(2)
            self._state("RUNNING", True)
            d = self._duration
            self.elapsed_lbl.setText(f"{fb.elapsed:.1f} / {d:.1f} s" if d else f"{fb.elapsed:.1f} s")
            self.elapsed_bar.setValue(int(1000 * fb.elapsed / d) if d else 0)
            if fb.action:
                self.action_lbl.setText("  ".join(f"{a:+.3f}" for a in fb.action))
            if fb.wrench:
                for i, k in enumerate(AXES):
                    self.run_grid.set(k, f"{fb.wrench[i]:+7.2f} {'N' if i < 3 else 'Nm'}")
            if fb.message:
                self.fb_note.setText(fb.message)
        elif fb.message:
            self.result_lbl.setText(fb.message)

    def refresh(self, s):
        if not self.isVisible():
            return
        self._update_state()
        m = self._manifest()
        if m and not self.win.policies.busy and time.monotonic() - self._last_check > 1.0:
            self._check(m)
