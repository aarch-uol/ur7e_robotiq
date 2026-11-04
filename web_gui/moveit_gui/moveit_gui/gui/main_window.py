"""MoveIt GUI main window: a task-oriented shell.

  header      left: robot, robot state · safety.  centre: TCP, control mode · hardware interface.
              right: speed, connectivity (secondary), profile (muted), STOP
  left nav    OPERATE | PROGRAMS | AUTOMATION | SETUP | DIAGNOSTICS
  bottom      collapsed log drawer ("Logs ▸"); full log under DIAGNOSTICS

The window owns the shared services (bridge, profile, calibration store, geometry, mode
manager, program runner, policy library) and the actions several pages use. Pages read
state from the 10 Hz snapshot; nothing ROS-side touches a widget.

Stop concepts, kept visibly apart:
  * STOP GUI MOTION (header, Esc): stops the GUI program, cancels the MoveIt trajectory and
    stops a running policy. Not the PolyScope program, and not an emergency stop.
  * PROGRAMS ■ Stop: ends the GUI program.
  * OPERATE robot card "PolyScope ■": the program on the pendant (UR Dashboard).
"""
from __future__ import annotations

import sys
from pathlib import Path

from PyQt5 import QtCore, QtGui, QtWidgets, sip

from ..control_modes import CONTROL_STATE_UNKNOWN
from ..core import RobotCore
from ..profile import RobotProfile
from ..program import RunState
from .theme import SPACE, STYLESHEET, T
from .widgets import Dispatcher, Led, button, label, run_in_background, set_kind, vline

NAV = ("OPERATE", "PROGRAMS", "AUTOMATION", "SETUP", "DIAGNOSTICS")
SERIOUS = ("FAILED", "Cannot", "error", "refused", "failed", "rejected", "stale")


def _core_attr(name):
    return property(lambda self: getattr(self.core, name), lambda self, v: setattr(self.core, name, v))


class MainWindow(QtWidgets.QMainWindow):
    profile_change_requested = QtCore.pyqtSignal(str)
    calibration_changed = QtCore.pyqtSignal()
    # pages read and write these; the core is the single owner
    mode = _core_attr("mode")
    busy = _core_attr("busy")
    command_source = _core_attr("command_source")
    last_refusal = _core_attr("last_refusal")
    last_snap = _core_attr("last_snap")

    def __init__(self, bridge, profile: RobotProfile, profiles_dir: Path, programs_dir: Path,
                 policies_dir: Path, fake: bool = False):
        super().__init__()
        self.bridge, self.profile, self.fake = bridge, profile, fake
        self.profiles_dir, self.programs_dir = profiles_dir, programs_dir
        self.dispatch = Dispatcher()
        # Services and the motion rules live in RobotCore, shared with the web server.
        self.core = RobotCore(bridge, profile, profiles_dir, programs_dir, policies_dir, fake=fake)
        self.store, self.geo, self.modes = self.core.store, self.core.geo, self.core.modes
        self.policies, self.runner = self.core.policies, self.core.runner
        self.core.step_listeners.append(lambda i: self.dispatch.later(self._on_runner_step, i))
        self.core.state_listeners.append(lambda st: self.dispatch.later(self._on_runner_state, st))
        self.core.done_listeners.append(lambda r: self.dispatch.later(self._on_runner_done, r))
        self.fd_buttons = []
        self._log_unread = 0
        self._log_serious = False

        self.setWindowTitle(f"MoveIt GUI — {profile.display_name}" + ("  [FAKE ROBOT]" if fake else ""))
        self.setStyleSheet(STYLESHEET)
        QtWidgets.QApplication.instance().setStyleSheet(STYLESHEET)   # dialogs and popups too
        central = QtWidgets.QWidget()
        root = QtWidgets.QVBoxLayout(central)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)
        root.addWidget(self._build_header())
        body = QtWidgets.QHBoxLayout()
        body.setSpacing(0)
        self.nav = QtWidgets.QListWidget()
        self.nav.setObjectName("nav")
        self.nav.setFixedWidth(118)
        self.nav.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.nav.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        for n in NAV:
            self.nav.addItem(n)
        body.addWidget(self.nav)
        self.stack = QtWidgets.QStackedWidget()
        body.addWidget(self.stack, 1)
        root.addLayout(body, 1)
        root.addWidget(self._build_log_drawer())
        self.setCentralWidget(central)

        from .automation import AutomationPage
        from .diagnostics import DiagnosticsPage
        from .operate import OperatePage
        from .programs_page import ProgramsPage
        from .setup_page import SetupPage
        self.pages = {"OPERATE": OperatePage(self), "PROGRAMS": ProgramsPage(self),
                      "AUTOMATION": AutomationPage(self), "SETUP": SetupPage(self),
                      "DIAGNOSTICS": DiagnosticsPage(self)}
        for n in NAV:
            self.stack.addWidget(self.pages[n])
        self.nav.currentRowChanged.connect(self.stack.setCurrentIndex)
        self.nav.setCurrentRow(0)
        QtWidgets.QShortcut(QtGui.QKeySequence("Esc"), self, activated=self.stop_all)
        for i, n in enumerate(NAV):
            QtWidgets.QShortcut(QtGui.QKeySequence(f"Ctrl+{i + 1}"), self,
                                activated=lambda i=i: self.nav.setCurrentRow(i))

        self.calibration_changed.connect(self._on_calibration_changed)
        self._on_calibration_changed()
        self._timer = QtCore.QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(100)
        if self.policies.errors:
            for f, e in self.policies.errors.items():
                self.bridge.log(f"policy manifest {f}: {e}")
        self._fit_to_screen()

    def _fit_to_screen(self):
        scr = QtWidgets.QApplication.primaryScreen()
        if scr is None:
            self.resize(1280, 760)
            return
        g = scr.availableGeometry()
        self.resize(min(1440, g.width()), min(900, g.height()))

    # ================================================================ header
    def _build_header(self):
        w = QtWidgets.QFrame()
        w.setObjectName("header")
        row = QtWidgets.QHBoxLayout(w)
        row.setContentsMargins(SPACE["m"], 6, SPACE["m"], 6)
        row.setSpacing(SPACE["l"])

        left = QtWidgets.QVBoxLayout()
        left.setSpacing(1)
        left.addWidget(label(self.profile.display_name, "robot"))
        st = QtWidgets.QHBoxLayout()
        st.setSpacing(5)
        self.state_dot = Led("", small=False)
        st.addWidget(self.state_dot.dot)
        self.state_lbl = label("—", "state")
        st.addWidget(self.state_lbl)
        st.addStretch(1)
        left.addLayout(st)
        row.addLayout(left)
        row.addWidget(vline())

        tcp = QtWidgets.QVBoxLayout()
        tcp.setSpacing(1)
        tcp.addWidget(label("TCP", "field"))
        self.tcp_combo = QtWidgets.QComboBox()
        self.tcp_combo.setSizeAdjustPolicy(QtWidgets.QComboBox.AdjustToContents)
        self.tcp_combo.setToolTip("Active TCP: Cartesian targets, displays and recorded poses refer to it")
        self.tcp_combo.activated.connect(lambda i: self.set_active_tcp(self.tcp_combo.itemData(i)))
        tcp.addWidget(self.tcp_combo)
        row.addLayout(tcp)

        ctrl = QtWidgets.QVBoxLayout()
        ctrl.setSpacing(1)
        ctrl.addWidget(label("CONTROL", "field"))
        self.mode_badge = label("—", "hvalue")
        self.mode_badge.setToolTip("Active arm control mode · hardware command interface")
        ctrl.addWidget(self.mode_badge)
        row.addLayout(ctrl)
        row.addStretch(1)

        meta = QtWidgets.QVBoxLayout()
        meta.setSpacing(3)
        lights = QtWidgets.QHBoxLayout()
        lights.setSpacing(SPACE["s"])
        self.leds = {}
        for key, text in (("ROS", "ROS"), ("UR", "UR"), ("MoveIt", "MoveIt"), ("F/T", "ATI"),
                          ("Gripper", "Robotiq"), ("ExtControl", "ExtCtrl")):
            self.leds[key] = Led(text, small=True, kind="led")
            lights.addWidget(self.leds[key])
        self.leds["ExtControl"].setToolTip("External Control program on the pendant")
        self.conn_ok = label("All connected", "led")
        lights.insertWidget(0, self.conn_ok)
        meta.addLayout(lights)
        ip = label(f"{self.profile.ur_type} · {self.profile.robot_ip} · {self.profile.name}", "meta")
        ip.setAlignment(QtCore.Qt.AlignRight)
        meta.addWidget(ip)
        row.addLayout(meta)
        row.addWidget(vline())

        sp = QtWidgets.QVBoxLayout()
        sp.setSpacing(1)
        sp.addWidget(label("SPEED", "field"))
        self.speed_lbl = label("—", "hvalue")
        sp.addWidget(self.speed_lbl)
        row.addLayout(sp)
        self.stop_all_btn = button("■ STOP", "stop",
                                   "Stop GUI motion: stops the GUI program, cancels the MoveIt trajectory and stops a "
                                   "running policy (Esc).\nDoes not stop the PolyScope program. Not an emergency stop.")
        self.stop_all_btn.setMinimumHeight(36)
        self.stop_all_btn.clicked.connect(self.stop_all)
        row.addWidget(self.stop_all_btn)
        return w

    # ============================================================ log drawer
    def _build_log_drawer(self):
        w = QtWidgets.QFrame()
        w.setObjectName("logbar")
        lay = QtWidgets.QVBoxLayout(w)
        lay.setContentsMargins(SPACE["s"], 2, SPACE["s"], 2)
        lay.setSpacing(2)
        row = QtWidgets.QHBoxLayout()
        self.log_toggle = QtWidgets.QToolButton()
        self.log_toggle.setCheckable(True)
        self.log_toggle.toggled.connect(self._toggle_log)
        row.addWidget(self.log_toggle)
        self.log_last = label("", "muted")
        row.addWidget(self.log_last, 1)
        lay.addLayout(row)
        self.log_drawer = QtWidgets.QPlainTextEdit()
        self.log_drawer.setReadOnly(True)
        self.log_drawer.setMaximumBlockCount(500)
        self.log_drawer.setFixedHeight(130)
        self.log_drawer.setVisible(False)
        lay.addWidget(self.log_drawer)
        self._update_log_toggle()
        return w

    def _toggle_log(self, on: bool):
        self.log_drawer.setVisible(on)
        if on:
            self._log_unread, self._log_serious = 0, False
        self._update_log_toggle()

    def _update_log_toggle(self):
        open_ = self.log_toggle.isChecked()
        badge = ""
        if self._log_unread and not open_:
            badge = f"  ({self._log_unread} new{', ⚠' if self._log_serious else ''})"
        self.log_toggle.setText(f"Logs {'▾' if open_ else '▸'}{badge}")
        self.log_toggle.setStyleSheet(f"color:{T['danger']};" if self._log_serious and not open_ else "")

    def _append_logs(self, lines):
        if not lines:
            return
        for line in lines:
            self.log_drawer.appendPlainText(line)
            self.pages["DIAGNOSTICS"].log_view.appendPlainText(line)
            if any(k in line for k in SERIOUS):
                self._log_serious = True
        self.log_last.setText(lines[-1])
        set_kind(self.log_last, "error" if any(k in lines[-1] for k in SERIOUS) else "muted")
        if not self.log_toggle.isChecked():
            self._log_unread += len(lines)
        self._update_log_toggle()

    # ================================================================ refresh
    def _refresh(self):
        self._append_logs(self.bridge.drain_logs())
        s = self.core.update()
        all_ok = True
        for k, led in self.leds.items():
            state = s.program_running if k == "ExtControl" else s.connected.get(k)
            led.set_state(state)
            led.setVisible(state is not True)      # only what needs attention
            all_ok = all_ok and state is True
        self.conn_ok.setVisible(all_ok)
        healthy = None if s.robot_mode == "?" else (s.robot_mode == "RUNNING" and s.safety_mode in ("NORMAL", "REDUCED"))
        self.state_dot.set_state(healthy)
        self.state_lbl.setText(f"{s.robot_mode} · {s.safety_mode}")
        self.speed_lbl.setText(f"{s.speed_scaling * 100:.0f}%" if s.speed_scaling is not None else "—")
        m = self.profile.modes.get(self.mode)
        if self.mode == CONTROL_STATE_UNKNOWN:
            text, color = "CONTROL STATE UNKNOWN", "#f0827a"
        elif m:
            text = f"{m.label} · {'Direct torque' if m.direct_torque else m.command_interface.capitalize()}"
            color = "#e3ae5b" if m.direct_torque or m.experimental else T["on_chrome"]
        else:
            text, color = self.mode.capitalize(), "#f0827a" if self.mode == "unknown" else T["on_chrome2"]
        if self.mode_badge.text() != text:
            self.mode_badge.setText(text)
            self.mode_badge.setStyleSheet(f"color:{color};")
        self.fd_buttons = [b for b in self.fd_buttons if not sip.isdeleted(b)]
        for b in self.fd_buttons:
            b.setChecked(s.freedrive)
            b.setText("End freedrive" if s.freedrive else "Freedrive")
        page = self.stack.currentWidget()
        if hasattr(page, "refresh"):
            page.refresh(s)

    # ============================================================== freedrive
    def freedrive_button(self) -> QtWidgets.QPushButton:
        """Teach: drag the arm by hand, then record. Every page that records poses gets one of these."""
        b = button("Freedrive", "secondary", "Hand-guide the arm (UR freedrive). Ending it returns to the "
                                             "previous control mode.")
        b.setCheckable(True)
        b.clicked.connect(lambda: (b.setChecked(self.last_snap.freedrive), self.toggle_freedrive()))
        self.fd_buttons.append(b)          # buttons of closed dialogs are pruned in _refresh
        return b

    def toggle_freedrive(self):
        on = not self.last_snap.freedrive
        self.bg(lambda: self.core.freedrive(on), lambda r: self.report(r) if r[0] else self.show_refusal(r[1]))

    # ================================================================ helpers
    def bg(self, fn, on_done=None):
        run_in_background(self.dispatch, fn, on_done, lambda e: self.bridge.log(f"error: {e}"))

    def report(self, res):
        """Operator feedback goes to the log drawer line (no separate status bar: saves height)."""
        ok, msg = res if isinstance(res, tuple) else (bool(res), "")
        if not msg:
            return
        self.bridge.log(msg if ok else f"FAILED: {msg}")
        self.log_last.setText(msg)
        set_kind(self.log_last, "secondary" if ok else "error")

    def confirm(self, title: str, text: str) -> bool:
        return QtWidgets.QMessageBox.question(self, title, text) == QtWidgets.QMessageBox.Yes

    def dashboard(self, cmd: str, confirm: bool = False):
        if confirm and not self.confirm("UR Dashboard", f"Send '{cmd}' to the robot?"):
            return
        self.bg(lambda: self.bridge.dashboard(cmd), self.report)

    def stop_all(self):
        self.core.stop_all()

    def show_refusal(self, msg: str):
        self.last_refusal = msg
        self.bridge.log(msg)
        self.log_last.setText(msg)
        set_kind(self.log_last, "error")

    def start_motion(self, fn, desc: str, confirm: bool):
        """Manual MoveIt motion. fn runs in a worker thread; the core decides whether it may."""
        why = self.core.motion_refusal()
        if why:
            self.report((False, why))
            return
        if confirm and not self.confirm("Move robot", desc + "\n\nExecute?"):
            return
        why = self.core.begin_motion()
        if why:
            self.report((False, why))
            return

        def done(ok):
            self.core.end_motion()
            self.report((ok, "motion done" if ok else "motion failed / cancelled"))
        self.bg(fn, done)

    def set_active_tcp(self, name: str):
        if not name:
            return
        self.store.set_active_tcp(name)
        self.bridge.log(f"active TCP: {name}")
        self.calibration_changed.emit()

    def _on_calibration_changed(self):
        self.tcp_combo.blockSignals(True)
        self.tcp_combo.clear()
        for n, t in self.store.tcps.items():
            self.tcp_combo.addItem(n if t.builtin else f"{n} · virtual", n)
        self.tcp_combo.setCurrentIndex(self.tcp_combo.findData(self.store.active_tcp))
        self.tcp_combo.blockSignals(False)
        for p in self.pages.values() if hasattr(self, "pages") else ():
            if hasattr(p, "calibration_changed"):
                p.calibration_changed()

    # ---------------------------------------------------------- runner events
    def _on_runner_step(self, i: int):
        self.pages["PROGRAMS"].highlight_step(i)

    def _on_runner_state(self, s: RunState):
        self.pages["PROGRAMS"].runner_state_changed(s)

    def _on_runner_done(self, res):
        ok, msg = res
        self.report((ok, f"program {msg}"))

    def request_profile_change(self, path: str):
        if self.runner.state != RunState.IDLE:
            self.report((False, "stop the program first"))
            return
        if self.confirm("Change robot", f"Reconnect with profile '{Path(path).stem}'?\n\nThis is not a live switch: "
                        "URDF, kinematics, limits and calibration change. The ROS stack for that robot must "
                        "already be running."):
            self.profile_change_requested.emit(path)

    def closeEvent(self, e):
        self._timer.stop()
        try:
            self.core.stop_all("GUI closed")             # zeroes a force, stops jog / program / policy / motion
            self.core.recorder.stop_and_wait("GUI closed")
        except Exception as ex:                          # closing must go on: the bridge zeroes wrenches again
            print(f"shutdown: {ex}", file=sys.stderr)
        self.runner.stop()
        self.policies.cancel()
        super().closeEvent(e)
