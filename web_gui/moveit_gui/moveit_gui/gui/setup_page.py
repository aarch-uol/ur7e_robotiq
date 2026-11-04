"""SETUP: engineering / calibration tasks, one section at a time, each in a scroll area."""
from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from ..profile import list_profiles
from .calibration_tab import CalibrationPages
from .cameras_page import CamerasPage
from .setup_tasks import PayloadPage, VialPage
from .theme import SPACE
from .widgets import Card, ReadoutGrid, button, muted, page_widget, scrolled

SECTIONS = ("TCPs", "Fixtures", "Vial Geometry", "F/T + Payload", "Cameras", "Robot Profile")
# Vision baselines stay in CalibrationPages (tested, not in the normal nav); a future advanced Perception
# section will host them once a detector and a camera calibration exist.


class SetupPage(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.cal = CalibrationPages(win.store, win.geo, win.bridge, win.dispatch, win.report, fake=win.fake)
        self.cal.changed.connect(win.calibration_changed.emit)
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(SPACE["s"], SPACE["s"], SPACE["s"], SPACE["s"])
        lay.setSpacing(SPACE["s"])
        self.sub = QtWidgets.QListWidget()
        self.sub.setObjectName("subnav")
        self.sub.setFixedWidth(128)
        self.sub.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        for s in SECTIONS:
            self.sub.addItem(s)
        lay.addWidget(self.sub)
        self.stack = QtWidgets.QStackedWidget()
        lay.addWidget(self.stack, 1)
        self.vial = VialPage(win, win.calibration_changed.emit)
        self.payload = PayloadPage(win)
        self.cameras = CamerasPage(win)
        pages = {"Cameras": self.cameras, "TCPs": self.cal.pages["TCPs"], "Fixtures": self.cal.pages["Fixtures"], "Vial Geometry": self.vial,
                 "F/T + Payload": self.payload, "Robot Profile": self._profile_page()}
        for s in SECTIONS:
            self.stack.addWidget(scrolled(pages[s]))
        self.sub.currentRowChanged.connect(self.stack.setCurrentIndex)
        self.sub.setCurrentRow(0)

    def _profile_page(self):
        w, p = self.win, self.win.profile
        page = page_widget()
        lay = QtWidgets.QVBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(SPACE["s"])
        c = Card("Robot profile")
        grid = ReadoutGrid(spacing=SPACE["xs"])
        for k, v in (("Profile", f"{p.display_name} ({p.name})"), ("File", str(p.source_path)),
                     ("Robot", f"{p.ur_type} at {p.robot_ip}"), ("MoveIt group", p.moveit.get("group")),
                     ("Base frame", p.base_link), ("Planning link", f"{p.planning_link} (not the TCP)"),
                     ("Arm controller manager", p.services.get("controller_manager")),
                     ("Gripper controller manager", p.gripper.get("controller_manager", "—")),
                     ("F/T topic", p.topics.get("wrench", "—")),
                     ("Calibration file", str(w.store.path))):
            grid.add(k, k).setText(str(v))
        c.body.addWidget(grid)
        lay.addWidget(c)
        modes = Card("Control modes")
        t = QtWidgets.QTableWidget(len(p.modes), 6)
        t.setHorizontalHeaderLabels(["Mode", "Interface", "Source", "MoveIt controller", "F/T", "Status"])
        for r, m in enumerate(p.modes.values()):
            for col, v in enumerate((m.label, m.command_interface.upper(), m.command_source,
                                     m.moveit_controller or "—", "required" if m.requires_ft else "",
                                     ("EXPERIMENTAL · " if m.experimental else "") +
                                     ("enabled" if m.allow_activation else "disabled"))):
                t.setItem(r, col, QtWidgets.QTableWidgetItem(v))
        t.verticalHeader().setVisible(False)
        t.resizeColumnsToContents()
        t.setMinimumHeight(150)
        t.setShowGrid(False)
        modes.body.addWidget(t)
        lay.addWidget(modes)
        sw = Card("Change robot")
        sw.body.addWidget(muted("Changing the robot model is a reconnect, not a live switch: URDF, kinematics, "
                                "limits and calibration file change, and that robot's ROS stack must be running."))
        row = QtWidgets.QHBoxLayout()
        cb = QtWidgets.QComboBox()
        for path in list_profiles(w.profiles_dir):
            cb.addItem(path.stem, str(path))
        cb.setCurrentText(p.source_path.stem if p.source_path else "")
        row.addWidget(cb)
        b = button("Reconnect…", "secondary")
        b.clicked.connect(lambda: w.request_profile_change(cb.currentData()))
        row.addWidget(b)
        row.addStretch(1)
        sw.body.addLayout(row)
        lay.addWidget(sw)
        lay.addStretch(1)
        return page

    def calibration_changed(self):
        self.cal.refresh()
        self.vial.refresh_labels()

    def refresh(self, s):
        if self.cameras.isVisible():
            self.cameras.refresh()
