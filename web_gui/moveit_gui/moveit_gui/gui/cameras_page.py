"""SETUP > Cameras: is each expected camera publishing, and what does any image topic look like right now.

Only one image topic is subscribed at a time, and only while this page is on screen.
"""
from __future__ import annotations

import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets

from .theme import SPACE, T
from .widgets import Card, Dot, button, label, muted, page_widget, section_label


class CameraRow(QtWidgets.QFrame):
    clicked = QtCore.pyqtSignal(str)

    def __init__(self, name, topic):
        super().__init__()
        self.topic = topic
        self.setCursor(QtCore.Qt.PointingHandCursor)
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(SPACE["s"], SPACE["xs"], SPACE["s"], SPACE["xs"])
        self.dot = Dot(8)
        lay.addWidget(self.dot, 0, QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        col = QtWidgets.QVBoxLayout()
        col.setSpacing(0)
        col.addWidget(label(name))
        t = muted(topic, wrap=False)
        t.setToolTip(topic)
        col.addWidget(t)
        lay.addLayout(col, 1)
        self.state = muted("", wrap=False)
        lay.addWidget(self.state, 0, QtCore.Qt.AlignTop)

    def set_publishers(self, n: int):
        self.dot.set_state(True if n > 0 else None)       # grey, not red: not connected is not a fault
        self.state.setText("publishing" if n > 0 else "no publisher")

    def set_selected(self, on: bool):
        self.setStyleSheet(f"CameraRow {{ background: {T['selected']}; border-radius: 3px; }}" if on else "")

    def mousePressEvent(self, e):
        self.clicked.emit(self.topic)


class CamerasPage(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        self.topic = ""
        self._status_t = 0.0
        page = page_widget()
        QtWidgets.QVBoxLayout(self).addWidget(page)
        self.layout().setContentsMargins(0, 0, 0, 0)
        lay = QtWidgets.QHBoxLayout(page)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(SPACE["s"])

        left = Card("Cameras")
        left.setFixedWidth(300)
        self.rows = []
        for cam in win.profile.cameras:
            r = CameraRow(cam.get("name", cam["topic"]), cam["topic"])
            r.clicked.connect(self.watch)
            self.rows.append(r)
            left.body.addWidget(r)
        if not self.rows:
            left.body.addWidget(muted("No cameras in this robot profile (cameras: in the profile YAML)."))
        left.body.addSpacing(SPACE["m"])
        left.body.addWidget(section_label("Any image topic"))
        self.combo = QtWidgets.QComboBox()
        self.combo.setEditable(True)
        self.combo.setInsertPolicy(QtWidgets.QComboBox.NoInsert)
        self.combo.activated.connect(lambda _i: self.watch(self.combo.currentText().strip()))
        self.combo.lineEdit().returnPressed.connect(lambda: self.watch(self.combo.currentText().strip()))
        left.body.addWidget(self.combo)
        rescan = button("Rescan topics", "quiet")
        rescan.clicked.connect(self.rescan)
        left.body.addWidget(rescan, 0, QtCore.Qt.AlignLeft)
        left.body.addStretch(1)
        lay.addWidget(left)

        right = Card()
        head = QtWidgets.QHBoxLayout()
        self.title = label("No topic selected", "title")
        head.addWidget(self.title, 1)
        self.pause = button("Pause", "secondary")
        self.pause.setCheckable(True)
        self.pause.toggled.connect(lambda on: self.pause.setText("Resume" if on else "Pause"))
        head.addWidget(self.pause)
        right.body.addLayout(head)
        self.info = label("", "secondary")
        right.body.addWidget(self.info)
        self.view = QtWidgets.QLabel()
        self.view.setAlignment(QtCore.Qt.AlignCenter)
        self.view.setMinimumSize(320, 240)
        self.view.setSizePolicy(QtWidgets.QSizePolicy.Ignored, QtWidgets.QSizePolicy.Ignored)
        self.view.setStyleSheet(f"background: {T['chrome']}; color: {T['on_chrome2']}; border-radius: 3px;")
        right.body.addWidget(self.view, 1)
        lay.addWidget(right, 1)
        self.rescan()

    # ----------------------------------------------------------------- topics
    def rescan(self):
        cur = self.combo.currentText()
        self.combo.blockSignals(True)
        self.combo.clear()
        try:
            topics = self.win.bridge.image_topics()
        except Exception as e:                       # graph query failed: say so, keep the page usable
            self.win.report((False, f"cannot list image topics: {e}"))
            topics = []
        for t, typ in topics:
            self.combo.addItem(t)
            self.combo.setItemData(self.combo.count() - 1, typ, QtCore.Qt.ToolTipRole)
        self.combo.setEditText(cur or self.topic)
        self.combo.blockSignals(False)
        self._update_status(force=True)

    def _update_status(self, force=False):
        now = time.monotonic()
        if not force and now - self._status_t < 1.0:
            return
        self._status_t = now
        for r in self.rows:
            r.set_publishers(self.win.bridge.topic_publishers(r.topic))

    def watch(self, topic: str):
        self.topic = topic
        for r in self.rows:
            r.set_selected(r.topic == topic)
        names = {c["topic"]: c.get("name", "") for c in self.win.profile.cameras}
        self.title.setText(names.get(topic) or topic or "No topic selected")
        self.combo.setEditText(topic)
        self.view.clear()
        if not self.isVisible():
            return
        ok, msg = self.win.bridge.watch_image(topic or None)
        if not ok:
            self.view.setText(f"{msg}\n\nIs the camera driver running?")
            self.info.setText("")

    # ----------------------------------------------------------------- live
    def showEvent(self, e):
        super().showEvent(e)
        self.rescan()
        if self.topic:
            self.watch(self.topic)
        elif self.rows:
            self.watch(self.rows[0].topic)

    def hideEvent(self, e):
        super().hideEvent(e)
        self.win.bridge.watch_image(None)       # no image traffic while nobody looks

    def refresh(self):
        self._update_status()
        w = self.win.bridge.watched_image()
        if w is None or w.topic != self.topic:
            return
        rate, age = w.stats.rate(), w.stats.age()
        f = w.latest()
        if f is None:
            self.info.setText("Subscribed, no image received yet")
            if not self.view.pixmap():
                self.view.setText("Waiting for the first image…")
            return
        stale = age is None or age > 2.0
        size = f"{f.width}×{f.height} · " if f.width else ""
        self.info.setText(f"{size}{f.encoding} · {rate:.1f} Hz · {f.frame_id or 'no frame_id'}"
                          + (f" · last image {age:.0f} s ago" if stale and age is not None else ""))
        if self.pause.isChecked():
            return
        if f.error:
            self.view.setText(f.error)
            return
        img = self._qimage(f)
        if img is None or img.isNull():
            self.view.setText("cannot decode this image")
            return
        self.view.setPixmap(QtGui.QPixmap.fromImage(img).scaled(self.view.size(), QtCore.Qt.KeepAspectRatio,
                                                                QtCore.Qt.SmoothTransformation))

    @staticmethod
    def _qimage(f):
        if f.compressed is not None:
            return QtGui.QImage.fromData(f.compressed)
        a = np.ascontiguousarray(f.rgb)
        h, w = a.shape[:2]
        return QtGui.QImage(a.data, w, h, 3 * w, QtGui.QImage.Format_RGB888).copy()
