"""Small reusable widgets and the thread -> Qt plumbing. All styling comes from theme.py.

Button roles (visual priority):
  primary       the one main action on a screen
  secondary     neutral, the default
  quiet         tertiary actions, no outline
  danger        Delete, Stop
  stop          the header STOP only
  experimental  anything that enters direct torque (amber, not red)
"""
from __future__ import annotations

import threading
import traceback

from PyQt5 import QtCore, QtGui, QtWidgets

from .theme import CONTROL_H, SPACE, STYLESHEET, T, mono_font  # noqa: F401  (STYLESHEET re-exported)

COLORS = {"primary": T["accent"], "danger": T["danger"], "stop": T["danger"], "experimental": T["warn"],
          "ok": T["ok"], "warn": T["warn"], "bad": T["danger"], "muted": T["muted"]}


class Dispatcher(QtCore.QObject):
    """Runs a callable on the Qt thread. Emit from any thread."""
    call = QtCore.pyqtSignal(object, object)

    def __init__(self):
        super().__init__()
        self.call.connect(lambda fn, arg: fn(arg))

    def later(self, fn, arg=None):
        self.call.emit(fn, arg)


def run_in_background(dispatcher: Dispatcher, fn, on_done=None, on_error=None):
    """Run fn() in a daemon thread; deliver its result to on_done on the Qt thread."""
    def work():
        try:
            res = fn()
        except Exception as e:  # pragma: no cover
            traceback.print_exc()
            if on_error:
                dispatcher.later(on_error, e)
            return
        if on_done:
            dispatcher.later(on_done, res)
    t = threading.Thread(target=work, daemon=True)
    t.start()
    return t


def button(text: str, role: str = "secondary", tooltip: str = "") -> QtWidgets.QPushButton:
    b = QtWidgets.QPushButton(text)
    b.setProperty("role", role)
    b.setMinimumHeight(CONTROL_H)
    if tooltip:
        b.setToolTip(tooltip)
    return b


def big_button(text: str, color: str = None, tooltip: str = "") -> QtWidgets.QPushButton:
    """Kept for the calibration pages: maps the old colour argument onto a role."""
    role = "primary" if color in ("#2563eb", "#16a34a") else "secondary"
    return button(text, role, tooltip)


def repolish(w: QtWidgets.QWidget):
    w.style().unpolish(w)
    w.style().polish(w)


def label(text: str = "", kind: str = "", wrap: bool = False, selectable: bool = False) -> QtWidgets.QLabel:
    lab = QtWidgets.QLabel(text)
    if kind:
        lab.setProperty("kind", kind)
    lab.setWordWrap(wrap)
    if selectable:
        lab.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
    return lab


def set_kind(lab: QtWidgets.QLabel, kind: str):
    if lab.property("kind") != kind:
        lab.setProperty("kind", kind)
        repolish(lab)


def section_label(text: str) -> QtWidgets.QLabel:
    return label(text.upper(), "section")


def hline() -> QtWidgets.QFrame:
    f = QtWidgets.QFrame()
    f.setProperty("line", "h")
    return f


def vline() -> QtWidgets.QFrame:
    f = QtWidgets.QFrame()
    f.setProperty("line", "v")
    return f


class Card(QtWidgets.QFrame):
    """A surface with an optional section heading. No outline: grouping comes from the background level."""
    def __init__(self, title: str = "", parent=None, right_widget: QtWidgets.QWidget = None):
        super().__init__(parent)
        self.setProperty("surface", "panel")
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(SPACE["m"], SPACE["m"], SPACE["m"], SPACE["m"])
        outer.setSpacing(SPACE["s"])
        if title or right_widget:
            head = QtWidgets.QHBoxLayout()
            if title:
                head.addWidget(section_label(title))
            head.addStretch(1)
            if right_widget:
                head.addWidget(right_widget)
            outer.addLayout(head)
        self.body = QtWidgets.QVBoxLayout()
        self.body.setSpacing(SPACE["s"] - 2)
        outer.addLayout(self.body)


class Collapsible(QtWidgets.QWidget):
    """'Title ▸' toggle that reveals its content (progressive disclosure)."""
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        lay = QtWidgets.QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(SPACE["xs"])
        self.toggle = QtWidgets.QToolButton()
        self.toggle.setText(f"{title} ▸")
        self.toggle.setCheckable(True)
        self._title = title
        self.toggle.toggled.connect(self._set_open)
        lay.addWidget(self.toggle, 0, QtCore.Qt.AlignLeft)
        self.content = QtWidgets.QWidget()
        self.content_layout = QtWidgets.QVBoxLayout(self.content)
        self.content_layout.setContentsMargins(0, 0, 0, 0)
        self.content.setVisible(False)
        lay.addWidget(self.content)

    def _set_open(self, on: bool):
        self.toggle.setText(f"{self._title} {'▾' if on else '▸'}")
        self.content.setVisible(on)


def scrolled(widget: QtWidgets.QWidget) -> QtWidgets.QScrollArea:
    """Wrap a dense form so it stays usable on 1366x768 / high-DPI screens."""
    sa = QtWidgets.QScrollArea()
    sa.setWidgetResizable(True)
    sa.setFrameShape(QtWidgets.QFrame.NoFrame)
    sa.setWidget(widget)
    return sa


def page_widget() -> QtWidgets.QWidget:
    w = QtWidgets.QWidget()
    w.setObjectName("page")
    return w


def muted(text: str, wrap: bool = True) -> QtWidgets.QLabel:
    return label(text, "muted", wrap)


class Dot(QtWidgets.QWidget):
    """A small status dot. Painted, so it does not depend on font glyphs. None = unknown."""
    COLORS = {True: T["ok"], False: T["danger"], None: T["off"], "warn": T["warn"]}

    def __init__(self, d: int = 8, parent=None):
        super().__init__(parent)
        self._d, self._state = d, None
        self.setFixedSize(d, d)

    def set_state(self, state):
        if state != self._state:
            self._state = state
            self.update()

    def paintEvent(self, _e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        p.setPen(QtCore.Qt.NoPen)
        p.setBrush(QtGui.QColor(self.COLORS.get(self._state, T["off"])))
        p.drawEllipse(self.rect().adjusted(0, 0, -1, -1))


class Led(QtWidgets.QWidget):
    """Dot + label."""
    def __init__(self, text: str, parent=None, small: bool = False, kind: str = ""):
        super().__init__(parent)
        lay = QtWidgets.QHBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.setSpacing(4)
        self.dot = Dot(6 if small else 8)
        self.label = label(text, kind)
        lay.addWidget(self.dot, 0, QtCore.Qt.AlignVCenter)
        lay.addWidget(self.label, 0, QtCore.Qt.AlignVCenter)

    def set_state(self, ok):
        self.dot.set_state(ok)


class ReadoutGrid(QtWidgets.QWidget):
    """Read-only aligned name / value rows. Values use a fixed font so digits do not jump."""
    def __init__(self, rows=(), parent=None, spacing: int = 2):
        super().__init__(parent)
        self.grid = QtWidgets.QGridLayout(self)
        self.grid.setContentsMargins(0, 0, 0, 0)
        self.grid.setVerticalSpacing(spacing)
        self.grid.setHorizontalSpacing(SPACE["m"])
        self.grid.setColumnStretch(1, 1)
        self.values = {}
        for key, name in rows:
            self.add(key, name)

    def add(self, key: str, name: str, tip: str = "") -> QtWidgets.QLabel:
        r = self.grid.rowCount()
        n = label(name, "field")
        if tip:
            n.setToolTip(tip)
        v = label("—", "value", selectable=True)
        v.setFont(mono_font())
        v.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)
        self.grid.addWidget(n, r, 0)
        self.grid.addWidget(v, r, 1)
        self.values[key] = v
        return v

    def set(self, key: str, text: str):
        self.values[key].setText(text)


def mono_label(text: str = "—") -> QtWidgets.QLabel:
    lab = QtWidgets.QLabel(text)
    lab.setFont(mono_font())
    lab.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
    return lab


def section(title: str) -> QtWidgets.QGroupBox:
    return QtWidgets.QGroupBox(title)


class CenterBar(QtWidgets.QWidget):
    """Thin zero-centred bar for signed values."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self._v = 0.0
        self.setMinimumSize(40, 4)
        self.setMaximumHeight(4)

    def set_fraction(self, v: float):
        self._v = max(-1.0, min(1.0, float(v)))
        self.update()

    def paintEvent(self, _e):
        p = QtGui.QPainter(self)
        r = self.rect()
        p.fillRect(r, QtGui.QColor(T["sunken"]))
        mid = r.left() + r.width() // 2
        w = int(self._v * r.width() / 2)
        color = QtGui.QColor(T["text2"] if abs(self._v) < 0.8 else T["warn"])
        if w >= 0:
            p.fillRect(mid, r.top(), w, r.height(), color)
        else:
            p.fillRect(mid + w, r.top(), -w, r.height(), color)
        p.fillRect(mid, r.top(), 1, r.height(), QtGui.QColor(T["muted"]))


class FlowView(QtWidgets.QWidget):
    """The command path on one line: nodes joined by arrows."""
    def __init__(self, parent=None):
        super().__init__(parent)
        self.lay = QtWidgets.QVBoxLayout(self)
        self.lay.setContentsMargins(0, 0, 0, 0)
        self.lay.setSpacing(0)
        self._nodes = None

    def set_nodes(self, nodes):
        nodes = list(nodes)
        if nodes == self._nodes:
            return
        self._nodes = nodes
        while self.lay.count():
            it = self.lay.takeAt(0)
            w = it.widget()
            if w:
                w.setParent(None)
                w.deleteLater()
        if nodes:
            self.lay.addWidget(label(" → ".join(nodes), "flow", wrap=True))
