"""Setup pages: TCPs (incl. pivot calibration), fixtures (wizard), vision baselines.

CalibrationPages builds one widget per page (self.pages); the SETUP screen places each
in its own scroll area.

UI units: millimetres and degrees. Stored: metres and quaternions (xyzw).
All robot reads go through Geometry, so every captured value is explicit about
which TCP and which frame it refers to.
"""
from __future__ import annotations

import math
import time
from pathlib import Path
from typing import List, Optional

import numpy as np
from PyQt5 import QtCore, QtWidgets

from .. import transforms as tf
from ..calibration import (CORRECTION_SIDES, CalibrationError, CalibrationStore, FixtureDef, PivotResult,
                           TcpDef, VisionBaseline, frame_from_three_points, now_iso, pivot_quality,
                           solve_pivot, solve_vision_baseline)
from ..geometry import Geometry, GeometryError
from .theme import T
from .widgets import Card, Collapsible, button, mono_label, muted, page_widget, run_in_background

VIRTUAL_NOTE = ("A virtual TCP changes what Cartesian poses refer to. It does not change the "
                "MoveIt collision model: a physically different tool needs a robot-description update.")


def xyz_rpy_inputs(layout: QtWidgets.QGridLayout, row: int, rng_mm: float = 2000.0):
    """Six spin boxes (X Y Z in mm, R P Y in degrees) placed in two rows."""
    spins = []
    for i, (lab, suf, rng, dec) in enumerate((("X", " mm", rng_mm, 2), ("Y", " mm", rng_mm, 2),
                                              ("Z", " mm", rng_mm, 2), ("Roll", " °", 180, 3),
                                              ("Pitch", " °", 180, 3), ("Yaw", " °", 180, 3))):
        sp = QtWidgets.QDoubleSpinBox()
        sp.setRange(-rng, rng)
        sp.setDecimals(dec)
        sp.setSuffix(suf)
        r, c = row + i // 3, (i % 3) * 2
        layout.addWidget(QtWidgets.QLabel(lab), r, c)
        layout.addWidget(sp, r, c + 1)
        spins.append(sp)
    return spins


def set_T_spins(spins, T: np.ndarray):
    xyz, rpy = tf.T_to_mm_deg(T)
    for sp, v in zip(spins, xyz + rpy):
        sp.setValue(v)


def T_from_spins(spins) -> np.ndarray:
    v = [sp.value() for sp in spins]
    return tf.mm_deg_to_T(v[:3], v[3:])


class CalibrationPages(QtCore.QObject):
    changed = QtCore.pyqtSignal()        # TCPs / frames changed: header + combos must refresh

    def __init__(self, store: CalibrationStore, geometry: Geometry, bridge, dispatch, report,
                 fake: bool = False, parent=None):
        super().__init__(parent)
        self.fake = fake
        self.store, self.geo, self.bridge, self.dispatch, self.report = store, geometry, bridge, dispatch, report
        self.path_label = QtWidgets.QLabel()
        self.path_label.setProperty("kind", "muted")
        self.pages = {"TCPs": self._build_tcps(), "Fixtures": self._build_fixtures(),
                      "Vision baselines": self._build_vision()}
        self._pivot_samples: List[np.ndarray] = []
        self._pivot_result: Optional[PivotResult] = None
        self._teach_pts = {}
        self._vision_expected: Optional[np.ndarray] = None
        self._vision_samples: List[np.ndarray] = []
        self.refresh()

    # ============================================================== common
    def _err(self, e: Exception):
        QtWidgets.QMessageBox.warning(None, "Calibration", str(e))

    def _fill_frames(self, cb: QtWidgets.QComboBox):
        cur = cb.currentText()
        cb.blockSignals(True)
        cb.clear()
        cb.addItems(self.store.frame_names())
        i = cb.findText(cur)
        cb.setCurrentIndex(i if i >= 0 else 0)
        cb.blockSignals(False)

    def _fill_tcps(self, cb: QtWidgets.QComboBox):
        cur = cb.currentText()
        cb.blockSignals(True)
        cb.clear()
        cb.addItems(list(self.store.tcps))
        i = cb.findText(cur)
        cb.setCurrentIndex(i if i >= 0 else cb.findText(self.store.active_tcp))
        cb.blockSignals(False)

    def refresh(self):
        self.path_label.setText(f"Calibration file: {self.store.path}   (robot: {self.store.profile.name})")
        self._refresh_tcp_table()
        self._refresh_fixture_table()
        self._refresh_vision_table()
        self._fill_tcps(self.pivot_parent)
        for cb in (self.fx_parent, self.vb_reference, self.vb_gt_frame):
            self._fill_frames(cb)
        self.teach_tcp_lbl.setText(f"Points are captured with the active TCP: <b>{self.store.active_tcp}</b>")

    def _emit(self):
        self.refresh()
        self.changed.emit()

    # ================================================================ TCPs
    def _build_tcps(self):
        page = page_widget()
        lay = QtWidgets.QVBoxLayout(page)
        lib = Card("TCP library")
        lay.addWidget(lib)
        self.tcp_table = QtWidgets.QTableWidget(0, 7)
        self.tcp_table.setHorizontalHeaderLabels(["TCP", "Type", "Parent / link", "Offset [mm]",
                                                  "Method", "Samples / RMS", "Active"])
        self.tcp_table.horizontalHeader().setStretchLastSection(True)
        self.tcp_table.verticalHeader().setVisible(False)
        self.tcp_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.tcp_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.tcp_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.tcp_table.setMinimumHeight(150)
        lib.body.addWidget(self.tcp_table)
        row = QtWidgets.QHBoxLayout()
        for text, fn in (("Set active", self.set_active_tcp), ("New…", self.new_tcp), ("Edit…", self.edit_tcp)):
            b = button(text, "secondary")
            b.clicked.connect(fn)
            row.addWidget(b)
        more = button("More ▾", "quiet")
        mm = QtWidgets.QMenu(more)
        mm.addAction("Duplicate", self.duplicate_tcp)
        mm.addAction("Import…", self.import_calib)
        mm.addAction("Export…", self.export_calib)
        more.setMenu(mm)
        row.addWidget(more)
        row.addStretch(1)
        d = button("Delete", "quiet")
        d.clicked.connect(self.delete_tcp)
        row.addWidget(d)
        lib.body.addLayout(row)
        lib.body.addWidget(muted("Model-link TCPs come from the robot profile and are read-only. " + VIRTUAL_NOTE))
        lib.body.addWidget(self.path_label)

        g = Card()
        fold = Collapsible("Calibrate a new TCP (pivot)")
        fold.content_layout.addWidget(g)
        lay.addWidget(fold)
        gl = QtWidgets.QGridLayout()
        g.body.addLayout(gl)
        gl.addWidget(muted(
            "Keep the tip on one fixed point, tilt the tool about two different axes (freedrive) and capture "
            "6–12 samples. This finds the tip POSITION only; orientation is inherited from the parent unless "
            "set below (manual / CAD, not calibrated)."), 0, 0, 1, 4)
        gl.addWidget(QtWidgets.QLabel("Parent"), 1, 0)
        self.pivot_parent = QtWidgets.QComboBox()
        self.pivot_parent.currentTextChanged.connect(lambda _: self.reset_pivot())
        gl.addWidget(self.pivot_parent, 1, 1, 1, 3)
        self.pivot_list = QtWidgets.QListWidget()
        self.pivot_list.setMinimumHeight(110)
        gl.addWidget(self.pivot_list, 2, 0, 1, 4)
        for i, (text, fn, role) in enumerate((("Capture sample", self.capture_pivot, "secondary"),
                                              ("Remove selected", self.remove_pivot, "secondary"),
                                              ("Reset", self.reset_pivot, "secondary"),
                                              ("Solve", self.solve_pivot, "secondary"))):
            b = button(text, role)
            b.clicked.connect(fn)
            gl.addWidget(b, 3, i)
        self.pivot_quality_lbl = QtWidgets.QLabel("0 samples")
        gl.addWidget(self.pivot_quality_lbl, 4, 0, 1, 4)
        self.pivot_result_lbl = mono_label("")
        gl.addWidget(self.pivot_result_lbl, 5, 0, 1, 4)
        gl.addWidget(QtWidgets.QLabel("Orientation RPY [°]"), 6, 0)
        self.pivot_rpy = []
        for k in range(3):
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-180, 180)
            sp.setDecimals(2)
            self.pivot_rpy.append(sp)
            gl.addWidget(sp, 6, 1 + k)
        gl.addWidget(QtWidgets.QLabel("Save as"), 7, 0)
        self.pivot_name = QtWidgets.QLineEdit("spatula_tip")
        gl.addWidget(self.pivot_name, 7, 1, 1, 2)
        sv = button("Save TCP", "primary")
        sv.clicked.connect(self.save_pivot)
        gl.addWidget(sv, 7, 3)
        lay.addStretch(1)
        return page

    def _selected_tcp(self) -> Optional[str]:
        rows = self.tcp_table.selectionModel().selectedRows()
        if not rows:
            return None
        return self.tcp_table.item(rows[0].row(), 0).text()

    def _refresh_tcp_table(self):
        tcps = list(self.store.tcps.values())
        self.tcp_table.setRowCount(len(tcps))
        for r, t in enumerate(tcps):
            if t.builtin:
                vals = [t.name, "model link", t.link, "(from TF)", "profile", "",
                        "● active" if t.name == self.store.active_tcp else ""]
            else:
                xyz, rpy = tf.T_to_mm_deg(t.T_parent_tcp)
                m = t.meta
                rms = f"{m['sample_count']} / {m['residual_rms_m'] * 1000:.2f} mm" if "residual_rms_m" in m else ""
                vals = [t.name, "virtual", t.parent, "  ".join(f"{v:.2f}" for v in xyz),
                        m.get("method", "manual"), rms, "● active" if t.name == self.store.active_tcp else ""]
            for c, v in enumerate(vals):
                self.tcp_table.setItem(r, c, QtWidgets.QTableWidgetItem(v))
        self.tcp_table.resizeColumnsToContents()

    def _tcp_dialog(self, title: str, tcp: Optional[TcpDef], name_editable: bool) -> Optional[TcpDef]:
        dlg = QtWidgets.QDialog()
        dlg.setWindowTitle(title)
        gl = QtWidgets.QGridLayout(dlg)
        gl.addWidget(QtWidgets.QLabel("Name"), 0, 0)
        name = QtWidgets.QLineEdit(tcp.name if tcp else "")
        name.setEnabled(name_editable)
        gl.addWidget(name, 0, 1, 1, 5)
        gl.addWidget(QtWidgets.QLabel("Parent TCP / link"), 1, 0)
        parent = QtWidgets.QComboBox()
        parent.addItems([n for n in self.store.tcps if not tcp or n != tcp.name])
        if tcp:
            parent.setCurrentText(tcp.parent)
        gl.addWidget(parent, 1, 1, 1, 5)
        spins = xyz_rpy_inputs(gl, 2)
        if tcp and tcp.T_parent_tcp is not None:
            set_T_spins(spins, tcp.T_parent_tcp)
        note = QtWidgets.QLabel("Offset of the TCP in the parent frame (mm), orientation as fixed-axis "
                                "roll/pitch/yaw (degrees).\n" + VIRTUAL_NOTE)
        note.setWordWrap(True)
        gl.addWidget(note, 4, 0, 1, 6)
        bb = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.Ok | QtWidgets.QDialogButtonBox.Cancel)
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        gl.addWidget(bb, 5, 0, 1, 6)
        if dlg.exec_() != QtWidgets.QDialog.Accepted:
            return None
        meta = dict(tcp.meta) if tcp else {}
        meta["method"] = "manual" if not tcp or tcp.T_parent_tcp is None or not np.allclose(
            T_from_spins(spins), tcp.T_parent_tcp, atol=1e-7) else meta.get("method", "manual")
        if meta["method"] == "manual":
            for k in ("sample_count", "residual_rms_m", "residual_max_m"):
                meta.pop(k, None)
        return TcpDef(name.text().strip(), "virtual", parent=parent.currentText(),
                      T_parent_tcp=T_from_spins(spins), meta=meta)

    def new_tcp(self):
        t = self._tcp_dialog("New virtual TCP", None, True)
        if t:
            try:
                self.store.put_tcp(t)
                self._emit()
            except CalibrationError as e:
                self._err(e)

    def edit_tcp(self):
        name = self._selected_tcp()
        if not name:
            return
        t = self.store.tcps[name]
        if t.builtin:
            self._err(CalibrationError(f"'{name}' is a model link from the robot profile (read-only)"))
            return
        new = self._tcp_dialog(f"Edit {name}", t, False)
        if new:
            try:
                self.store.put_tcp(new, replace=True)
                self._emit()
            except CalibrationError as e:
                self._err(e)

    def duplicate_tcp(self):
        name = self._selected_tcp()
        if not name or self.store.tcps[name].builtin:
            return
        t = self.store.tcps[name]
        new, ok = QtWidgets.QInputDialog.getText(None, "Duplicate TCP", "New name", text=f"{name}_copy")
        if ok:
            try:
                meta = {k: v for k, v in t.meta.items() if k not in ("created", "modified")}
                self.store.put_tcp(TcpDef(new.strip(), "virtual", parent=t.parent,
                                          T_parent_tcp=t.T_parent_tcp.copy(), meta=meta))
                self._emit()
            except CalibrationError as e:
                self._err(e)

    def delete_tcp(self):
        name = self._selected_tcp()
        if name and QtWidgets.QMessageBox.question(None, "Delete TCP", f"Delete TCP '{name}'?") \
                == QtWidgets.QMessageBox.Yes:
            try:
                self.store.delete_tcp(name)
                self._emit()
            except CalibrationError as e:
                self._err(e)

    def set_active_tcp(self):
        name = self._selected_tcp()
        if name:
            self.store.set_active_tcp(name)
            self._emit()

    def import_calib(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(None, "Import calibration", str(Path.home()),
                                                        "Calibration (*.yaml *.yml)")
        if path:
            try:
                self.store.import_file(Path(path))
                self._emit()
                self.report((True, f"imported {path}"))
            except Exception as e:      # YAML errors included: show, never apply partially
                self._err(e)

    def export_calib(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            None, "Export calibration", str(Path.home() / f"{self.store.profile.name}_calibration.yaml"),
            "Calibration (*.yaml)")
        if path:
            self.store.export(Path(path))
            self.report((True, f"exported {path}"))

    # ---------------------------------------------------------------- pivot
    def capture_pivot(self):
        parent = self.pivot_parent.currentText()
        try:
            T = self.geo.current_T_base_tcp(parent)
        except GeometryError as e:
            self._err(e)
            return
        self._pivot_samples.append(T)
        xyz, rpy = tf.T_to_mm_deg(T)
        self.pivot_list.addItem(f"#{len(self._pivot_samples)}  {parent} at "
                                f"[{xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}] mm  "
                                f"rpy [{rpy[0]:.1f}, {rpy[1]:.1f}, {rpy[2]:.1f}]°")
        self._update_pivot_quality()

    def remove_pivot(self):
        r = self.pivot_list.currentRow()
        if r >= 0:
            del self._pivot_samples[r]
            self.pivot_list.takeItem(r)
            self._update_pivot_quality()

    def reset_pivot(self):
        self._pivot_samples = []
        self._pivot_result = None
        self.pivot_list.clear()
        self.pivot_result_lbl.setText("")
        self._update_pivot_quality()

    def _update_pivot_quality(self):
        q, reason, _ = pivot_quality(self._pivot_samples)
        color = {"insufficient": T["muted"], "poor": T["danger"], "acceptable": T["warn"], "good": T["ok"]}[q]
        self.pivot_quality_lbl.setText(f"<b style='color:{color}'>{q.upper()}</b> — {reason}")
        self._pivot_result = None

    def solve_pivot(self):
        try:
            res = solve_pivot(self._pivot_samples)
        except CalibrationError as e:
            self._err(e)
            return
        self._pivot_result = res
        p = res.p_parent_tcp * 1000
        pv = res.p_base_pivot * 1000
        worst = int(np.argmax(res.residuals_m)) + 1
        self.pivot_result_lbl.setText(
            f"tip in {self.pivot_parent.currentText()}: [{p[0]:.2f}, {p[1]:.2f}, {p[2]:.2f}] mm\n"
            f"pivot in base: [{pv[0]:.1f}, {pv[1]:.1f}, {pv[2]:.1f}] mm\n"
            f"RMS {res.rms_m * 1000:.3f} mm, max {res.max_m * 1000:.3f} mm (sample #{worst}), "
            f"{res.sample_count} samples, quality {res.quality}")

    def save_pivot(self):
        res = self._pivot_result
        if res is None:
            self._err(CalibrationError("solve first"))
            return
        q = tf.rpy_to_quat(*[math.radians(sp.value()) for sp in self.pivot_rpy])
        meta = {"method": "pivot", "sample_count": res.sample_count,
                "residual_rms_m": round(res.rms_m, 7), "residual_max_m": round(res.max_m, 7),
                "orientation": "inherited from parent" if not any(sp.value() for sp in self.pivot_rpy)
                else "manual RPY (not calibrated)"}
        name = self.pivot_name.text().strip()
        tcp = TcpDef(name, "virtual", parent=self.pivot_parent.currentText(),
                     T_parent_tcp=tf.make(res.p_parent_tcp, q), meta=meta)
        try:
            replace = name in self.store.tcps
            if replace and QtWidgets.QMessageBox.question(
                    None, "Overwrite", f"Replace existing TCP '{name}'?") != QtWidgets.QMessageBox.Yes:
                return
            self.store.put_tcp(tcp, replace=replace)
            self._emit()
            self.report((True, f"saved TCP {name}"))
        except CalibrationError as e:
            self._err(e)

    # ============================================================ fixtures
    def _build_fixtures(self):
        page = page_widget()
        lay = QtWidgets.QVBoxLayout(page)
        lib = Card("Fixture frames")
        lay.addWidget(lib)
        self.fx_table = QtWidgets.QTableWidget(0, 5)
        self.fx_table.setHorizontalHeaderLabels(["Fixture", "Parent", "XYZ [mm]", "RPY [°]", "Method"])
        self.fx_table.horizontalHeader().setStretchLastSection(True)
        self.fx_table.verticalHeader().setVisible(False)
        self.fx_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.fx_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.fx_table.setMinimumHeight(120)
        lib.body.addWidget(self.fx_table)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(muted("Static workcell frames. Poses stored relative to a fixture move with it when it "
                            "is re-taught."), 1)
        b = button("Delete", "quiet")
        b.clicked.connect(self.delete_fixture)
        row.addWidget(b)
        lib.body.addLayout(row)

        wiz = Card()
        fold = Collapsible("New fixture")
        fold.content_layout.addWidget(wiz)
        lay.addWidget(fold)
        top = QtWidgets.QGridLayout()
        top.addWidget(QtWidgets.QLabel("Name"), 0, 0)
        self.fx_name = QtWidgets.QLineEdit("vial_rack")
        top.addWidget(self.fx_name, 0, 1)
        top.addWidget(QtWidgets.QLabel("Parent frame"), 0, 2)
        self.fx_parent = QtWidgets.QComboBox()
        top.addWidget(self.fx_parent, 0, 3)
        top.addWidget(QtWidgets.QLabel("Method"), 1, 0)
        self.fx_method_teach = QtWidgets.QRadioButton("Teach 3 points")
        self.fx_method_manual = QtWidgets.QRadioButton("Manual transform")
        self.fx_method_teach.setChecked(True)
        top.addWidget(self.fx_method_teach, 1, 1)
        top.addWidget(self.fx_method_manual, 1, 2)
        wiz.body.addLayout(top)
        self.fx_stack = QtWidgets.QStackedWidget()
        wiz.body.addWidget(self.fx_stack)
        self.fx_method_teach.toggled.connect(lambda on: self.fx_stack.setCurrentIndex(0 if on else 1))

        teach = QtWidgets.QWidget()
        tl = QtWidgets.QGridLayout(teach)
        tl.setContentsMargins(0, 0, 0, 0)
        self.teach_tcp_lbl = muted("")
        tl.addWidget(self.teach_tcp_lbl, 0, 0, 1, 3)
        self.teach_lbls, self.teach_marks = {}, {}
        for r, (key, title) in enumerate((("origin", "Step 1  Teach origin"), ("x", "Step 2  Teach a point on +X"),
                                          ("y", "Step 3  Teach a point on the +Y side")), 1):
            mark = QtWidgets.QLabel("○")
            self.teach_marks[key] = mark
            tl.addWidget(mark, r, 0)
            tl.addWidget(QtWidgets.QLabel(title), r, 1)
            lbl = mono_label("—")
            self.teach_lbls[key] = lbl
            tl.addWidget(lbl, r, 2)
        cap = button("Capture current TCP", "secondary")
        cap.clicked.connect(self.capture_next_teach)
        tl.addWidget(cap, 4, 1)
        redo = button("Restart", "secondary")
        redo.clicked.connect(self.reset_teach)
        tl.addWidget(redo, 4, 2, QtCore.Qt.AlignLeft)
        self.teach_preview = mono_label("")
        tl.addWidget(self.teach_preview, 5, 0, 1, 3)
        sb = button("Save taught fixture", "primary")
        sb.clicked.connect(self.save_taught_fixture)
        tl.addWidget(sb, 6, 1)
        self.fx_stack.addWidget(teach)

        man = QtWidgets.QWidget()
        ml = QtWidgets.QGridLayout(man)
        ml.setContentsMargins(0, 0, 0, 0)
        self.fx_spins = xyz_rpy_inputs(ml, 0)
        mb = button("Save manual fixture", "secondary")
        mb.clicked.connect(self.save_manual_fixture)
        ml.addWidget(mb, 2, 0, 1, 2)
        self.fx_stack.addWidget(man)
        lay.addStretch(1)
        return page

    def _refresh_fixture_table(self):
        fxs = list(self.store.fixtures.values())
        self.fx_table.setRowCount(len(fxs))
        for r, f in enumerate(fxs):
            xyz, rpy = tf.T_to_mm_deg(f.T_parent_fixture)
            for c, v in enumerate((f.name, f.parent, "  ".join(f"{v:.1f}" for v in xyz),
                                   "  ".join(f"{v:.2f}" for v in rpy), f.meta.get("method", "manual"))):
                self.fx_table.setItem(r, c, QtWidgets.QTableWidgetItem(v))
        self.fx_table.resizeColumnsToContents()

    def save_manual_fixture(self):
        try:
            self.store.put_fixture(FixtureDef(self.fx_name.text().strip(), self.fx_parent.currentText(),
                                              T_from_spins(self.fx_spins), {"method": "manual"}))
            self._emit()
        except CalibrationError as e:
            self._err(e)

    def capture_next_teach(self):
        for key in ("origin", "x", "y"):
            if key not in self._teach_pts:
                self.capture_teach(key)
                return
        self.report((False, "all three points captured; save or restart"))

    def reset_teach(self):
        self._teach_pts = {}
        for k in self.teach_lbls:
            self.teach_lbls[k].setText("—")
            self.teach_marks[k].setText("○")
        self.teach_preview.setText("")

    def capture_teach(self, key: str):
        tcp = self.store.active_tcp
        try:
            p = self.geo.current_T_base_tcp(tcp)[:3, 3]
        except GeometryError as e:
            self._err(e)
            return
        self._teach_pts[key] = (p, tcp)
        self.teach_lbls[key].setText(f"[{p[0] * 1000:.1f}, {p[1] * 1000:.1f}, {p[2] * 1000:.1f}] mm in base")
        self.teach_marks[key].setText(f"<b style='color:{T['ok']}'>✓</b>")
        if set(self._teach_pts) == {"origin", "x", "y"}:
            try:
                T = frame_from_three_points(*(self._teach_pts[k][0] for k in ("origin", "x", "y")))
                xyz, rpy = tf.T_to_mm_deg(T)
                self.teach_preview.setText(f"Result in base: [{xyz[0]:.1f}, {xyz[1]:.1f}, {xyz[2]:.1f}] mm, "
                                           f"rpy [{rpy[0]:.2f}, {rpy[1]:.2f}, {rpy[2]:.2f}]°")
            except CalibrationError as e:
                self.teach_preview.setText(f"✗ {e}")

    def save_taught_fixture(self):
        if set(self._teach_pts) != {"origin", "x", "y"}:
            self._err(CalibrationError("capture all three points first"))
            return
        tcps = {v[1] for v in self._teach_pts.values()}
        if len(tcps) != 1:
            self._err(CalibrationError("all three points must be taught with the same TCP"))
            return
        try:
            T_base_fx = frame_from_three_points(*(self._teach_pts[k][0] for k in ("origin", "x", "y")))
            parent = self.fx_parent.currentText()
            T_parent_fx = tf.inv(self.geo.T_base_frame(parent)) @ T_base_fx
            meta = {"method": "three_point", "tcp": tcps.pop(), "taught": now_iso(),
                    "points_base_m": {k: [round(float(x), 6) for x in v[0]] for k, v in self._teach_pts.items()}}
            self.store.put_fixture(FixtureDef(self.fx_name.text().strip(), parent, T_parent_fx, meta),
                                   replace=self.fx_name.text().strip() in self.store.fixtures)
            self.reset_teach()
            self._emit()
        except (CalibrationError, GeometryError) as e:
            self._err(e)

    def delete_fixture(self):
        rows = self.fx_table.selectionModel().selectedRows()
        if rows:
            name = self.fx_table.item(rows[0].row(), 0).text()
            try:
                self.store.delete_fixture(name)
                self._emit()
            except CalibrationError as e:
                self._err(e)

    # ============================================================== vision
    def _build_vision(self):
        page = page_widget()
        lay = QtWidgets.QVBoxLayout(page)
        lib = Card("Vision baselines")
        lay.addWidget(lib)
        self.vb_table = QtWidgets.QTableWidget(0, 6)
        self.vb_table.setHorizontalHeaderLabels(["Object", "Source", "Reference", "Side / components",
                                                 "Correction", "Samples / spread"])
        self.vb_table.horizontalHeader().setStretchLastSection(True)
        self.vb_table.verticalHeader().setVisible(False)
        self.vb_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.vb_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.vb_table.setMinimumHeight(110)
        lib.body.addWidget(self.vb_table)
        row = QtWidgets.QHBoxLayout()
        row.addWidget(muted("Corrects systematic bias of a detector (not camera intrinsics). Programs can use the "
                            "object name as a frame; a missing or stale detection fails the step."), 1)
        b = button("Delete", "danger")
        b.clicked.connect(self.delete_vision)
        row.addWidget(b)
        lib.body.addLayout(row)

        g = Card("New baseline")
        lay.addWidget(g)
        gl = QtWidgets.QGridLayout()
        g.body.addLayout(gl)
        r = 0
        gl.addWidget(QtWidgets.QLabel("Object name"), r, 0)
        self.vb_name = QtWidgets.QLineEdit("vial_7")
        gl.addWidget(self.vb_name, r, 1, 1, 2)
        gl.addWidget(QtWidgets.QLabel("Reference"), r, 3)
        self.vb_reference = QtWidgets.QComboBox()
        gl.addWidget(self.vb_reference, r, 4, 1, 2)
        r += 1
        gl.addWidget(QtWidgets.QLabel("Pose source"), r, 0)
        self.vb_type = QtWidgets.QComboBox()
        self.vb_type.addItems(["tf", "pose_topic"])
        gl.addWidget(self.vb_type, r, 1)
        self.vb_source = QtWidgets.QLineEdit("fake_detector/object" if self.fake else "")
        self.vb_source.setPlaceholderText("TF frame or PoseStamped topic")
        gl.addWidget(self.vb_source, r, 2, 1, 2)
        gl.addWidget(QtWidgets.QLabel("Side"), r, 4)
        self.vb_side = QtWidgets.QComboBox()
        self.vb_side.addItems(list(CORRECTION_SIDES))
        self.vb_side.setToolTip("reference: camera / extrinsic error;  object: offset in the detector's object model")
        gl.addWidget(self.vb_side, r, 5)
        r += 1
        gl.addWidget(QtWidgets.QLabel("<b>1. Ground truth</b>"), r, 0, 1, 6)
        r += 1
        gl.addWidget(QtWidgets.QLabel("Source"), r, 0)
        self.vb_gt_source = QtWidgets.QComboBox()
        self.vb_gt_source.addItems(["Probe touch (active TCP) — position only",
                                    "Known fixture / frame — position + orientation",
                                    "Manual entry — position + orientation"])
        self.vb_gt_source.currentIndexChanged.connect(self._vision_gt_changed)
        gl.addWidget(self.vb_gt_source, r, 1, 1, 3)
        self.vb_gt_frame = QtWidgets.QComboBox()
        gl.addWidget(self.vb_gt_frame, r, 4, 1, 2)
        r += 1
        self.vb_spins = xyz_rpy_inputs(gl, r)
        r += 2
        self.vb_gt_btn = button("Capture ground truth", "primary")
        self.vb_gt_btn.clicked.connect(self.capture_vision_truth)
        gl.addWidget(self.vb_gt_btn, r, 0, 1, 2)
        self.vb_truth_lbl = muted("manual", wrap=False)
        gl.addWidget(self.vb_truth_lbl, r, 2, 1, 4)
        r += 1
        gl.addWidget(QtWidgets.QLabel("Correct"), r, 0)
        self.vb_comp_pos = QtWidgets.QCheckBox("Position")
        self.vb_comp_pos.setChecked(True)
        self.vb_comp_pos.setEnabled(False)
        self.vb_comp_rot = QtWidgets.QCheckBox("Orientation")
        gl.addWidget(self.vb_comp_pos, r, 1)
        gl.addWidget(self.vb_comp_rot, r, 2)
        self.vb_comp_note = muted("", wrap=True)
        gl.addWidget(self.vb_comp_note, r, 3, 1, 3)
        r += 1
        gl.addWidget(QtWidgets.QLabel("<b>2. Detector samples</b>"), r, 0, 1, 2)
        self.vb_count = QtWidgets.QSpinBox()
        self.vb_count.setRange(3, 200)
        self.vb_count.setValue(20)
        gl.addWidget(self.vb_count, r, 2)
        b = button("Capture samples", "primary")
        b.clicked.connect(self.capture_vision_samples)
        gl.addWidget(b, r, 3, 1, 2)
        r += 1
        self.vb_result = mono_label("")
        gl.addWidget(self.vb_result, r, 0, 1, 6)
        r += 1
        b = button("3. Solve and save baseline", "primary")
        b.clicked.connect(self.save_vision)
        gl.addWidget(b, r, 0, 1, 3)
        lay.addStretch(1)
        self._vision_gt_changed(0)
        return page

    def _vision_gt_changed(self, i: int):
        """A probe touch gives a position; orientation needs a deliberate source."""
        probe, fixture = i == 0, i == 1
        self.vb_gt_frame.setVisible(fixture)
        self.vb_gt_btn.setText({0: "Capture probe position", 1: "Use frame pose", 2: "Use entered pose"}[i])
        self.vb_comp_rot.setEnabled(not probe)
        if probe:
            self.vb_comp_rot.setChecked(False)
            self.vb_comp_note.setText("A probe touch does not define the object's orientation.")
        else:
            self.vb_comp_note.setText("Orientation correction uses this source's orientation.")
        for sp in self.vb_spins[3:]:
            sp.setEnabled(i == 2)

    def _refresh_vision_table(self):
        vbs = list(self.store.vision.values())
        self.vb_table.setRowCount(len(vbs))
        for r, v in enumerate(vbs):
            xyz, rpy = tf.T_to_mm_deg(v.T_correction)
            st = v.meta.get("stats", {})
            spread = (f"{st.get('sample_count', '?')} / {st.get('position_rms_m', 0) * 1000:.2f} mm, "
                      f"{st.get('angle_rms_deg', 0):.2f}°") if st else ""
            comps = "+".join(st.get("components", ["position", "orientation"])) if st else "?"
            for c, val in enumerate((v.name, f"{v.source_type}: {v.source}", v.reference_frame,
                                     f"{v.correction_side} / {comps}",
                                     f"{np.linalg.norm(xyz):.2f} mm / {math.degrees(tf.rotation_angle(v.T_correction[:3, :3])):.2f}°",
                                     spread)):
                self.vb_table.setItem(r, c, QtWidgets.QTableWidgetItem(val))
        self.vb_table.resizeColumnsToContents()

    def capture_vision_truth(self):
        ref = self.vb_reference.currentText()
        src = self.vb_gt_source.currentIndex()
        try:
            T_base_ref = self.geo.T_base_frame(ref)
            if src == 0:                              # probe: position of the active TCP only
                tcp = self.store.active_tcp
                p = (tf.inv(T_base_ref) @ self.geo.current_T_base_tcp(tcp))[:3, 3]
                for sp, v in zip(self.vb_spins[:3], p * 1000):
                    sp.setValue(v)
                label = f"position from probe TCP {tcp} at {time.strftime('%H:%M:%S')}"
            elif src == 1:                            # known frame: full pose
                frame = self.vb_gt_frame.currentText()
                set_T_spins(self.vb_spins, tf.inv(T_base_ref) @ self.geo.T_base_frame(frame))
                label = f"pose of frame {frame}"
            else:
                label = "manual entry"
        except GeometryError as e:
            self._err(e)
            return
        self.vb_truth_lbl.setText(label)

    def capture_vision_samples(self):
        src_type, src, ref, n = (self.vb_type.currentText(), self.vb_source.text().strip(),
                                 self.vb_reference.currentText(), self.vb_count.value())
        self.vb_result.setText(f"capturing {n} samples…")

        def work():
            out, misses = [], 0
            deadline = time.monotonic() + max(5.0, n * 0.3)
            while len(out) < n and time.monotonic() < deadline:
                T = self.bridge.read_pose_source(src_type, src, ref)
                if T is None or not tf.is_valid(T, tol=1e-4):
                    misses += 1
                else:
                    out.append(T)
                time.sleep(0.1)
            return out, misses

        def done(res):
            self._vision_samples, misses = res
            _, stats = tf.average_transforms(self._vision_samples) if self._vision_samples else (None, {})
            self.vb_result.setText(
                f"{len(self._vision_samples)} samples ({misses} missing/invalid)\n"
                + (f"spread: pos RMS {stats['position_rms_m'] * 1000:.2f} mm, "
                   f"angle RMS {stats['angle_rms_deg']:.2f}°" if stats else "no valid detections"))
        run_in_background(self.dispatch, work, done, self._err)

    def save_vision(self):
        try:
            T_R_expected = T_from_spins(self.vb_spins)
            side = self.vb_side.currentText()
            orient = self.vb_comp_rot.isChecked() and self.vb_gt_source.currentIndex() != 0
            T_corr, stats = solve_vision_baseline(T_R_expected, self._vision_samples, side, orientation=orient)
            vb = VisionBaseline(self.vb_name.text().strip(), self.vb_type.currentText(),
                                self.vb_source.text().strip(), self.vb_reference.currentText(), T_corr,
                                {"method": "baseline", "ground_truth": self.vb_truth_lbl.text(),
                                 "stats": {k: (round(v, 7) if isinstance(v, float) else v)
                                           for k, v in stats.items()}}, side)
            replace = vb.name in self.store.vision
            if replace and QtWidgets.QMessageBox.question(
                    None, "Overwrite", f"Replace baseline '{vb.name}'?") != QtWidgets.QMessageBox.Yes:
                return
            self.store.put_vision(vb, replace=replace)
            self.vb_result.setText(self.vb_result.text() + f"\nsaved: correction "
                                   f"{stats['correction_translation_m'] * 1000:.2f} mm / "
                                   f"{stats['correction_angle_deg']:.2f}°")
            self._emit()
        except CalibrationError as e:
            self._err(e)

    def delete_vision(self):
        rows = self.vb_table.selectionModel().selectedRows()
        if rows:
            self.store.delete_vision(self.vb_table.item(rows[0].row(), 0).text())
            self._emit()
