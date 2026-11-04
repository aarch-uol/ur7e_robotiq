"""DIAGNOSTICS: the engineering view of the control stack, gripper, freshness and the full log."""
from __future__ import annotations

from PyQt5 import QtCore, QtWidgets

from ..control_modes import claim_problems
from .theme import SPACE, T, mono_font
from .widgets import Card, label, mono_label, page_widget


class DiagnosticsPage(QtWidgets.QWidget):
    def __init__(self, win):
        super().__init__()
        self.win = win
        page = page_widget()
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.addWidget(page)
        lay = QtWidgets.QVBoxLayout(page)
        lay.setContentsMargins(SPACE["s"], SPACE["s"], SPACE["s"], SPACE["s"])
        lay.setSpacing(SPACE["s"])
        self.unknown = label("", "banner_bad", wrap=True)
        lay.addWidget(self.unknown)
        top = QtWidgets.QHBoxLayout()
        top.setSpacing(SPACE["s"])
        lay.addLayout(top, 3)

        arm = Card("Arm control · " + str(win.profile.services.get("controller_manager")))
        self.arm_lbl = QtWidgets.QLabel()
        self.arm_lbl.setTextFormat(QtCore.Qt.RichText)
        self.arm_lbl.setWordWrap(True)
        self.arm_lbl.setAlignment(QtCore.Qt.AlignTop)
        sa = QtWidgets.QScrollArea()
        sa.setWidgetResizable(True)
        sa.setFrameShape(QtWidgets.QFrame.NoFrame)
        sa.setWidget(self.arm_lbl)
        arm.body.addWidget(sa, 1)
        top.addWidget(arm, 3)

        mid = QtWidgets.QVBoxLayout()
        g = win.profile.gripper
        grip = Card("Gripper · " + str(g.get("controller_manager", "no separate controller manager")))
        self.grip_lbl = QtWidgets.QLabel()
        self.grip_lbl.setWordWrap(True)
        grip.body.addWidget(self.grip_lbl)
        mid.addWidget(grip)
        fresh = Card("State")
        self.fresh = {}
        form = QtWidgets.QFormLayout()
        for k in ("Joint state", "Joint velocities", "Stationary", "F/T source", "Command source",
                  "Policy runner", "Active TCP", "Planning link", "Last refusal"):
            lab = mono_label()
            lab.setWordWrap(True)
            self.fresh[k] = lab
            form.addRow(k, lab)
        fresh.body.addLayout(form)
        mid.addWidget(fresh)
        mid.addStretch(1)
        mid_host = QtWidgets.QWidget()
        mid_host.setLayout(mid)
        mid.setContentsMargins(0, 0, 0, 0)
        mid_scroll = QtWidgets.QScrollArea()
        mid_scroll.setWidgetResizable(True)
        mid_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        mid_scroll.setWidget(mid_host)
        top.addWidget(mid_scroll, 2)

        logc = Card("Log")
        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(5000)
        self.log_view.setFont(mono_font(12))
        logc.body.addWidget(self.log_view)
        lay.addWidget(logc, 2)

    def refresh(self, s):
        w, p = self.win, self.win.profile
        m = p.modes.get(w.mode)
        claims = s.claimed_interfaces
        self.unknown.setVisible(w.modes.control_state_unknown)
        self.unknown.setText("CONTROL STATE UNKNOWN. " + w.modes.unknown_reason)
        html = [f"<b>Mode</b> {m.label if m else w.mode}"
                + (f" &nbsp; <b>Hardware command</b> {m.command_interface.upper()}"
                   + (f" — <span style='color:{T['warn']}'>DIRECT TORQUE</span>" if m.direct_torque else "") if m else "")]
        if m:
            chain = m.chain or m.activate
            html.append("<pre style='margin:4px 0'>" + ("MoveIt FollowJointTrajectory\n       ↓\n"
                                                         if m.supports_moveit else
                                                         "Cartesian reference\n       ↓\n" if m.direct_torque else ""))
            for ctrl in chain:
                html[-1] += f"{ctrl}  [{s.controllers.get(ctrl, 'not loaded')}]\n       ↓\n"
            last = chain[-1]
            hw = [c for c in claims.get(last, [])]
            html[-1] += ("\n".join(hw) if hw else f"(no claims reported for {last})") + "</pre>"
            bad = claim_problems(p, w.mode, claims) if claims else ["claims not reported"]
            html.append(f"<span style='color:{T['ok']}'>✓ joint interfaces claimed as expected</span>" if not bad else
                        f"<span style='color:{T['danger']}'>✗ " + "<br>✗ ".join(bad) + "</span>")
            if m.direct_torque:
                html.append(f"<span style='color:{T['warn']}'>⚠ Position/velocity motion controllers must remain "
                            "inactive.</span>")
        html.append("<br><b>All arm controllers</b>")
        rows = []
        for name, st in sorted(s.controllers.items()):
            col = T["ok"] if st == "active" else T["muted"]
            cl = ", ".join(claims.get(name, [])[:2]) + (" …" if len(claims.get(name, [])) > 2 else "")
            rows.append(f"<tr><td>{name}</td><td style='color:{col}'>&nbsp;{st}&nbsp;</td>"
                        f"<td style='color:{T['muted']}'>{cl}</td></tr>")
        html.append("<table>" + "".join(rows) + "</table>")
        self.arm_lbl.setText("<br>".join(html))

        g = p.gripper
        if g.get("enabled"):
            ctrls = "<br>".join(f"{k}: <b>{v}</b>" for k, v in sorted(s.gripper_controllers.items())) or "not reported"
            self.grip_lbl.setText(f"{ctrls}<br>Action {g.get('action')}<br>Interface {g.get('action_type')}<br>"
                                  f"Joint {g.get('joint_name')}<br>Action server: "
                                  f"{'up' if s.connected.get('Gripper') else 'DOWN'}<br>"
                                  f"<span style='color:{T['muted']}'>Not part of the arm's mode switching.</span>")
        else:
            self.grip_lbl.setText("No gripper in this profile.")

        age = s.joint_state_age
        lim = p.joint_state_timeout_s
        self.fresh["Joint state"].setText("never received" if age is None else
                                          f"{age * 1000:.0f} ms ({'fresh' if age <= lim else 'STALE'})")
        self.fresh["Joint velocities"].setText("available" if w.bridge.joint_velocities_available()
                                               else "MISSING → switches refused")
        self.fresh["Stationary"].setText({True: "yes", False: "moving", None: "cannot verify"}[w.bridge.is_stationary()])
        self.fresh["F/T source"].setText(f"{p.topics.get('wrench', '—')} ({'live' if s.connected.get('F/T') else 'silent'})")
        self.fresh["Command source"].setText(w.command_source)
        pr = w.policies
        self.fresh["Policy runner"].setText(f"{type(pr).__name__} · {'busy' if pr.busy else 'idle'}"
                                            + ("" if pr.supports_feedback else " · no feedback"))
        self.fresh["Active TCP"].setText(w.store.active_tcp)
        self.fresh["Planning link"].setText(p.planning_link)
        self.fresh["Last refusal"].setText(w.last_refusal or "—")
