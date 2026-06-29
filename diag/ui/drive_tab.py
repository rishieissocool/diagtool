"""
drive_tab.py — multi-robot movement, including grSim.

Drive a whole scope of robots at once (all / a team / just the grSim ones / just
the real ones) with a hold-to-drive direction pad and speed sliders. Each robot
is driven through its own target protocol — grSim robots get a grSim packet, real
robots get a RobotCommand — so the same controls work in the simulator and on the
field. The arena brake / emergency stop still apply to any robot vision can see.

Hold a direction button to move; release (or hit STOP ALL) to halt. A polling
watchdog stops everything if a button release is ever missed.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton,
    QComboBox, QSlider, QGroupBox,
)

from .widgets import COL_TEXT, COL_TEXT_DIM, COL_GOOD

# scope label -> predicate on a RobotInfo
_SCOPES = [
    ("All robots", lambda r: True),
    ("Yellow team", lambda r: r.is_yellow),
    ("Blue team", lambda r: not r.is_yellow),
    ("Sim (grSim)", lambda r: r.mode == "grsim"),
    ("Real robots", lambda r: r.mode != "grsim"),
]

# (label, vx, vy, w, row, col) in body frame: +x forward, +y left, +w CCW
_DIRS = [
    ("↑\nForward", 1, 0, 0, 0, 1),
    ("←\nLeft", 0, 1, 0, 1, 0),
    ("↺ Turn", 0, 0, 1, 1, 1),
    ("Turn ↻", 0, 0, -1, 1, 2),
    ("→\nRight", 0, -1, 0, 1, 3),
    ("↓\nBack", -1, 0, 0, 2, 1),
]


class DriveTab(QWidget):
    def __init__(self, engine, parent=None):
        super().__init__(parent)
        self.engine = engine
        self._active = None        # (fx, fy, fw, button) while a button is held
        self._scope_pred = _SCOPES[0][1]

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        head = QHBoxLayout()
        title = QLabel("Drive")
        title.setStyleSheet(f"color:{COL_TEXT.name()}; font-size:20px; font-weight:bold;")
        hint = QLabel("hold a direction to move · release or STOP ALL to halt · "
                      "grSim robots drive in the simulator")
        hint.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
        head.addWidget(title)
        head.addSpacing(10)
        head.addWidget(hint)
        head.addStretch()
        root.addLayout(head)

        # scope + speeds
        opts = QHBoxLayout()
        opts.setSpacing(14)
        opts.addWidget(QLabel("Drive:"))
        self.scope = QComboBox()
        for name, _ in _SCOPES:
            self.scope.addItem(name)
        self.scope.currentIndexChanged.connect(self._on_scope)
        opts.addWidget(self.scope)

        lim = self.engine.lim
        self.sld_lin, self.lbl_lin = self._slider(
            opts, "Speed", 5, int(lim.max_speed * 100), 30, "m/s")
        self.sld_ang, self.lbl_ang = self._slider(
            opts, "Turn", 10, max(20, int(lim.max_w * 100)), 80, "rad/s")
        opts.addStretch()
        root.addLayout(opts)

        body = QHBoxLayout()
        body.setSpacing(16)

        # direction pad
        pad_box = QGroupBox("Direction (body frame)")
        pad = QGridLayout(pad_box)
        pad.setSpacing(8)
        for label, fx, fy, fw, r, c in _DIRS:
            b = QPushButton(label)
            b.setMinimumSize(96, 64)
            b.setStyleSheet("font-size:14px; font-weight:bold;")
            # hold-to-drive: press starts motion, release stops
            b.pressed.connect(lambda fx=fx, fy=fy, fw=fw, bt=b: self._press(fx, fy, fw, bt))
            b.released.connect(self._release)
            pad.addWidget(b, r, c)
        stop_pad = QPushButton("■\nSTOP")
        stop_pad.setMinimumSize(96, 64)
        stop_pad.setStyleSheet(
            "background:#e05050; color:white; font-weight:bold; font-size:14px;")
        stop_pad.clicked.connect(self.stop_all)
        pad.addWidget(stop_pad, 0, 0)
        body.addWidget(pad_box)

        # status / big stop
        side = QVBoxLayout()
        self.status = QLabel("idle")
        self.status.setWordWrap(True)
        self.status.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:12px;")
        self.status.setAlignment(Qt.AlignTop)
        side.addWidget(self.status, 1)
        big_stop = QPushButton("STOP ALL")
        big_stop.setMinimumHeight(56)
        big_stop.setStyleSheet(
            "background:#e05050; color:white; font-weight:bold; font-size:18px;"
            "border-radius:8px;")
        big_stop.clicked.connect(self.stop_all)
        side.addWidget(big_stop)
        body.addLayout(side, 1)

        root.addLayout(body, 1)
        root.addStretch()

        # watchdog: poll whether the held button is still down (catches a missed
        # release), plus a hard cap so nothing drives forever unattended.
        self._poll = QTimer(self)
        self._poll.setInterval(60)
        self._poll.timeout.connect(self._watch)
        self._cap = QTimer(self)
        self._cap.setSingleShot(True)
        self._cap.timeout.connect(self.stop_all)

        self._refresh_status()

    # -- helpers --
    def _slider(self, parent_layout, name, lo, hi, val, unit):
        parent_layout.addWidget(QLabel(name + ":"))
        s = QSlider(Qt.Horizontal)
        s.setMinimum(lo)
        s.setMaximum(hi)
        s.setValue(min(max(val, lo), hi))
        s.setFixedWidth(130)
        lbl = QLabel()
        lbl.setStyleSheet(f"color:{COL_TEXT.name()};")
        lbl.setFixedWidth(74)

        def _upd(v):
            lbl.setText(f"{v/100:.2f} {unit}")
            if self._active is not None:           # live-adjust while driving
                self._apply()
        s.valueChanged.connect(_upd)
        _upd(s.value())
        parent_layout.addWidget(s)
        parent_layout.addWidget(lbl)
        return s, lbl

    def _on_scope(self, idx):
        self._scope_pred = _SCOPES[idx][1]
        self._refresh_status()

    def _scope_robots(self):
        return [r for r in self.engine.robots() if self._scope_pred(r)]

    def _refresh_status(self):
        rs = self._scope_robots()
        labels = ", ".join(r.label for r in rs) or "(none)"
        sim = sum(1 for r in rs if r.mode == "grsim")
        self.status.setText(
            f"scope: {self.scope.currentText()} — {len(rs)} robot(s)"
            f"  [{sim} grSim, {len(rs) - sim} real]\n{labels}")

    # -- hold-to-drive --
    def _press(self, fx, fy, fw, btn):
        if self.engine.commander is None:
            return
        self._active = (fx, fy, fw, btn)
        self._apply()
        self._poll.start()
        self._cap.start(20000)        # 20 s hard cap, re-armed each press

    def _release(self):
        self.stop_all()

    def _apply(self):
        if self._active is None:
            return
        fx, fy, fw, _btn = self._active
        lin = self.sld_lin.value() / 100.0
        ang = self.sld_ang.value() / 100.0
        rs = self._scope_robots()
        for r in rs:
            self.engine.commander.set_velocity(
                r.is_yellow, r.robot_id,
                vx=fx * lin, vy=fy * lin, w=fw * ang,
                frame="body", safe=False)
        dir_name = {(1, 0, 0): "forward", (-1, 0, 0): "back", (0, 1, 0): "left",
                    (0, -1, 0): "right", (0, 0, 1): "turn ↺",
                    (0, 0, -1): "turn ↻"}.get((fx, fy, fw), "move")
        self.status.setText(
            f"▶ driving {len(rs)} robot(s) {dir_name} — "
            f"{lin:.2f} m/s / {ang:.2f} rad/s\n"
            f"{', '.join(r.label for r in rs) or '(none)'}")

    def _watch(self):
        # if the held button is no longer pressed, a release was missed — stop.
        if self._active is not None and not self._active[3].isDown():
            self.stop_all()

    def stop_all(self):
        self._poll.stop()
        self._cap.stop()
        self._active = None
        if self.engine.commander is not None:
            self.engine.commander.stop_all()
        self._refresh_status()

    # stop driving if the user navigates away or the tab hides
    def hideEvent(self, ev):
        self.stop_all()
        super().hideEvent(ev)
