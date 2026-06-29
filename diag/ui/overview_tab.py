"""
overview_tab.py — the Competition dashboard.

A responsive wall of live robot cards (battery, vision/telemetry link, ball,
pose, state) plus a summary header (vision fps, telemetry rate, field size,
robots online, low batteries). Designed to be glanced at during a match: a
robot that goes dark, drops its link, or runs low on battery stands out by
colour without anyone having to read numbers.

Fed a snapshot dict from main_window every refresh tick; holds no engine state.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QScrollArea,
    QFrame, QSizePolicy,
)

from .widgets import (
    RobotCard, battery_status, COL_TEXT, COL_TEXT_DIM, COL_GOOD, COL_WARN,
    COL_CRIT, COL_IDLE,
)


_CARD_MIN_W = 264   # keep in step with RobotCard.minimumWidth + spacing


def _metric(title: str):
    box = QFrame()
    box.setObjectName("metric")
    box.setStyleSheet(
        "#metric { background:#1b2330; border:1px solid #2b3647; border-radius:8px; }")
    lay = QVBoxLayout(box)
    lay.setContentsMargins(12, 6, 12, 6)
    lay.setSpacing(0)
    val = QLabel("—")
    val.setStyleSheet(f"color:{COL_TEXT.name()}; font-size:18px; font-weight:bold;")
    cap = QLabel(title)
    cap.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
    lay.addWidget(val)
    lay.addWidget(cap)
    return box, val


class OverviewTab(QWidget):
    robot_selected = Signal(str)

    def __init__(self, robots, settings: dict, parent=None):
        super().__init__(parent)
        self._settings = settings
        self._cols = 0

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # --- summary header ---
        head = QHBoxLayout()
        head.setSpacing(10)
        title = QLabel("Competition")
        title.setStyleSheet(f"color:{COL_TEXT.name()}; font-size:20px; font-weight:bold;")
        head.addWidget(title)
        head.addSpacing(8)
        self._m_online, self._v_online = _metric("robots online")
        self._m_batt, self._v_batt = _metric("low battery")
        self._m_vis, self._v_vis = _metric("vision fps")
        self._m_tel, self._v_tel = _metric("telemetry")
        self._m_field, self._v_field = _metric("field (mm)")
        for box in (self._m_online, self._m_batt, self._m_vis, self._m_tel,
                    self._m_field):
            head.addWidget(box)
        head.addStretch()
        root.addLayout(head)

        # --- card grid in a scroll area ---
        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.NoFrame)
        self._holder = QWidget()
        self._grid = QGridLayout(self._holder)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setHorizontalSpacing(10)
        self._grid.setVerticalSpacing(10)
        self._grid.setAlignment(Qt.AlignTop)
        self._scroll.setWidget(self._holder)
        root.addWidget(self._scroll, 1)

        self._cards: dict[str, RobotCard] = {}
        self._order: list[str] = []
        for r in robots:
            card = RobotCard(r.label, r.is_yellow)
            card.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
            card.clicked.connect(self.robot_selected)
            self._cards[r.label] = card
            self._order.append(r.label)

        if not self._cards:
            empty = QLabel("No robots in ipconfig.yaml.")
            empty.setStyleSheet(f"color:{COL_TEXT_DIM.name()};")
            self._grid.addWidget(empty, 0, 0)
        self._relayout(force=True)

    # responsive reflow: pick a column count from the available width
    def resizeEvent(self, ev):
        self._relayout()
        super().resizeEvent(ev)

    def showEvent(self, ev):
        # the tab may have been laid out while hidden (stale viewport width);
        # reflow now that it has its real size.
        super().showEvent(ev)
        self._relayout(force=True)

    def _relayout(self, force: bool = False):
        if not self._cards:
            return
        avail = max(self._scroll.viewport().width(), _CARD_MIN_W)
        cols = max(1, avail // _CARD_MIN_W)
        cols = min(cols, len(self._cards))
        if cols == self._cols and not force:
            return
        self._cols = cols
        while self._grid.count():
            self._grid.takeAt(0)
        for i, label in enumerate(self._order):
            self._grid.addWidget(self._cards[label], i // cols, i % cols)
        for c in range(cols):
            self._grid.setColumnStretch(c, 1)

    def update_snapshot(self, snapshot: dict):
        self._relayout()   # converge column count once the tab has its real width
        robots = snapshot.get("robots", [])
        online = low = 0
        for snap in robots:
            card = self._cards.get(snap["label"])
            if card is not None:
                card.update_snapshot(snap, self._settings)
            if snap.get("vision_seen") or snap.get("tel_seen"):
                online += 1
            bat = battery_status(snap.get("voltage"), self._settings)
            if bat["level"] in ("warn", "crit"):
                low += 1

        self._v_online.setText(f"{online}/{len(robots)}")
        self._v_batt.setText(str(low))
        self._v_batt.setStyleSheet(
            f"color:{(COL_CRIT if low else COL_TEXT).name()}; "
            "font-size:18px; font-weight:bold;")

        v = snapshot.get("vision", {})
        if v.get("error"):
            self._v_vis.setText("ERR")
            self._v_vis.setStyleSheet(
                f"color:{COL_CRIT.name()}; font-size:18px; font-weight:bold;")
        else:
            fps = v.get("fps") or 0.0
            self._v_vis.setText(f"{fps:.0f}")
            self._v_vis.setStyleSheet(
                f"color:{(COL_GOOD if fps > 5 else COL_WARN).name()}; "
                "font-size:18px; font-weight:bold;")

        t = snapshot.get("telemetry", {})
        if t.get("error"):
            self._v_tel.setText("ERR")
        else:
            self._v_tel.setText(f"{(t.get('rate_hz') or 0.0):.1f} Hz")

        fi = snapshot.get("field", {})
        if fi:
            self._v_field.setText(f"{fi.get('length_mm', 0):.0f}×{fi.get('width_mm', 0):.0f}")
