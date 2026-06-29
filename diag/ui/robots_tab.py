"""
robots_tab.py — the Robots gallery.

One large card per robot with its picture (a real photo dropped into
assets/robots/<LABEL>.png, or a drawn team-coloured placeholder), the robot's
name, an online/offline chip, a battery gauge and a one-line link summary. It is
the "who's who" view: walk up to the table, see every robot and its state.

Fed the same snapshot dict as the Competition tab; holds no engine state.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtGui import QColor
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QScrollArea,
    QFrame, QSizePolicy,
)

from .widgets import (
    BatteryBar, StatusPill, robot_pixmap, battery_status,
    COL_TEXT, COL_TEXT_DIM, COL_GOOD, COL_IDLE, COL_YELLOW, COL_BLUE,
)


_PHOTO_PX = 184
_CARD_MIN_W = 224


class GalleryCard(QFrame):
    clicked = Signal(str)

    def __init__(self, label: str, is_yellow: bool, photo_dir, parent=None):
        super().__init__(parent)
        self.label_id = label
        self.setObjectName("galleryCard")
        self.setCursor(Qt.PointingHandCursor)
        self.setStyleSheet(
            "#galleryCard { background:#1b2330; border:1px solid #2b3647; "
            "border-radius:12px; }")

        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.setSpacing(8)

        self.photo = QLabel()
        self.photo.setAlignment(Qt.AlignCenter)
        self.photo.setPixmap(robot_pixmap(label, is_yellow, _PHOTO_PX, photo_dir))
        self.photo.setFixedHeight(_PHOTO_PX)
        lay.addWidget(self.photo, 0, Qt.AlignCenter)

        name_row = QHBoxLayout()
        name_row.setSpacing(6)
        dot = QLabel("●")
        dot.setStyleSheet(
            f"color:{(COL_YELLOW if is_yellow else COL_BLUE).name()}; font-size:16px;")
        name = QLabel(label)
        name.setStyleSheet(f"color:{COL_TEXT.name()}; font-size:18px; font-weight:bold;")
        team = QLabel("Yellow" if is_yellow else "Blue")
        team.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:12px;")
        self.pill = StatusPill()
        name_row.addWidget(dot)
        name_row.addWidget(name)
        name_row.addWidget(team)
        name_row.addStretch()
        name_row.addWidget(self.pill)
        lay.addLayout(name_row)

        self.battery = BatteryBar()
        lay.addWidget(self.battery)

        self.sub = QLabel("—")
        self.sub.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
        lay.addWidget(self.sub)

    def mousePressEvent(self, ev):
        self.clicked.emit(self.label_id)
        super().mousePressEvent(ev)

    def update_snapshot(self, snap: dict, settings: dict):
        self.battery.set_status(battery_status(snap.get("voltage"), settings))
        vis, tel = snap.get("vision_seen"), snap.get("tel_seen")
        online = vis or tel
        self.pill.set_state(
            "ONLINE" if online else ("OFFLINE" if snap.get("is_real") else "SIM"),
            COL_GOOD if online else COL_IDLE,
            QColor("#0b0f14") if online else COL_TEXT)
        bits = [snap.get("ip", "")]
        bits.append("vision ●" if vis else "vision ○")
        rate = snap.get("rate_hz") or 0.0
        bits.append(f"telem {rate:.0f}Hz" if tel else "telem ○")
        if snap.get("ball"):
            bits.append("ball ●")
        self.sub.setText("   ·   ".join(b for b in bits if b))


class RobotsTab(QWidget):
    robot_selected = Signal(str)

    def __init__(self, robots, settings: dict, parent=None):
        super().__init__(parent)
        self._settings = settings
        self._cols = 0

        root = QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        head = QHBoxLayout()
        title = QLabel("Robots")
        title.setStyleSheet(f"color:{COL_TEXT.name()}; font-size:20px; font-weight:bold;")
        hint = QLabel("drop photos into diag/ui/assets/robots as Y0.png, B1.png …")
        hint.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
        head.addWidget(title)
        head.addSpacing(10)
        head.addWidget(hint)
        head.addStretch()
        root.addLayout(head)

        self._scroll = QScrollArea()
        self._scroll.setWidgetResizable(True)
        self._scroll.setFrameShape(QFrame.NoFrame)
        self._holder = QWidget()
        self._grid = QGridLayout(self._holder)
        self._grid.setContentsMargins(0, 0, 0, 0)
        self._grid.setHorizontalSpacing(12)
        self._grid.setVerticalSpacing(12)
        self._grid.setAlignment(Qt.AlignTop)
        self._scroll.setWidget(self._holder)
        root.addWidget(self._scroll, 1)

        photo_dir = settings.get("robot_photo_dir")
        self._cards: dict[str, GalleryCard] = {}
        self._order: list[str] = []
        for r in robots:
            card = GalleryCard(r.label, r.is_yellow, photo_dir)
            card.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
            card.clicked.connect(self.robot_selected)
            self._cards[r.label] = card
            self._order.append(r.label)

        if not self._cards:
            empty = QLabel("No robots in ipconfig.yaml.")
            empty.setStyleSheet(f"color:{COL_TEXT_DIM.name()};")
            self._grid.addWidget(empty, 0, 0)
        self._relayout(force=True)

    def resizeEvent(self, ev):
        self._relayout()
        super().resizeEvent(ev)

    def showEvent(self, ev):
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
        for snap in snapshot.get("robots", []):
            card = self._cards.get(snap["label"])
            if card is not None:
                card.update_snapshot(snap, self._settings)
