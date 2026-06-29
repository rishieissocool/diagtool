"""
widgets.py — shared UI building blocks for DiagTool's dashboard tabs.

Pure-Qt helpers with no dependency on the engine, so both the Competition
overview and the Robots gallery can reuse them:

  * battery_status()  — map a telemetry voltage to a percentage + colour band,
  * robot_pixmap()    — load a robot's photo (assets/robots/<LABEL>.png) or draw
                        a clean team-coloured placeholder when none exists,
  * BatteryBar        — a small painted battery gauge,
  * StatusPill        — a coloured online/offline chip,
  * RobotCard         — the compact per-robot card used on the Competition tab.

Everything is fed a plain "robot snapshot" dict (see main_window._collect_snapshot)
so these widgets never reach back into the engine.
"""

from __future__ import annotations

from pathlib import Path

from PySide6.QtCore import Qt, QRectF, Signal
from PySide6.QtGui import (
    QColor, QPainter, QPen, QBrush, QFont, QPixmap, QLinearGradient,
    QPainterPath, QPolygonF,
)
from PySide6.QtCore import QPointF
from PySide6.QtWidgets import (
    QWidget, QFrame, QVBoxLayout, QHBoxLayout, QLabel, QGridLayout, QSizePolicy,
)

import math

# ── palette ──────────────────────────────────────────────────────────────
COL_GOOD = QColor("#46c25a")
COL_WARN = QColor("#e0b341")
COL_CRIT = QColor("#e05050")
COL_IDLE = QColor("#7d8590")
COL_YELLOW = QColor("#e8d44d")
COL_BLUE = QColor("#4d8ce8")
COL_CARD = QColor("#1b2330")
COL_CARD_HDR = QColor("#222c3c")
COL_TEXT = QColor("#e6edf3")
COL_TEXT_DIM = QColor("#9aa7b4")

_PHOTO_EXTS = (".png", ".jpg", ".jpeg", ".webp", ".bmp")
_ASSET_DIR = Path(__file__).resolve().parent / "assets" / "robots"


# ── battery maths ──────────────────────────────────────────────────────────
def battery_status(voltage, settings: dict | None = None) -> dict:
    """voltage (V or None) -> {pct, level, color, text}.

    level is one of "good" / "warn" / "crit" / "unknown". The voltage->% range
    and the warn/crit thresholds come from settings (battery_full_v etc.) with
    6S-LiPo defaults.
    """
    s = settings or {}
    full = float(s.get("battery_full_v", 25.2) or 25.2)
    empty = float(s.get("battery_empty_v", 19.8) or 19.8)
    if full <= empty:                                  # guard against a bad config
        full, empty = 25.2, 19.8
    warn = float(s.get("battery_warn_pct", 40.0) or 40.0)
    crit = float(s.get("battery_crit_pct", 15.0) or 15.0)

    if voltage is None:
        return {"pct": None, "level": "unknown", "color": COL_IDLE, "text": "— V"}

    pct = max(0.0, min(100.0, (float(voltage) - empty) / (full - empty) * 100.0))
    if pct <= crit:
        level, color = "crit", COL_CRIT
    elif pct <= warn:
        level, color = "warn", COL_WARN
    else:
        level, color = "good", COL_GOOD
    return {"pct": pct, "level": level, "color": color,
            "text": f"{float(voltage):.1f} V"}


# ── robot imagery ──────────────────────────────────────────────────────────
def robot_photo_path(label: str, photo_dir: str | None) -> Path | None:
    """First existing <photo_dir>/<LABEL>.<ext>, else None."""
    dirs = []
    if photo_dir:
        dirs.append(Path(photo_dir))
    dirs.append(_ASSET_DIR)
    for d in dirs:
        for ext in _PHOTO_EXTS:
            cand = d / f"{label}{ext}"
            if cand.is_file():
                return cand
    return None


def _draw_placeholder(label: str, is_yellow: bool, size: int) -> QPixmap:
    """A clean, recognisable robot icon: dark panel + team-coloured puck with a
    heading wedge and the shell number. Used when no photo is supplied."""
    pm = QPixmap(size, size)
    pm.fill(Qt.transparent)
    p = QPainter(pm)
    p.setRenderHint(QPainter.Antialiasing, True)

    # background panel
    bg = QLinearGradient(0, 0, 0, size)
    bg.setColorAt(0.0, QColor("#243042"))
    bg.setColorAt(1.0, QColor("#161d29"))
    panel = QRectF(2, 2, size - 4, size - 4)
    path = QPainterPath()
    path.addRoundedRect(panel, size * 0.08, size * 0.08)
    p.fillPath(path, QBrush(bg))
    p.setPen(QPen(QColor("#33425a"), 2))
    p.drawPath(path)

    # robot puck
    team = COL_YELLOW if is_yellow else COL_BLUE
    cx, cy = size / 2.0, size * 0.52
    r = size * 0.30
    grad = QLinearGradient(cx - r, cy - r, cx + r, cy + r)
    grad.setColorAt(0.0, team.lighter(115))
    grad.setColorAt(1.0, team.darker(115))
    p.setBrush(QBrush(grad))
    p.setPen(QPen(QColor("#10151c"), max(2.0, size * 0.02)))
    p.drawEllipse(QPointF(cx, cy), r, r)

    # heading wedge (flat front, like the real robots) pointing up
    wedge = QPolygonF([
        QPointF(cx - r * 0.62, cy - r * 0.78),
        QPointF(cx + r * 0.62, cy - r * 0.78),
        QPointF(cx + r * 0.50, cy - r * 1.02),
        QPointF(cx - r * 0.50, cy - r * 1.02),
    ])
    p.setBrush(QBrush(QColor("#10151c")))
    p.setPen(Qt.NoPen)
    p.drawPolygon(wedge)

    # shell number
    num = "".join(ch for ch in label if ch.isdigit()) or label
    f = QFont("Arial", int(size * 0.22))
    f.setBold(True)
    p.setFont(f)
    p.setPen(QPen(QColor("#10151c")))
    p.drawText(QRectF(cx - r, cy - r, 2 * r, 2 * r), Qt.AlignCenter, num)

    # team label chip
    fr = QFont("Arial", int(size * 0.085))
    fr.setBold(True)
    p.setFont(fr)
    p.setPen(QPen(COL_TEXT))
    p.drawText(QRectF(0, size * 0.80, size, size * 0.16), Qt.AlignCenter,
               ("YELLOW " if is_yellow else "BLUE ") + label)
    p.end()
    return pm


def robot_pixmap(label: str, is_yellow: bool, size: int,
                 photo_dir: str | None = None) -> QPixmap:
    """A square QPixmap for the robot: its photo (cover-cropped) if one exists,
    else a drawn placeholder."""
    path = robot_photo_path(label, photo_dir)
    if path is not None:
        src = QPixmap(str(path))
        if not src.isNull():
            scaled = src.scaled(size, size, Qt.KeepAspectRatioByExpanding,
                                Qt.SmoothTransformation)
            # centre-crop to a square
            x = max(0, (scaled.width() - size) // 2)
            y = max(0, (scaled.height() - size) // 2)
            return scaled.copy(x, y, size, size)
    return _draw_placeholder(label, is_yellow, size)


# ── battery gauge widget ───────────────────────────────────────────────────
class BatteryBar(QWidget):
    """Horizontal battery gauge: filled proportion + colour + overlaid text."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._pct = None
        self._color = COL_IDLE
        self._text = "— V"
        self.setMinimumHeight(22)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_status(self, status: dict):
        self._pct = status.get("pct")
        self._color = status.get("color", COL_IDLE)
        pct = status.get("pct")
        self._text = (f"{status.get('text', '')}"
                      + (f"  ·  {pct:.0f}%" if pct is not None else "  ·  no telemetry"))
        self.update()

    def paintEvent(self, _ev):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        r = QRectF(1, 1, self.width() - 6, self.height() - 2)  # leave room for nub
        # shell
        p.setPen(QPen(QColor("#3a4658"), 1.5))
        p.setBrush(QBrush(QColor("#0e141d")))
        p.drawRoundedRect(r, 4, 4)
        # positive terminal nub
        nub = QRectF(r.right(), r.center().y() - r.height() * 0.22,
                     4, r.height() * 0.44)
        p.setBrush(QBrush(QColor("#3a4658")))
        p.setPen(Qt.NoPen)
        p.drawRoundedRect(nub, 1.5, 1.5)
        # fill
        if self._pct is not None and self._pct > 0:
            fw = (r.width() - 4) * (self._pct / 100.0)
            fill = QRectF(r.left() + 2, r.top() + 2, max(2.0, fw), r.height() - 4)
            p.setPen(Qt.NoPen)
            p.setBrush(QBrush(self._color))
            p.drawRoundedRect(fill, 3, 3)
        # text
        p.setPen(QPen(COL_TEXT))
        f = QFont("Arial", 8)
        f.setBold(True)
        p.setFont(f)
        p.drawText(r, Qt.AlignCenter, self._text)
        p.end()


# ── small status chip ──────────────────────────────────────────────────────
class StatusPill(QLabel):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumWidth(64)

    def set_state(self, text: str, color: QColor, fg: QColor = QColor("#0b0f14")):
        self.setText(text)
        self.setStyleSheet(
            f"background:{color.name()}; color:{fg.name()}; border-radius:7px;"
            "padding:1px 8px; font-weight:bold; font-size:11px;")


def _dot(on: bool, color: QColor) -> str:
    c = color.name() if on else COL_IDLE.name()
    return f"<span style='color:{c}; font-size:15px;'>●</span>"


# ── compact per-robot card (Competition tab) ───────────────────────────────
class RobotCard(QFrame):
    """Data-dense card: battery, vision/telemetry link, ball, pose, state.

    The whole card border/tint reflects overall health so problems pop out at a
    glance across a wall of robots during a match.
    """

    clicked = Signal(str)   # emits the robot label

    def __init__(self, label: str, is_yellow: bool, parent=None):
        super().__init__(parent)
        self.label_id = label
        self.is_yellow = is_yellow
        self.setObjectName("robotCard")
        self.setFrameShape(QFrame.StyledPanel)
        self.setMinimumWidth(248)
        self.setCursor(Qt.PointingHandCursor)

        root = QVBoxLayout(self)
        root.setContentsMargins(10, 8, 10, 10)
        root.setSpacing(6)

        # header: team dot + label + ip
        hdr = QHBoxLayout()
        hdr.setSpacing(6)
        self.lbl_team = QLabel()
        self.lbl_team.setText(_dot(True, COL_YELLOW if is_yellow else COL_BLUE))
        self.lbl_name = QLabel(label)
        self.lbl_name.setStyleSheet(
            f"color:{COL_TEXT.name()}; font-weight:bold; font-size:16px;")
        self.lbl_ip = QLabel("")
        self.lbl_ip.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
        self.lbl_ip.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        hdr.addWidget(self.lbl_team)
        hdr.addWidget(self.lbl_name)
        hdr.addStretch()
        hdr.addWidget(self.lbl_ip)
        root.addLayout(hdr)

        # battery
        self.battery = BatteryBar()
        root.addWidget(self.battery)

        # link pills row
        links = QHBoxLayout()
        links.setSpacing(6)
        self.pill_vis = StatusPill()
        self.pill_tel = StatusPill()
        links.addWidget(self.pill_vis)
        links.addWidget(self.pill_tel)
        links.addStretch()
        root.addLayout(links)

        # detail grid
        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(2)
        self.val_pose = QLabel("—")
        self.val_head = QLabel("—")
        self.val_ball = QLabel("—")
        self.val_state = QLabel("—")
        rows = [("Position", self.val_pose), ("Heading", self.val_head),
                ("Ball", self.val_ball), ("State", self.val_state)]
        for i, (k, v) in enumerate(rows):
            key = QLabel(k)
            key.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
            v.setStyleSheet(f"color:{COL_TEXT.name()}; font-size:11px;")
            v.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            grid.addWidget(key, i, 0)
            grid.addWidget(v, i, 1)
        grid.setColumnStretch(1, 1)
        root.addLayout(grid)

        self._apply_tint("idle")

    def mousePressEvent(self, ev):
        self.clicked.emit(self.label_id)
        super().mousePressEvent(ev)

    def _apply_tint(self, level: str):
        edge = {"good": COL_GOOD, "warn": COL_WARN, "crit": COL_CRIT,
                "idle": QColor("#33425a")}.get(level, QColor("#33425a"))
        self.setStyleSheet(
            f"#robotCard {{ background:{COL_CARD.name()}; border:1px solid "
            f"{edge.name()}; border-radius:10px; }}")

    def update_snapshot(self, snap: dict, settings: dict):
        self.lbl_ip.setText(snap.get("ip", "") + ("  ⚠DUP" if snap.get("dup") else ""))

        bat = battery_status(snap.get("voltage"), settings)
        self.battery.set_status(bat)

        vis = snap.get("vision_seen")
        self.pill_vis.set_state(
            "VISION ●" if vis else "VISION ○",
            COL_GOOD if vis else COL_IDLE,
            QColor("#0b0f14") if vis else COL_TEXT)
        tel = snap.get("tel_seen")
        rate = snap.get("rate_hz") or 0.0
        self.pill_tel.set_state(
            (f"TELEM {rate:.0f}Hz" if tel else "TELEM ○"),
            COL_GOOD if tel else COL_IDLE,
            QColor("#0b0f14") if tel else COL_TEXT)

        if vis and snap.get("x") is not None:
            self.val_pose.setText(f"{snap['x']:.0f}, {snap['y']:.0f} mm")
            self.val_head.setText(f"{snap.get('o_deg', 0):.0f}°")
        else:
            self.val_pose.setText("not seen")
            self.val_head.setText("—")

        ball = snap.get("ball")
        if ball is None:
            self.val_ball.setText("—")
            self.val_ball.setStyleSheet(f"color:{COL_TEXT.name()}; font-size:11px;")
        else:
            self.val_ball.setText("● has ball" if ball else "no")
            self.val_ball.setStyleSheet(
                f"color:{(COL_WARN if ball else COL_TEXT_DIM).name()}; font-size:11px;")
        self.val_state.setText(str(snap.get("state") or "—"))

        # overall health tint: crit if totally dark or battery critical;
        # warn if half-linked or low battery; good if fully linked.
        if not vis and not tel:
            level = "crit" if snap.get("is_real") else "idle"
        elif bat["level"] == "crit":
            level = "crit"
        elif (not vis or not tel) or bat["level"] == "warn":
            level = "warn"
        else:
            level = "good"
        self._apply_tint(level)
