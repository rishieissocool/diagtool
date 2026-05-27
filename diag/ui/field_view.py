"""
field_view.py — lightweight top-down field canvas.

Draws the field, the safety margin (the keep-off-walls zone DiagTool will
never knowingly drive a robot past), the centre cross, and every robot vision
currently sees, with a heading tick. Purely a viewer; it never sends commands.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, QPointF, QRectF
from PySide6.QtGui import QColor, QPainter, QPen, QBrush, QFont
from PySide6.QtWidgets import QWidget

import math

_BG = QColor("#0b3d0b")
_LINE = QColor("#cfe8cf")
_SAFE = QColor("#e0b341")
_YELLOW = QColor("#e8d44d")
_BLUE = QColor("#4d8ce8")
_SEL = QColor("#ff5555")


class FieldView(QWidget):
    def __init__(self, limits, parent=None):
        super().__init__(parent)
        self._lim = limits
        self._poses: dict[str, tuple] = {}   # label -> (x, y, o, is_yellow)
        self._selected: str | None = None
        self.setMinimumSize(420, 280)

    def set_limits(self, limits):
        self._lim = limits
        self.update()

    def set_poses(self, poses: dict):
        self._poses = dict(poses)
        self.update()

    def set_selected(self, label: str | None):
        self._selected = label
        self.update()

    # -- world(mm) -> screen(px) --
    def _transform(self):
        lim = self._lim
        margin_px = 14
        w = self.width() - 2 * margin_px
        h = self.height() - 2 * margin_px
        fw, fh = 2 * lim.half_len, 2 * lim.half_wid
        scale = min(w / fw, h / fh) if fw and fh else 1.0
        cx, cy = self.width() / 2, self.height() / 2

        def to_px(x, y):
            return QPointF(cx + x * scale, cy - y * scale)   # flip y
        return to_px, scale

    def paintEvent(self, _ev):
        lim = self._lim
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, True)
        p.fillRect(self.rect(), _BG)
        to_px, scale = self._transform()

        # field border
        tl = to_px(-lim.half_len, lim.half_wid)
        br = to_px(lim.half_len, -lim.half_wid)
        field = QRectF(tl, br)
        p.setPen(QPen(_LINE, 2))
        p.setBrush(Qt.NoBrush)
        p.drawRect(field)

        # halfway line + centre circle
        p.drawLine(to_px(0, lim.half_wid), to_px(0, -lim.half_wid))
        r_px = 500 * scale
        c = to_px(0, 0)
        p.drawEllipse(c, r_px, r_px)

        # safety margin (keep-off zone)
        m = lim.safe_margin
        st = to_px(-lim.half_len + m, lim.half_wid - m)
        sb = to_px(lim.half_len - m, -lim.half_wid + m)
        pen = QPen(_SAFE, 1, Qt.DashLine)
        p.setPen(pen)
        p.drawRect(QRectF(st, sb))

        # robots
        rr = max(lim.robot_radius * scale, 6)
        f = QFont(); f.setPointSize(8); p.setFont(f)
        for label, (x, y, o, is_yellow) in self._poses.items():
            ctr = to_px(x, y)
            col = _YELLOW if is_yellow else _BLUE
            if label == self._selected:
                p.setPen(QPen(_SEL, 3))
            else:
                p.setPen(QPen(_LINE, 1))
            p.setBrush(QBrush(col))
            p.drawEllipse(ctr, rr, rr)
            # heading tick
            hx = ctr.x() + math.cos(o) * rr * 1.6
            hy = ctr.y() - math.sin(o) * rr * 1.6
            p.setPen(QPen(QColor("#111111"), 2))
            p.drawLine(ctr, QPointF(hx, hy))
            p.setPen(QPen(QColor("#000000")))
            p.drawText(QPointF(ctr.x() - rr, ctr.y() - rr - 2), label)
        p.end()
