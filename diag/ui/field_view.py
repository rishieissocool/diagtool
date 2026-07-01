"""
field_view.py — lightweight top-down field canvas.

Draws the field, the safety margin (the keep-off-walls zone DiagTool will
never knowingly drive a robot past), the drive zone (the tighter boundary the
robot is actively kept inside), the centre cross, and every robot vision
currently sees, with a heading tick.

Beyond viewing, the drive zone can be *set by dragging a box* on the canvas:
press, drag out a rectangle, release. That's the "test zone" — handy when a
competition only gives you half a field. A `zone_changed` signal carries the
new rectangle (world mm); the owner clamps + applies it. Dragging never sends
a command itself.
"""

from __future__ import annotations

from PySide6.QtCore import Qt, QPointF, QRectF, Signal
from PySide6.QtGui import QColor, QPainter, QPen, QBrush, QFont
from PySide6.QtWidgets import QWidget

import math

_BG = QColor("#0b3d0b")
_LINE = QColor("#cfe8cf")
_SAFE = QColor("#e0b341")
_ARENA = QColor("#ff8c42")
_ZONE = QColor("#46c2e0")          # custom (drag-set) test zone
_ZONE_FILL = QColor(70, 194, 224, 38)
_DRAG = QColor("#7CFC8A")          # rectangle being dragged out right now
_DRAG_FILL = QColor(124, 252, 138, 50)
_YELLOW = QColor("#e8d44d")
_BLUE = QColor("#4d8ce8")
_SEL = QColor("#ff5555")
_BALL = QColor("#ff8000")          # SSL ball (orange)

# ignore tiny press-drags (a click / a slip) so the zone isn't set by accident
_MIN_DRAG_PX = 12
_MIN_DRAG_MM = 250.0


class FieldView(QWidget):
    # new test-zone rectangle in world mm: (x_min, x_max, y_min, y_max)
    zone_changed = Signal(float, float, float, float)

    def __init__(self, limits, parent=None):
        super().__init__(parent)
        self._lim = limits
        self._poses: dict[str, tuple] = {}   # label -> (x, y, o, is_yellow)
        self._ball: tuple[float, float] | None = None   # (x, y) world mm, or None
        self._selected: str | None = None
        self._edit = True                    # drag-to-set-zone enabled
        self._drag_start: QPointF | None = None   # screen px
        self._drag_cur: QPointF | None = None
        self.setMinimumSize(420, 280)
        self.setCursor(Qt.CrossCursor)

    def set_limits(self, limits):
        self._lim = limits
        self.update()

    def set_poses(self, poses: dict):
        self._poses = dict(poses)
        self.update()

    def set_ball(self, ball: tuple[float, float] | None):
        """Show (or hide, with None) the ball at world-mm (x, y)."""
        self._ball = tuple(ball) if ball is not None else None
        self.update()

    def set_selected(self, label: str | None):
        self._selected = label
        self.update()

    def set_zone_edit_enabled(self, enabled: bool):
        """Allow/forbid dragging out a new zone (e.g. lock it while a test runs)."""
        self._edit = bool(enabled)
        if not enabled:
            self._drag_start = self._drag_cur = None
        self.setCursor(Qt.CrossCursor if enabled else Qt.ArrowCursor)
        self.update()

    # -- world(mm) <-> screen(px) --
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

    def _to_world(self, pt: QPointF):
        """Screen px -> world mm (inverse of _transform's to_px)."""
        _, scale = self._transform()
        if not scale:
            return 0.0, 0.0
        x = (pt.x() - self.width() / 2) / scale
        y = (self.height() / 2 - pt.y()) / scale
        return x, y

    # -- drag-to-set the test zone --
    def mousePressEvent(self, ev):
        if self._edit and ev.button() == Qt.LeftButton:
            self._drag_start = ev.position()
            self._drag_cur = ev.position()
            self.update()
        else:
            super().mousePressEvent(ev)

    def mouseMoveEvent(self, ev):
        if self._drag_start is not None:
            self._drag_cur = ev.position()
            self.update()
        else:
            super().mouseMoveEvent(ev)

    def mouseReleaseEvent(self, ev):
        if self._drag_start is None or ev.button() != Qt.LeftButton:
            super().mouseReleaseEvent(ev)
            return
        start, cur = self._drag_start, ev.position()
        self._drag_start = self._drag_cur = None
        self.update()
        if (abs(cur.x() - start.x()) < _MIN_DRAG_PX or
                abs(cur.y() - start.y()) < _MIN_DRAG_PX):
            return                                   # too small -> treat as a click
        x0, y0 = self._to_world(start)
        x1, y1 = self._to_world(cur)
        x_min, x_max = sorted((x0, x1))
        y_min, y_max = sorted((y0, y1))
        # clamp to the keep-off-walls safe box (Limits clamps again, defensively)
        lim = self._lim
        sx = max(0.0, lim.half_len - lim.safe_margin)
        sy = max(0.0, lim.half_wid - lim.safe_margin)
        x_min, x_max = max(-sx, x_min), min(sx, x_max)
        y_min, y_max = max(-sy, y_min), min(sy, y_max)
        if (x_max - x_min) < _MIN_DRAG_MM or (y_max - y_min) < _MIN_DRAG_MM:
            return
        self.zone_changed.emit(x_min, x_max, y_min, y_max)

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

        # drive zone — the tighter boundary the robot is actively kept inside.
        # A custom (drag-set) zone is tinted + labelled "test zone"; otherwise
        # it's the default symmetric "arena".
        bounds = getattr(lim, "arena_bounds", None)
        if callable(bounds):
            xlo, xhi, ylo, yhi = lim.arena_bounds()
            custom = getattr(lim, "has_custom_zone", False)
            zt = to_px(xlo, yhi)
            zb = to_px(xhi, ylo)
            rect = QRectF(zt, zb)
            col = _ZONE if custom else _ARENA
            if custom:
                p.fillRect(rect, _ZONE_FILL)
            p.setPen(QPen(col, 2, Qt.DashLine))
            p.setBrush(Qt.NoBrush)
            p.drawRect(rect)
            p.setPen(QPen(col))
            f = QFont(); f.setPointSize(8); p.setFont(f)
            label = "test zone" if custom else "arena"
            if custom:
                label += f"  {xhi - xlo:.0f}×{yhi - ylo:.0f} mm"
            p.drawText(QPointF(rect.left() + 4, rect.top() + 14), label)

        # rectangle currently being dragged out
        if self._drag_start is not None and self._drag_cur is not None:
            drag = QRectF(self._drag_start, self._drag_cur).normalized()
            p.fillRect(drag, _DRAG_FILL)
            p.setPen(QPen(_DRAG, 2, Qt.SolidLine))
            p.setBrush(Qt.NoBrush)
            p.drawRect(drag)

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

        # ball (drawn last so it stays visible on top of any robot over it)
        if self._ball is not None:
            bc = to_px(self._ball[0], self._ball[1])
            br_px = max(21.5 * scale, 4)          # SSL ball ≈ 43 mm dia
            p.setPen(QPen(QColor("#000000"), 1))
            p.setBrush(QBrush(_BALL))
            p.drawEllipse(bc, br_px, br_px)
        p.end()
