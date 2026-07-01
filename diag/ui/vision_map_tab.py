"""
vision_map_tab.py — standalone live vision map.

A dedicated, read-only top-down view of what SSL-Vision is putting on the local
network (the ethernet LAN): every robot both teams' cameras can see, plus the
ball, drawn live. Unlike the little field on the Diagnostics tab it does NOT
edit the test zone and it is not limited to the ipconfig inventory — it shows
*everything* on the wire, so it doubles as a "is vision actually reaching this
machine?" check at a competition.

Data comes straight from the Engine's VisionSource (which joins the SSL-Vision
multicast group on the configured interface — see Engine.vision_net_info); the
tab is refreshed once per UI tick by MainWindow.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QGroupBox, QTableWidget,
    QTableWidgetItem, QHeaderView, QSplitter,
)

from .field_view import FieldView
from .widgets import (
    COL_TEXT, COL_TEXT_DIM, COL_GOOD, COL_WARN, COL_CRIT, COL_IDLE,
    COL_YELLOW, COL_BLUE,
)

_MONO = "monospace"


class VisionMapTab(QWidget):
    def __init__(self, engine, parent=None):
        super().__init__(parent)
        self.engine = engine

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(10)

        # -- header: title + where on the LAN we're listening --
        head = QVBoxLayout()
        head.setSpacing(2)
        title = QLabel("Vision Map")
        title.setStyleSheet(f"color:{COL_TEXT.name()}; font-size:20px; font-weight:bold;")
        net = self.engine.vision_net_info()
        iface = net["interface"]
        iface_txt = "all interfaces" if iface in ("0.0.0.0", "", None) else iface
        src = QLabel(
            f"SSL-Vision multicast {net['group']}:{net['port']}  ·  "
            f"interface {iface} ({iface_txt})")
        src.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
        src.setFont(QFont(_MONO, 10))
        src.setToolTip(
            "Vision is read live off the local network (ethernet) by joining the "
            "SSL-Vision multicast group. The interface comes from "
            "ipconfig.yaml → network.vision_ip; 0.0.0.0 means listen on every "
            "network adapter. Set it to this PC's ethernet IP to pin it to the "
            "field LAN.")
        head.addWidget(title)
        head.addWidget(src)
        root.addLayout(head)

        # -- live status line (fps / visible / latency / ball) --
        self.lbl_status = QLabel("waiting for vision…")
        self.lbl_status.setFont(QFont(_MONO, 10))
        self.lbl_status.setStyleSheet(f"color:{COL_IDLE.name()};")
        root.addWidget(self.lbl_status)

        # -- map (left, read-only) + detections table (right) --
        split = QSplitter(Qt.Horizontal)
        root.addWidget(split, 1)

        self.map = FieldView(self.engine.lim)
        self.map.set_zone_edit_enabled(False)   # pure monitor — never sets a zone
        split.addWidget(self.map)

        side = QGroupBox("On the wire")
        sl = QVBoxLayout(side)
        self.tbl = QTableWidget(0, 3)
        self.tbl.setHorizontalHeaderLabels(["Object", "X (mm)", "Y (mm)"])
        self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl.setEditTriggers(QTableWidget.NoEditTriggers)
        self.tbl.setSelectionMode(QTableWidget.NoSelection)
        self.tbl.setFont(QFont(_MONO, 9))
        sl.addWidget(self.tbl)
        split.addWidget(side)
        split.setSizes([900, 320])

    # -- called each UI tick by MainWindow --
    def refresh(self):
        vis = self.engine.vision

        # every robot vision currently sees (both teams, all ids — not just
        # the ipconfig inventory), fresh within half a second
        poses: dict[str, tuple] = {}
        for (is_y, rid) in vis.visible_robots():
            s = vis.get_pose_sample(is_y, rid, max_age=0.5)
            if s:
                poses[f"{'Y' if is_y else 'B'}{rid}"] = (s.x, s.y, s.o, is_y)
        ball = vis.get_ball(max_age=0.5)

        self.map.set_limits(self.engine.lim)      # picks up live field geometry
        self.map.set_poses(poses)
        self.map.set_ball((ball.x, ball.y) if ball else None)

        self._update_status(vis.status(), len(poses), ball is not None)
        self._update_table(poses, ball)

    def _update_status(self, st: dict, n_visible: int, has_ball: bool):
        if st.get("error"):
            self.lbl_status.setText(f"vision ERROR — {st['error']}")
            self.lbl_status.setStyleSheet(f"color:{COL_CRIT.name()};")
            return
        fps = st.get("fps") or 0.0
        lat = (st.get("sent_latency_ms") or {}).get("median")
        lat_txt = f"{lat:.0f} ms" if isinstance(lat, (int, float)) else "—"
        ball_txt = "yes" if has_ball else "no"
        self.lbl_status.setText(
            f"vision {fps:5.1f} fps   robots={n_visible}   ball={ball_txt}   "
            f"drops={st.get('drops', 0)}   latency(med)={lat_txt}")
        if fps <= 0:
            col = COL_IDLE       # nothing arriving — off the LAN or vision down
        elif fps > 5 and n_visible > 0:
            col = COL_GOOD
        else:
            col = COL_WARN
        self.lbl_status.setStyleSheet(f"color:{col.name()};")

    def _update_table(self, poses: dict, ball):
        rows = sorted(poses.items())
        self.tbl.setRowCount(len(rows) + (1 if ball else 0))
        r = 0
        if ball:
            self._set_row(r, "● ball", ball.x, ball.y, COL_WARN)
            r += 1
        for label, (x, y, o, is_y) in rows:
            self._set_row(r, label, x, y, COL_YELLOW if is_y else COL_BLUE)
            r += 1

    def _set_row(self, row: int, name: str, x: float, y: float, color):
        item = QTableWidgetItem(name)
        item.setForeground(color)
        self.tbl.setItem(row, 0, item)
        self.tbl.setItem(row, 1, QTableWidgetItem(f"{x:7.0f}"))
        self.tbl.setItem(row, 2, QTableWidgetItem(f"{y:7.0f}"))
