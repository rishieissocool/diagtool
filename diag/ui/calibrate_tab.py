"""
calibrate_tab.py — one-click auto-calibration for a whole team of robots.

Calibrate every robot on YOUR half of the field, one at a time, completely
hands-off, and get a per-robot report plus a combined summary in a generated
output folder. Built entirely on the existing wall-safe Commander + test-zone +
diagnostic battery, so it can never drive a robot vision can see into a wall.

Workflow at a comp:
  1. pick our team colour (Yellow / Blue) and our side of the field,
  2. tick "restrict to our half" (you usually only get one side) — the drive
     zone shrinks to that half, clamped to the keep-off-walls box,
  3. fix any IPs inline (applies live), tick the robots to calibrate,
  4. press Start. Each robot is probed, then the full battery (incl. a pure
     spin-in-place calibration) runs on it; results stream into the table and a
     report folder is written.

The tab never drives robots itself — it hands a job to the single worker thread
in MainWindow (so only ONE robot ever moves at a time) and renders the results.
"""

from __future__ import annotations

import os

from PySide6.QtCore import Qt
from PySide6.QtGui import QColor, QFont
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QCheckBox, QTableWidget, QTableWidgetItem, QHeaderView, QGroupBox,
    QGridLayout, QProgressBar, QPlainTextEdit, QSplitter, QMessageBox,
)

from ..diagnostics import CALIBRATION_TESTS, BY_NAME
from .field_view import FieldView
from .widgets import (COL_TEXT, COL_TEXT_DIM, COL_GOOD, COL_WARN, COL_CRIT,
                      COL_IDLE, battery_status)

# result/status table columns
C_CHECK, C_ROBOT, C_IP, C_REAL, C_LINK, C_SPD, C_W, C_DRIFT, C_SPIN, C_LAT, C_STAT = range(11)
_HEADERS = ["✓", "Robot", "IP (editable)", "Real", "Link", "speed\nscale",
            "w\nscale", "drift\nmm/m", "spin drift\nmm", "cmd lat\nms", "status"]

# trial-count presets layered on top of diag_settings.yaml
_PRESETS = {
    "Quick (fast)":   {"latency_trials": 4, "stop_trials": 3, "angular_trials": 3,
                       "speed_passes": 3, "spin_trials": 3},
    "Standard":       {},                      # use diag_settings.yaml as-is
    "Thorough (slow)": {"latency_trials": 8, "stop_trials": 6, "angular_trials": 6,
                        "speed_passes": 6, "spin_trials": 6},
}

# friendly names for the test toggles
_TEST_LABELS = {
    "vision_health": "Vision health",
    "telemetry_health": "Telemetry health",
    "command_latency": "Command latency",
    "stop_latency": "Stop latency & coast",
    "speed_scale": "Speed scale & drift",
    "angular": "Rotation latency / w-scale",
    "spin": "Spin-in-place calibration",
}


class CalibrateTab(QWidget):
    def __init__(self, engine, start_cb, stop_cb, parent=None):
        super().__init__(parent)
        self.engine = engine
        self._start_cb = start_cb        # MainWindow launches the worker job
        self._stop_cb = stop_cb
        self._row_by_label: dict[str, int] = {}
        self._populating = False
        self._busy = False
        self._last_output: str | None = None

        root = QVBoxLayout(self)
        root.setContentsMargins(14, 12, 14, 12)
        root.setSpacing(10)

        # -- header + safety banner --
        head = QHBoxLayout()
        title = QLabel("Auto-Calibrate")
        title.setStyleSheet(f"color:{COL_TEXT.name()}; font-size:20px; font-weight:bold;")
        head.addWidget(title)
        head.addSpacing(12)
        banner = QLabel("Drives one robot at a time, inside the safe zone, with the "
                        "predictive wall-stop active — it will not hit anything.")
        banner.setStyleSheet(f"color:{COL_GOOD.name()}; font-size:11px;")
        head.addWidget(banner)
        head.addStretch()
        root.addLayout(head)

        # -- options row --
        opts = QGroupBox("Setup")
        og = QGridLayout(opts)
        og.setHorizontalSpacing(14)
        og.setVerticalSpacing(8)

        og.addWidget(QLabel("Our team:"), 0, 0)
        self.cmb_team = QComboBox()
        self.cmb_team.addItems(["Yellow", "Blue"])
        self.cmb_team.setCurrentIndex(0 if engine.config_flag("us_yellow", True) else 1)
        self.cmb_team.currentIndexChanged.connect(self._on_team_changed)
        og.addWidget(self.cmb_team, 0, 1)

        og.addWidget(QLabel("Our side:"), 0, 2)
        self.cmb_side = QComboBox()
        self.cmb_side.addItems(["+X half", "−X half"])
        self.cmb_side.setCurrentIndex(0 if engine.config_flag("us_positive", True) else 1)
        self.cmb_side.currentIndexChanged.connect(self._apply_zone)
        og.addWidget(self.cmb_side, 0, 3)

        self.chk_half = QCheckBox("Restrict to our half (only one side available)")
        self.chk_half.setChecked(True)
        self.chk_half.toggled.connect(self._apply_zone)
        og.addWidget(self.chk_half, 0, 4, 1, 2)

        og.addWidget(QLabel("Trials:"), 1, 0)
        self.cmb_preset = QComboBox()
        self.cmb_preset.addItems(list(_PRESETS.keys()))
        self.cmb_preset.setCurrentText("Standard")
        og.addWidget(self.cmb_preset, 1, 1)

        # test toggles
        og.addWidget(QLabel("Tests:"), 1, 2)
        tests_row = QHBoxLayout()
        tests_row.setSpacing(10)
        self.test_checks: dict[str, QCheckBox] = {}
        for name in CALIBRATION_TESTS:
            cb = QCheckBox(_TEST_LABELS.get(name, name))
            cb.setChecked(True)
            self.test_checks[name] = cb
            tests_row.addWidget(cb)
        tests_row.addStretch()
        tw = QWidget(); tw.setLayout(tests_row)
        og.addWidget(tw, 1, 3, 1, 3)
        root.addWidget(opts)

        # -- main split: robot table (left) + field/log (right) --
        split = QSplitter(Qt.Horizontal)
        root.addWidget(split, 1)

        left = QWidget(); ll = QVBoxLayout(left)
        ll.setContentsMargins(0, 0, 0, 0)
        sel_row = QHBoxLayout()
        sel_row.addWidget(QLabel("Robots to calibrate:"))
        sel_row.addStretch()
        self.btn_all = QPushButton("Select reachable")
        self.btn_all.clicked.connect(lambda: self._select(all_real=True))
        self.btn_none = QPushButton("Select none")
        self.btn_none.clicked.connect(lambda: self._select(none=True))
        sel_row.addWidget(self.btn_all)
        sel_row.addWidget(self.btn_none)
        ll.addLayout(sel_row)

        self.tbl = QTableWidget(0, len(_HEADERS))
        self.tbl.setHorizontalHeaderLabels(_HEADERS)
        self.tbl.verticalHeader().setVisible(False)
        hh = self.tbl.horizontalHeader()
        hh.setSectionResizeMode(QHeaderView.ResizeToContents)
        hh.setSectionResizeMode(C_IP, QHeaderView.Stretch)
        self.tbl.itemChanged.connect(self._on_item_changed)
        ll.addWidget(self.tbl, 1)
        split.addWidget(left)

        right = QWidget(); rl = QVBoxLayout(right)
        rl.setContentsMargins(0, 0, 0, 0)
        self.field = FieldView(self.engine.lim)
        self.field.zone_changed.connect(self._on_zone_dragged)
        rl.addWidget(self.field, 1)
        self.lbl_zone = QLabel("")
        self.lbl_zone.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
        rl.addWidget(self.lbl_zone)
        rl.addWidget(QLabel("Log:"))
        self.log = QPlainTextEdit(); self.log.setReadOnly(True)
        self.log.setFont(QFont("monospace", 9))
        self.log.setMaximumHeight(220)
        rl.addWidget(self.log)
        split.addWidget(right)
        split.setSizes([720, 520])

        # -- run controls --
        run = QHBoxLayout()
        self.btn_start = QPushButton("▶  Start Auto-Calibration")
        self.btn_start.setStyleSheet(
            "font-weight:bold; padding:8px; font-size:13px;")
        self.btn_start.clicked.connect(self._start)
        self.btn_stop = QPushButton("■ STOP")
        self.btn_stop.setStyleSheet(
            "background:#e05050; color:white; font-weight:bold; padding:8px;")
        self.btn_stop.clicked.connect(lambda: self._stop_cb())
        self.btn_stop.setEnabled(False)
        self.btn_open = QPushButton("Open report folder")
        self.btn_open.clicked.connect(self._open_folder)
        self.btn_open.setEnabled(False)
        run.addWidget(self.btn_start, 2)
        run.addWidget(self.btn_stop, 1)
        run.addWidget(self.btn_open)
        root.addLayout(run)

        self.progress = QProgressBar(); self.progress.setRange(0, 100)
        root.addWidget(self.progress)
        self.lbl_status = QLabel("idle")
        self.lbl_status.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
        root.addWidget(self.lbl_status)

        self.populate()
        # Reflect the current zone but do NOT mutate the shared engine zone just
        # by constructing the tab — the half-field is applied on an explicit
        # toggle, or at Start (so launching the app never silently shrinks the
        # field for the other tabs).
        self.field.set_limits(self.engine.lim)
        self.lbl_zone.setText("drive zone: " + self.engine.zone_desc())

    # ── robot table ──────────────────────────────────────────────────────
    def _team_yellow(self) -> bool:
        return self.cmb_team.currentText() == "Yellow"

    def _team_robots(self):
        return [r for r in self.engine.robots() if r.is_yellow == self._team_yellow()]

    def populate(self):
        self._populating = True
        self.tbl.setRowCount(0)
        self._row_by_label.clear()
        for r in self._team_robots():
            row = self.tbl.rowCount()
            self.tbl.insertRow(row)
            self._row_by_label[r.label] = row

            chk = QTableWidgetItem()
            chk.setFlags((chk.flags() | Qt.ItemIsUserCheckable) & ~Qt.ItemIsEditable)
            chk.setCheckState(Qt.Checked if r.is_real else Qt.Unchecked)
            self.tbl.setItem(row, C_CHECK, chk)

            self.tbl.setItem(row, C_ROBOT, self._ro(r.label))
            self.tbl.setItem(row, C_IP, QTableWidgetItem(r.ip))           # editable
            self.tbl.setItem(row, C_REAL, self._ro("yes" if r.is_real else "sim"))
            self.tbl.setItem(row, C_LINK, self._ro("—"))
            for c in (C_SPD, C_W, C_DRIFT, C_SPIN, C_LAT):
                self.tbl.setItem(row, c, self._ro("—"))
            self.tbl.setItem(row, C_STAT, self._ro("queued"))
        self._populating = False

    @staticmethod
    def _ro(text: str) -> QTableWidgetItem:
        it = QTableWidgetItem(text)
        it.setFlags(it.flags() & ~Qt.ItemIsEditable)
        return it

    def _on_team_changed(self):
        if self._busy:
            return
        self.populate()

    def _on_item_changed(self, item: QTableWidgetItem):
        if self._populating or self._busy:
            return
        if item.column() != C_IP:
            return
        label = self.tbl.item(item.row(), C_ROBOT).text()
        ip = item.text().strip()
        if not ip:
            return
        self.engine.set_robot_target(label, ip=ip)     # applies live
        self.lbl_status.setText(f"{label} IP set to {ip} (live).")

    def _select(self, all_real=False, none=False):
        if self._busy:
            return
        self._populating = True
        for r in self._team_robots():
            row = self._row_by_label.get(r.label)
            if row is None:
                continue
            it = self.tbl.item(row, C_CHECK)
            if none:
                it.setCheckState(Qt.Unchecked)
            elif all_real:
                it.setCheckState(Qt.Checked if r.is_real else Qt.Unchecked)
        self._populating = False

    def _checked_labels(self) -> list[str]:
        out = []
        for label, row in self._row_by_label.items():
            it = self.tbl.item(row, C_CHECK)
            if it and it.checkState() == Qt.Checked:
                out.append(label)
        return out

    # ── field zone ───────────────────────────────────────────────────────
    def _apply_zone(self, *_):
        if self._busy:
            return
        if self.chk_half.isChecked():
            self.engine.set_field_half(positive=(self.cmb_side.currentIndex() == 0))
        else:
            self.engine.clear_test_zone()
        self.field.set_limits(self.engine.lim)
        self.lbl_zone.setText("drive zone: " + self.engine.zone_desc())

    def _on_zone_dragged(self, x_min, x_max, y_min, y_max):
        if self._busy:
            return
        # a manual drag overrides the half-field selector
        self.chk_half.blockSignals(True)
        self.chk_half.setChecked(False)
        self.chk_half.blockSignals(False)
        self.engine.set_test_zone(x_min, x_max, y_min, y_max)
        self.field.set_limits(self.engine.lim)
        self.lbl_zone.setText("drive zone: " + self.engine.zone_desc())

    # ── live status (fed by MainWindow's timer snapshot) ─────────────────
    def update_live(self, snapshot: dict):
        poses = {}
        yellow = self._team_yellow()
        for s in snapshot.get("robots", []):
            if s.get("x") is not None and s.get("o_deg") is not None:
                import math
                poses[s["label"]] = (s["x"], s["y"], math.radians(s["o_deg"]),
                                     s["is_yellow"])
            if s.get("is_yellow") != yellow:
                continue
            row = self._row_by_label.get(s["label"])
            if row is None:
                continue
            v = "V" if s.get("vision_seen") else "·"
            t = "T" if s.get("tel_seen") else "·"
            bat = battery_status(s.get("voltage"), self.engine.settings)
            pct = bat.get("pct")
            link = f"{v} {t}" + (f"  {pct:.0f}%" if pct is not None else "")
            item = self.tbl.item(row, C_LINK)
            if item:
                item.setText(link)
                item.setForeground(COL_GOOD if s.get("vision_seen") else COL_IDLE)
        if poses:
            self.field.set_poses(poses)

    # ── run ──────────────────────────────────────────────────────────────
    def _start(self):
        if self._busy:
            return
        # make sure the drive zone matches the half-field selector before driving
        self._apply_zone()
        labels = self._checked_labels()
        robots = [self.engine.find_robot(l) for l in labels]
        robots = [r for r in robots if r is not None]
        if not robots:
            QMessageBox.information(self, "No robots",
                                    "Tick at least one robot to calibrate.")
            return
        tests = [n for n in CALIBRATION_TESTS if self.test_checks[n].isChecked()]
        if not tests:
            QMessageBox.information(self, "No tests", "Select at least one test.")
            return

        # safety confirmation — spell out exactly what will move and where
        zone = self.engine.zone_desc()
        warn = ""
        if not self.engine.lim.has_custom_zone:
            warn = ("\n\n⚠ No half-field restriction is set — robots may use the "
                    "WHOLE field. Tick 'Restrict to our half' if you only have "
                    "one side.")
        msg = (f"Auto-calibrate {len(robots)} robot(s), one at a time:\n"
               f"   {', '.join(r.label for r in robots)}\n\n"
               f"Drive zone: {zone}\n"
               f"Tests each: {', '.join(tests)}\n\n"
               "Each robot is kept inside the safe zone and emergency-stopped by "
               "vision. Make sure the field is clear of people." + warn)
        if QMessageBox.question(self, "Start auto-calibration?", msg,
                                QMessageBox.Yes | QMessageBox.No,
                                QMessageBox.Yes) != QMessageBox.Yes:
            return

        # reset result cells
        self._populating = True
        for r in robots:
            row = self._row_by_label.get(r.label)
            if row is not None:
                for c in (C_SPD, C_W, C_DRIFT, C_SPIN, C_LAT):
                    self.tbl.item(row, c).setText("—")
                self.tbl.item(row, C_STAT).setText("queued")
                self.tbl.item(row, C_STAT).setForeground(COL_TEXT_DIM)
        self._populating = False

        self.log.clear()
        preset = _PRESETS.get(self.cmb_preset.currentText(), {})
        self._start_cb({"kind": "calibrate", "robots": robots, "tests": tests,
                        "preset": dict(preset)})

    # ── worker callbacks (forwarded by MainWindow) ───────────────────────
    def append_log(self, msg: str):
        self.log.appendPlainText(msg)

    def set_progress(self, frac: float, text: str = ""):
        self.progress.setValue(int(max(0.0, min(1.0, frac)) * 100))
        if text:
            self.lbl_status.setText(text)

    def on_robot_update(self, payload: dict):
        label = payload.get("label")
        row = self._row_by_label.get(label)
        if row is None:
            return
        status = payload.get("status")
        stat_item = self.tbl.item(row, C_STAT)
        if status == "running":
            stat_item.setText("running…")
            stat_item.setForeground(COL_WARN)
        elif status == "skipped":
            stat_item.setText("SKIPPED — unreachable")
            stat_item.setForeground(COL_CRIT)
        elif status == "done":
            cal = payload.get("calibration", {}) or {}
            self._set(row, C_SPD, cal.get("speed_scale"))
            self._set(row, C_W, cal.get("w_scale"))
            self._set(row, C_DRIFT, cal.get("lateral_drift_per_m"))
            self._set(row, C_SPIN, cal.get("spin_center_drift_mm"))
            self._set(row, C_LAT, cal.get("command_latency_ms"))
            warns = cal.get("warnings", [])
            if warns:
                stat_item.setText(f"done · {len(warns)} warning(s)")
                stat_item.setForeground(COL_WARN)
                stat_item.setToolTip("\n".join(warns))
            else:
                stat_item.setText("✓ done")
                stat_item.setForeground(COL_GOOD)
            # colour the speed-scale cell if far from 1.0 (the unit-bug tell)
            ss = cal.get("speed_scale")
            if ss is not None:
                cell = self.tbl.item(row, C_SPD)
                cell.setForeground(COL_GOOD if 0.85 <= ss <= 1.15 else COL_CRIT)

    def _set(self, row, col, val):
        self._populating = True
        self.tbl.item(row, col).setText("—" if val is None else f"{val}")
        self._populating = False

    def on_done(self, rep: dict):
        self._last_output = rep.get("_output_dir")
        self.btn_open.setEnabled(bool(self._last_output and os.path.isdir(self._last_output)))
        paths = rep.get("_paths", {})
        skipped = rep.get("_skipped", [])
        msg = "Auto-calibration finished."
        if skipped:
            msg += f"  Skipped (unreachable): {', '.join(skipped)}."
        if paths.get("txt"):
            msg += f"  Report: {paths['txt']}"
        self.lbl_status.setText(msg)
        self.progress.setValue(100)

    def set_busy(self, busy: bool):
        self._busy = busy
        for w in (self.cmb_team, self.cmb_side, self.chk_half, self.cmb_preset,
                  self.btn_start, self.btn_all, self.btn_none, self.tbl):
            w.setEnabled(not busy)
        for cb in self.test_checks.values():
            cb.setEnabled(not busy)
        self.field.set_zone_edit_enabled(not busy)
        self.btn_stop.setEnabled(busy)

    def _open_folder(self):
        if self._last_output and os.path.isdir(self._last_output):
            try:
                os.startfile(self._last_output)        # Windows
            except Exception:
                self.lbl_status.setText(f"Output: {self._last_output}")
