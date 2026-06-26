"""
main_window.py — PySide6 dashboard for DiagTool.

Live view of vision/telemetry health and robot positions, one-click buttons to
run each diagnostic on the selected robot or a full sweep over the real robots,
a streaming log, and a calibration/summary panel. All measurement work runs on
a worker thread so the UI never freezes; the worker talks back over Qt signals.
"""

from __future__ import annotations

import threading

from PySide6.QtCore import Qt, QObject, QThread, Signal, Slot, QTimer
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QLabel, QPushButton, QPlainTextEdit, QProgressBar, QTableWidget,
    QTableWidgetItem, QGroupBox, QSplitter, QHeaderView, QMessageBox,
)

from ..engine import Engine
from ..diagnostics import ALL_DIAGNOSTICS
from ..report import render_text
from .field_view import FieldView


_DOT_OK = "color:#46c25a; font-weight:bold;"
_DOT_BAD = "color:#e05050; font-weight:bold;"
_DOT_WARN = "color:#e0b341; font-weight:bold;"


class Worker(QObject):
    sig_log = Signal(str)
    sig_progress = Signal(float, str)
    sig_done = Signal(dict)

    def __init__(self, engine: Engine, job: dict):
        super().__init__()
        self.engine = engine
        self.job = job
        self._stop = threading.Event()

    def stop(self):
        self._stop.set()

    @Slot()
    def run(self):
        log = self.sig_log.emit
        prog = lambda f, t="": self.sig_progress.emit(float(f), str(t))
        try:
            if self.job["kind"] == "test":
                res = self.engine.run_diagnostic(
                    self.job["robot"], self.job["name"],
                    log=log, progress=prog, stop=self._stop.is_set)
                self.sig_done.emit({"kind": "test", "name": self.job["name"],
                                    "robot": self.job["robot"].label, "result": res})
            elif self.job["kind"] == "selftest":
                from ..selftest import run_selftest
                rep = run_selftest(
                    log=log,
                    include_sim=self.job.get("include_sim", True),
                    include_net=self.job.get("include_net", False),
                    stop=self._stop.is_set)
                self.sig_done.emit({"kind": "selftest", "report": rep})
            elif self.job["kind"] == "testall":
                robot = self.job["robot"]
                # 1) wake/prove the link first — stream a heartbeat and listen
                log(f"== Reachability probe :: {robot.label} ==")
                probe = self.engine.probe_robot(robot, seconds=2.0,
                                                move_speed=0.0, log=log)
                # 2) then run the whole battery on this robot
                tests = [d.name for d in ALL_DIAGNOSTICS]
                rep = self.engine.run_sweep(
                    [robot], tests, log=log, progress=prog,
                    stop=self._stop.is_set)
                self.sig_done.emit({"kind": "sweep", "report": rep, "probe": probe})
            else:
                rep = self.engine.run_sweep(
                    self.job["robots"], self.job["tests"],
                    log=log, progress=prog, stop=self._stop.is_set)
                self.sig_done.emit({"kind": "sweep", "report": rep})
        except Exception as e:
            self.sig_log.emit(f"[worker error] {type(e).__name__}: {e}")
            self.sig_done.emit({"kind": "error"})


class MainWindow(QMainWindow):
    def __init__(self, engine: Engine):
        super().__init__()
        self.engine = engine
        self.setWindowTitle("DiagTool — robot diagnostics & calibration")
        self.resize(1180, 760)

        self._thread: QThread | None = None
        self._worker: Worker | None = None
        self._robots = engine.robots()
        self._row_by_label = {}
        self._jog_target = None

        self._build_ui()
        self.engine.start()

        fi = self.engine.field_info()
        self._append_log(
            f"[field] {fi['length_mm']:.0f} x {fi['width_mm']:.0f} mm "
            f"(source: {fi['source']}); arena ±{fi['arena_half_len_mm']:.0f} x "
            f"±{fi['arena_half_wid_mm']:.0f} mm")

        for ip, labels in self.engine.ip_conflicts().items():
            self._append_log(f"[!] CONFIG: {ip} is shared by {', '.join(labels)} "
                             "— commands for one go to whichever robot answers at "
                             "that IP. Fix ipconfig.yaml.")

        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self._refresh_status)
        self._timer.start()

        # auto-stop timer for manual jog pulses
        self._jog_timer = QTimer(self)
        self._jog_timer.setSingleShot(True)
        self._jog_timer.timeout.connect(self._jog_stop)

    # -- UI --
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # top status bar
        bar = QHBoxLayout()
        self.lbl_vision = QLabel("vision: —")
        self.lbl_telem = QLabel("telemetry: —")
        for lb in (self.lbl_vision, self.lbl_telem):
            lb.setFont(QFont("monospace", 10))
        bar.addWidget(self.lbl_vision)
        bar.addSpacing(20)
        bar.addWidget(self.lbl_telem)
        bar.addStretch()
        root.addLayout(bar)

        split = QSplitter(Qt.Horizontal)
        root.addWidget(split, 1)

        # left: robot table + field
        left = QWidget(); ll = QVBoxLayout(left)
        ll.addWidget(QLabel("Robots (from ipconfig):"))
        self.tbl = QTableWidget(0, 5)
        self.tbl.setHorizontalHeaderLabels(["Robot", "IP", "Real", "Vis", "Tel"])
        self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl.setSelectionBehavior(QTableWidget.SelectRows)
        self.tbl.setSelectionMode(QTableWidget.SingleSelection)
        self.tbl.itemSelectionChanged.connect(self._on_select)
        conflicts = self.engine.ip_conflicts()
        for r in self._robots:
            row = self.tbl.rowCount()
            self.tbl.insertRow(row)
            self._row_by_label[r.label] = row
            self.tbl.setItem(row, 0, QTableWidgetItem(r.label))
            ip_item = QTableWidgetItem(
                r.ip + ("  ⚠ DUP" if r.ip in conflicts else ""))
            if r.ip in conflicts:
                ip_item.setToolTip(
                    f"{r.ip} is shared by {', '.join(conflicts[r.ip])} — give "
                    "each robot a unique IP in ipconfig.yaml.")
            self.tbl.setItem(row, 1, ip_item)
            self.tbl.setItem(row, 2, QTableWidgetItem("yes" if r.is_real else "no"))
            self.tbl.setItem(row, 3, QTableWidgetItem("—"))
            self.tbl.setItem(row, 4, QTableWidgetItem("—"))
        ll.addWidget(self.tbl, 1)
        self.field = FieldView(self.engine.lim)
        ll.addWidget(self.field, 1)
        split.addWidget(left)

        # right: controls + results + log
        right = QWidget(); rl = QVBoxLayout(right)

        ctrl = QGroupBox("Run diagnostics (on selected robot)")
        g = QGridLayout(ctrl)
        self._test_btns = []
        for i, d in enumerate(ALL_DIAGNOSTICS):
            b = QPushButton(d.title)
            b.clicked.connect(lambda _=False, name=d.name: self._run_test(name))
            g.addWidget(b, i // 2, i % 2)
            self._test_btns.append(b)
        rl.addWidget(ctrl)

        # -- manual jog: plainly drive the robot a little, to see it move --
        jog = QGroupBox("Manual jog — selected robot · 0.5 s pulse · body frame")
        jg = QGridLayout(jog)
        self._jog_btns = []

        def _jog_btn(label, fx, fy, fw, row, col, tip):
            b = QPushButton(label)
            b.setToolTip(tip)
            b.clicked.connect(
                lambda _=False, a=fx, c=fy, d=fw: self._jog(a, c, d))
            jg.addWidget(b, row, col)
            self._jog_btns.append(b)

        _jog_btn("↑ Forward", 1, 0, 0, 0, 1, "drive forward (body +X)")
        _jog_btn("← Left", 0, 1, 0, 1, 0, "strafe left (body +Y)")
        _jog_btn("↺ Turn", 0, 0, 1, 1, 1, "rotate counter-clockwise")
        _jog_btn("Turn ↻", 0, 0, -1, 1, 2, "rotate clockwise")
        _jog_btn("Right →", 0, -1, 0, 1, 3, "strafe right (body −Y)")
        _jog_btn("↓ Back", -1, 0, 0, 2, 1, "drive backward (body −X)")
        bstop = QPushButton("■ STOP")
        bstop.setStyleSheet("background:#e05050; color:white; font-weight:bold;")
        bstop.clicked.connect(self._jog_stop)
        jg.addWidget(bstop, 2, 3)
        rl.addWidget(jog)

        testall_row = QHBoxLayout()
        self.btn_testall = QPushButton("▶ Probe + Run All Tests (selected robot)")
        self.btn_testall.setStyleSheet("font-weight:bold; padding:6px;")
        self.btn_testall.setToolTip(
            "One click: first stream a heartbeat and check the robot is "
            "reachable (probe), then run the whole diagnostic battery on the "
            "highlighted robot and write a report.")
        self.btn_testall.clicked.connect(self._run_test_all)
        testall_row.addWidget(self.btn_testall, 1)
        rl.addLayout(testall_row)

        sweep_row = QHBoxLayout()
        self.btn_sweep = QPushButton("Run Full Sweep (real robots)")
        self.btn_sweep.clicked.connect(self._run_sweep)
        self.btn_stop = QPushButton("STOP")
        self.btn_stop.setStyleSheet("background:#e05050; color:white; font-weight:bold;")
        self.btn_stop.clicked.connect(self._stop_job)
        self.btn_stop.setEnabled(False)
        sweep_row.addWidget(self.btn_sweep, 1)
        sweep_row.addWidget(self.btn_stop)
        rl.addLayout(sweep_row)

        selftest_row = QHBoxLayout()
        self.btn_selftest = QPushButton("Self-Test (offline · no robots)")
        self.btn_selftest.setToolTip(
            "Run DiagTool's built-in self-test: unit checks of every module "
            "plus a simulated end-to-end sweep against a fake robot.\n"
            "Needs no robots or field network — safe to run from anywhere.")
        self.btn_selftest.clicked.connect(self._run_selftest)
        selftest_row.addWidget(self.btn_selftest, 1)
        rl.addLayout(selftest_row)

        self.progress = QProgressBar()
        self.progress.setRange(0, 100)
        self.lbl_action = QLabel("idle")
        rl.addWidget(self.progress)
        rl.addWidget(self.lbl_action)

        rl.addWidget(QLabel("Summary / calibration:"))
        self.summary = QPlainTextEdit(); self.summary.setReadOnly(True)
        self.summary.setFont(QFont("monospace", 9))
        self.summary.setMaximumHeight(220)
        rl.addWidget(self.summary)

        rl.addWidget(QLabel("Log:"))
        self.log = QPlainTextEdit(); self.log.setReadOnly(True)
        self.log.setFont(QFont("monospace", 9))
        rl.addWidget(self.log, 1)

        split.addWidget(right)
        split.setSizes([440, 740])

    # -- helpers --
    def _selected_robot(self):
        items = self.tbl.selectedItems()
        if not items:
            return None
        label = self.tbl.item(items[0].row(), 0).text()
        return self.engine.find_robot(label)

    def _on_select(self):
        r = self._selected_robot()
        self.field.set_selected(r.label if r else None)

    def _busy(self, busy: bool):
        for b in self._test_btns:
            b.setEnabled(not busy)
        for b in self._jog_btns:
            b.setEnabled(not busy)
        self.btn_testall.setEnabled(not busy)
        self.btn_sweep.setEnabled(not busy)
        self.btn_selftest.setEnabled(not busy)
        self.btn_stop.setEnabled(busy)

    def _append_log(self, msg: str):
        self.log.appendPlainText(msg)

    # -- jobs --
    def _start_worker(self, job: dict):
        if self._thread is not None:
            return
        self._busy(True)
        self.progress.setValue(0)
        self._thread = QThread(self)
        self._worker = Worker(self.engine, job)
        self._worker.moveToThread(self._thread)
        self._thread.started.connect(self._worker.run)
        self._worker.sig_log.connect(self._append_log)
        self._worker.sig_progress.connect(self._on_progress)
        self._worker.sig_done.connect(self._on_done)
        self._thread.start()

    @Slot(float, str)
    def _on_progress(self, frac, text):
        self.progress.setValue(int(max(0.0, min(1.0, frac)) * 100))
        self.lbl_action.setText(text or "")

    @Slot(dict)
    def _on_done(self, payload: dict):
        if self._thread:
            self._thread.quit()
            self._thread.wait(2000)
        self._thread = None
        self._worker = None
        self._busy(False)
        self.lbl_action.setText("done")
        if payload.get("kind") == "sweep":
            rep = payload.get("report", {})
            txt = render_text(rep)
            probe = payload.get("probe")
            if probe:
                txt = (f"PROBE {probe.get('robot')} @ {probe.get('ip')}:"
                       f"{probe.get('port')} — {probe.get('verdict', '')}\n\n" + txt)
            self.summary.setPlainText(txt)
        elif payload.get("kind") == "test":
            self.summary.setPlainText(
                f"{payload['robot']} :: {payload['name']}\n"
                + _short(payload.get("result", {})))
        elif payload.get("kind") == "selftest":
            rep = payload.get("report", {})
            self.summary.setPlainText(rep.get("summary", "(no result)"))
            self.lbl_action.setText(
                f"self-test: {rep.get('passed', 0)} passed, "
                f"{rep.get('failed', 0)} failed, {rep.get('skipped', 0)} skipped")

    def _run_test(self, name: str):
        r = self._selected_robot()
        if r is None:
            QMessageBox.information(self, "Select a robot",
                                    "Pick a robot in the table first.")
            return
        self._start_worker({"kind": "test", "name": name, "robot": r})

    def _run_test_all(self):
        r = self._selected_robot()
        if r is None:
            QMessageBox.information(self, "Select a robot",
                                    "Pick a robot in the table first.")
            return
        self._append_log(f"One-click: probe + full battery on {r.label}...")
        self._start_worker({"kind": "testall", "robot": r})

    # -- manual jog --
    def _jog(self, fx: float, fy: float, fw: float):
        """Plainly drive the selected robot for a short pulse, then auto-stop.

        Direct send, so it moves even when vision is flaky — but while vision
        CAN see the robot it is still arena-braked and cannot leave the field.
        """
        if self._thread is not None:
            return                                   # a test job is running
        r = self._selected_robot()
        if r is None:
            QMessageBox.information(self, "Select a robot",
                                    "Pick a robot in the table first.")
            return
        speed, wspeed, dur_ms = 0.15, 0.5, 400
        vx, vy, w = fx * speed, fy * speed, fw * wspeed
        self.engine.commander.set_velocity(
            r.is_yellow, r.robot_id, vx=vx, vy=vy, w=w, frame="body", safe=False)
        self._jog_target = r
        self._append_log(
            f"jog {r.label}: vx={vx:.2f} vy={vy:.2f} m/s  w={w:.2f} rad/s  ({dur_ms} ms)")
        self._jog_timer.start(dur_ms)

    def _jog_stop(self):
        self._jog_timer.stop()
        r = self._jog_target or self._selected_robot()
        if r is not None and self.engine.commander is not None:
            self.engine.commander.stop_robot(r.is_yellow, r.robot_id)

    def _run_sweep(self):
        robots = self.engine.robots(real_only=True)
        if not robots:
            r = self._selected_robot()
            if r is None:
                QMessageBox.information(
                    self, "No robots",
                    "No real robots in ipconfig and none selected.")
                return
            robots = [r]
        tests = [d.name for d in ALL_DIAGNOSTICS]
        self._start_worker({"kind": "sweep", "robots": robots, "tests": tests})

    def _run_selftest(self):
        self.log.clear()
        self._append_log("Running offline self-test (no robots needed)...")
        self.summary.setPlainText("")
        self._start_worker({"kind": "selftest",
                            "include_sim": True, "include_net": False})

    def _stop_job(self):
        if self._worker:
            self._worker.stop()
            self._append_log("[stop requested]")

    # -- live status --
    def _refresh_status(self):
        # pick up the real field size as soon as SSL-Vision sends geometry
        if self.engine.refresh_field_geometry():
            self.field.set_limits(self.engine.lim)
            fi = self.engine.field_info()
            self._append_log(
                f"[field] updated to {fi['length_mm']:.0f} x {fi['width_mm']:.0f} "
                f"mm (source: {fi['source']}); arena ±{fi['arena_half_len_mm']:.0f} "
                f"x ±{fi['arena_half_wid_mm']:.0f} mm")
        st = self.engine.source_status()
        v, t = st["vision"], st["telemetry"]
        if v.get("error"):
            self.lbl_vision.setText(f"vision: ERROR — {v['error'][:60]}")
            self.lbl_vision.setStyleSheet(_DOT_BAD)
        else:
            fps = v.get("fps") or 0
            self.lbl_vision.setText(
                f"vision: {fps:.1f} fps  visible={v.get('visible')}  "
                f"drops={v.get('drops')}")
            self.lbl_vision.setStyleSheet(_DOT_OK if fps > 5 else _DOT_WARN)
        if t.get("error"):
            self.lbl_telem.setText(f"telemetry: ERROR — {t['error'][:60]}")
            self.lbl_telem.setStyleSheet(_DOT_BAD)
        else:
            rate = t.get("rate_hz") or 0
            self.lbl_telem.setText(
                f"telemetry: {rate:.2f} Hz  packets={t.get('packets')}")
            self.lbl_telem.setStyleSheet(_DOT_OK if rate >= 5 else _DOT_WARN)

        # field poses + table indicators
        poses = {}
        for (is_y, rid) in self.engine.vision.visible_robots():
            s = self.engine.vision.get_pose_sample(is_y, rid, max_age=0.5)
            if s:
                poses[f"{'Y' if is_y else 'B'}{rid}"] = (s.x, s.y, s.o, is_y)
        self.field.set_poses(poses)

        for r in self._robots:
            row = self._row_by_label[r.label]
            seen_v = r.label in poses
            self.tbl.item(row, 3).setText("●" if seen_v else "—")
            rs = self.engine.telemetry.robot_status(r.is_yellow, r.robot_id)
            self.tbl.item(row, 4).setText("●" if rs.get("seen") else "—")

    def closeEvent(self, ev):
        self._timer.stop()
        self._jog_timer.stop()
        if self._worker:
            self._worker.stop()
        if self._thread:
            self._thread.quit(); self._thread.wait(2000)
        self.engine.stop()
        super().closeEvent(ev)


def _short(result: dict) -> str:
    import json
    return json.dumps(result, indent=2, default=str)


def launch(engine: Engine | None = None) -> int:
    app = QApplication.instance() or QApplication([])
    eng = engine or Engine()
    win = MainWindow(eng)
    win.show()
    return app.exec()
