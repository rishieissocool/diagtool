"""
setup_tab.py — edit robot addresses live.

An editable table of every robot's IP, port and target (real RobotFramework vs
grSim). Changes apply immediately to the running command streamer — no restart —
so you can fix an IP mid-session and the next command goes to the new address.
You can also reload ipconfig.yaml from disk, or save the current table back to it.
"""

from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QTableWidget,
    QTableWidgetItem, QComboBox, QHeaderView, QMessageBox,
)

from .widgets import COL_TEXT, COL_TEXT_DIM

_COLS = ["Robot", "Team", "IP address", "Port", "Target", "Type"]


class SetupTab(QWidget):
    def __init__(self, engine, on_changed=None, parent=None):
        super().__init__(parent)
        self.engine = engine
        self._on_changed = on_changed or (lambda: None)
        self._combos: list[QComboBox] = []

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        head = QHBoxLayout()
        title = QLabel("Setup")
        title.setStyleSheet(f"color:{COL_TEXT.name()}; font-size:20px; font-weight:bold;")
        hint = QLabel("edit IP / port / target, then Apply — changes take effect "
                      "immediately on the running command stream")
        hint.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
        head.addWidget(title)
        head.addSpacing(10)
        head.addWidget(hint)
        head.addStretch()
        root.addLayout(head)

        self.tbl = QTableWidget(0, len(_COLS))
        self.tbl.setHorizontalHeaderLabels(_COLS)
        self.tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.tbl.verticalHeader().setVisible(False)
        root.addWidget(self.tbl, 1)

        btns = QHBoxLayout()
        self.btn_apply = QPushButton("Apply changes")
        self.btn_apply.setStyleSheet("font-weight:bold;")
        self.btn_apply.clicked.connect(self._apply)
        self.btn_reload = QPushButton("Reload from file")
        self.btn_reload.clicked.connect(self._reload)
        self.btn_save = QPushButton("Save to file")
        self.btn_save.clicked.connect(self._save)
        btns.addWidget(self.btn_apply)
        btns.addWidget(self.btn_reload)
        btns.addWidget(self.btn_save)
        btns.addStretch()
        self.lbl_grsim = QLabel("")
        self.lbl_grsim.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
        btns.addWidget(self.lbl_grsim)
        root.addLayout(btns)

        self.status = QLabel("")
        self.status.setStyleSheet(f"color:{COL_TEXT_DIM.name()}; font-size:11px;")
        root.addWidget(self.status)

        self.populate()

    # -- table <-> inventory --
    def populate(self):
        robots = self.engine.robots()
        self.tbl.setRowCount(0)
        self._combos = []
        for r in robots:
            row = self.tbl.rowCount()
            self.tbl.insertRow(row)

            def _ro(text):
                it = QTableWidgetItem(text)
                it.setFlags(it.flags() & ~Qt.ItemIsEditable)
                return it

            self.tbl.setItem(row, 0, _ro(r.label))
            self.tbl.setItem(row, 1, _ro("Yellow" if r.is_yellow else "Blue"))
            self.tbl.setItem(row, 2, QTableWidgetItem(r.ip))     # editable
            self.tbl.setItem(row, 3, QTableWidgetItem(str(r.port)))  # editable
            combo = QComboBox()
            combo.addItems(["real", "grSim"])
            combo.setCurrentIndex(1 if r.mode == "grsim" else 0)
            self.tbl.setCellWidget(row, 4, combo)
            self._combos.append(combo)
            self.tbl.setItem(row, 5, _ro("real LAN" if r.is_real else "loopback"))

        addr = self.engine.grsim_addr
        self.lbl_grsim.setText(
            f"grSim address: {addr[0]}:{addr[1]}" if addr else "grSim address: (none)")

    def _apply(self):
        applied, errors = 0, []
        for row in range(self.tbl.rowCount()):
            label = self.tbl.item(row, 0).text()
            ip = self.tbl.item(row, 2).text().strip()
            port_txt = self.tbl.item(row, 3).text().strip()
            mode = "grsim" if self._combos[row].currentText() == "grSim" else "real"
            try:
                port = int(port_txt)
                if not 0 < port < 65536:
                    raise ValueError("port out of range")
            except ValueError:
                errors.append(f"{label}: bad port '{port_txt}'")
                continue
            if not ip:
                errors.append(f"{label}: empty IP")
                continue
            self.engine.set_robot_target(label, ip=ip, port=port, mode=mode)
            applied += 1
        self.populate()
        self._on_changed()
        msg = f"Applied {applied} robot(s)."
        if errors:
            msg += "  Skipped: " + "; ".join(errors)
        self.status.setText(msg)

    def _reload(self):
        try:
            summary = self.engine.reload_config()
        except Exception as e:
            QMessageBox.warning(self, "Reload failed", str(e))
            return
        self.populate()
        self._on_changed()
        parts = []
        for k in ("changed", "added", "removed"):
            if summary.get(k):
                parts.append(f"{k}: {', '.join(summary[k])}")
        extra = ("  (restart to rebuild tabs for added/removed robots)"
                 if summary.get("added") or summary.get("removed") else "")
        self.status.setText("Reloaded ipconfig.yaml. "
                            + ("  ".join(parts) or "no address changes.") + extra)

    def _save(self):
        try:
            path = self.engine.save_config()
        except Exception as e:
            QMessageBox.warning(self, "Save failed", str(e))
            return
        self.status.setText(f"Saved current addresses to {path}")
