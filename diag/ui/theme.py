"""
theme.py — a single dark stylesheet for the DiagTool dashboard.

Applied once to the QApplication so the new Competition/Robots tabs (which draw
on a dark background) and the existing Diagnostics tab share one look. Kept
deliberately mild on inputs/tables so the dense diagnostics text stays readable.
"""

DARK_QSS = """
QWidget { background:#0f141b; color:#e6edf3; font-size:12px; }
QMainWindow, QScrollArea { background:#0f141b; }

QTabWidget::pane { border:1px solid #2b3647; border-radius:6px; top:-1px; }
QTabBar::tab {
    background:#1b2330; color:#9aa7b4; padding:8px 18px; margin-right:2px;
    border-top-left-radius:6px; border-top-right-radius:6px;
    font-size:13px; font-weight:bold;
}
QTabBar::tab:selected { background:#243042; color:#e6edf3; }
QTabBar::tab:hover { color:#e6edf3; }

QGroupBox {
    border:1px solid #2b3647; border-radius:8px; margin-top:10px;
    padding-top:6px; font-weight:bold; color:#c7d2dc;
}
QGroupBox::title { subcontrol-origin:margin; left:10px; padding:0 4px; }

QPushButton {
    background:#243042; color:#e6edf3; border:1px solid #34465c;
    border-radius:6px; padding:6px 10px;
}
QPushButton:hover { background:#2d3b4f; border-color:#4a6079; }
QPushButton:pressed { background:#1c2734; }
QPushButton:disabled { background:#161d27; color:#5b6672; border-color:#252f3d; }

QTableWidget {
    background:#131a23; alternate-background-color:#161d27; gridline-color:#2b3647;
    border:1px solid #2b3647; border-radius:6px; selection-background-color:#2f70c2;
    selection-color:#ffffff;
}
QHeaderView::section {
    background:#1b2330; color:#9aa7b4; border:none; border-bottom:1px solid #2b3647;
    padding:4px;
}

QPlainTextEdit {
    background:#0c1117; color:#cdd6df; border:1px solid #2b3647; border-radius:6px;
    selection-background-color:#2f70c2;
}

QProgressBar {
    background:#131a23; border:1px solid #2b3647; border-radius:6px; text-align:center;
    color:#e6edf3;
}
QProgressBar::chunk { background:#3a78c2; border-radius:5px; }

QScrollBar:vertical { background:#0f141b; width:11px; margin:0; }
QScrollBar::handle:vertical { background:#34465c; border-radius:5px; min-height:24px; }
QScrollBar::handle:vertical:hover { background:#4a6079; }
QScrollBar::add-line, QScrollBar::sub-line { height:0; }
QScrollBar:horizontal { background:#0f141b; height:11px; margin:0; }
QScrollBar::handle:horizontal { background:#34465c; border-radius:5px; min-width:24px; }

QToolTip { background:#1b2330; color:#e6edf3; border:1px solid #34465c; }
"""
