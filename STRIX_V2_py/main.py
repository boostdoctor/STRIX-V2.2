#!/usr/bin/env python3
"""STRIX V2 entry point."""
import sys
from pathlib import Path

# Allow running from package root without install
sys.path.insert(0, Path(__file__).resolve().parent.as_posix())

from PySide6.QtWidgets import QApplication
from strix_v2.constants import DARK_STYLE
from strix_v2.main_window import MainWindow


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(DARK_STYLE)
    app.setApplicationName("STRIX V2")
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
