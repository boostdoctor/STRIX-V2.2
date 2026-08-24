"""Dashboard driven by JSON layout files (MegaTunix dash-style)."""
from __future__ import annotations

import json
from pathlib import Path

from PySide6.QtCore import Qt, QTimer, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QFont
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QWidget,
    QPushButton, QSizePolicy, QComboBox, QFileDialog,
)


def dashboard_dir() -> Path:
    return Path(__file__).resolve().parent.parent / "dashboards"


def list_layouts() -> list[Path]:
    d = dashboard_dir()
    if not d.is_dir():
        return []
    return sorted(d.glob("*.json"))


def load_layout(path: str | Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


class _Gauge(QWidget):
    def __init__(self, title: str, unit: str = "", vmin=0, vmax=100, hero=False, key="rpm", parent=None):
        super().__init__(parent)
        self.title = title
        self.unit = unit
        self.key = key
        self.vmin = float(vmin)
        self.vmax = float(vmax)
        self.value = 0.0
        self.hero = hero
        self.setMinimumSize(200 if hero else 140, 200 if hero else 140)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_value(self, v: float):
        self.value = float(v)
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        side = min(w, h) - 12
        cx, cy = w / 2, h / 2 + 4
        r = side / 2
        p.setPen(QPen(QColor("#2a3a55"), 3))
        p.setBrush(QColor("#0d121a"))
        p.drawEllipse(QRectF(cx - r, cy - r, 2 * r, 2 * r))
        p.setPen(QPen(QColor("#1e2a3c"), 10))
        p.drawArc(QRectF(cx - r + 12, cy - r + 12, 2 * r - 24, 2 * r - 24), 225 * 16, -270 * 16)
        t = 0.0 if self.vmax <= self.vmin else (self.value - self.vmin) / (self.vmax - self.vmin)
        t = max(0.0, min(1.0, t))
        col = QColor("#44ff88") if t < 0.75 else (QColor("#ffcc44") if t < 0.9 else QColor("#ff5566"))
        p.setPen(QPen(col, 10, Qt.SolidLine, Qt.RoundCap))
        p.drawArc(QRectF(cx - r + 12, cy - r + 12, 2 * r - 24, 2 * r - 24), 225 * 16, int(-270 * 16 * t))
        p.setPen(QColor("#f0f4ff"))
        f = QFont("Segoe UI", 28 if self.hero else 18, QFont.Bold)
        p.setFont(f)
        if self.hero or self.title in ("RPM",):
            txt = f"{int(self.value)}"
        elif self.title in ("AFR", "INJ", "LAM", "BAT"):
            txt = f"{self.value:.1f}"
        else:
            txt = f"{self.value:.0f}"
        p.drawText(QRectF(0, cy - 18, w, 36), Qt.AlignCenter, txt)
        p.setPen(QColor("#6a7a90"))
        p.setFont(QFont("Segoe UI", 10, QFont.Bold))
        p.drawText(QRectF(0, cy + 20, w, 18), Qt.AlignCenter, self.title + (f" {self.unit}" if self.unit else ""))


class DashboardDialog(QDialog):
    def __init__(self, live_getter, parent=None, layout_path: str | Path | None = None):
        super().__init__(parent)
        self._live = live_getter
        self.setWindowTitle("STRIX Dashboard")
        self.setWindowState(Qt.WindowFullScreen)
        self.setStyleSheet("background:#080b10; color:#d0d8e8;")
        self._gauges: list[_Gauge] = []
        self._grid = None

        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 12)
        top = QHBoxLayout()
        title = QLabel("STRIX  ·  LIVE DASH")
        title.setStyleSheet("font-size:14px;font-weight:700;letter-spacing:3px;color:#5a7aaa;")
        top.addWidget(title)
        top.addWidget(QLabel("Layout"))
        self.combo = QComboBox()
        for p in list_layouts():
            self.combo.addItem(p.stem, str(p))
        self.combo.currentIndexChanged.connect(self._combo_layout)
        top.addWidget(self.combo)
        btn_open = QPushButton("Open JSON")
        btn_open.clicked.connect(self._open_json)
        top.addWidget(btn_open)
        top.addStretch(1)
        self.sync = QLabel("SYNC —")
        self.sync.setStyleSheet("font-weight:700;")
        top.addWidget(self.sync)
        btn = QPushButton("Close (Esc)")
        btn.clicked.connect(self.close)
        top.addWidget(btn)
        root.addLayout(top)

        self._grid_host = QWidget()
        self._grid = QGridLayout(self._grid_host)
        self._grid.setSpacing(12)
        root.addWidget(self._grid_host, 1)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

        if layout_path:
            self.apply_layout_file(layout_path)
        elif self.combo.count():
            self.apply_layout_file(self.combo.currentData())
        else:
            self.apply_layout({
                "name": "Default",
                "gauges": [
                    {"key": "rpm", "title": "RPM", "min": 0, "max": 8000, "hero": True, "row": 0, "col": 0, "rowspan": 2},
                    {"key": "tps", "title": "TPS", "unit": "%", "min": 0, "max": 100, "row": 0, "col": 1},
                    {"key": "map", "title": "MAP", "unit": "kPa", "min": 0, "max": 250, "row": 0, "col": 2},
                    {"key": "afr", "title": "AFR", "min": 10, "max": 20, "row": 1, "col": 1},
                    {"key": "ign", "title": "IGN", "unit": "°", "min": -10, "max": 45, "row": 1, "col": 2},
                ],
            })

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Escape:
            self.close()
        else:
            super().keyPressEvent(e)

    def _combo_layout(self, *_):
        p = self.combo.currentData()
        if p:
            self.apply_layout_file(p)

    def _open_json(self):
        path, _ = QFileDialog.getOpenFileName(self, "Dashboard layout", "", "JSON (*.json)")
        if path:
            self.apply_layout_file(path)

    def apply_layout_file(self, path: str | Path):
        try:
            self.apply_layout(load_layout(path))
        except Exception as e:
            self.sync.setText(f"layout error: {e}")

    def apply_layout(self, layout: dict):
        while self._grid.count():
            item = self._grid.takeAt(0)
            w = item.widget()
            if w:
                w.setParent(None)
        self._gauges = []
        for spec in layout.get("gauges") or []:
            g = _Gauge(
                spec.get("title") or spec.get("key", ""),
                spec.get("unit") or "",
                spec.get("min", 0),
                spec.get("max", 100),
                hero=bool(spec.get("hero")),
                key=spec.get("key") or "rpm",
            )
            r = int(spec.get("row", 0))
            c = int(spec.get("col", 0))
            rs = int(spec.get("rowspan", 1))
            cs = int(spec.get("colspan", 1))
            self._grid.addWidget(g, r, c, rs, cs)
            self._gauges.append(g)
        name = layout.get("name") or "dash"
        self.setWindowTitle(f"STRIX Dashboard — {name}")

    def _tick(self):
        live = self._live() if callable(self._live) else self._live
        for g in self._gauges:
            v = float(live.get(g.key) or 0)
            if g.key in ("pw", "inj") and v > 80:
                v *= 0.001
            if g.key == "afr" and v <= 0:
                o2 = float(live.get("o2") or 0)
                v = (14.7 if o2 < 0.01 else max(10.0, min(22.0, 18 - (o2 / 0.9) * 6)))
            g.set_value(v)
        sync = int(live.get("sync") or 0)
        self.sync.setText("SYNC LOCK" if sync else "SYNC —")
        self.sync.setStyleSheet(
            "font-weight:700;color:#44ff88;" if sync else "font-weight:700;color:#ff6666;"
        )
