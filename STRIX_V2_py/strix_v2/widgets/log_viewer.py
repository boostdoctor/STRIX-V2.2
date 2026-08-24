"""CSV datalog viewer — channels, cursor, simple overlay plot."""
from __future__ import annotations

import csv
from pathlib import Path

from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QFont
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QFileDialog,
    QListWidget, QListWidgetItem, QSlider, QSizePolicy, QAbstractItemView,
)

from strix_v2.datalog import DEFAULT_FIELDS


_PALETTE = (
    "#4aa3ff", "#44ff88", "#ffcc44", "#ff6688",
    "#c080ff", "#80e0ff", "#ffaa66", "#a0ffc0",
)


class _Plot(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.t: list[float] = []
        self.series: dict[str, list[float]] = {}
        self.visible: list[str] = []
        self.cursor = 0
        self.setMinimumHeight(220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_data(self, t, series, visible):
        self.t = t
        self.series = series
        self.visible = list(visible)
        if self.cursor >= len(self.t):
            self.cursor = max(0, len(self.t) - 1)
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#0c1018"))
        if len(self.t) < 2 or not self.visible:
            p.setPen(QColor("#667"))
            p.drawText(self.rect(), Qt.AlignCenter, "Open a CSV log")
            return
        l, t0, r, b = 48, 12, self.width() - 12, self.height() - 24
        tmin, tmax = self.t[0], self.t[-1]
        span = max(1e-6, tmax - tmin)
        p.setPen(QPen(QColor("#1e2a3c"), 1))
        for i in range(5):
            y = t0 + i * (b - t0) / 4
            p.drawLine(int(l), int(y), int(r), int(y))

        for si, key in enumerate(self.visible):
            ys = self.series.get(key) or []
            if len(ys) < 2:
                continue
            ymin, ymax = min(ys), max(ys)
            if ymax <= ymin:
                ymax = ymin + 1
            col = QColor(_PALETTE[si % len(_PALETTE)])
            p.setPen(QPen(col, 1.6))
            last = None
            step = max(1, len(ys) // max(1, int(r - l)))
            for i in range(0, min(len(ys), len(self.t)), step):
                x = l + (self.t[i] - tmin) / span * (r - l)
                y = b - (ys[i] - ymin) / (ymax - ymin) * (b - t0)
                pt = (x, y)
                if last:
                    p.drawLine(int(last[0]), int(last[1]), int(pt[0]), int(pt[1]))
                last = pt
            p.setFont(QFont("Segoe UI", 8))
            p.drawText(int(l + 4 + si * 70), 12, key)

        if 0 <= self.cursor < len(self.t):
            x = l + (self.t[self.cursor] - tmin) / span * (r - l)
            p.setPen(QPen(QColor("#ffe080"), 1, Qt.DashLine))
            p.drawLine(int(x), t0, int(x), b)
            p.setPen(QColor("#c8d4e8"))
            p.setFont(QFont("Segoe UI", 8))
            p.drawText(QRectF(l, b + 2, r - l, 18), Qt.AlignLeft,
                       f"t = {self.t[self.cursor]:.2f}s   n = {self.cursor + 1}/{len(self.t)}")


class LogViewer(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.path: Path | None = None
        self.t: list[float] = []
        self.series: dict[str, list[float]] = {}
        self.fields: list[str] = []

        root = QVBoxLayout(self)
        top = QHBoxLayout()
        self.btn_open = QPushButton("Open CSV")
        self.btn_open.clicked.connect(self.open_file)
        self.lbl = QLabel("No log loaded")
        self.lbl.setStyleSheet("color:#9ab;")
        top.addWidget(self.btn_open)
        top.addWidget(self.lbl, 1)
        root.addLayout(top)

        body = QHBoxLayout()
        self.chan = QListWidget()
        self.chan.setSelectionMode(QAbstractItemView.MultiSelection)
        self.chan.setMaximumWidth(140)
        self.chan.itemSelectionChanged.connect(self._refresh_plot)
        body.addWidget(self.chan)
        right = QVBoxLayout()
        self.plot = _Plot()
        right.addWidget(self.plot, 1)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.valueChanged.connect(self._cursor)
        right.addWidget(self.slider)
        self.readout = QLabel("")
        self.readout.setStyleSheet("font-family:Consolas,monospace;color:#c8d4e8;")
        right.addWidget(self.readout)
        body.addLayout(right, 1)
        root.addLayout(body, 1)

    def open_file(self, path: str | None = None):
        if not path:
            path, _ = QFileDialog.getOpenFileName(self, "Open log", "", "CSV (*.csv)")
        if not path:
            return
        self.load_csv(path)

    def load_csv(self, path: str | Path):
        path = Path(path)
        with path.open(newline="", encoding="utf-8") as f:
            rows = list(csv.reader(f))
        if not rows:
            return
        header = [h.strip() for h in rows[0]]
        t_idx = 0
        for i, h in enumerate(header):
            if h.lower() in ("t_s", "t", "time", "time_s"):
                t_idx = i
                break
        fields = [h for i, h in enumerate(header) if i != t_idx]
        series = {k: [] for k in fields}
        t: list[float] = []
        for row in rows[1:]:
            if not row:
                continue
            try:
                t.append(float(row[t_idx]))
            except (ValueError, IndexError):
                continue
            for i, h in enumerate(header):
                if i == t_idx:
                    continue
                try:
                    series[h].append(float(row[i]) if i < len(row) and row[i] != "" else 0.0)
                except ValueError:
                    series[h].append(0.0)
        self.path = path
        self.t = t
        self.series = series
        self.fields = fields
        self.lbl.setText(f"{path.name}  ·  {len(t)} samples")
        self.chan.blockSignals(True)
        self.chan.clear()
        prefer = [k for k in ("rpm", "map", "tps", "afr", "ect", "ign", "pw") if k in fields]
        for k in fields:
            it = QListWidgetItem(k)
            self.chan.addItem(it)
            if k in prefer:
                it.setSelected(True)
        if not prefer and self.chan.count():
            self.chan.item(0).setSelected(True)
        self.chan.blockSignals(False)
        self.slider.setRange(0, max(0, len(t) - 1))
        self.slider.setValue(0)
        self._refresh_plot()

    def _visible(self) -> list[str]:
        return [it.text() for it in self.chan.selectedItems()]

    def _refresh_plot(self):
        self.plot.set_data(self.t, self.series, self._visible())
        self._cursor(self.slider.value())

    def _cursor(self, i: int):
        self.plot.cursor = i
        self.plot.update()
        if not self.t or i < 0 or i >= len(self.t):
            self.readout.setText("")
            return
        bits = [f"t={self.t[i]:.2f}s"]
        for k in self._visible() or DEFAULT_FIELDS:
            ys = self.series.get(k)
            if ys and i < len(ys):
                bits.append(f"{k}={ys[i]:.2f}")
        self.readout.setText("  ".join(bits))
