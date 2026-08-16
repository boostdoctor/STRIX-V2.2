"""Full-screen car-style dashboard (V1 layout, Qt gauges)."""
from __future__ import annotations

from PySide6.QtCore import Qt, QTimer, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QConicalGradient
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QWidget, QPushButton, QSizePolicy,
)


class _Gauge(QWidget):
    def __init__(self, title: str, unit: str = "", vmin=0, vmax=100, hero=False, parent=None):
        super().__init__(parent)
        self.title = title
        self.unit = unit
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
        # bezel
        p.setPen(QPen(QColor("#2a3a55"), 3))
        p.setBrush(QColor("#0d121a"))
        p.drawEllipse(QRectF(cx - r, cy - r, 2 * r, 2 * r))
        # arc track
        p.setPen(QPen(QColor("#1e2a3c"), 10))
        p.drawArc(QRectF(cx - r + 12, cy - r + 12, 2 * r - 24, 2 * r - 24), 225 * 16, -270 * 16)
        # value arc
        t = 0.0 if self.vmax <= self.vmin else (self.value - self.vmin) / (self.vmax - self.vmin)
        t = max(0.0, min(1.0, t))
        col = QColor("#44ff88") if t < 0.75 else (QColor("#ffcc44") if t < 0.9 else QColor("#ff5566"))
        p.setPen(QPen(col, 10, Qt.SolidLine, Qt.RoundCap))
        p.drawArc(QRectF(cx - r + 12, cy - r + 12, 2 * r - 24, 2 * r - 24), 225 * 16, int(-270 * 16 * t))
        # readout
        p.setPen(QColor("#f0f4ff"))
        f = QFont("Segoe UI", 28 if self.hero else 18, QFont.Bold)
        p.setFont(f)
        if self.hero:
            txt = f"{int(self.value)}"
        elif self.title in ("MAP",):
            txt = f"{self.value:.0f}"
        elif self.title in ("AFR", "INJ"):
            txt = f"{self.value:.1f}"
        else:
            txt = f"{self.value:.0f}"
        p.drawText(QRectF(0, cy - 18, w, 36), Qt.AlignCenter, txt)
        p.setPen(QColor("#6a7a90"))
        p.setFont(QFont("Segoe UI", 10, QFont.Bold))
        p.drawText(QRectF(0, cy + 20, w, 18), Qt.AlignCenter, self.title + (f" {self.unit}" if self.unit else ""))


class DashboardDialog(QDialog):
    """V1-style order: RPM (large) → TPS → MAP → IGN → AFR → INJ → VVT IN → VVT EX."""

    def __init__(self, live_getter, parent=None):
        super().__init__(parent)
        self._live = live_getter
        self.setWindowTitle("STRIX Dashboard")
        self.setWindowState(Qt.WindowFullScreen)
        self.setStyleSheet("background:#080b10; color:#d0d8e8;")
        root = QVBoxLayout(self)
        root.setContentsMargins(16, 12, 16, 12)
        top = QHBoxLayout()
        title = QLabel("STRIX  ·  LIVE DASH")
        title.setStyleSheet("font-size:14px;font-weight:700;letter-spacing:3px;color:#5a7aaa;")
        top.addWidget(title)
        top.addStretch(1)
        self.sync = QLabel("SYNC —")
        self.sync.setStyleSheet("font-weight:700;")
        top.addWidget(self.sync)
        btn = QPushButton("Close (Esc)")
        btn.clicked.connect(self.close)
        top.addWidget(btn)
        root.addLayout(top)

        grid = QGridLayout()
        grid.setSpacing(12)
        self.g_rpm = _Gauge("RPM", "", 0, 8000, hero=True)
        self.g_tps = _Gauge("TPS", "%", 0, 100)
        self.g_map = _Gauge("MAP", "kPa", 0, 250)
        self.g_ign = _Gauge("IGN", "°", -10, 45)
        self.g_afr = _Gauge("AFR", "", 10, 20)
        self.g_inj = _Gauge("INJ", "ms", 0, 15)
        self.g_v1 = _Gauge("VVT IN", "°", 0, 50)
        self.g_v2 = _Gauge("VVT EX", "°", 0, 50)
        grid.addWidget(self.g_rpm, 0, 0, 2, 1)
        grid.addWidget(self.g_tps, 0, 1)
        grid.addWidget(self.g_map, 0, 2)
        grid.addWidget(self.g_ign, 0, 3)
        grid.addWidget(self.g_afr, 1, 1)
        grid.addWidget(self.g_inj, 1, 2)
        grid.addWidget(self.g_v1, 1, 3)
        grid.addWidget(self.g_v2, 1, 4)
        # keep rpm spanning; place vvt ex
        grid.addWidget(self.g_v2, 0, 4, 2, 1)
        root.addLayout(grid, 1)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._timer.start(50)

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Escape:
            self.close()
        else:
            super().keyPressEvent(e)

    def _tick(self):
        live = self._live() if callable(self._live) else self._live
        rpm = float(live.get("rpm") or 0)
        self.g_rpm.set_value(rpm)
        self.g_tps.set_value(float(live.get("tps") or 0))
        self.g_map.set_value(float(live.get("map") or 0))
        self.g_ign.set_value(float(live.get("ign") or 0))
        afr = float(live.get("afr") or 0)
        if afr <= 0:
            o2 = float(live.get("o2") or 0)
            afr = (14.7 if o2 < 0.01 else max(10.0, min(22.0, 18 - (o2 / 0.9) * 6)))
        self.g_afr.set_value(afr)
        inj = float(live.get("pw") or live.get("inj") or 0)
        if inj > 100:  # µs → ms
            inj /= 1000.0
        self.g_inj.set_value(inj)
        self.g_v1.set_value(float(live.get("vvt1") or live.get("cam1") or 0))
        self.g_v2.set_value(float(live.get("vvt2") or live.get("cam2") or 0))
        sync = int(live.get("sync") or 0)
        self.sync.setText("SYNC LOCK" if sync else "SYNC —")
        self.sync.setStyleSheet("font-weight:700;color:#44ff88;" if sync else "font-weight:700;color:#ff6666;")
