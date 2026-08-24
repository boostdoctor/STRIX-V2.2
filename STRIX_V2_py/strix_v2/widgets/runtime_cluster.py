"""Runtime bar cluster + status lamps (MegaTunix runtime page)."""
from __future__ import annotations

from PySide6.QtCore import Qt, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QFont
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QLabel, QFrame, QSizePolicy,
)


class _Bar(QWidget):
    def __init__(self, title, unit, vmin, vmax, parent=None):
        super().__init__(parent)
        self.title = title
        self.unit = unit
        self.vmin = float(vmin)
        self.vmax = float(vmax)
        self.value = 0.0
        self.setMinimumHeight(22)
        self.setMinimumWidth(160)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

    def set_value(self, v: float):
        self.value = float(v)
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(self.rect(), QColor("#0e141c"))
        p.setPen(QColor("#6a7a90"))
        p.setFont(QFont("Segoe UI", 8, QFont.Bold))
        p.drawText(QRectF(4, 1, 70, 12), Qt.AlignLeft, self.title)
        t = 0.0 if self.vmax <= self.vmin else (self.value - self.vmin) / (self.vmax - self.vmin)
        t = max(0.0, min(1.0, t))
        track = QRectF(76, 5, w - 150, h - 10)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor("#1a2433"))
        p.drawRoundedRect(track, 3, 3)
        fill = QRectF(track.x(), track.y(), track.width() * t, track.height())
        col = QColor("#3d9cff") if t < 0.8 else (QColor("#ffcc44") if t < 0.92 else QColor("#ff5566"))
        p.setBrush(col)
        p.drawRoundedRect(fill, 3, 3)
        p.setPen(QColor("#e8f0ff"))
        p.setFont(QFont("Consolas", 10, QFont.Bold))
        txt = f"{self.value:.0f}{self.unit}" if abs(self.value) >= 20 else f"{self.value:.1f}{self.unit}"
        p.drawText(QRectF(w - 70, 1, 66, h - 2), Qt.AlignVCenter | Qt.AlignRight, txt)


class _Lamp(QFrame):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self.on = False
        self.warn = False
        self.setFixedSize(86, 36)
        self.setFrameShape(QFrame.NoFrame)

    def set_state(self, on: bool, warn: bool = False):
        self.on = bool(on)
        self.warn = bool(warn)
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        if self.warn:
            bg, fg, ring = "#5a2218", "#ff8866", "#ff5566"
        elif self.on:
            bg, fg, ring = "#163524", "#44ff88", "#33cc66"
        else:
            bg, fg, ring = "#161b24", "#556070", "#2a3548"
        p.setBrush(QColor(bg))
        p.setPen(QPen(QColor(ring), 1))
        p.drawRoundedRect(self.rect().adjusted(1, 1, -1, -1), 6, 6)
        p.setPen(QColor(fg))
        p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        p.drawText(self.rect(), Qt.AlignCenter, self.title)


LAMPS = (
    ("sync", "SYNC"),
    ("cam", "CAM"),
    ("fp", "PUMP"),
    ("fan", "FAN"),
    ("ase", "ASE"),
    ("dfco", "DFCO"),
    ("cut", "REV CUT"),
)


BARS = (
    ("rpm", "RPM", "", 0, 8000),
    ("map", "MAP", " kPa", 0, 250),
    ("tps", "TPS", " %", 0, 100),
    ("load", "LOAD", "", 0, 120),
    ("ect", "ECT", " °C", -20, 120),
    ("iat", "IAT", " °C", -20, 80),
    ("afr", "AFR", "", 10, 20),
    ("bat", "BAT", " V", 8, 16),
    ("ign", "IGN", " °", -10, 45),
    ("pw", "PW", " ms", 0, 15),
)


class RuntimeCluster(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        root = QHBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        left = QVBoxLayout()
        hdr = QLabel("RUNTIME")
        hdr.setStyleSheet("color:#5a7aaa;letter-spacing:2px;font-weight:700;")
        left.addWidget(hdr)
        self.bars: dict[str, _Bar] = {}
        for key, title, unit, vmin, vmax in BARS:
            b = _Bar(title, unit, vmin, vmax)
            self.bars[key] = b
            left.addWidget(b)
        left.addStretch(1)
        root.addLayout(left, 3)

        right = QVBoxLayout()
        rh = QLabel("STATUS")
        rh.setStyleSheet("color:#5a7aaa;letter-spacing:2px;font-weight:700;")
        right.addWidget(rh)
        grid = QGridLayout()
        self.lamps: dict[str, _Lamp] = {}
        for i, (key, title) in enumerate(LAMPS):
            lamp = _Lamp(title)
            self.lamps[key] = lamp
            grid.addWidget(lamp, i // 2, i % 2)
        right.addLayout(grid)
        self.note = QLabel("Lamps follow ECU status bits.")
        self.note.setStyleSheet("color:#667;font-size:11px;")
        self.note.setWordWrap(True)
        right.addWidget(self.note)
        right.addStretch(1)
        root.addLayout(right, 1)

    def update_live(self, live: dict):
        for key, bar in self.bars.items():
            v = float(live.get(key) or 0)
            if key == "pw" and v > 80:
                v *= 0.001
            bar.set_value(v)
        self.lamps["sync"].set_state(int(live.get("sync") or 0) != 0)
        self.lamps["cam"].set_state(int(live.get("cam") or 0) != 0)
        self.lamps["fp"].set_state(int(live.get("fp") or 0) != 0)
        self.lamps["fan"].set_state(int(live.get("fan") or 0) != 0)
        self.lamps["ase"].set_state(int(live.get("ase") or 0) != 0)
        self.lamps["dfco"].set_state(int(live.get("dfco") or 0) != 0)
        rpm = float(live.get("rpm") or 0)
        cut = int(live.get("cut") or 0) or rpm > 7900
        self.lamps["cut"].set_state(bool(cut), warn=bool(cut))
