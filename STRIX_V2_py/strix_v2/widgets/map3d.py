"""3D map surface with pan/tilt/zoom and XYZ scales from map axes."""
from __future__ import annotations

import math
from PySide6.QtCore import Qt, QPointF
from PySide6.QtGui import QPainter, QColor, QPen, QPolygonF, QFont
from PySide6.QtWidgets import QWidget, QVBoxLayout, QLabel, QSizePolicy, QHBoxLayout


class _Surface(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.table: list[list[float]] = [[0.0]]
        self.title = "3D"
        self.rpm_bins: list[float] = []
        self.load_bins: list[float] = []
        self.x_label = "RPM"
        self.y_label = "LOAD"
        self.z_label = "VALUE"
        self.yaw = 35.0
        self.pitch = 28.0
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self._drag = None
        self.setMinimumHeight(220)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setToolTip("LMB: orbit  ·  RMB: pan  ·  Wheel: zoom  ·  R: reset")

    def set_table(self, table, title="3D", rpm_bins=None, load_bins=None,
                  x_label="RPM", y_label="LOAD", z_label="VALUE"):
        self.table = [list(row) for row in table] if table else [[0.0]]
        self.title = title
        rows = len(self.table)
        cols = len(self.table[0]) if rows else 1
        if rpm_bins is not None:
            self.rpm_bins = list(rpm_bins)[:cols]
        if not self.rpm_bins or len(self.rpm_bins) < cols:
            self.rpm_bins = [i * 500 for i in range(cols)]
        if load_bins is not None:
            self.load_bins = list(load_bins)[:rows]
        if not self.load_bins or len(self.load_bins) < rows:
            self.load_bins = [i * 10 for i in range(rows)]
        self.x_label = x_label
        self.y_label = y_label
        self.z_label = z_label
        self.update()

    def reset_view(self):
        self.yaw, self.pitch, self.zoom = 35.0, 28.0, 1.0
        self.pan_x = self.pan_y = 0.0
        self.update()

    def wheelEvent(self, e):
        d = e.angleDelta().y()
        if d > 0:
            self.zoom = min(4.0, self.zoom * 1.12)
        elif d < 0:
            self.zoom = max(0.35, self.zoom / 1.12)
        self.update()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self._drag = ("orbit", e.position().toPoint())
        elif e.button() in (Qt.RightButton, Qt.MiddleButton):
            self._drag = ("pan", e.position().toPoint())

    def mouseMoveEvent(self, e):
        if not self._drag:
            return
        mode, prev = self._drag
        pos = e.position().toPoint()
        dx, dy = pos.x() - prev.x(), pos.y() - prev.y()
        if mode == "orbit":
            self.yaw = (self.yaw + dx * 0.4) % 360.0
            self.pitch = max(5.0, min(85.0, self.pitch + dy * 0.35))
        else:
            self.pan_x += dx
            self.pan_y += dy
        self._drag = (mode, pos)
        self.update()

    def mouseReleaseEvent(self, _):
        self._drag = None

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_R:
            self.reset_view()
        else:
            super().keyPressEvent(e)

    def _proj_fn(self, scale, cx, cy):
        yaw = math.radians(self.yaw)
        pitch = math.radians(self.pitch)
        cy_, sy = math.cos(yaw), math.sin(yaw)
        cp_, sp = math.cos(pitch), math.sin(pitch)
        rows = len(self.table)
        cols = len(self.table[0]) if rows else 1
        vals = [float(v) for row in self.table for v in row]
        vmin = min(vals) if vals else 0.0
        vmax = max(vals) if vals else 1.0
        span = max(1e-6, vmax - vmin)

        def proj(r, c, z):
            x = (c - (cols - 1) / 2) * scale
            y = (r - (rows - 1) / 2) * scale
            zz = ((float(z) - vmin) / span) * scale * 1.6
            x1 = x * cy_ - y * sy
            y1 = x * sy + y * cy_
            y2 = y1 * cp_ - zz * sp
            return QPointF(cx + x1, cy + y2), vmin, vmax, span

        return proj, vmin, vmax, span, rows, cols

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#0c1018"))
        rows = len(self.table)
        cols = len(self.table[0]) if rows else 1
        if rows < 1 or cols < 1:
            return
        cx = self.width() / 2 + self.pan_x
        cy = self.height() / 2 + self.pan_y + 10
        scale = min(self.width(), self.height()) / (max(rows, cols) + 2) * 0.75 * self.zoom
        proj, vmin, vmax, span, rows, cols = self._proj_fn(scale, cx, cy)

        # surface quads
        quads = []
        for r in range(rows - 1):
            for c in range(cols - 1):
                z00 = float(self.table[r][c])
                z01 = float(self.table[r][c + 1])
                z10 = float(self.table[r + 1][c])
                z11 = float(self.table[r + 1][c + 1])
                pts = [proj(r, c, z00)[0], proj(r, c + 1, z01)[0],
                       proj(r + 1, c + 1, z11)[0], proj(r + 1, c, z10)[0]]
                depth = sum(pt.y() for pt in pts) / 4.0
                avg = (z00 + z01 + z10 + z11) / 4.0
                quads.append((depth, avg, pts))
        quads.sort(key=lambda q: q[0])
        for _, avg, pts in quads:
            t = (avg - vmin) / span
            col = QColor(
                int(40 + 180 * t),
                int(80 + 100 * (1 - abs(t - 0.5) * 2)),
                int(200 - 140 * t),
                210,
            )
            p.setBrush(col)
            p.setPen(QPen(QColor(15, 20, 30), 1))
            p.drawPolygon(QPolygonF(pts))

        # Axis frame: origin at (0,0,vmin), X along cols, Y along rows, Z up
        o, _, _, _ = proj(0, 0, vmin)
        x_end, _, _, _ = proj(0, cols - 1, vmin)
        y_end, _, _, _ = proj(rows - 1, 0, vmin)
        z_end, _, _, _ = proj(0, 0, vmax)
        p.setPen(QPen(QColor("#ff6688"), 2))
        p.drawLine(o, x_end)
        p.setPen(QPen(QColor("#66ff99"), 2))
        p.drawLine(o, y_end)
        p.setPen(QPen(QColor("#66aaff"), 2))
        p.drawLine(o, z_end)

        p.setFont(QFont("Segoe UI", 9, QFont.Bold))
        p.setPen(QColor("#ff8899"))
        p.drawText(x_end + QPointF(4, 0), f"{self.x_label}")
        p.setPen(QColor("#88ffaa"))
        p.drawText(y_end + QPointF(4, 0), f"{self.y_label}")
        p.setPen(QColor("#88bbff"))
        p.drawText(z_end + QPointF(4, -4), f"{self.z_label}")

        # Tick labels along axes (subset)
        p.setFont(QFont("Segoe UI", 8))
        n_xt = min(6, cols)
        for i in range(n_xt):
            c = int(round(i * (cols - 1) / max(1, n_xt - 1)))
            pt, _, _, _ = proj(0, c, vmin)
            val = self.rpm_bins[c] if c < len(self.rpm_bins) else c
            p.setPen(QColor("#cc8899"))
            p.drawText(pt + QPointF(-8, 14), f"{int(val)}")
        n_yt = min(6, rows)
        for i in range(n_yt):
            r = int(round(i * (rows - 1) / max(1, n_yt - 1)))
            pt, _, _, _ = proj(r, 0, vmin)
            val = self.load_bins[r] if r < len(self.load_bins) else r
            p.setPen(QColor("#88cc99"))
            p.drawText(pt + QPointF(-28, 4), f"{int(val)}")
        for i, zv in enumerate((vmin, (vmin + vmax) / 2, vmax)):
            pt, _, _, _ = proj(0, 0, zv)
            p.setPen(QColor("#88aadd"))
            txt = f"{zv:.0f}" if abs(zv) >= 10 else f"{zv:.1f}"
            p.drawText(pt + QPointF(-30, -2), txt)

        # HUD
        p.setPen(QColor("#c8d4e8"))
        p.setFont(QFont("Segoe UI", 10, QFont.Bold))
        p.drawText(10, 18, self.title)
        p.setPen(QColor("#6a7a90"))
        p.setFont(QFont("Segoe UI", 9))
        p.drawText(10, 36, f"X {self.x_label}  Y {self.y_label}  Z {self.z_label}   "
                           f"[{int(vmin)} … {int(vmax) if abs(vmax)>=10 else round(vmax,1)}]")
        p.drawText(10, self.height() - 8,
                   f"yaw {self.yaw:.0f}°  pitch {self.pitch:.0f}°  zoom {self.zoom:.2f}")


class Map3DView(QWidget):
    PRESETS = [
        ("Default", 35, 28, 1.0, 0, 0),
        ("Top-down", 0, 85, 1.1, 0, 0),
        ("Front", 0, 15, 1.0, 0, 20),
        ("Side (RPM)", 90, 20, 1.0, 0, 0),
        ("Corner high", 45, 40, 1.15, 0, -10),
        ("Low angle", 25, 12, 1.2, 0, 30),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        from PySide6.QtWidgets import QComboBox, QPushButton
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 4, 4, 4)
        head = QHBoxLayout()
        self.lbl = QLabel("3D map")
        self.lbl.setStyleSheet("color:#9ab;font-size:11px;")
        head.addWidget(self.lbl)
        head.addStretch(1)
        head.addWidget(QLabel("View"))
        self.preset = QComboBox()
        for name, *_ in self.PRESETS:
            self.preset.addItem(name)
        self.preset.currentIndexChanged.connect(self._apply_preset_idx)
        head.addWidget(self.preset)
        btn = QPushButton("Reset")
        btn.clicked.connect(lambda: self.surface.reset_view())
        head.addWidget(btn)
        lay.addLayout(head)
        self.surface = _Surface()
        lay.addWidget(self.surface, 1)

    def _apply_preset_idx(self, i: int):
        if i < 0 or i >= len(self.PRESETS):
            return
        _n, yaw, pitch, zoom, px, py = self.PRESETS[i]
        self.surface.yaw = float(yaw)
        self.surface.pitch = float(pitch)
        self.surface.zoom = float(zoom)
        self.surface.pan_x = float(px)
        self.surface.pan_y = float(py)
        self.surface.update()

    def set_table(self, table, title="3D", rpm_bins=None, load_bins=None,
                  x_label="RPM", y_label="LOAD", z_label="VALUE"):
        self.surface.set_table(table, title, rpm_bins, load_bins, x_label, y_label, z_label)
        self.lbl.setText(title)
