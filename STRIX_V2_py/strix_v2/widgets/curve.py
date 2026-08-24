"""2-D curve editor — WUE / ASE / IAT / BAT (MegaTunix-style)."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal, QPointF, QRectF
from PySide6.QtGui import QPainter, QColor, QPen, QFont, QPainterPath
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QDoubleSpinBox,
    QPushButton, QSizePolicy, QComboBox,
)


class CurveCanvas(QWidget):
    point_moved = Signal(int, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.xs: list[float] = []
        self.ys: list[float] = []
        self.x_label = "X"
        self.y_label = "Y"
        self.sel = 0
        self._drag = False
        self.live_x: float | None = None
        self.setMinimumHeight(180)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMouseTracking(True)

    def set_curve(self, xs, ys, x_label="X", y_label="Y"):
        self.xs = [float(x) for x in xs]
        self.ys = [float(y) for y in ys]
        if len(self.ys) < len(self.xs):
            self.ys.extend([0.0] * (len(self.xs) - len(self.ys)))
        self.x_label = x_label
        self.y_label = y_label
        if self.sel >= len(self.xs):
            self.sel = max(0, len(self.xs) - 1)
        self.update()

    def set_live_x(self, x: float | None):
        self.live_x = x
        self.update()

    def _bounds(self):
        if not self.xs:
            return 0.0, 1.0, 0.0, 1.0
        xmin, xmax = min(self.xs), max(self.xs)
        ymin, ymax = min(self.ys), max(self.ys)
        if xmax <= xmin:
            xmax = xmin + 1
        pad = max(5.0, (ymax - ymin) * 0.15)
        if ymax <= ymin:
            ymax, ymin = ymin + 10, ymin - 2
        return xmin, xmax, ymin - pad, ymax + pad

    def _to_px(self, x, y) -> QPointF:
        l, t, r, b = 48, 16, self.width() - 12, self.height() - 28
        xmin, xmax, ymin, ymax = self._bounds()
        px = l + (x - xmin) / (xmax - xmin) * (r - l)
        py = b - (y - ymin) / (ymax - ymin) * (b - t)
        return QPointF(px, py)

    def _from_px(self, px, py) -> tuple[float, float]:
        l, t, r, b = 48, 16, self.width() - 12, self.height() - 28
        xmin, xmax, ymin, ymax = self._bounds()
        x = xmin + (px - l) / max(1, r - l) * (xmax - xmin)
        y = ymin + (b - py) / max(1, b - t) * (ymax - ymin)
        return x, y

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor("#0c1018"))
        if len(self.xs) < 2:
            return
        l, t, r, b = 48, 16, self.width() - 12, self.height() - 28
        p.setPen(QPen(QColor("#1e2a3c"), 1))
        for i in range(5):
            y = t + i * (b - t) / 4
            p.drawLine(QPointF(l, y), QPointF(r, y))
        xmin, xmax, ymin, ymax = self._bounds()

        path = QPainterPath()
        for i, (x, y) in enumerate(zip(self.xs, self.ys)):
            pt = self._to_px(x, y)
            if i == 0:
                path.moveTo(pt)
            else:
                path.lineTo(pt)
        p.setPen(QPen(QColor("#4aa3ff"), 2))
        p.setBrush(Qt.NoBrush)
        p.drawPath(path)

        if self.live_x is not None:
            pt = self._to_px(self.live_x, ymin)
            pt2 = self._to_px(self.live_x, ymax)
            p.setPen(QPen(QColor("#44ff88"), 1, Qt.DashLine))
            p.drawLine(pt, pt2)

        for i, (x, y) in enumerate(zip(self.xs, self.ys)):
            pt = self._to_px(x, y)
            col = QColor("#ffe080") if i == self.sel else QColor("#7eb8ff")
            p.setBrush(col)
            p.setPen(QPen(QColor("#0c1018"), 1))
            rad = 6 if i == self.sel else 4
            p.drawEllipse(pt, rad, rad)

        p.setPen(QColor("#6a7a90"))
        p.setFont(QFont("Segoe UI", 8))
        p.drawText(QRectF(0, b + 2, self.width(), 20), Qt.AlignCenter, self.x_label)
        p.save()
        p.translate(4, self.height() / 2)
        p.rotate(-90)
        p.drawText(QRectF(-60, 0, 120, 16), Qt.AlignCenter, self.y_label)
        p.restore()
        p.setPen(QColor("#9ab"))
        p.drawText(4, 14, f"{ymax:.0f}")
        p.drawText(4, b, f"{ymin:.0f}")

    def mousePressEvent(self, e):
        if not self.xs:
            return
        best, bd = 0, 1e9
        for i, (x, y) in enumerate(zip(self.xs, self.ys)):
            pt = self._to_px(x, y)
            d = (pt.x() - e.position().x()) ** 2 + (pt.y() - e.position().y()) ** 2
            if d < bd:
                best, bd = i, d
        self.sel = best
        self._drag = bd < 400
        self.update()

    def mouseMoveEvent(self, e):
        if not self._drag or not self.xs:
            return
        _x, y = self._from_px(e.position().x(), e.position().y())
        self.ys[self.sel] = round(y, 1)
        self.point_moved.emit(self.sel, self.xs[self.sel], self.ys[self.sel])
        self.update()

    def mouseReleaseEvent(self, _):
        self._drag = False


# Named default curves used by the Curves tab and warmup wizard.
DEFAULT_CURVES = {
    "wue": {
        "title": "Warm-up enrichment (WUE)",
        "x_label": "ECT °C",
        "y_label": "Fuel add %",
        "xs": [-20, 0, 10, 20, 30, 40, 50, 60, 70, 80],
        "ys": [80, 55, 40, 28, 18, 12, 7, 3, 0, 0],
        "y_range": (0.0, 120.0),
    },
    "ase": {
        "title": "After-start enrichment (ASE)",
        "x_label": "ECT °C",
        "y_label": "Initial add %",
        "xs": [-20, 0, 20, 40, 60, 80],
        "ys": [60, 45, 30, 18, 8, 0],
        "y_range": (0.0, 100.0),
    },
    "iat": {
        "title": "IAT fuel compensation",
        "x_label": "IAT °C",
        "y_label": "Fuel add %",
        "xs": [-20, 0, 20, 40, 60, 80, 100],
        "ys": [12, 6, 0, -4, -8, -12, -16],
        "y_range": (-30.0, 30.0),
    },
    "bat": {
        "title": "Battery voltage compensation",
        "x_label": "BAT V",
        "y_label": "Fuel add %",
        "xs": [8, 10, 12, 13.2, 14.4, 16],
        "ys": [18, 10, 4, 0, -2, -4],
        "y_range": (-10.0, 30.0),
    },
}


class CurvePage(QWidget):
    """Tab host for the four enrichment curves."""
    changed = Signal(str)  # curve key

    def __init__(self, engine: dict, parent=None):
        super().__init__(parent)
        self.engine = engine
        lay = QVBoxLayout(self)
        lay.setContentsMargins(6, 6, 6, 6)
        row = QHBoxLayout()
        row.addWidget(QLabel("Curve"))
        self.combo = QComboBox()
        for k, spec in DEFAULT_CURVES.items():
            self.combo.addItem(spec["title"], k)
        self.combo.currentIndexChanged.connect(self._reload)
        row.addWidget(self.combo)
        self.hint = QLabel("")
        self.hint.setStyleSheet("color:#8a9;")
        row.addWidget(self.hint, 1)
        lay.addLayout(row)

        self.canvas = CurveCanvas()
        self.canvas.point_moved.connect(self._on_move)
        lay.addWidget(self.canvas, 1)

        edit = QHBoxLayout()
        self.sp_x = QDoubleSpinBox()
        self.sp_y = QDoubleSpinBox()
        self.sp_x.setRange(-40, 200)
        self.sp_y.setRange(-40, 150)
        self.sp_x.setDecimals(1)
        self.sp_y.setDecimals(1)
        self.sp_x.valueChanged.connect(self._spin)
        self.sp_y.valueChanged.connect(self._spin)
        btn_def = QPushButton("Defaults")
        btn_def.clicked.connect(self._defaults)
        edit.addWidget(QLabel("X"))
        edit.addWidget(self.sp_x)
        edit.addWidget(QLabel("Y"))
        edit.addWidget(self.sp_y)
        edit.addWidget(btn_def)
        edit.addStretch(1)
        lay.addLayout(edit)
        self._reloading = False
        self._reload()

    def key(self) -> str:
        return self.combo.currentData() or "wue"

    def select_curve(self, which: str):
        for i in range(self.combo.count()):
            if self.combo.itemData(i) == which:
                self.combo.setCurrentIndex(i)
                return

    def _store_get(self, key: str):
        spec = DEFAULT_CURVES[key]
        blob = self.engine.get("curves") or {}
        rec = blob.get(key) or {}
        xs = rec.get("xs") or list(spec["xs"])
        ys = rec.get("ys") or list(spec["ys"])
        return xs, ys, spec

    def _store_put(self, key: str, xs, ys):
        blob = dict(self.engine.get("curves") or {})
        blob[key] = {"xs": list(xs), "ys": list(ys)}
        self.engine["curves"] = blob

    def _reload(self, *_):
        self._reloading = True
        key = self.key()
        xs, ys, spec = self._store_get(key)
        self.canvas.set_curve(xs, ys, spec["x_label"], spec["y_label"])
        lo, hi = spec["y_range"]
        self.sp_y.setRange(lo, hi)
        self._sync_spin()
        self.hint.setText("Drag a point  ·  live cursor from ECT/IAT/BAT")
        self._reloading = False

    def _sync_spin(self):
        i = self.canvas.sel
        if 0 <= i < len(self.canvas.xs):
            self.sp_x.blockSignals(True)
            self.sp_y.blockSignals(True)
            self.sp_x.setValue(self.canvas.xs[i])
            self.sp_y.setValue(self.canvas.ys[i])
            self.sp_x.blockSignals(False)
            self.sp_y.blockSignals(False)

    def _on_move(self, i, x, y):
        self._store_put(self.key(), self.canvas.xs, self.canvas.ys)
        self._sync_spin()
        self.changed.emit(self.key())

    def _spin(self, *_):
        if self._reloading:
            return
        i = self.canvas.sel
        if 0 <= i < len(self.canvas.xs):
            self.canvas.xs[i] = float(self.sp_x.value())
            self.canvas.ys[i] = float(self.sp_y.value())
            self.canvas.update()
            self._store_put(self.key(), self.canvas.xs, self.canvas.ys)
            self.changed.emit(self.key())

    def _defaults(self):
        key = self.key()
        spec = DEFAULT_CURVES[key]
        self._store_put(key, spec["xs"], spec["ys"])
        self._reload()
        self.changed.emit(key)

    def set_live(self, live: dict):
        key = self.key()
        if key in ("wue", "ase"):
            self.canvas.set_live_x(float(live.get("ect") or 0))
        elif key == "iat":
            self.canvas.set_live_x(float(live.get("iat") or 0))
        elif key == "bat":
            self.canvas.set_live_x(float(live.get("bat") or 0))
