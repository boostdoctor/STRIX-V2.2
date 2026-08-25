"""STRIX V2.1 map view — heatmap, crosshair, dirty cells, copy/paste, legend."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal, QRectF
from PySide6.QtGui import QColor, QFont, QPen, QPainter, QFontMetrics, QKeySequence
from PySide6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel, QInputDialog, QSizePolicy, QCheckBox

from strix_v2.constants import ROWS, COLS, RPM_BINS


def _heat(v: float, vmax: float, kind: str = "ign") -> QColor:
    if vmax <= 0:
        vmax = 1.0
    t = max(0.0, min(1.0, float(v) / vmax))
    if kind in ("ign", "vvt"):
        r = int(min(255, max(0, 60 + t * 420)))
        g = int(min(255, max(0, 180 - abs(t - 0.45) * 280)))
        b = int(min(255, max(0, 200 - t * 180)))
        return QColor(r, g, b, 220)
    if kind == "boost":
        if t < 0.33:
            u = t / 0.33
            r, g, b = int(80 + 40 * u), int(40 + 120 * u), int(160 + 60 * u)
        elif t < 0.66:
            u = (t - 0.33) / 0.33
            r, g, b = int(120 + 100 * u), int(160 + 40 * u), int(220 - 160 * u)
        else:
            u = (t - 0.66) / 0.34
            r, g, b = int(220 + 35 * u), int(200 - 160 * u), int(60 - 40 * u)
        return QColor(max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b)), 230)
    # inj / VE
    if t < 0.25:
        u = t / 0.25
        r, g, b = int(20 + 20 * u), int(40 + 80 * u), int(120 + 100 * u)
    elif t < 0.5:
        u = (t - 0.25) / 0.25
        r, g, b = int(40 + 20 * u), int(120 + 100 * u), int(220 - 100 * u)
    elif t < 0.75:
        u = (t - 0.5) / 0.25
        r, g, b = int(60 + 160 * u), int(220 - 40 * u), int(120 - 100 * u)
    else:
        u = (t - 0.75) / 0.25
        r, g, b = int(220 + 35 * u), int(180 - 140 * u), int(20)
    return QColor(max(0, min(255, r)), max(0, min(255, g)), max(0, min(255, b)), 230)


class _Canvas(QWidget):
    def __init__(self, view: "MapView"):
        super().__init__()
        self.v = view
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMinimumHeight(180)
        self.setFocusPolicy(Qt.StrongFocus)
        self._drag0 = None

    def paintEvent(self, _):
        v = self.v
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing, False)
        w, h = self.width(), self.height()
        left, top, bot = 48, 22, 28
        gw = max(1, w - left - 8)
        gh = max(1, h - top - bot)
        cw = gw / v.cols
        ch = gh / v.rows
        font = QFont("Segoe UI", 9, QFont.Bold)
        p.setFont(font)
        # cells
        for r in range(v.rows):
            for c in range(v.cols):
                val = float(v.table[r][c])
                rect = QRectF(left + c * cw, top + r * ch, cw - 1, ch - 1)
                p.fillRect(rect, _heat(val, v.vmax, v.kind))
                if (r, c) in v.dirty:
                    p.setPen(QPen(QColor("#ffcc00"), 2))
                    p.drawRect(rect.adjusted(1, 1, -1, -1))
                if (r, c) in v.selected:
                    p.setPen(QPen(QColor("#ffffff"), 3))
                    p.drawRect(rect.adjusted(0.5, 0.5, -0.5, -0.5))
                elif (r, c) == (v.sel_r, v.sel_c):
                    p.setPen(QPen(QColor("#ffff66"), 2))
                    p.drawRect(rect.adjusted(0.5, 0.5, -0.5, -0.5))
                txt = f"{int(val)}" if v.kind in ("ign", "vvt") else f"{val:.1f}"
                p.setPen(QColor("#ffffff"))
                p.drawText(rect, Qt.AlignCenter, txt)
        # live trail (oldest → newest fade)
        trail = getattr(v, "trail", None) or []
        ntr = len(trail)
        for i, (tr, tc) in enumerate(trail):
            if not (0 <= tr < v.rows and 0 <= tc < v.cols):
                continue
            age = (i + 1) / max(1, ntr)
            alpha = int(40 + 140 * age)
            rr = QRectF(left + tc * cw, top + tr * ch, cw - 1, ch - 1)
            p.setPen(QPen(QColor(0, 200, 255, alpha), 1))
            p.setBrush(QColor(0, 180, 220, int(18 + 40 * age)))
            p.drawRect(rr)
        # live crosshair (current cell)
        if 0 <= v.live_r < v.rows and 0 <= v.live_c < v.cols:
            lr = QRectF(left + v.live_c * cw, top + v.live_r * ch, cw - 1, ch - 1)
            p.setPen(QPen(QColor("#00ffcc"), 3))
            p.drawRect(lr.adjusted(-1, -1, 1, 1))
            p.setPen(QPen(QColor("#00ffcc"), 1, Qt.DashLine))
            p.drawLine(int(lr.center().x()), top, int(lr.center().x()), top + gh)
            p.drawLine(left, int(lr.center().y()), left + gw, int(lr.center().y()))
        # axis labels (fixed)
        p.setPen(QColor("#e8eef8"))
        af = QFont("Segoe UI", 8)
        p.setFont(af)
        for c in range(v.cols):
            lab = str(int(v.rpm_bins[c])) if c < len(v.rpm_bins) else str(c)
            p.drawText(QRectF(left + c * cw, 2, cw, top - 2), Qt.AlignCenter, lab)
        for r in range(v.rows):
            lab = str(int(v.load_bins[r])) if r < len(v.load_bins) else str(r)
            p.drawText(QRectF(2, top + r * ch, left - 4, ch), Qt.AlignCenter, lab)
        p.drawText(QRectF(2, h - bot + 4, 40, 16), Qt.AlignLeft, v.load_label)
        p.drawText(QRectF(left, h - bot + 4, 80, 16), Qt.AlignLeft, "RPM →")

    def mousePressEvent(self, e):
        self.setFocus()
        rc = self._hit(e.position().x(), e.position().y())
        if not rc:
            return
        r, c = rc
        if e.modifiers() & Qt.ControlModifier:
            if (r, c) in self.v.selected:
                self.v.selected.discard((r, c))
            else:
                self.v.selected.add((r, c))
        else:
            self.v.sel_r, self.v.sel_c = r, c
            self.v.selected = {(r, c)}
            self._drag0 = (r, c)
        self.update()

    def mouseMoveEvent(self, e):
        if self._drag0 is None or not (e.buttons() & Qt.LeftButton):
            return
        rc = self._hit(e.position().x(), e.position().y())
        if not rc:
            return
        r0, c0 = self._drag0
        r1, c1 = rc
        self.v.selected = {
            (r, c)
            for r in range(min(r0, r1), max(r0, r1) + 1)
            for c in range(min(c0, c1), max(c0, c1) + 1)
        }
        self.v.sel_r, self.v.sel_c = r1, c1
        self.update()

    def mouseReleaseEvent(self, _):
        self._drag0 = None

    def _hit(self, x, y):
        v = self.v
        left, top, bot = 48, 22, 28
        gw = max(1, self.width() - left - 8)
        gh = max(1, self.height() - top - bot)
        if x < left or y < top or y > top + gh:
            return None
        c = int((x - left) / (gw / v.cols))
        r = int((y - top) / (gh / v.rows))
        if 0 <= r < v.rows and 0 <= c < v.cols:
            return r, c
        return None


class MapView(QWidget):
    cell_changed = Signal(int, int, float)
    dirty_changed = Signal()

    def __init__(
        self,
        title: str,
        is_ign: bool = True,
        rows: int = ROWS,
        cols: int = COLS,
        kind: str = "ign",
        vmax: float = 45.0,
        rpm_bins=None,
        parent=None,
    ):
        super().__init__(parent)
        self.kind = kind if kind else ("ign" if is_ign else "inj")
        self.rows = rows
        self.cols = cols
        self.vmax = vmax
        self.rpm_bins = list(rpm_bins) if rpm_bins is not None else list(RPM_BINS[:cols])
        while len(self.rpm_bins) < cols:
            self.rpm_bins.append(self.rpm_bins[-1] + 500 if self.rpm_bins else 0)
        self.load_bins = [int(20 + i * (220 / max(1, rows - 1))) for i in range(rows)]
        self.load_label = "MAP"
        self.table = [[0.0 for _ in range(cols)] for _ in range(rows)]
        self.baseline = [[0.0 for _ in range(cols)] for _ in range(rows)]
        self.dirty: set[tuple[int, int]] = set()
        self.sel_r = self.sel_c = 0
        self.selected: set[tuple[int, int]] = {(0, 0)}
        self.live_r = self.live_c = -1
        self._stable_live = (-1, -1)
        self._live_hold = 0
        self.snap_live = True
        self.trail: list[tuple[int, int]] = []
        self._trail_max = 50
        self._clip = None  # copy buffer

        root = QVBoxLayout(self)
        root.setContentsMargins(2, 2, 2, 2)
        root.setSpacing(2)
        head = QHBoxLayout()
        self.hdr = QLabel(title)
        self.hdr.setStyleSheet("font-weight:700;font-size:13px;color:#c8d8ff;")
        head.addWidget(self.hdr)
        self.chk_snap = QCheckBox("Snap live")
        self.chk_snap.setChecked(True)
        self.chk_snap.toggled.connect(lambda on: setattr(self, "snap_live", on))
        head.addWidget(self.chk_snap)
        self.dirty_lbl = QLabel("")
        self.dirty_lbl.setStyleSheet("color:#ffcc00;font-size:11px;")
        head.addWidget(self.dirty_lbl)
        head.addStretch(1)
        root.addLayout(head)
        self._canvas = _Canvas(self)
        root.addWidget(self._canvas, 1)
        # colour legend
        self.legend = QLabel("")
        self.legend.setFixedHeight(18)
        self.legend.setStyleSheet("color:#8899aa;font-size:10px;")
        root.addWidget(self.legend)
        self._update_legend()
        self.setFocusPolicy(Qt.StrongFocus)

    def _update_legend(self):
        self.legend.setText(f"Scale 0 → {self.vmax:g}  |  cyan=live  |  trail=last 50  |  Arrows select  |  +/- or PgUp/Dn  |  Shift×5  |  Ctrl+C/V  |  Ctrl+P %")

    def set_load_bins(self, bins, label="MAP"):
        self.load_bins = list(bins)[: self.rows]
        while len(self.load_bins) < self.rows:
            self.load_bins.append(self.load_bins[-1] if self.load_bins else 0)
        self.load_label = label
        self._canvas.update()

    def set_table(self, data, mark_clean: bool = True):
        for r in range(min(self.rows, len(data))):
            for c in range(min(self.cols, len(data[r]))):
                self.table[r][c] = data[r][c]
        if mark_clean:
            self.mark_clean()
        self._canvas.update()

    def mark_clean(self):
        self.baseline = [row[:] for row in self.table]
        self.dirty.clear()
        self.dirty_lbl.setText("")
        self.dirty_changed.emit()
        self._canvas.update()

    def _mark_dirty(self, r, c):
        if float(self.table[r][c]) != float(self.baseline[r][c]):
            self.dirty.add((r, c))
        else:
            self.dirty.discard((r, c))
        n = len(self.dirty)
        self.dirty_lbl.setText(f"{n} unsaved" if n else "")
        self.dirty_changed.emit()

    def set_live_cell(self, r: int, c: int):
        """Place live crosshair on ECU-reported cell (MCELL)."""
        r = max(0, min(self.rows - 1, int(r)))
        c = max(0, min(self.cols - 1, int(c)))
        prev = (self.live_r, self.live_c)
        if (r, c) != prev:
            self.trail.append((r, c))
            if len(self.trail) > self._trail_max:
                self.trail = self.trail[-self._trail_max:]
        self.live_r, self.live_c = r, c
        self._stable_live = (r, c)
        self._live_hold = 0
        self._canvas.update()

    def set_live(self, rpm: float, load: float):
        # find nearest bin indices
        c = 0
        for i, b in enumerate(self.rpm_bins):
            if rpm >= b:
                c = i
        r = 0
        for i, b in enumerate(self.load_bins):
            if load >= b:
                r = i
        r = max(0, min(self.rows - 1, r))
        c = max(0, min(self.cols - 1, c))
        prev = (self.live_r, self.live_c)
        if not self.snap_live:
            self.live_r, self.live_c = r, c
            if (r, c) != prev and r >= 0:
                self.trail.append((r, c))
                if len(self.trail) > self._trail_max:
                    self.trail = self.trail[-self._trail_max:]
            self._canvas.update()
            return
        # stable hold filter
        if (r, c) == self._stable_live:
            self._live_hold = 0
            self.live_r, self.live_c = r, c
            self._canvas.update()
            return
        if (r, c) == (self.live_r, self.live_c):
            self._live_hold += 1
        else:
            self.live_r, self.live_c = r, c
            self._live_hold = 1
        if self._live_hold >= 1:
            if self._stable_live != (r, c) and r >= 0:
                self.trail.append((r, c))
                if len(self.trail) > self._trail_max:
                    self.trail = self.trail[-self._trail_max:]
            self._stable_live = (r, c)
            self._canvas.update()

    def keyPressEvent(self, e):
        if e.matches(QKeySequence.Copy):
            self._copy()
            return
        if e.matches(QKeySequence.Paste):
            self._paste()
            return
        if e.key() == Qt.Key_P and (e.modifiers() & Qt.ControlModifier):
            self._pct_dialog()
            return
        step = 1.0 if self.kind in ("ign", "vvt", "boost") else 0.1
        if e.modifiers() & Qt.ShiftModifier:
            step *= 5.0
        r, c = self.sel_r, self.sel_c
        if e.key() == Qt.Key_Up:
            self.sel_r = max(0, r - 1)
        elif e.key() == Qt.Key_Down:
            self.sel_r = min(self.rows - 1, r + 1)
        elif e.key() == Qt.Key_Left:
            self.sel_c = max(0, c - 1)
        elif e.key() == Qt.Key_Right:
            self.sel_c = min(self.cols - 1, c + 1)
        elif e.key() == Qt.Key_Home:
            self.sel_c = 0
        elif e.key() == Qt.Key_End:
            self.sel_c = self.cols - 1
        elif e.key() == Qt.Key_A and (e.modifiers() & Qt.ControlModifier):
            self.selected = {(rr, cc) for rr in range(self.rows) for cc in range(self.cols)}
            self._canvas.update()
            return
        elif e.key() in (Qt.Key_Plus, Qt.Key_Equal, Qt.Key_PageUp):
            self._nudge_sel(step)
            return
        elif e.key() in (Qt.Key_Minus, Qt.Key_PageDown):
            self._nudge_sel(-step)
            return
        elif e.key() == Qt.Key_BracketLeft:
            self._nudge_sel(-step * 10)
            return
        elif e.key() == Qt.Key_BracketRight:
            self._nudge_sel(step * 10)
            return
        else:
            super().keyPressEvent(e)
            return
        if not (e.modifiers() & Qt.ShiftModifier):
            self.selected = {(self.sel_r, self.sel_c)}
        else:
            self.selected.add((self.sel_r, self.sel_c))
        self._canvas.update()

    def _copy(self):
        cells = sorted(self.selected or {(self.sel_r, self.sel_c)})
        if not cells:
            return
        rs = [r for r, _ in cells]
        cs = [c for _, c in cells]
        r0, r1 = min(rs), max(rs)
        c0, c1 = min(cs), max(cs)
        self._clip = [
            [float(self.table[r][c]) for c in range(c0, c1 + 1)]
            for r in range(r0, r1 + 1)
        ]

    def _paste(self):
        if not self._clip:
            return
        r0, c0 = self.sel_r, self.sel_c
        for dr, row in enumerate(self._clip):
            for dc, val in enumerate(row):
                r, c = r0 + dr, c0 + dc
                if r < self.rows and c < self.cols:
                    v = int(round(val)) if self.kind in ("ign", "vvt") else round(val, 1)
                    self.table[r][c] = v
                    self._mark_dirty(r, c)
                    self.cell_changed.emit(r, c, float(v))
        self._canvas.update()

    def _pct_dialog(self):
        cells = self.selected or {(self.sel_r, self.sel_c)}
        pct, ok = QInputDialog.getDouble(self, "Percentage change", "Change selected %:", 0.0, -90.0, 200.0, 1)
        if not ok:
            return
        for r, c in cells:
            v = float(self.table[r][c]) * (1.0 + pct / 100.0)
            v = int(round(v)) if self.kind in ("ign", "vvt") else round(v, 1)
            self.table[r][c] = v
            self._mark_dirty(r, c)
            self.cell_changed.emit(r, c, float(v))
        self._canvas.update()

    def _nudge_sel(self, step):
        cells = self.selected or {(self.sel_r, self.sel_c)}
        for r, c in cells:
            v = float(self.table[r][c]) + step
            v = int(round(v)) if self.kind in ("ign", "vvt") else round(v, 1)
            self.table[r][c] = v
            self._mark_dirty(r, c)
            self.cell_changed.emit(r, c, float(v))
        self._canvas.update()

    def smooth_selected(self, passes=1):
        cells = self.selected or {(r, c) for r in range(self.rows) for c in range(self.cols)}
        for _ in range(passes):
            snap = [row[:] for row in self.table]
            for r, c in cells:
                acc, n = 0.0, 0
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < self.rows and 0 <= cc < self.cols:
                            acc += float(snap[rr][cc])
                            n += 1
                v = acc / max(1, n)
                v = int(round(v)) if self.kind in ("ign", "vvt") else round(v, 1)
                self.table[r][c] = v
                self._mark_dirty(r, c)
                self.cell_changed.emit(r, c, float(v))
        self._canvas.update()

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self._canvas.update()
