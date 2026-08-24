"""Per-cylinder fuel trim + injector disable for diagnostics."""
from __future__ import annotations

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QSlider, QPushButton,
    QDoubleSpinBox, QFrame, QGridLayout,
)


class CylTrimPage(QWidget):
    """One column per cylinder: trim % slider + Disable injector button."""

    trim_changed = Signal(int, float)   # cyl 1..N, pct
    disable_changed = Signal(int)       # bitmask bit0=cyl1

    def __init__(self, cylinders: int = 4, parent=None):
        super().__init__(parent)
        self._cyl = max(1, min(8, int(cylinders)))
        self._mask = 0
        root = QVBoxLayout(self)
        root.setContentsMargins(8, 8, 8, 8)
        hdr = QLabel("CYLINDER FUEL TRIM  ·  ±25 %")
        hdr.setStyleSheet("color:#5a7aaa;letter-spacing:2px;font-weight:700;")
        root.addWidget(hdr)
        note = QLabel(
            "Trim scales pulse width for that cylinder only. "
            "Disable cuts the injector for leak-down / misfire diagnosis — "
            "restore before driving."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color:#8899aa;font-size:11px;")
        root.addWidget(note)

        grid = QGridLayout()
        grid.setSpacing(12)
        self._sliders: list[QSlider] = []
        self._spins: list[QDoubleSpinBox] = []
        self._btns: list[QPushButton] = []
        for i in range(self._cyl):
            col = QVBoxLayout()
            lab = QLabel(f"Cyl {i + 1}")
            lab.setAlignment(Qt.AlignCenter)
            lab.setStyleSheet("font-weight:700;color:#c8d8ff;")
            col.addWidget(lab)
            s = QSlider(Qt.Vertical)
            s.setRange(-250, 250)  # tenths of %
            s.setValue(0)
            s.setTickPosition(QSlider.TicksBothSides)
            s.setTickInterval(50)
            s.setMinimumHeight(180)
            s.valueChanged.connect(lambda v, c=i: self._on_slider(c, v))
            col.addWidget(s, 1, Qt.AlignHCenter)
            spin = QDoubleSpinBox()
            spin.setRange(-25.0, 25.0)
            spin.setDecimals(1)
            spin.setSuffix(" %")
            spin.setValue(0.0)
            spin.valueChanged.connect(lambda v, c=i: self._on_spin(c, v))
            col.addWidget(spin)
            btn = QPushButton("Disable")
            btn.setCheckable(True)
            btn.setStyleSheet(
                "QPushButton:checked { background:#7a2222; color:#ffe0e0; border-color:#ff5566; }"
            )
            btn.toggled.connect(lambda on, c=i: self._on_disable(c, on))
            col.addWidget(btn)
            self._sliders.append(s)
            self._spins.append(spin)
            self._btns.append(btn)
            grid.addLayout(col, 0, i)
        root.addLayout(grid)

        row = QHBoxLayout()
        btn_z = QPushButton("Zero all trims")
        btn_z.clicked.connect(self.zero_all)
        btn_en = QPushButton("Enable all injectors")
        btn_en.clicked.connect(self.enable_all)
        row.addWidget(btn_z)
        row.addWidget(btn_en)
        row.addStretch(1)
        root.addLayout(row)
        root.addStretch(1)

    def set_cylinders(self, n: int):
        # fixed at construct for simplicity; N from engine on rebuild
        pass

    def _on_slider(self, idx: int, tenths: int):
        pct = tenths / 10.0
        sp = self._spins[idx]
        sp.blockSignals(True)
        sp.setValue(pct)
        sp.blockSignals(False)
        self.trim_changed.emit(idx + 1, pct)

    def _on_spin(self, idx: int, pct: float):
        s = self._sliders[idx]
        s.blockSignals(True)
        s.setValue(int(round(pct * 10)))
        s.blockSignals(False)
        self.trim_changed.emit(idx + 1, float(pct))

    def _on_disable(self, idx: int, on: bool):
        bit = 1 << idx
        if on:
            self._mask |= bit
        else:
            self._mask &= ~bit
        self.disable_changed.emit(self._mask)

    def zero_all(self):
        for i in range(self._cyl):
            self._sliders[i].setValue(0)
            self.trim_changed.emit(i + 1, 0.0)

    def enable_all(self):
        for b in self._btns:
            b.setChecked(False)
        self._mask = 0
        self.disable_changed.emit(0)

    def mask(self) -> int:
        return self._mask
