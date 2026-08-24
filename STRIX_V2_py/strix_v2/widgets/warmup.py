"""Warm-up wizard — step ECT bins and write WUE / ASE."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QStackedWidget,
    QWidget, QDoubleSpinBox, QFormLayout, QDialogButtonBox,
)

from strix_v2.widgets.curve import DEFAULT_CURVES, CurveCanvas


STEPS_ECT = [-20, 0, 20, 40, 60, 80]


class WarmupWizardDialog(QDialog):
    def __init__(self, engine: dict, live_getter, parent=None):
        super().__init__(parent)
        self.engine = engine
        self._live = live_getter
        self.setWindowTitle("Warm-up wizard")
        self.resize(560, 420)
        self.wue_xs = list((engine.get("curves") or {}).get("wue", {}).get("xs") or DEFAULT_CURVES["wue"]["xs"])
        self.wue_ys = list((engine.get("curves") or {}).get("wue", {}).get("ys") or DEFAULT_CURVES["wue"]["ys"])
        self.ase_pct = float(engine.get("ase_initial_pct") or 35.0)
        self.ase_decay = float(engine.get("ase_decay_sec") or 8.0)
        self.ase_min = float(engine.get("ase_min_ect") or 60.0)

        root = QVBoxLayout(self)
        self.stack = QStackedWidget()
        self.stack.addWidget(self._page_intro())
        self.stack.addWidget(self._page_ase())
        self.stack.addWidget(self._page_wue())
        self.stack.addWidget(self._page_done())
        root.addWidget(self.stack, 1)

        nav = QHBoxLayout()
        self.btn_back = QPushButton("Back")
        self.btn_next = QPushButton("Next")
        self.btn_back.clicked.connect(self._back)
        self.btn_next.clicked.connect(self._next)
        nav.addWidget(self.btn_back)
        nav.addStretch(1)
        nav.addWidget(self.btn_next)
        root.addLayout(nav)
        self._sync_nav()

    def _page_intro(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        t = QLabel("Warm-up enrichment")
        t.setStyleSheet("font-size:16px;font-weight:700;color:#7eb8ff;")
        lay.addWidget(t)
        lay.addWidget(QLabel(
            "WUE adds fuel while coolant is below ~80 °C.\n"
            "ASE adds extra fuel for a few seconds after the engine fires.\n\n"
            "This wizard writes both to tuner RAM. Press Flash at RPM 0\n"
            "to store them in ECU NVM (SET:WUE / SET:ASE)."
        ))
        self.ect_live = QLabel("ECT —")
        self.ect_live.setStyleSheet("font-size:20px;font-weight:700;")
        lay.addWidget(self.ect_live)
        lay.addStretch(1)
        return w

    def _page_ase(self) -> QWidget:
        w = QWidget()
        f = QFormLayout(w)
        self.sp_ase = QDoubleSpinBox()
        self.sp_ase.setRange(0, 100)
        self.sp_ase.setSuffix(" %")
        self.sp_ase.setValue(self.ase_pct)
        self.sp_dec = QDoubleSpinBox()
        self.sp_dec.setRange(0.5, 30)
        self.sp_dec.setSuffix(" s")
        self.sp_dec.setValue(self.ase_decay)
        self.sp_min = QDoubleSpinBox()
        self.sp_min.setRange(-20, 90)
        self.sp_min.setSuffix(" °C")
        self.sp_min.setValue(self.ase_min)
        f.addRow(QLabel("After-start (ASE)"))
        f.addRow("Initial extra fuel", self.sp_ase)
        f.addRow("Decay time", self.sp_dec)
        f.addRow("Skip if ECT above", self.sp_min)
        return w

    def _page_wue(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        lay.addWidget(QLabel("Drag WUE vs coolant. Live ECT is the dashed line."))
        self.canvas = CurveCanvas()
        self.canvas.set_curve(self.wue_xs, self.wue_ys, "ECT °C", "Fuel add %")
        self.canvas.point_moved.connect(self._wue_moved)
        lay.addWidget(self.canvas, 1)
        rec = QPushButton("Fill recommended (MS-style)")
        rec.clicked.connect(self._fill_rec)
        lay.addWidget(rec)
        return w

    def _page_done(self) -> QWidget:
        w = QWidget()
        lay = QVBoxLayout(w)
        self.summary = QLabel("")
        self.summary.setWordWrap(True)
        lay.addWidget(self.summary)
        lay.addStretch(1)
        return w

    def _fill_rec(self):
        spec = DEFAULT_CURVES["wue"]
        self.wue_xs, self.wue_ys = list(spec["xs"]), list(spec["ys"])
        self.canvas.set_curve(self.wue_xs, self.wue_ys, "ECT °C", "Fuel add %")

    def _wue_moved(self, i, x, y):
        self.wue_xs = list(self.canvas.xs)
        self.wue_ys = list(self.canvas.ys)

    def _back(self):
        i = self.stack.currentIndex()
        if i > 0:
            self.stack.setCurrentIndex(i - 1)
        self._sync_nav()

    def _next(self):
        i = self.stack.currentIndex()
        if i == self.stack.count() - 1:
            self._commit()
            self.accept()
            return
        self.stack.setCurrentIndex(i + 1)
        if self.stack.currentIndex() == 3:
            self._build_summary()
        self._sync_nav()

    def _sync_nav(self):
        i = self.stack.currentIndex()
        self.btn_back.setEnabled(i > 0)
        self.btn_next.setText("Apply" if i == self.stack.count() - 1 else "Next")
        live = self._live() if callable(self._live) else {}
        ect = float(live.get("ect") or 0)
        if hasattr(self, "ect_live"):
            self.ect_live.setText(f"Live ECT  {ect:.0f} °C")
        if hasattr(self, "canvas"):
            self.canvas.set_live_x(ect)

    def _build_summary(self):
        self.ase_pct = float(self.sp_ase.value())
        self.ase_decay = float(self.sp_dec.value())
        self.ase_min = float(self.sp_min.value())
        pts = ", ".join(f"{x:.0f}°={y:.0f}%" for x, y in zip(self.wue_xs, self.wue_ys))
        self.summary.setText(
            f"ASE {self.ase_pct:.0f}% over {self.ase_decay:.1f}s "
            f"(skip above {self.ase_min:.0f} °C)\n\nWUE: {pts}"
        )

    def _commit(self):
        self.ase_pct = float(self.sp_ase.value())
        self.ase_decay = float(self.sp_dec.value())
        self.ase_min = float(self.sp_min.value())
        blob = dict(self.engine.get("curves") or {})
        blob["wue"] = {"xs": list(self.wue_xs), "ys": list(self.wue_ys)}
        self.engine["curves"] = blob
        self.engine["ase_initial_pct"] = self.ase_pct
        self.engine["ase_decay_sec"] = self.ase_decay
        self.engine["ase_min_ect"] = self.ase_min
