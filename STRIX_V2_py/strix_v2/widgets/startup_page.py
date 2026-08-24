"""Startup / priming / WUE·ASE links / cranking advance / flood clear."""
from __future__ import annotations

from PySide6.QtCore import Signal
from PySide6.QtWidgets import (
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QFormLayout, QGroupBox,
    QSpinBox, QDoubleSpinBox, QCheckBox, QPushButton, QFrame,
)


class StartupPage(QWidget):
    apply_requested = Signal()
    open_wue = Signal()
    open_ase = Signal()

    def __init__(self, engine: dict, parent=None):
        super().__init__(parent)
        self.engine = engine
        root = QVBoxLayout(self)
        root.setContentsMargins(10, 10, 10, 10)

        title = QLabel("STARTUP")
        title.setStyleSheet("color:#5a7aaa;letter-spacing:2px;font-weight:700;")
        root.addWidget(title)

        # Priming
        prime = QGroupBox("Priming")
        pf = QFormLayout(prime)
        self.fp_prime = QSpinBox()
        self.fp_prime.setRange(0, 15000)
        self.fp_prime.setSuffix(" ms")
        self.fp_prime.setSingleStep(100)
        self.fp_prime.setValue(int(engine.get("fp_prime_ms") or 2000))
        self.inj_prime_en = QCheckBox("Enable start injector prime")
        self.inj_prime_en.setChecked(bool(engine.get("start_prime_enable", True)))
        self.inj_prime = QSpinBox()
        self.inj_prime.setRange(0, 500)
        self.inj_prime.setSuffix(" ms")
        self.inj_prime.setValue(int(engine.get("start_prime_ms") or 50))
        self.inj_prime.setEnabled(self.inj_prime_en.isChecked())
        self.inj_prime_en.toggled.connect(self.inj_prime.setEnabled)
        pf.addRow("Fuel pump prime", self.fp_prime)
        pf.addRow(self.inj_prime_en)
        pf.addRow("Start inj. prime", self.inj_prime)
        root.addWidget(prime)

        # Warm-up links
        warm = QGroupBox("Warm-up enrichment")
        wl = QHBoxLayout(warm)
        btn_wue = QPushButton("Edit WUE curve…")
        btn_wue.clicked.connect(self.open_wue.emit)
        btn_ase = QPushButton("Edit ASE curve…")
        btn_ase.clicked.connect(self.open_ase.emit)
        wl.addWidget(btn_wue)
        wl.addWidget(btn_ase)
        wl.addStretch(1)
        root.addWidget(warm)

        # Cranking advance
        crank = QGroupBox("Cranking advance")
        cf = QFormLayout(crank)
        self.crank_en = QCheckBox("Enable fixed advance while cranking")
        self.crank_en.setChecked(bool(engine.get("crank_adv_enable", True)))
        self.crank_deg = QDoubleSpinBox()
        self.crank_deg.setRange(-5.0, 30.0)
        self.crank_deg.setDecimals(1)
        self.crank_deg.setSuffix(" ° BTDC")
        self.crank_deg.setValue(float(engine.get("crank_adv_deg") or 10.0))
        self.crank_rpm = QSpinBox()
        self.crank_rpm.setRange(100, 1200)
        self.crank_rpm.setSuffix(" RPM")
        self.crank_rpm.setValue(int(engine.get("crank_adv_rpm") or 400))
        self.crank_deg.setEnabled(self.crank_en.isChecked())
        self.crank_rpm.setEnabled(self.crank_en.isChecked())
        self.crank_en.toggled.connect(self.crank_deg.setEnabled)
        self.crank_en.toggled.connect(self.crank_rpm.setEnabled)
        cf.addRow(self.crank_en)
        cf.addRow("Advance", self.crank_deg)
        cf.addRow("Below RPM", self.crank_rpm)
        root.addWidget(crank)

        # Flood clear
        flood = QGroupBox("Flood clear")
        ff = QFormLayout(flood)
        self.flood_en = QCheckBox("Cut injectors when TPS ≥ threshold (cranking)")
        self.flood_en.setChecked(bool(engine.get("flood_clear_enable", True)))
        self.flood_tps = QDoubleSpinBox()
        self.flood_tps.setRange(50.0, 100.0)
        self.flood_tps.setDecimals(0)
        self.flood_tps.setSuffix(" %")
        self.flood_tps.setValue(float(engine.get("flood_clear_tps") or 85.0))
        self.flood_tps.setEnabled(self.flood_en.isChecked())
        self.flood_en.toggled.connect(self.flood_tps.setEnabled)
        ff.addRow(self.flood_en)
        ff.addRow("TPS threshold", self.flood_tps)
        tip = QLabel("Hold throttle ≥ threshold while cranking to clear a flooded engine.")
        tip.setStyleSheet("color:#8899aa;font-size:11px;")
        tip.setWordWrap(True)
        ff.addRow(tip)
        root.addWidget(flood)

        btn = QPushButton("Apply startup settings")
        btn.clicked.connect(self.apply_requested.emit)
        root.addWidget(btn)
        root.addStretch(1)

    def apply_to(self, engine: dict):
        engine["fp_prime_ms"] = int(self.fp_prime.value())
        engine["start_prime_enable"] = self.inj_prime_en.isChecked()
        engine["start_prime_ms"] = int(self.inj_prime.value())
        engine["crank_adv_enable"] = self.crank_en.isChecked()
        engine["crank_adv_deg"] = float(self.crank_deg.value())
        engine["crank_adv_rpm"] = int(self.crank_rpm.value())
        engine["flood_clear_enable"] = self.flood_en.isChecked()
        engine["flood_clear_tps"] = float(self.flood_tps.value())

    def load_from(self, engine: dict):
        self.fp_prime.setValue(int(engine.get("fp_prime_ms") or 2000))
        self.inj_prime_en.setChecked(bool(engine.get("start_prime_enable", True)))
        self.inj_prime.setValue(int(engine.get("start_prime_ms") or 50))
        self.crank_en.setChecked(bool(engine.get("crank_adv_enable", True)))
        self.crank_deg.setValue(float(engine.get("crank_adv_deg") or 10.0))
        self.crank_rpm.setValue(int(engine.get("crank_adv_rpm") or 400))
        self.flood_en.setChecked(bool(engine.get("flood_clear_enable", True)))
        self.flood_tps.setValue(float(engine.get("flood_clear_tps") or 85.0))
