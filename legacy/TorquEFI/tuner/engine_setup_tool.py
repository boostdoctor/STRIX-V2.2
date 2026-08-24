#!/usr/bin/env python3
"""
Engine Setup Tool – builds ecu_firmware/ecu_config.h for the Arduino ECU.

Select cylinders, crank trigger (36-1, 60-2, …), sensor profiles, coil type,
and load strategy (Speed-Density vs Alpha-N). Generates a config header the
main firmware includes so the tuner stays focused on maps.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

from PySide6.QtCore import Qt
from PySide6.QtGui import QFont, QDoubleValidator
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox, QSpinBox, QDoubleSpinBox, QPushButton, QGroupBox,
    QFormLayout, QTextEdit, QMessageBox, QCheckBox, QFrame, QDialog,
    QTableWidget, QTableWidgetItem, QHeaderView, QTabWidget, QLineEdit,
    QAbstractItemView,
)

ROOT = Path(__file__).resolve().parent
FIRMWARE_DIR = ROOT / "ecu_firmware"
CONFIG_H = FIRMWARE_DIR / "ecu_config.h"
PROFILE_JSON = ROOT / "engine_profiles.json"
CUSTOM_SENSORS_JSON = ROOT / "custom_sensors.json"

# ── Trigger wheel presets ──────────────────────────────────────
TRIGGERS = {
    "12-1 (simple)":          {"teeth": 12, "missing": 1, "trig": 60},
    "24-1":                   {"teeth": 24, "missing": 1, "trig": 40},
    "24-2":                   {"teeth": 24, "missing": 2, "trig": 40},
    "36-1 (common 4-cyl)":    {"teeth": 36, "missing": 1, "trig": 30},
    "36-2":                   {"teeth": 36, "missing": 2, "trig": 30},
    "60-2 (common modern)":   {"teeth": 60, "missing": 2, "trig": 20},
    "60-1":                   {"teeth": 60, "missing": 1, "trig": 20},
}

# ── Sensor profiles: CLT / IAT = (temps[], adcs[]) ─────────────
# Approximate 5 V + pull-up curves – fine-tune later in Calibrate Sensors
CLT_PROFILES = {
    "Generic 10k NTC": {
        "id": 0,
        "temp": [-10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110],
        "adc":  [920, 850, 780, 700, 620, 540, 470, 400, 340, 290, 245, 210, 180],
    },
    "GM 3/8 NTC": {
        "id": 1,
        "temp": [-10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110],
        "adc":  [940, 880, 810, 730, 640, 550, 470, 400, 340, 290, 250, 215, 185],
    },
    "Bosch 024 CLT": {
        "id": 2,
        "temp": [-10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110],
        "adc":  [900, 830, 760, 680, 600, 520, 450, 390, 335, 285, 245, 210, 180],
    },
    "Honda CLT": {
        "id": 3,
        "temp": [-10, 0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110],
        "adc":  [910, 840, 770, 690, 610, 530, 460, 395, 340, 290, 250, 215, 185],
    },
}

IAT_PROFILES = {
    "Generic 10k NTC": {
        "id": 0,
        "temp": [-20, -6, 8, 22, 36, 50, 64, 78, 92, 106, 120],
        "adc":  [900, 820, 740, 660, 580, 500, 430, 370, 320, 280, 240],
    },
    "GM open-element": {
        "id": 1,
        "temp": [-20, -6, 8, 22, 36, 50, 64, 78, 92, 106, 120],
        "adc":  [920, 850, 770, 690, 600, 520, 450, 390, 340, 295, 255],
    },
    "Bosch air temp": {
        "id": 2,
        "temp": [-20, -6, 8, 22, 36, 50, 64, 78, 92, 106, 120],
        "adc":  [890, 810, 730, 650, 570, 490, 420, 360, 310, 270, 235],
    },
    "Honda IAT": {
        "id": 3,
        "temp": [-20, -6, 8, 22, 36, 50, 64, 78, 92, 106, 120],
        "adc":  [905, 825, 745, 665, 585, 505, 435, 375, 325, 280, 245],
    },
}

# MAP: offset_kPa + V * gain → kPa (linear approx of common sensors)
MAP_PROFILES = {
    "MPX4250 (1 bar / ~250 kPa)": {"id": 0, "offset": 10.0, "gain": 50.0},
    "MPX5700 (up to ~700 kPa)":   {"id": 1, "offset": 15.0, "gain": 140.0},
    "GM 1-bar":                   {"id": 2, "offset": 10.0, "gain": 51.0},
    "GM 2-bar":                   {"id": 3, "offset": 8.0,  "gain": 100.0},
}

COIL_TYPES = {
    "Dumb (inductive – ECU dwell)": 0,
    "Smart (logic-level / CDI fire)": 1,
}

LOAD_MODES = {
    "Speed-Density (MAP)": 0,
    "Alpha-N (TPS)": 1,
}

# Runtime-merged profile dicts (builtins + custom from JSON)
def _load_custom_sensors() -> dict:
    if not CUSTOM_SENSORS_JSON.is_file():
        return {"clt": {}, "iat": {}, "map": {}}
    try:
        data = json.loads(CUSTOM_SENSORS_JSON.read_text(encoding="utf-8"))
        return {
            "clt": data.get("clt") or {},
            "iat": data.get("iat") or {},
            "map": data.get("map") or {},
        }
    except Exception:
        return {"clt": {}, "iat": {}, "map": {}}


def _save_custom_sensors(store: dict) -> None:
    CUSTOM_SENSORS_JSON.write_text(
        json.dumps(store, indent=2), encoding="utf-8")


def all_clt_profiles() -> dict:
    d = dict(CLT_PROFILES)
    d.update(_load_custom_sensors()["clt"])
    return d


def all_iat_profiles() -> dict:
    d = dict(IAT_PROFILES)
    d.update(_load_custom_sensors()["iat"])
    return d


def all_map_profiles() -> dict:
    d = dict(MAP_PROFILES)
    d.update(_load_custom_sensors()["map"])
    return d


DARK = """
QMainWindow, QWidget, QDialog { background: #1a1e26; color: #d0d8e8; }
QGroupBox {
    font-weight: bold; color: #a0c8ff;
    border: 1px solid #2a4a6a; border-radius: 8px;
    margin-top: 12px; padding: 12px 8px 8px 8px;
}
QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; }
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {
    background: #222a38; color: #c8d8ff;
    border: 1px solid #3a5a8a; border-radius: 4px; padding: 4px 8px;
}
QComboBox QAbstractItemView {
    background: #1e2736; selection-background-color: #3a5a8a; color: #c8d8ff;
}
QPushButton {
    background: #2a3a5a; color: #c8d8ff;
    border: 1px solid #3a5a8a; border-radius: 5px; padding: 8px 16px;
}
QPushButton:hover { background: #3a5a8a; }
QTextEdit, QTableWidget {
    background: #12151c; color: #a0b8d0;
    border: 1px solid #2a3a58; font-family: monospace; font-size: 11px;
    gridline-color: #2a3a58;
}
QHeaderView::section {
    background: #1e2736; color: #a0c8ff;
    border: 1px solid #2a3a58; padding: 4px;
}
QTabWidget::pane { border: 1px solid #2a4a6a; }
QTabBar::tab {
    background: #222a38; color: #a0b0c8; padding: 6px 14px;
    border: 1px solid #2a3a58; border-bottom: none;
}
QTabBar::tab:selected { background: #2a4a6a; color: #ffffff; }
QLabel { color: #c0cce0; }
"""


# ──────────────────────────────────────────────────────────────
#  SENSOR PROFILER DIALOG
# ──────────────────────────────────────────────────────────────
class SensorProfilerDialog(QDialog):
    """Create / edit custom CLT, IAT (temp↔ADC tables) and MAP (gain) sensors."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Sensor Profiler – custom sensors")
        self.setMinimumSize(640, 520)
        self.resize(700, 560)
        self.store = _load_custom_sensors()

        lay = QVBoxLayout(self)
        tip = QLabel(
            "Enter measured points: temperature (°C) vs ADC (0–1023) for NTC sensors, "
            "or MAP offset/gain for linear pressure sensors. Saved profiles appear in the "
            "Engine Setup dropdowns."
        )
        tip.setWordWrap(True)
        tip.setStyleSheet("color:#8090b0;")
        lay.addWidget(tip)

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_ntc_tab("clt", 13), "CLT / ECT")
        self.tabs.addTab(self._build_ntc_tab("iat", 11), "IAT")
        self.tabs.addTab(self._build_map_tab(), "MAP")
        lay.addWidget(self.tabs)

        row = QHBoxLayout()
        row.addWidget(QPushButton("Save profile", clicked=self._save_current))
        row.addWidget(QPushButton("Delete selected custom", clicked=self._delete_current))
        row.addStretch()
        row.addWidget(QPushButton("Close", clicked=self.accept))
        lay.addLayout(row)
        self.status = QLabel("")
        self.status.setStyleSheet("color:#8090b0;")
        lay.addWidget(self.status)

    def _build_ntc_tab(self, kind: str, n_pts: int) -> QWidget:
        page = QWidget()
        v = QVBoxLayout(page)

        form = QFormLayout()
        name = QLineEdit()
        name.setPlaceholderText("e.g. My custom 10k NTC")
        form.addRow("Profile name", name)
        v.addLayout(form)

        # Existing custom list
        existing = QComboBox()
        existing.addItem("— new profile —")
        customs = self.store.get(kind, {})
        for k in customs:
            existing.addItem(k)
        form.addRow("Load custom", existing)

        tbl = QTableWidget(n_pts, 2)
        tbl.setHorizontalHeaderLabels(["Temperature °C", "ADC (0–1023)"])
        tbl.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        tbl.setSelectionBehavior(QAbstractItemView.SelectRows)
        # Seed blank / default ladder
        if kind == "clt":
            temps = [-10 + i * 10 for i in range(n_pts)]
            adcs = [920 - i * 60 for i in range(n_pts)]
        else:
            temps = [-20 + i * 14 for i in range(n_pts)]
            adcs = [900 - i * 60 for i in range(n_pts)]
        for i in range(n_pts):
            tbl.setItem(i, 0, QTableWidgetItem(f"{temps[i]:.1f}"))
            tbl.setItem(i, 1, QTableWidgetItem(str(int(adcs[i]))))
        v.addWidget(tbl)

        def load_existing(text: str):
            if text.startswith("—"):
                return
            p = self.store.get(kind, {}).get(text)
            if not p:
                return
            name.setText(text)
            tlist = p.get("temp") or []
            alist = p.get("adc") or []
            for i in range(n_pts):
                if i < len(tlist):
                    tbl.item(i, 0).setText(f"{float(tlist[i]):.1f}")
                if i < len(alist):
                    tbl.item(i, 1).setText(str(int(alist[i])))

        existing.currentTextChanged.connect(load_existing)

        # Stash widgets for save
        setattr(self, f"_{kind}_name", name)
        setattr(self, f"_{kind}_table", tbl)
        setattr(self, f"_{kind}_existing", existing)
        setattr(self, f"_{kind}_npts", n_pts)
        return page

    def _build_map_tab(self) -> QWidget:
        page = QWidget()
        v = QVBoxLayout(page)
        form = QFormLayout()
        name = QLineEdit()
        name.setPlaceholderText("e.g. Custom 3-bar MAP")
        form.addRow("Profile name", name)

        existing = QComboBox()
        existing.addItem("— new profile —")
        for k in self.store.get("map", {}):
            existing.addItem(k)
        form.addRow("Load custom", existing)

        offset = QDoubleSpinBox()
        offset.setRange(-50, 100)
        offset.setDecimals(2)
        offset.setValue(10.0)
        offset.setSuffix(" kPa")
        form.addRow("Offset @ 0 V", offset)

        gain = QDoubleSpinBox()
        gain.setRange(1, 500)
        gain.setDecimals(2)
        gain.setValue(50.0)
        gain.setSuffix(" kPa/V")
        form.addRow("Gain", gain)

        hint = QLabel(
            "kPa = offset + voltage × gain\n"
            "Example: 0.5 V → 10 + 0.5×50 = 35 kPa; 4.5 V → 10 + 4.5×50 = 235 kPa"
        )
        hint.setStyleSheet("color:#708090;")
        hint.setWordWrap(True)

        v.addLayout(form)
        v.addWidget(hint)
        v.addStretch()

        def load_existing(text: str):
            if text.startswith("—"):
                return
            p = self.store.get("map", {}).get(text)
            if not p:
                return
            name.setText(text)
            offset.setValue(float(p.get("offset", 10)))
            gain.setValue(float(p.get("gain", 50)))

        existing.currentTextChanged.connect(load_existing)
        self._map_name = name
        self._map_offset = offset
        self._map_gain = gain
        self._map_existing = existing
        return page

    @staticmethod
    def _validate_profile_name(name: str) -> str:
        name = (name or "").strip()
        if not name:
            raise ValueError("Enter a profile name.")
        if len(name) > 48:
            raise ValueError("Profile name must be 48 characters or fewer.")
        for ch in name:
            if not (ch.isalnum() or ch in " _-+./()"):
                raise ValueError(
                    f"Invalid character “{ch}” in name. "
                    "Use letters, numbers, spaces, and _ - + . / ( )"
                )
        return name

    def _read_ntc_table(self, kind: str):
        tbl = getattr(self, f"_{kind}_table")
        n = getattr(self, f"_{kind}_npts")
        temps, adcs = [], []
        for i in range(n):
            it0 = tbl.item(i, 0)
            it1 = tbl.item(i, 1)
            if it0 is None or it1 is None or not it0.text().strip() or not it1.text().strip():
                raise ValueError(f"Row {i + 1}: both Temperature and ADC are required.")
            try:
                t = float(it0.text().replace(",", "."))
                a = float(it1.text().replace(",", "."))
            except Exception:
                raise ValueError(f"Row {i + 1}: “{it0.text()}” / “{it1.text()}” is not a valid number.")
            if not (-80.0 <= t <= 200.0):
                raise ValueError(f"Row {i + 1}: temperature {t} °C out of range (−80…200).")
            if a != a:  # NaN
                raise ValueError(f"Row {i + 1}: ADC is not a number.")
            ai = int(round(a))
            if ai < 0 or ai > 1023:
                raise ValueError(f"Row {i + 1}: ADC {ai} must be 0–1023.")
            temps.append(t)
            adcs.append(ai)
        # Temps should be strictly increasing for sensible interpolation
        for i in range(1, len(temps)):
            if temps[i] <= temps[i - 1]:
                raise ValueError(
                    f"Temperatures must increase down the table "
                    f"(row {i} ≤ row {i + 1}: {temps[i - 1]} → {temps[i]})."
                )
        # ADC for NTC typically decreases with temp – warn only if flat/duplicate
        if len(set(adcs)) < 3:
            raise ValueError("ADC column needs more variation (almost all values identical).")
        return temps, adcs

    def _save_current(self):
        idx = self.tabs.currentIndex()
        try:
            if idx == 0:
                name = self._validate_profile_name(self._clt_name.text())
                if name in CLT_PROFILES:
                    raise ValueError("Name collides with a built-in profile – choose another.")
                temps, adcs = self._read_ntc_table("clt")
                next_id = 100 + len(self.store.get("clt", {}))
                self.store.setdefault("clt", {})[name] = {
                    "id": next_id, "temp": temps, "adc": adcs, "custom": True,
                }
            elif idx == 1:
                name = self._validate_profile_name(self._iat_name.text())
                if name in IAT_PROFILES:
                    raise ValueError("Name collides with a built-in profile – choose another.")
                temps, adcs = self._read_ntc_table("iat")
                next_id = 100 + len(self.store.get("iat", {}))
                self.store.setdefault("iat", {})[name] = {
                    "id": next_id, "temp": temps, "adc": adcs, "custom": True,
                }
            else:
                name = self._validate_profile_name(self._map_name.text())
                if name in MAP_PROFILES:
                    raise ValueError("Name collides with a built-in profile – choose another.")
                offset = float(self._map_offset.value())
                gain = float(self._map_gain.value())
                if gain <= 0:
                    raise ValueError("MAP gain must be greater than 0.")
                if offset < -50 or offset > 200:
                    raise ValueError("MAP offset out of range (−50…200 kPa).")
                next_id = 100 + len(self.store.get("map", {}))
                self.store.setdefault("map", {})[name] = {
                    "id": next_id,
                    "offset": offset,
                    "gain": gain,
                    "custom": True,
                }
            try:
                _save_custom_sensors(self.store)
            except OSError as e:
                raise ValueError(f"Could not write custom_sensors.json: {e}") from e
            self.status.setText(f"Saved custom profile “{name}” → {CUSTOM_SENSORS_JSON.name}")
            self.status.setStyleSheet("color:#44ff88;")
            self._refresh_existing_combos()
        except ValueError as e:
            self.status.setText(str(e))
            self.status.setStyleSheet("color:#ff8866;")
            QMessageBox.warning(self, "Invalid input", str(e))
        except Exception as e:
            self.status.setText(f"Unexpected error: {e}")
            self.status.setStyleSheet("color:#ff8866;")
            QMessageBox.critical(self, "Error", f"Unexpected error while saving:\n{e}")

    def _delete_current(self):
        idx = self.tabs.currentIndex()
        kind = ["clt", "iat", "map"][idx]
        box = getattr(self, f"_{kind}_existing")
        name = box.currentText()
        if name.startswith("—") or name not in self.store.get(kind, {}):
            QMessageBox.information(self, "Delete", "Select a custom profile to delete.")
            return
        del self.store[kind][name]
        _save_custom_sensors(self.store)
        self.status.setText(f"Deleted “{name}”")
        self.status.setStyleSheet("color:#ffaa66;")
        self._refresh_existing_combos()

    def _refresh_existing_combos(self):
        for kind in ("clt", "iat", "map"):
            box = getattr(self, f"_{kind}_existing")
            cur = box.currentText()
            box.blockSignals(True)
            box.clear()
            box.addItem("— new profile —")
            for k in self.store.get(kind, {}):
                box.addItem(k)
            i = box.findText(cur)
            box.setCurrentIndex(i if i >= 0 else 0)
            box.blockSignals(False)


def fmt_float_array(name: str, values: list, per_line: int = 8) -> str:
    parts = [f"{v:g}" for v in values]
    lines = []
    for i in range(0, len(parts), per_line):
        lines.append("  " + ", ".join(parts[i:i + per_line]))
    body = ",\n".join(lines)
    return f"static const float {name}[{len(values)}] = {{\n{body}\n}};"


def fmt_u16_array(name: str, values: list, per_line: int = 8) -> str:
    parts = [str(int(v)) for v in values]
    lines = []
    for i in range(0, len(parts), per_line):
        lines.append("  " + ", ".join(parts[i:i + per_line]))
    body = ",\n".join(lines)
    return f"static const uint16_t {name}[{len(values)}] = {{\n{body}\n}};"


def generate_config_h(cfg: dict) -> str:
    clt = cfg["clt"]
    iat = cfg["iat"]
    mp = cfg["map"]
    return f"""/*
 * ecu_config.h – generated by Engine Setup Tool
 * Engine: {cfg['cylinders']} cyl | Trigger: {cfg['trigger_name']}
 * Load: {cfg['load_name']} | Coil: {cfg['coil_name']}
 * CLT: {cfg['clt_name']} | IAT: {cfg['iat_name']} | MAP: {cfg['map_name']}
 */
#ifndef ECU_CONFIG_H
#define ECU_CONFIG_H

/* ── Engine ─────────────────────────────────────────────────── */
#define CFG_CYLINDERS        {cfg['cylinders']}
#define CFG_TEETH            {cfg['teeth']}
#define CFG_MISSING          {cfg['missing']}
#define CFG_TRIG_ANGLE       {cfg['trig']}
#define CFG_RPM_LIMIT        {cfg['rpm_limit']}
#define CFG_FAN_C            {cfg['fan_c']}

/* 0 = Speed-Density (MAP), 1 = Alpha-N (TPS) */
#define CFG_LOAD_ALPHA_N     {cfg['load_alpha_n']}

/* 0 = dumb coil (ECU controls dwell), 1 = smart coil (logic-level fire only) */
#define CFG_COIL_SMART       {cfg['coil_smart']}

/* Dwell (dumb coils only) – microseconds @ 14 V nominal */
#define CFG_DWELL_NOM_US     {cfg['dwell_nom']}
#define CFG_DWELL_MIN_US     {cfg['dwell_min']}
#define CFG_DWELL_MAX_US     {cfg['dwell_max']}

/* ── Sensor profile IDs ─────────────────────────────────────── */
#define CFG_PROF_CLT         {clt['id']}
#define CFG_PROF_IAT         {iat['id']}
#define CFG_PROF_MAP         {mp['id']}

/* MAP: engMap = MAP_OFFSET_KPA + volts * MAP_GAIN_KPA_PER_V */
#define CFG_MAP_OFFSET_KPA   {mp['offset']}f
#define CFG_MAP_GAIN_KPA_V   {mp['gain']}f

/* Battery ADC voltage divider: Vbat = (ADC/1023 * Vref) * ratio */
#define CFG_BAT_DIVIDER      {cfg['bat_divider']:.4f}f
#define CFG_BAT_ADC_REF_V    {cfg['bat_vref']:.2f}f

/* NTC: 5V—Rpull—ADC—NTC—GND  (0=beta equation, 1=tables) */
#define CFG_USE_NTC_TABLE    0
#define CFG_NTC_R0_OHM       {cfg['ntc_r0']:.1f}f
#define CFG_NTC_BETA         {cfg['ntc_beta']:.1f}f
#define CFG_NTC_PULLUP_OHM   {cfg['ntc_pullup']:.1f}f

/* CLT (ECT) table (used only if CFG_USE_NTC_TABLE=1) */
#define CFG_ECT_N            {len(clt['temp'])}
{fmt_float_array("CFG_ECT_TEMP", clt["temp"])}
{fmt_u16_array("CFG_ECT_ADC", clt["adc"])}

/* IAT table */
#define CFG_IAT_N            {len(iat['temp'])}
{fmt_float_array("CFG_IAT_TEMP", iat["temp"])}
{fmt_u16_array("CFG_IAT_ADC", iat["adc"])}

#endif /* ECU_CONFIG_H */
"""


class EngineSetupTool(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ECU Engine Setup Tool")
        self.setMinimumSize(560, 480)
        self.resize(620, 520)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setSpacing(8)
        root.setContentsMargins(12, 12, 12, 12)

        # Header – compact
        head = QHBoxLayout()
        title = QLabel("Engine Setup")
        title.setFont(QFont("Segoe UI", 15, QFont.Bold))
        title.setStyleSheet("color:#a8d0ff;")
        head.addWidget(title)
        head.addStretch()
        sub = QLabel("Configure once → flash → tune maps")
        sub.setStyleSheet("color:#708090; font-size:11px;")
        head.addWidget(sub)
        root.addLayout(head)

        # ── Tabs (one topic each) ───────────────────────────────
        tabs = QTabWidget()
        tabs.addTab(self._tab_engine(), "1 · Engine")
        tabs.addTab(self._tab_sensors(), "2 · Sensors")
        tabs.addTab(self._tab_ignition(), "3 · Ignition")
        tabs.addTab(self._tab_battery(), "4 · Battery")
        root.addWidget(tabs, 1)

        self.trigger.currentTextChanged.connect(self._on_trigger)
        self._on_trigger(self.trigger.currentText())

        # Summary strip (replaces large preview box)
        self.preview = QLabel("")
        self.preview.setWordWrap(True)
        self.preview.setStyleSheet(
            "background:#12151c; border:1px solid #2a3a58; border-radius:6px; "
            "padding:8px 10px; color:#a0b8d0; font-size:11px;"
        )
        self.preview.setMinimumHeight(48)
        root.addWidget(self.preview)

        # Actions
        btns = QHBoxLayout()
        gen = QPushButton("Generate ecu_config.h")
        gen.setStyleSheet(
            "background:#1a4a2a; color:#88ffaa; border:1px solid #44aa66; "
            "border-radius:5px; padding:8px 16px; font-weight:bold;"
        )
        gen.clicked.connect(self.generate)
        btns.addWidget(gen)
        btns.addWidget(QPushButton("Firmware folder", clicked=self._show_path))
        btns.addStretch()
        q = QPushButton("Quit")
        q.setStyleSheet("background:#5a2020;color:#ffaaaa;border:1px solid #8a3030;")
        q.clicked.connect(self.close)
        btns.addWidget(q)
        root.addLayout(btns)

        self.status = QLabel("Ready")
        self.status.setStyleSheet("color:#8090b0; font-size:11px;")
        root.addWidget(self.status)

        for w in (self.cyl, self.trigger, self.trig_angle, self.rpm_limit,
                  self.fan_c, self.clt, self.iat, self.map_s, self.coil, self.load,
                  self.bat_div, self.bat_vref,
                  self.ntc_pullup, self.ntc_r0, self.ntc_beta):
            if hasattr(w, "currentTextChanged"):
                w.currentTextChanged.connect(self._refresh_preview)
            elif hasattr(w, "valueChanged"):
                w.valueChanged.connect(self._refresh_preview)
        self._refresh_preview()

    def _form(self, parent_lay=None):
        """Consistent form layout for tabs."""
        f = QFormLayout()
        f.setLabelAlignment(Qt.AlignRight | Qt.AlignVCenter)
        f.setFormAlignment(Qt.AlignTop)
        f.setHorizontalSpacing(16)
        f.setVerticalSpacing(10)
        f.setContentsMargins(12, 16, 12, 12)
        return f

    def _tab_engine(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 8, 0, 0)
        form = self._form()

        self.cyl = QSpinBox()
        self.cyl.setRange(1, 8)
        self.cyl.setValue(4)
        form.addRow("Cylinders", self.cyl)

        self.trigger = QComboBox()
        self.trigger.addItems(list(TRIGGERS.keys()))
        self.trigger.setCurrentText("36-1 (common 4-cyl)")
        form.addRow("Crank trigger", self.trigger)

        self.trig_angle = QSpinBox()
        self.trig_angle.setRange(0, 90)
        self.trig_angle.setSuffix(" ° BTDC")
        self.trig_angle.setValue(30)
        form.addRow("Trigger angle", self.trig_angle)

        self.rpm_limit = QSpinBox()
        self.rpm_limit.setRange(3000, 12000)
        self.rpm_limit.setSingleStep(100)
        self.rpm_limit.setValue(7000)
        self.rpm_limit.setSuffix(" rpm")
        form.addRow("RPM limit", self.rpm_limit)

        self.fan_c = QSpinBox()
        self.fan_c.setRange(70, 110)
        self.fan_c.setValue(95)
        self.fan_c.setSuffix(" °C")
        form.addRow("Fan on above", self.fan_c)

        lay.addLayout(form)
        tip = QLabel("Wheel presets fill a typical trigger angle — adjust if your timing mark differs.")
        tip.setStyleSheet("color:#607080; font-size:11px; padding:0 12px;")
        tip.setWordWrap(True)
        lay.addWidget(tip)
        lay.addStretch()
        return page

    def _tab_sensors(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 8, 0, 0)
        form = self._form()

        self.clt = QComboBox()
        self.iat = QComboBox()
        self.map_s = QComboBox()
        self._reload_sensor_combos()
        form.addRow("CLT / ECT profile", self.clt)
        form.addRow("IAT profile", self.iat)
        form.addRow("MAP profile", self.map_s)

        # NTC electrical model (beta equation – used when CFG_USE_NTC_TABLE=0)
        self.ntc_pullup = QDoubleSpinBox()
        self.ntc_pullup.setRange(100, 100000)
        self.ntc_pullup.setDecimals(0)
        self.ntc_pullup.setSingleStep(100)
        self.ntc_pullup.setValue(10000)
        self.ntc_pullup.setSuffix(" Ω")
        self.ntc_pullup.setToolTip(
            "Bias resistor from 5V to ADC node. 10k common DIY; 1k common on OE sensors.")
        form.addRow("NTC pull-up", self.ntc_pullup)

        self.ntc_r0 = QDoubleSpinBox()
        self.ntc_r0.setRange(1000, 100000)
        self.ntc_r0.setDecimals(0)
        self.ntc_r0.setValue(10000)
        self.ntc_r0.setSuffix(" Ω")
        self.ntc_r0.setToolTip("NTC resistance at 25 °C")
        form.addRow("NTC R@25°C", self.ntc_r0)

        self.ntc_beta = QDoubleSpinBox()
        self.ntc_beta.setRange(2000, 5000)
        self.ntc_beta.setDecimals(0)
        self.ntc_beta.setValue(3950)
        self.ntc_beta.setToolTip("Beta constant of the thermistor (typical 10k ≈ 3950)")
        form.addRow("NTC beta", self.ntc_beta)

        lay.addLayout(form)

        row = QHBoxLayout()
        row.setContentsMargins(12, 4, 12, 0)
        prof = QPushButton("Sensor Profiler…")
        prof.setToolTip("Optional table curves (only if you set CFG_USE_NTC_TABLE=1 in firmware)")
        prof.clicked.connect(self._open_profiler)
        row.addWidget(prof)
        row.addStretch()
        lay.addLayout(row)

        tip = QLabel(
            "Wiring: 5V → pull-up → ADC pin → NTC → GND.\n"
            "At ~25°C with 10k/10k, ADC should be near 512. OE 1k bias → set pull-up to 1000."
        )
        tip.setStyleSheet("color:#607080; font-size:11px; padding:8px 12px;")
        tip.setWordWrap(True)
        lay.addWidget(tip)
        lay.addStretch()
        return page

    def _tab_ignition(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 8, 0, 0)
        form = self._form()

        self.coil = QComboBox()
        self.coil.addItems(list(COIL_TYPES.keys()))
        form.addRow("Coil type", self.coil)

        self.load = QComboBox()
        self.load.addItems(list(LOAD_MODES.keys()))
        form.addRow("Load strategy", self.load)
        lay.addLayout(form)

        tip = QLabel(
            "• Dumb coil — ECU times dwell (inductive).\n"
            "• Smart coil — short logic-level fire pulse.\n"
            "• Speed-Density — MAP load (boost OK).  Alpha-N — TPS only (ITB)."
        )
        tip.setStyleSheet("color:#607080; font-size:11px; padding:8px 12px;")
        tip.setWordWrap(True)
        lay.addWidget(tip)
        lay.addStretch()
        return page

    def _tab_battery(self) -> QWidget:
        page = QWidget()
        lay = QVBoxLayout(page)
        lay.setContentsMargins(0, 8, 0, 0)
        form = self._form()

        self.bat_vref = QDoubleSpinBox()
        self.bat_vref.setRange(3.0, 5.5)
        self.bat_vref.setDecimals(2)
        self.bat_vref.setSingleStep(0.01)
        self.bat_vref.setValue(5.00)
        self.bat_vref.setSuffix(" V")
        form.addRow("ADC Vref (AVCC)", self.bat_vref)

        self.bat_div = QDoubleSpinBox()
        self.bat_div.setRange(1.0, 20.0)
        self.bat_div.setDecimals(4)
        self.bat_div.setSingleStep(0.01)
        self.bat_div.setValue(3.00)
        self.bat_div.setToolTip("Vbat = (ADC/1023 × Vref) × divider")
        form.addRow("Divider ratio", self.bat_div)
        lay.addLayout(form)

        # Calibration helper – secondary row
        cal = QGroupBox("Calibrate from measurement")
        cal.setStyleSheet("QGroupBox { font-weight: normal; color:#90a8c0; }")
        cf = QHBoxLayout(cal)
        cf.setSpacing(8)
        self.bat_known = QDoubleSpinBox()
        self.bat_known.setRange(5.0, 20.0)
        self.bat_known.setDecimals(2)
        self.bat_known.setValue(12.60)
        self.bat_known.setSuffix(" V")
        self.bat_adc = QSpinBox()
        self.bat_adc.setRange(0, 1023)
        self.bat_adc.setValue(860)
        cal_btn = QPushButton("Compute ratio")
        cal_btn.setToolTip("ratio = KnownV / (ADC/1023 × Vref)")
        cal_btn.clicked.connect(self._calibrate_bat_divider)
        cf.addWidget(QLabel("Known"))
        cf.addWidget(self.bat_known)
        cf.addWidget(QLabel("ADC"))
        cf.addWidget(self.bat_adc)
        cf.addWidget(cal_btn)
        lay.addWidget(cal)

        tip = QLabel("Typical dividers: 20k/10k ≈ 3.0 · 30k/10k ≈ 4.0 — keep ADC pin under 5 V.")
        tip.setStyleSheet("color:#607080; font-size:11px; padding:4px 12px;")
        tip.setWordWrap(True)
        lay.addWidget(tip)
        lay.addStretch()
        return page

    def _calibrate_bat_divider(self):
        """ratio = Vknown / (ADC/1023 * Vref)"""
        try:
            adc = int(self.bat_adc.value())
            vref = float(self.bat_vref.value())
            vknown = float(self.bat_known.value())
        except Exception:
            QMessageBox.warning(self, "Calibration", "Invalid number in calibration fields.")
            return
        if adc <= 0 or adc > 1023:
            QMessageBox.warning(self, "Calibration", "ADC must be in range 1–1023.")
            return
        if vref < 3.0 or vref > 5.5:
            QMessageBox.warning(self, "Calibration", "ADC reference must be between 3.0 and 5.5 V.")
            return
        if vknown < 5.0 or vknown > 20.0:
            QMessageBox.warning(
                self, "Calibration",
                "Known battery voltage looks out of range (expected about 5–20 V)."
            )
            return
        vadc = (adc / 1023.0) * vref
        if vadc < 0.05:
            QMessageBox.warning(self, "Calibration", "ADC voltage too low – check wiring / ADC value.")
            return
        if vadc >= vref * 0.98:
            QMessageBox.warning(
                self, "Calibration",
                "ADC is near full scale – increase top resistor so the pin stays under 5 V "
                "at maximum battery voltage."
            )
            # still allow compute so user can see the number
        ratio = vknown / vadc
        if ratio < 1.0 or ratio > 20.0:
            QMessageBox.warning(
                self, "Calibration",
                f"Computed ratio {ratio:.3f} is outside 1–20.\n"
                "Check that Known V and ADC belong to the same measurement."
            )
            return
        self.bat_div.setValue(ratio)
        self.status.setText(
            f"Divider ratio set to {ratio:.4f}  "
            f"(ADC pin ≈ {vadc:.3f} V @ {adc}, known {vknown:.2f} V)"
        )
        self.status.setStyleSheet("color:#44ff88;")
        self._refresh_preview()

    def _on_trigger(self, name: str):
        t = TRIGGERS.get(name, {"trig": 30})
        self.trig_angle.setValue(int(t["trig"]))

    def _reload_sensor_combos(self):
        def refill(box: QComboBox, profiles: dict):
            cur = box.currentText() if box.count() else ""
            box.blockSignals(True)
            box.clear()
            box.addItems(list(profiles.keys()))
            i = box.findText(cur)
            if i >= 0:
                box.setCurrentIndex(i)
            box.blockSignals(False)

        refill(self.clt, all_clt_profiles())
        refill(self.iat, all_iat_profiles())
        refill(self.map_s, all_map_profiles())

    def _open_profiler(self):
        dlg = SensorProfilerDialog(self)
        dlg.exec()
        self._reload_sensor_combos()
        self._refresh_preview()
        self.status.setText("Sensor lists refreshed from custom_sensors.json")
        self.status.setStyleSheet("color:#8090b0;")

    def _collect(self) -> dict:
        tr_name = self.trigger.currentText()
        tr = TRIGGERS[tr_name]
        clt_name = self.clt.currentText()
        iat_name = self.iat.currentText()
        map_name = self.map_s.currentText()
        coil_name = self.coil.currentText()
        load_name = self.load.currentText()
        smart = COIL_TYPES[coil_name]
        clt_p = all_clt_profiles()
        iat_p = all_iat_profiles()
        map_p = all_map_profiles()
        if clt_name not in clt_p or iat_name not in iat_p or map_name not in map_p:
            raise KeyError("Sensor profile missing – refresh dropdowns")
        return {
            "cylinders": self.cyl.value(),
            "trigger_name": tr_name,
            "teeth": tr["teeth"],
            "missing": tr["missing"],
            "trig": self.trig_angle.value(),
            "rpm_limit": self.rpm_limit.value(),
            "fan_c": self.fan_c.value(),
            "load_alpha_n": LOAD_MODES[load_name],
            "load_name": load_name,
            "coil_smart": smart,
            "coil_name": coil_name,
            "dwell_nom": 1800 if smart else 3000,
            "dwell_min": 800 if smart else 1500,
            "dwell_max": 2500 if smart else 4500,
            "clt_name": clt_name,
            "iat_name": iat_name,
            "map_name": map_name,
            "clt": clt_p[clt_name],
            "iat": iat_p[iat_name],
            "map": map_p[map_name],
            "bat_divider": float(self.bat_div.value()),
            "bat_vref": float(self.bat_vref.value()),
            "ntc_pullup": float(self.ntc_pullup.value()),
            "ntc_r0": float(self.ntc_r0.value()),
            "ntc_beta": float(self.ntc_beta.value()),
        }

    def _refresh_preview(self):
        try:
            c = self._collect()
        except Exception:
            self.preview.setText("Select valid options on each tab.")
            return
        coil_short = "smart" if c["coil_smart"] else "dumb"
        load_short = "Alpha-N" if c["load_alpha_n"] else "SD"
        self.preview.setText(
            f"{c['cylinders']}-cyl · {c['teeth']}-{c['missing']} @ {c['trig']}° · "
            f"{c['rpm_limit']} rpm · {load_short} · coil {coil_short}\n"
            f"NTC pull-up {c['ntc_pullup']:.0f}Ω · R25 {c['ntc_r0']:.0f}Ω · β {c['ntc_beta']:.0f} · "
            f"BAT ×{c['bat_divider']:.3f}"
        )

    def _validate_config(self, cfg: dict) -> list[str]:
        """Return list of human-readable errors (empty if OK)."""
        errs = []
        if cfg["cylinders"] < 1 or cfg["cylinders"] > 8:
            errs.append("Cylinders must be 1–8.")
        if cfg["teeth"] < 12 or cfg["teeth"] > 60:
            errs.append("Crank teeth must be 12–60.")
        if cfg["missing"] < 0 or cfg["missing"] >= cfg["teeth"]:
            errs.append("Missing teeth must be ≥ 0 and less than total teeth.")
        if (cfg["teeth"] - cfg["missing"]) < 3:
            errs.append("Physical teeth (total − missing) must be at least 3.")
        if cfg["trig"] < 0 or cfg["trig"] > 90:
            errs.append("Trigger angle must be 0–90° BTDC.")
        if cfg["rpm_limit"] < 3000 or cfg["rpm_limit"] > 12000:
            errs.append("RPM limit must be 3000–12000.")
        if cfg["fan_c"] < 70 or cfg["fan_c"] > 110:
            errs.append("Fan setpoint must be 70–110 °C.")
        if cfg["bat_divider"] < 1.0 or cfg["bat_divider"] > 20.0:
            errs.append("Battery divider ratio must be 1.0–20.0.")
        if cfg["bat_vref"] < 3.0 or cfg["bat_vref"] > 5.5:
            errs.append("ADC reference must be 3.0–5.5 V.")
        clt, iat, mp = cfg["clt"], cfg["iat"], cfg["map"]
        if not isinstance(clt.get("temp"), list) or not isinstance(clt.get("adc"), list):
            errs.append("CLT profile is missing temp/ADC tables.")
        elif len(clt["temp"]) != 13 or len(clt["adc"]) != 13:
            errs.append("CLT table must have exactly 13 temp/ADC points.")
        else:
            for i, a in enumerate(clt["adc"]):
                if not (0 <= int(a) <= 1023):
                    errs.append(f"CLT ADC[{i}] out of range 0–1023.")
                    break
        if not isinstance(iat.get("temp"), list) or not isinstance(iat.get("adc"), list):
            errs.append("IAT profile is missing temp/ADC tables.")
        elif len(iat["temp"]) != 11 or len(iat["adc"]) != 11:
            errs.append("IAT table must have exactly 11 temp/ADC points.")
        else:
            for i, a in enumerate(iat["adc"]):
                if not (0 <= int(a) <= 1023):
                    errs.append(f"IAT ADC[{i}] out of range 0–1023.")
                    break
        try:
            g = float(mp.get("gain", 0))
            o = float(mp.get("offset", 0))
            if g <= 0:
                errs.append("MAP gain must be > 0.")
            if o < -50 or o > 200:
                errs.append("MAP offset out of range (−50…200).")
        except (TypeError, ValueError):
            errs.append("MAP profile has invalid offset/gain.")
        return errs

    def generate(self):
        try:
            cfg = self._collect()
        except KeyError as e:
            QMessageBox.warning(self, "Sensor profile", str(e))
            self.status.setText(str(e))
            self.status.setStyleSheet("color:#ff8866;")
            return
        except Exception as e:
            QMessageBox.warning(self, "Invalid selection", f"Cannot build config:\n{e}")
            self.status.setText(str(e))
            self.status.setStyleSheet("color:#ff8866;")
            return

        errs = self._validate_config(cfg)
        if errs:
            msg = "Fix the following before generating firmware:\n\n• " + "\n• ".join(errs)
            QMessageBox.warning(self, "Invalid inputs", msg)
            self.status.setText(errs[0])
            self.status.setStyleSheet("color:#ff8866;")
            return

        try:
            FIRMWARE_DIR.mkdir(parents=True, exist_ok=True)
            text = generate_config_h(cfg)
            CONFIG_H.write_text(text, encoding="utf-8")
            PROFILE_JSON.write_text(json.dumps({
                "cylinders": cfg["cylinders"],
                "teeth": cfg["teeth"],
                "missing": cfg["missing"],
                "trig_angle": cfg["trig"],
                "rpm_limit": cfg["rpm_limit"],
                "fan_c": cfg["fan_c"],
                "load_alpha_n": bool(cfg["load_alpha_n"]),
                "coil_smart": bool(cfg["coil_smart"]),
                "bat_divider": cfg["bat_divider"],
                "bat_vref": cfg["bat_vref"],
                "clt": cfg["clt_name"],
                "iat": cfg["iat_name"],
                "map": cfg["map_name"],
            }, indent=2), encoding="utf-8")
        except OSError as e:
            QMessageBox.critical(self, "Write failed", f"Could not write config files:\n{e}")
            self.status.setText(f"Write failed: {e}")
            self.status.setStyleSheet("color:#ff8866;")
            return
        except Exception as e:
            QMessageBox.critical(self, "Generate failed", f"Unexpected error:\n{e}")
            self.status.setText(str(e))
            self.status.setStyleSheet("color:#ff8866;")
            return

        self.status.setText(f"Wrote {CONFIG_H.name} and engine_profiles.json – flash firmware next")
        self.status.setStyleSheet("color:#44ff88;")
        QMessageBox.information(
            self, "Firmware config generated",
            f"Saved:\n{CONFIG_H}\n{PROFILE_JSON}\n\n"
            "Open ecu_firmware in Arduino IDE and Upload to the Uno.\n"
            "Then use the main tuner for maps only."
        )

    def _show_path(self):
        QMessageBox.information(self, "Firmware folder", str(FIRMWARE_DIR))


def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(DARK)
    w = EngineSetupTool()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
