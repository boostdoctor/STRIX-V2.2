"""STRIX V2 dialogs: program settings, sensor cal, engine settings, trigger wizard."""
from __future__ import annotations

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QDialog, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QComboBox,
    QCheckBox, QSpinBox, QDoubleSpinBox, QGroupBox, QFormLayout, QTabWidget,
    QWidget, QTableWidget, QTableWidgetItem, QFileDialog, QMessageBox,
    QDialogButtonBox, QListWidget, QSlider, QScrollArea,
)

from strix_v2.constants import (
    OPTIONAL_STRIP, MAP_SENSORS, TEMP_SENSORS, O2_MODES,
    MAP_KPA_MAX_LIMIT, MAP_KPA_MIN, make_map_bins, WHEEL_PROFILES,
    FIRING_ORDERS_BY_CYL, ALL_FIRING_ORDERS,
    DEFAULT_ECT_COMP, DEFAULT_IAT_COMP, DEFAULT_BAT_COMP,
)
from strix_v2.tcal import default_engine_settings, save_tcal, load_tcal


class ProgramSettingsDialog(QDialog):
    def __init__(self, strip_keys: set[str], parent=None):
        super().__init__(parent)
        self.setWindowTitle("Program settings")
        self.setMinimumWidth(360)
        lay = QVBoxLayout(self)
        lay.addWidget(QLabel("Optional live-strip fields (RPM/TPS/MAP/ECT/IAT/SYNC always shown):"))
        self.checks: dict[str, QCheckBox] = {}
        for key, lab in OPTIONAL_STRIP:
            cb = QCheckBox(lab)
            cb.setChecked(key in strip_keys)
            self.checks[key] = cb
            lay.addWidget(cb)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        lay.addWidget(bb)

    def selected_keys(self) -> set[str]:
        return {k for k, cb in self.checks.items() if cb.isChecked()}


class SensorCalDialog(QDialog):
    def __init__(self, settings: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Sensor calibration")
        self.setMinimumSize(520, 480)
        self.settings = settings
        sens = settings.setdefault("sensors", default_engine_settings()["sensors"])

        root = QVBoxLayout(self)
        tabs = QTabWidget()
        root.addWidget(tabs)

        # MAP
        wmap = QWidget()
        fl = QFormLayout(wmap)
        self.map_en = QCheckBox("MAP sensor enabled (fuel/timing load)")
        self.map_en.setChecked(sens.get("map", {}).get("enabled", True))
        fl.addRow(self.map_en)
        self.map_preset = QComboBox()
        for name, _ in MAP_SENSORS:
            self.map_preset.addItem(name)
        preset_name = sens.get("map", {}).get("preset", "Custom")
        i = self.map_preset.findText(preset_name)
        if i >= 0:
            self.map_preset.setCurrentIndex(i)
        fl.addRow("MAP sensor", self.map_preset)
        self.map_max = QSpinBox()
        self.map_max.setRange(MAP_KPA_MIN + 20, MAP_KPA_MAX_LIMIT)
        self.map_max.setValue(int(settings.get("map_kpa_max") or 240))
        self.map_max.setSuffix(" kPa max")
        fl.addRow("Scale max", self.map_max)
        self.map_preset.currentIndexChanged.connect(self._map_preset_changed)
        tabs.addTab(wmap, "MAP")

        # TPS
        wtps = QWidget()
        fl = QFormLayout(wtps)
        self.tps_en = QCheckBox("TPS enabled")
        self.tps_en.setChecked(sens.get("tps", {}).get("enabled", True))
        fl.addRow(self.tps_en)
        tabs.addTab(wtps, "TPS")

        # ECT
        wect = QWidget()
        fl = QFormLayout(wect)
        self.ect_en = QCheckBox("ECT enabled (fuel/timing compensation)")
        self.ect_en.setChecked(sens.get("ect", {}).get("enabled", True))
        fl.addRow(self.ect_en)
        self.ect_preset = QComboBox()
        for name, _ in TEMP_SENSORS:
            self.ect_preset.addItem(name)
        i = self.ect_preset.findText(sens.get("ect", {}).get("preset", "Custom"))
        if i >= 0:
            self.ect_preset.setCurrentIndex(i)
        fl.addRow("ECT sensor", self.ect_preset)
        fl.addRow(QLabel("ADC · Fuel % · Ign ° (14 rows)"))
        self.ect_table = QTableWidget(14, 3)
        self.ect_table.setHorizontalHeaderLabels(["ADC", "Fuel %", "Ign °"])
        ect_rows = sens.get("ect", {}).get("comp") or DEFAULT_ECT_COMP
        for r in range(14):
            row = ect_rows[r] if r < len(ect_rows) else DEFAULT_ECT_COMP[r]
            for c, v in enumerate(row[:3]):
                self.ect_table.setItem(r, c, QTableWidgetItem(str(v)))
        fl.addRow(self.ect_table)
        tabs.addTab(wect, "ECT")

        # IAT
        wiat = QWidget()
        fl = QFormLayout(wiat)
        self.iat_en = QCheckBox("IAT enabled (fuel/timing compensation)")
        self.iat_en.setChecked(sens.get("iat", {}).get("enabled", True))
        fl.addRow(self.iat_en)
        self.iat_preset = QComboBox()
        for name, _ in TEMP_SENSORS:
            self.iat_preset.addItem(name)
        i = self.iat_preset.findText(sens.get("iat", {}).get("preset", "Custom"))
        if i >= 0:
            self.iat_preset.setCurrentIndex(i)
        fl.addRow("IAT sensor", self.iat_preset)
        fl.addRow(QLabel("ADC · Fuel % · Ign ° (14 rows)"))
        self.iat_table = QTableWidget(14, 3)
        self.iat_table.setHorizontalHeaderLabels(["ADC", "Fuel %", "Ign °"])
        iat_rows = sens.get("iat", {}).get("comp") or DEFAULT_IAT_COMP
        for r in range(14):
            row = iat_rows[r] if r < len(iat_rows) else DEFAULT_IAT_COMP[r]
            for c, v in enumerate(row[:3]):
                self.iat_table.setItem(r, c, QTableWidgetItem(str(v)))
        fl.addRow(self.iat_table)
        tabs.addTab(wiat, "IAT")

        # BAT
        wbat = QWidget()
        bfl = QVBoxLayout(wbat)
        bfl.addWidget(QLabel("Battery ADC · Fuel % · Ign ° (10 rows)"))
        self.bat_table = QTableWidget(10, 3)
        self.bat_table.setHorizontalHeaderLabels(["ADC", "Fuel %", "Ign °"])
        bat_rows = sens.get("bat", {}).get("comp") or DEFAULT_BAT_COMP
        for r in range(10):
            row = bat_rows[r] if r < len(bat_rows) else DEFAULT_BAT_COMP[r]
            for c, v in enumerate(row[:3]):
                self.bat_table.setItem(r, c, QTableWidgetItem(str(v)))
        bfl.addWidget(self.bat_table)
        tabs.addTab(wbat, "Battery")

        # O2
        wo2 = QWidget()
        ol = QVBoxLayout(wo2)
        self.o2_en = QCheckBox("O2 sensor enabled (closed-loop / trims)")
        self.o2_en.setChecked(sens.get("o2", {}).get("enabled", False))
        ol.addWidget(self.o2_en)
        row = QHBoxLayout()
        row.addWidget(QLabel("Type"))
        self.o2_mode = QComboBox()
        self.o2_mode.addItems(list(O2_MODES))
        mode = sens.get("o2", {}).get("mode", "Disabled")
        i = self.o2_mode.findText(mode)
        if i >= 0:
            self.o2_mode.setCurrentIndex(i)
        row.addWidget(self.o2_mode)
        row.addStretch(1)
        ol.addLayout(row)
        ol.addWidget(QLabel("Wideband: AFR vs voltage (controller 0–5 V → ECU 0–3.3 V scaled)"))
        ol.addWidget(QLabel("Narrowband: voltage table 0.1–1.0 V"))
        self.o2_table = QTableWidget(10, 2)
        self.o2_table.setHorizontalHeaderLabels(["AFR / point", "Voltage"])
        ol.addWidget(self.o2_table)
        self.o2_mode.currentTextChanged.connect(self._o2_mode_changed)
        self._o2_mode_changed(self.o2_mode.currentText())
        # load values
        o2 = sens.get("o2", {})
        self._fill_o2_table(o2)
        tabs.addTab(wo2, "O2")

        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        root.addWidget(bb)

    def _map_preset_changed(self, _i=None):
        name = self.map_preset.currentText()
        for n, mx in MAP_SENSORS:
            if n == name and mx:
                self.map_max.setValue(int(mx))
                break

    def _o2_mode_changed(self, mode: str):
        if mode == "Narrowband":
            self.o2_table.setRowCount(5)
            self.o2_table.setHorizontalHeaderLabels(["Point", "Voltage (V)"])
            defaults = [0.1, 0.3, 0.5, 0.7, 0.9]
            for r, v in enumerate(defaults):
                self.o2_table.setItem(r, 0, QTableWidgetItem(str(r + 1)))
                self.o2_table.setItem(r, 1, QTableWidgetItem(f"{v:.1f}"))
        elif mode == "Wideband":
            self.o2_table.setRowCount(10)
            self.o2_table.setHorizontalHeaderLabels(["AFR", "Voltage (0–5 V)"])
            defaults = [
                (10.0, 0.0), (11.0, 0.5), (12.0, 1.0), (13.0, 1.5), (14.0, 2.0),
                (14.7, 2.5), (16.0, 3.0), (18.0, 3.5), (20.0, 4.0), (22.0, 4.5),
            ]
            for r, (a, v) in enumerate(defaults):
                self.o2_table.setItem(r, 0, QTableWidgetItem(f"{a:.1f}"))
                self.o2_table.setItem(r, 1, QTableWidgetItem(f"{v:.2f}"))
        else:
            self.o2_table.setRowCount(0)

    def _fill_o2_table(self, o2: dict):
        mode = o2.get("mode", "Disabled")
        self._o2_mode_changed(mode)
        if mode == "Narrowband":
            tbl = o2.get("nb_table") or []
            for r, row in enumerate(tbl[:5]):
                if len(row) >= 2:
                    self.o2_table.setItem(r, 0, QTableWidgetItem(str(row[0])))
                    self.o2_table.setItem(r, 1, QTableWidgetItem(str(row[1])))
        elif mode == "Wideband":
            tbl = o2.get("wb_table") or []
            for r, row in enumerate(tbl[:10]):
                if len(row) >= 2:
                    self.o2_table.setItem(r, 0, QTableWidgetItem(str(row[0])))
                    self.o2_table.setItem(r, 1, QTableWidgetItem(str(row[1])))

    def _read_o2_table(self) -> list:
        out = []
        for r in range(self.o2_table.rowCount()):
            a = self.o2_table.item(r, 0)
            b = self.o2_table.item(r, 1)
            try:
                out.append([float(a.text() if a else 0), float(b.text() if b else 0)])
            except ValueError:
                out.append([0.0, 0.0])
        return out


    def apply_to(self, settings: dict) -> None:
        sens = settings.setdefault("sensors", {})
        sens["map"] = {
            "enabled": self.map_en.isChecked(),
            "preset": self.map_preset.currentText(),
            "max_kpa": self.map_max.value(),
        }
        settings["map_kpa_max"] = self.map_max.value()
        settings["map_bins"] = make_map_bins(self.map_max.value())
        sens["tps"] = {"enabled": self.tps_en.isChecked(), "preset": "Custom"}
        def _read_comp(table, rows):
            out = []
            for r in range(rows):
                vals = []
                for c in range(3):
                    it = table.item(r, c)
                    try:
                        vals.append(float(it.text() if it else 0))
                    except ValueError:
                        vals.append(0.0)
                out.append(vals)
            return out
        sens["ect"] = {
            "enabled": self.ect_en.isChecked(),
            "preset": self.ect_preset.currentText(),
            "comp": _read_comp(self.ect_table, 14),
        }
        sens["iat"] = {
            "enabled": self.iat_en.isChecked(),
            "preset": self.iat_preset.currentText(),
            "comp": _read_comp(self.iat_table, 14),
        }
        sens["bat"] = {
            "enabled": True,
            "comp": _read_comp(self.bat_table, 10),
        }
        mode = self.o2_mode.currentText()
        tbl = self._read_o2_table()
        sens["o2"] = {
            "enabled": self.o2_en.isChecked() and mode != "Disabled",
            "mode": mode,
            "nb_table": tbl if mode == "Narrowband" else sens.get("o2", {}).get("nb_table", []),
            "wb_table": tbl if mode == "Wideband" else sens.get("o2", {}).get("wb_table", []),
        }




class EngineSettingsDialog(QDialog):
    """V1 engine options + V2 fuel/prime/hybrid — tabbed."""

    WHEELS = [(wid, name) for wid, name, _t, _m in WHEEL_PROFILES]
    FIRE_ORDERS = list(ALL_FIRING_ORDERS)

    def __init__(self, settings: dict, rpm: int = 0, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Engine settings")
        self.setMinimumWidth(640)
        self.setMinimumHeight(560)
        self.resize(720, 640)
        self._rpm = rpm
        locked = rpm > 0
        root = QVBoxLayout(self)
        if locked:
            root.addWidget(QLabel("⚠ RPM > 0 — wheel/cyl locked. Trigger angle slider stays live."))

        tabs = QTabWidget()
        root.addWidget(tabs)

        def _spin(lo, hi, val, suffix=""):
            s = QSpinBox(); s.setRange(lo, hi); s.setValue(int(val))
            if suffix:
                s.setSuffix(suffix)
            return s

        # ── Core / Trigger ────────────────────────────────
        core = QWidget(); form = QFormLayout(core)
        self.wheel = QComboBox()
        for wid, name in self.WHEELS:
            self.wheel.addItem(name, wid)
        wi = int(settings.get("wheel_id") or 0)
        idx = max(0, self.wheel.findData(wi))
        self.wheel.setCurrentIndex(idx)
        self.cyl = _spin(1, 8, settings.get("cylinders") or 4)
        self.cyl.setToolTip("1–4: sequential allowed. 5/6/8: batch injection only. Stop engine to change.")
        self.teeth = _spin(4, 60, settings.get("teeth") or 36)
        self.missing = _spin(0, 3, settings.get("missing") or 1)
        # Trigger angle — slider, adjustable while engine runs
        self.trig = QSlider(Qt.Horizontal)
        self.trig.setRange(0, 360)
        self.trig.setValue(int(settings.get("trig_angle") or 30))
        self.trig.setTickPosition(QSlider.TicksBelow)
        self.trig.setTickInterval(30)
        self.trig_lbl = QLabel(f"{self.trig.value()} °")
        self.trig_lbl.setMinimumWidth(48)
        self.trig.valueChanged.connect(lambda v: self.trig_lbl.setText(f"{v} °"))
        self.trig.setToolTip("Tooth #1 → TDC angle. Live-adjustable while running (SET:TRIG).")
        self.trig.setEnabled(True)  # always editable
        self.trig.valueChanged.connect(self._live_trig)
        trig_row = QHBoxLayout()
        trig_row.addWidget(self.trig, 1)
        trig_row.addWidget(self.trig_lbl)
        self.fire = QComboBox()
        self._refill_fire_orders(int(settings.get("cylinders") or 4), settings.get("firing_order") or "1-3-4-2")
        self.coil = QComboBox(); self.coil.addItems(["Smart", "Dumb", "Distributor"])
        ct = settings.get("coil_type") or "Smart"
        if ct in ("Smart", "Dumb", "Distributor"):
            self.coil.setCurrentText(ct)
        for w in (self.wheel, self.cyl, self.teeth, self.missing, self.fire, self.coil):
            w.setEnabled(not locked)
        form.addRow("Crank wheel profile", self.wheel)
        form.addRow("Cylinders", self.cyl)
        form.addRow("Trigger angle (live)", trig_row)
        self.cam_home = QCheckBox("Cam home present (full sequential)")
        self.cam_home.setChecked(bool(settings.get("cam_home", True)))
        self.cam_home.setEnabled(not locked)
        form.addRow(self.cam_home)
        form.addRow("Firing order", self.fire)
        form.addRow("Ignition coil", self.coil)
        # RPM limiter moved from Controls → Core
        self.rpm_lim = _spin(2000, 12000, settings.get("rpm_limit") or 7000, " RPM")
        self.rpm_cut = QComboBox(); self.rpm_cut.addItems(["Hard", "Soft"])
        rc = settings.get("rpm_cut_mode") or "Hard"
        self.rpm_cut.setCurrentText(rc if rc in ("Hard", "Soft") else "Hard")
        form.addRow("RPM limit", self.rpm_lim)
        form.addRow("Limiter type", self.rpm_cut)
        self.max_adv = _spin(0, 60, settings.get("max_advance") or 40, " °")
        self.max_adv.setToolTip("Maximum ignition advance (BTDC) — clamps map + live timing")
        self.max_ret = _spin(0, 30, settings.get("max_retard") or 10, " °")
        self.max_ret.setToolTip("Maximum retard past TDC (ATDC). Final timing ≥ −max_retard")
        form.addRow("Max advance", self.max_adv)
        form.addRow("Max retard", self.max_ret)
        tabs.addTab(core, "Core")

        # ── Fuel ──────────────────────────────────────────
        fuel_inner = QWidget()
        fl = QVBoxLayout(fuel_inner)
        fl.setSpacing(12)
        fl.setContentsMargins(12, 12, 12, 12)
        load_grp = QGroupBox("Load calculation"); load_form = QFormLayout(load_grp)
        self.load_mode = QComboBox(); self.load_mode.addItems(["MAP", "TPS", "HYBRID"])
        lm = (settings.get("load_mode") or "MAP").upper()
        if lm in ("ALPHA-N", "ALPHA_N"):
            lm = "TPS"
        if lm not in ("MAP", "TPS", "HYBRID"):
            lm = "MAP"
        self.load_mode.setCurrentText(lm)
        self.load_mode.setEnabled(not locked)
        self.map_kpa_max = _spin(100, 500, settings.get("map_kpa_max") or 240, " kPa")
        load_form.addRow("Load axis", self.load_mode)
        load_form.addRow("MAP scale max", self.map_kpa_max)
        fl.addWidget(load_grp)

        prime_grp = QGroupBox("Priming (V2)"); prime_form = QFormLayout(prime_grp)
        prime_form.setHorizontalSpacing(16)
        prime_form.setVerticalSpacing(10)
        prime_form.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)
        self.fp_prime = _spin(0, 15000, settings.get("fp_prime_ms") or 2000, " ms")
        self.fp_prime.setSingleStep(100)
        self.fp_prime.setToolTip("Fuel pump on after power-up / when applied")
        self.inj_prime_en = QCheckBox("Enable start injector prime pulse")
        self.inj_prime_en.setChecked(bool(settings.get("start_prime_enable", True)))
        self.inj_prime = _spin(0, 500, settings.get("start_prime_ms") or 50, " ms")
        self.inj_prime.setSingleStep(5)
        self.inj_prime.setEnabled(self.inj_prime_en.isChecked())
        self.inj_prime_en.toggled.connect(self.inj_prime.setEnabled)
        prime_form.addRow("Fuel pump prime", self.fp_prime)
        prime_form.addRow(self.inj_prime_en)
        prime_form.addRow("Start inj. prime", self.inj_prime)
        fl.addWidget(prime_grp)

        inj_grp = QGroupBox("Injection"); inj_form = QFormLayout(inj_grp)
        inj_form.setLabelAlignment(Qt.AlignRight)
        inj_form.setFormAlignment(Qt.AlignLeft | Qt.AlignTop)
        inj_form.setHorizontalSpacing(16)
        inj_form.setVerticalSpacing(10)
        inj_form.setFieldGrowthPolicy(QFormLayout.ExpandingFieldsGrow)
        self.inj_flow = QDoubleSpinBox()
        self.inj_flow.setMinimumWidth(140)
        self.inj_flow.setRange(50.0, 3000.0)
        self.inj_flow.setDecimals(0)
        self.inj_flow.setSingleStep(10)
        self.inj_flow.setSuffix(" cc/min")
        self.inj_flow.setValue(float(settings.get("inj_flow_cc") or 220))
        self.inj_flow.setToolTip("Injector flow at rated pressure (used for VE → pulse width)")
        self.req_fuel = QDoubleSpinBox()
        self.req_fuel.setMinimumWidth(140)
        self.req_fuel.setRange(0.5, 15.0)
        self.req_fuel.setDecimals(2)
        self.req_fuel.setSingleStep(0.1)
        self.req_fuel.setSuffix(" ms")
        self.req_fuel.setValue(float(settings.get("req_fuel_ms") or 2.5))
        self.req_fuel.setToolTip("Base pulse at 100% VE, 100 kPa absolute, 20 °C")
        self.ve_mode_cb = QCheckBox("Fuel map is VE % (not raw ms)")
        self.ve_mode_cb.setChecked(bool(settings.get("ve_mode", True)))
        inj_form.addRow("Injector flow", self.inj_flow)
        self.fuel_press = QDoubleSpinBox()
        self.fuel_press.setMinimumWidth(140)
        self.fuel_press.setRange(1.0, 10.0)
        self.fuel_press.setDecimals(2)
        self.fuel_press.setSingleStep(0.1)
        self.fuel_press.setSuffix(" bar")
        self.fuel_press.setValue(float(settings.get("fuel_pressure_bar") or 3.0))
        self.fuel_press.setToolTip("Actual fuel rail pressure (gauge/regulator)")
        self.fuel_press_rated = QDoubleSpinBox()
        self.fuel_press_rated.setMinimumWidth(140)
        self.fuel_press_rated.setRange(1.0, 10.0)
        self.fuel_press_rated.setDecimals(2)
        self.fuel_press_rated.setSingleStep(0.1)
        self.fuel_press_rated.setSuffix(" bar")
        self.fuel_press_rated.setValue(float(settings.get("fuel_pressure_rated_bar") or 3.0))
        self.fuel_press_rated.setToolTip("Pressure at which injector flow (cc/min) is rated")
        inj_form.addRow("Fuel pressure (rail)", self.fuel_press)
        inj_form.addRow("Flow rated at", self.fuel_press_rated)
        inj_form.addRow("Req fuel (100% VE)", self.req_fuel)
        inj_form.addRow(self.ve_mode_cb)
        self.max_inj = QDoubleSpinBox()
        self.max_inj.setMinimumWidth(140)
        self.max_inj.setRange(1.0, 30.0)
        self.max_inj.setDecimals(1)
        self.max_inj.setSingleStep(0.5)
        self.max_inj.setSuffix(" ms")
        self.max_inj.setValue(float(settings.get("max_inj_ms") or 15.0))
        self.max_inj.setToolTip("Hard maximum injector pulse width (after enrichments)")
        inj_form.addRow("Max injection", self.max_inj)
        self.inj_mode = QComboBox(); self.inj_mode.addItems(["Sequential", "Batch", "Batch above RPM"])
        im = settings.get("inj_mode") or "Sequential"
        if im in ("Sequential", "Batch", "Batch above RPM"):
            self.inj_mode.setCurrentText(im)
        self.batch_rpm = _spin(1000, 8000, settings.get("batch_above_rpm") or 3000, " RPM")
        self.batch_rpm.setEnabled(self.inj_mode.currentText() == "Batch above RPM")
        def _cyl_mode_sync(*_):
            c = self.cyl.value()
            mode = self.inj_mode.currentText()
            if c in (5, 6, 8) and mode == "Sequential":
                self.inj_mode.blockSignals(True)
                self.inj_mode.setCurrentText("Batch")
                self.inj_mode.blockSignals(False)
                self.batch_rpm.setEnabled(False)
            elif mode == "Sequential" and c > 4:
                self.cyl.blockSignals(True)
                self.cyl.setValue(4)
                self.cyl.blockSignals(False)
            self.batch_rpm.setEnabled(self.inj_mode.currentText() == "Batch above RPM")
        self.inj_mode.currentTextChanged.connect(_cyl_mode_sync)
        self.cyl.valueChanged.connect(_cyl_mode_sync)
        self.cyl.valueChanged.connect(lambda v: self._refill_fire_orders(int(v)))
        _cyl_mode_sync()
        def _cyl_changed(n):
            if int(n) in (5, 6, 8):
                self.inj_mode.setCurrentText("Batch")
                self.inj_mode.setEnabled(False)
                self.batch_rpm.setEnabled(False)
            else:
                self.inj_mode.setEnabled(True)
                self.batch_rpm.setEnabled(self.inj_mode.currentText() == "Batch above RPM")
        self.cyl.valueChanged.connect(_cyl_changed)
        _cyl_changed(self.cyl.value())
        inj_form.addRow("Injection mode", self.inj_mode)
        inj_form.addRow("Batch above", self.batch_rpm)
        fl.addWidget(inj_grp)
        fl.addStretch(1)
        fuel = QScrollArea()
        fuel.setWidgetResizable(True)
        fuel.setFrameShape(QScrollArea.NoFrame)
        fuel.setWidget(fuel_inner)
        tabs.addTab(fuel, "Fuel")

        # ── Throttle / Idle ───────────────────────────────
        thr = QWidget(); tf = QFormLayout(thr)
        self.thr_type = QComboBox(); self.thr_type.addItems(["Cable", "DBW"])
        tt0 = settings.get("throttle_type") or "Cable"
        self.thr_type.setCurrentText(tt0 if tt0 in ("Cable", "DBW") else "Cable")
        self.idle = QComboBox(); self.idle.addItems(["Disabled", "Single wire PWM", "Dual wire"])
        idl = settings.get("idle_control") or "Disabled"
        if idl not in ("Disabled", "Single wire PWM", "Dual wire"):
            idl = "Disabled"
        self.idle.setCurrentText(idl)
        def _thr_changed(s):
            self.idle.setEnabled(s == "Cable")
            if s == "DBW":
                self.idle.setCurrentText("Disabled")
        self.thr_type.currentTextChanged.connect(_thr_changed)
        self.idle.setEnabled(self.thr_type.currentText() == "Cable")
        tf.addRow("Throttle type", self.thr_type)
        tf.addRow("Idle control", self.idle)
        self.btn_pedal_wiz = QPushButton("Pedal position calibration wizard")
        self.btn_tps_wiz = QPushButton("Throttle position calibration wizard")
        self.btn_pedal_wiz.setEnabled(self.thr_type.currentText() == "DBW")
        self.btn_tps_wiz.setEnabled(True)  # cable + DBW
        def _dbw_wiz(s):
            self.btn_pedal_wiz.setEnabled(s == "DBW")
            self.btn_tps_wiz.setEnabled(True)
        self.thr_type.currentTextChanged.connect(_dbw_wiz)
        self.btn_pedal_wiz.clicked.connect(lambda: QMessageBox.information(
            self, "Pedal wizard",
            "1. Key on, engine off\n2. Foot off pedal → Store CLOSED\n3. Full pedal → Store OPEN + Save\n(SET:PEDAL handled by ECU when connected)"))
        self.btn_tps_wiz.clicked.connect(self._open_tps_wizard)
        tf.addRow(self.btn_pedal_wiz)
        tf.addRow(self.btn_tps_wiz)
        tf.addRow(QLabel("Cable: TPS wizard required.  DBW: run pedal + throttle wizards."))
        tabs.addTab(thr, "Throttle")

        # ── Sensors / I/O ─────────────────────────────────
        io = QWidget(); iof = QFormLayout(io)
        sens = settings.get("sensors") or {}
        self.ect_en = QCheckBox("ECT enabled (fuel/timing compensation)")
        self.ect_en.setChecked(bool(sens.get("ect", {}).get("enabled", True)))
        self.iat_en = QCheckBox("IAT enabled (fuel/timing compensation)")
        self.iat_en.setChecked(bool(sens.get("iat", {}).get("enabled", True)))
        self.o2_mode = QComboBox(); self.o2_mode.addItems(["Disabled", "Narrowband", "Wideband"])
        om = settings.get("o2_mode") or sens.get("o2", {}).get("mode") or "Disabled"
        if om not in ("Disabled", "Narrowband", "Wideband"):
            om = "Disabled"
        self.o2_mode.setCurrentText(om)
        self.boost_mode = QComboBox()
        self.boost_mode.addItems(["OFF", "Single value", "Closed-loop", "Open-loop"])
        bm = settings.get("boost_mode") or "OFF"
        if bm not in ("OFF", "Single value", "Closed-loop", "Open-loop"):
            bm = "OFF"
        self.boost_mode.setCurrentText(bm)
        self.vvt_mode = QComboBox()
        self.vvt_mode.addItems(["Disabled", "Intake", "Exhaust", "Intake & Exhaust"])
        vm = settings.get("vvt_mode") or "Disabled"
        if vm not in ("Disabled", "Intake", "Exhaust", "Intake & Exhaust"):
            vm = "Disabled"
        self.vvt_mode.setCurrentText(vm)
        self.fan_en = QCheckBox("Radiator fan control")
        self.fan_en.setChecked(bool(settings.get("fan_enable", True)))
        self.fan = _spin(60, 130, settings.get("fan_c") or 95, " °C")
        self.fan.setEnabled(self.fan_en.isChecked())
        self.fan_en.toggled.connect(self.fan.setEnabled)
        iof.addRow(self.ect_en)
        iof.addRow(self.iat_en)
        iof.addRow("Oxygen sensor", self.o2_mode)
        self.boost_en = QCheckBox("Boost control enabled")
        self.boost_en.setChecked(bm != "OFF")
        iof.addRow(self.boost_en)
        iof.addRow("Boost mode", self.boost_mode)
        self.boost_mode.setEnabled(self.boost_en.isChecked())
        def _boost_toggled(on):
            self.boost_mode.setEnabled(on)
            if not on:
                self.boost_mode.setCurrentText("OFF")
            elif self.boost_mode.currentText() == "OFF":
                self.boost_mode.setCurrentText("Closed-loop")
        self.boost_en.toggled.connect(_boost_toggled)
        iof.addRow("VVT", self.vvt_mode)
        self.cam1_vvt = QCheckBox("Cam 1 sensor (intake VVT closed-loop)")
        self.cam2_vvt = QCheckBox("Cam 2 sensor (exhaust VVT closed-loop)")
        self.cam1_vvt.setChecked(bool(settings.get("cam1_vvt_cl", True)))
        self.cam2_vvt.setChecked(bool(settings.get("cam2_vvt_cl", False)))
        def _vvt_changed(s):
            self.cam1_vvt.setEnabled(s in ("Intake", "Intake & Exhaust"))
            self.cam2_vvt.setEnabled(s in ("Exhaust", "Intake & Exhaust"))
            if s == "Disabled":
                self.cam1_vvt.setChecked(False)
                self.cam2_vvt.setChecked(False)
        self.vvt_mode.currentTextChanged.connect(_vvt_changed)
        _vvt_changed(self.vvt_mode.currentText())
        iof.addRow(self.cam1_vvt)
        iof.addRow(self.cam2_vvt)
        iof.addRow(self.fan_en)
        iof.addRow("Fan on temperature", self.fan)
        tabs.addTab(io, "I/O")

        
        # ── Sensor calibration (moved from main toolbar) ──
        sens_page = QWidget()
        sens_lay = QVBoxLayout(sens_page)
        sens_lay.addWidget(QLabel("Sensor calibration is applied with Engine settings OK."))
        self._sensor_cal = SensorCalDialog(settings, parent=self)
        self._sensor_cal.setWindowFlags(Qt.Widget)
        # hide OK/Cancel of nested dialog if present
        for child in self._sensor_cal.findChildren(QDialogButtonBox):
            child.hide()
        sens_lay.addWidget(self._sensor_cal)
        tabs.addTab(sens_page, "Sensors")


        # ── Fuel cut (DFCO / coast) ────────────────────────
        cut = QWidget(); cutf = QFormLayout(cut)
        cutf.setHorizontalSpacing(16)
        cutf.setVerticalSpacing(10)
        self.dfco_en = QCheckBox("Enable deceleration / coast fuel cut")
        self.dfco_en.setChecked(bool(settings.get("dfco_enable", True)))
        self.dfco_en.setToolTip("Cut injectors when closed-throttle coasting above enter RPM")
        self.dfco_enter = _spin(800, 6000, settings.get("dfco_enter_rpm") or 1600, " RPM")
        self.dfco_enter.setToolTip("RPM above which DFCO may arm (with TPS closed)")
        self.dfco_exit = _spin(500, 5000, settings.get("dfco_exit_rpm") or 1200, " RPM")
        self.dfco_exit.setToolTip("Resume fuel when RPM falls to this (hysteresis)")
        self.dfco_tps = QDoubleSpinBox()
        self.dfco_tps.setRange(0.0, 15.0)
        self.dfco_tps.setDecimals(1)
        self.dfco_tps.setSingleStep(0.5)
        self.dfco_tps.setSuffix(" %")
        self.dfco_tps.setValue(float(settings.get("dfco_max_tps") or 3.0))
        self.dfco_tps.setToolTip("Max TPS (and pedal) to treat as closed throttle")
        self.dfco_ect = QDoubleSpinBox()
        self.dfco_ect.setRange(0.0, 100.0)
        self.dfco_ect.setDecimals(0)
        self.dfco_ect.setSuffix(" °C")
        self.dfco_ect.setValue(float(settings.get("dfco_min_ect") or 50.0))
        self.dfco_ect.setToolTip("Minimum coolant temp before DFCO allowed (cold engine keeps fuel)")
        self.dfco_delay = _spin(0, 2000, settings.get("dfco_delay_ms") or 200, " ms")
        self.dfco_delay.setToolTip("Delay after conditions met before injectors cut")
        cutf.addRow(self.dfco_en)
        cutf.addRow("Enter RPM", self.dfco_enter)
        cutf.addRow("Exit RPM", self.dfco_exit)
        cutf.addRow("Max TPS (closed)", self.dfco_tps)
        cutf.addRow("Min ECT", self.dfco_ect)
        cutf.addRow("Arm delay", self.dfco_delay)
        note = QLabel(
            "Strategy: if Enable, SYNC locked, RPM ≥ Enter, ECT ≥ Min, TPS ≤ Max for Arm delay → fuel cut.\n"
            "Exit when RPM ≤ Exit, or TPS opens (> Max+2%), or sync lost. STFT learn freezes during cut."
        )
        note.setWordWrap(True)
        note.setStyleSheet("color:#8899aa;font-size:11px;")
        cutf.addRow(note)
        tabs.addTab(cut, "Fuel cut")

        # ── Tools (was Controls) ──────────────────────────
        tools = QWidget(); tl = QVBoxLayout(tools)
        tl.addWidget(QLabel("Trigger identification (RPM must be 0)"))
        self.btn_trig_wiz = QPushButton("Trigger wizard")
        def _open_trig():
            if self._rpm > 0:
                QMessageBox.warning(self, "Trigger wizard", "Stop the engine (RPM must be 0).")
                return
            from strix_v2.dialogs import TriggerWizardDialog
            dlg = TriggerWizardDialog(self)
            dlg.exec()
        self.btn_trig_wiz.clicked.connect(_open_trig)
        self.btn_trig_wiz.setEnabled(not locked)
        tl.addWidget(self.btn_trig_wiz)
        self.btn_tps_tools = QPushButton("TPS calibration wizard")
        self.btn_tps_tools.setToolTip("Calibrate closed / WOT throttle ADC (cable or DBW)")
        self.btn_tps_tools.clicked.connect(self._open_tps_wizard)
        tl.addWidget(self.btn_tps_tools)
        self.btn_setup_wiz = QPushButton("Setup wizard")
        def _open_setup():
            if self._rpm > 0:
                QMessageBox.warning(self, "Setup wizard", "Stop the engine (RPM must be 0).")
                return
            from strix_v2.dialogs import SetupWizardDialog
            dlg = SetupWizardDialog(self._settings_ref, parent=self)
            if dlg.exec():
                dlg.apply_to(self._settings_ref)
                QMessageBox.information(self, "Setup", "Settings applied — press OK on Engine settings to keep.")
        self.btn_setup_wiz.clicked.connect(_open_setup)
        self.btn_setup_wiz.setEnabled(not locked)
        tl.addWidget(self.btn_setup_wiz)
        tl.addStretch(1)
        tabs.addTab(tools, "Tools")

        row = QHBoxLayout()
        self.btn_export = QPushButton("Export .tcal")
        self.btn_import = QPushButton("Import .tcal")
        self.btn_export.clicked.connect(self._export)
        self.btn_import.clicked.connect(self._import)
        row.addWidget(self.btn_export)
        row.addWidget(self.btn_import)
        root.addLayout(row)

        self._settings_ref = settings
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        root.addWidget(bb)

    def _export(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export calibration", "", "TorquEFI Cal (*.tcal)")
        if path:
            if not path.endswith(".tcal"):
                path += ".tcal"
            self.apply_to(self._settings_ref)
            save_tcal(path, self._settings_ref)
            QMessageBox.information(self, "Export", f"Saved {path}")

    def _import(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import calibration", "", "TorquEFI Cal (*.tcal)")
        if path:
            try:
                data = load_tcal(path)
                self._settings_ref.clear()
                self._settings_ref.update(data)
                # reload dialog fields from data via reject/reopen is heavy — set widgets
                self.cyl.setValue(int(data.get("cylinders") or 4))
                self.teeth.setValue(int(data.get("teeth") or 36))
                self.missing.setValue(int(data.get("missing") or 1))
                self.trig.setValue(int(data.get("trig_angle") or 30))
                wi = int(data.get("wheel_id") or 0)
                i = self.wheel.findData(wi)
                if i >= 0:
                    self.wheel.setCurrentIndex(i)
                fo = data.get("firing_order") or "1-3-4-2"
                if fo in self.FIRE_ORDERS:
                    self.fire.setCurrentText(fo)
                ct = data.get("coil_type") or "Smart"
                if ct in ("Smart", "Dumb", "Distributor"):
                    self.coil.setCurrentText(ct)
                lm = (data.get("load_mode") or "MAP").upper()
                if lm in ("ALPHA-N", "ALPHA_N"):
                    lm = "TPS"
                if lm in ("MAP", "TPS", "HYBRID"):
                    self.load_mode.setCurrentText(lm)
                self.map_kpa_max.setValue(int(data.get("map_kpa_max") or 240))
                self.fp_prime.setValue(int(data.get("fp_prime_ms") or 2000))
                self.inj_prime.setValue(int(data.get("start_prime_ms") or 50))
                self.inj_prime_en.setChecked(bool(data.get("start_prime_enable", True)))
                im = data.get("inj_mode") or "Sequential"
                if im in ("Sequential", "Batch", "Batch above RPM"):
                    self.inj_mode.setCurrentText(im)
                self.batch_rpm.setValue(int(data.get("batch_above_rpm") or 3000))
                tt0 = data.get("throttle_type") or "Cable"
                if tt0 in ("Cable", "DBW"):
                    self.thr_type.setCurrentText(tt0)
                idl = data.get("idle_control") or "Disabled"
                if idl in ("Disabled", "Single wire PWM", "Dual wire"):
                    self.idle.setCurrentText(idl)
                sens = data.get("sensors") or {}
                self.ect_en.setChecked(bool(sens.get("ect", {}).get("enabled", True)))
                self.iat_en.setChecked(bool(sens.get("iat", {}).get("enabled", True)))
                om = data.get("o2_mode") or sens.get("o2", {}).get("mode") or "Disabled"
                if om in ("Disabled", "Narrowband", "Wideband"):
                    self.o2_mode.setCurrentText(om)
                bm = data.get("boost_mode") or "OFF"
                if bm in ("OFF", "Single value", "Closed-loop", "Open-loop"):
                    self.boost_mode.setCurrentText(bm)
                vm = data.get("vvt_mode") or "Disabled"
                if vm in ("Disabled", "Intake", "Exhaust", "Intake & Exhaust"):
                    self.vvt_mode.setCurrentText(vm)
                self.fan_en.setChecked(bool(data.get("fan_enable", True)))
                self.fan.setValue(int(data.get("fan_c") or 95))
                self.rpm_lim.setValue(int(data.get("rpm_limit") or 7000))
                rc = data.get("rpm_cut_mode") or "Hard"
                if rc in ("Hard", "Soft"):
                    self.rpm_cut.setCurrentText(rc)
                QMessageBox.information(self, "Import", "Calibration loaded")
            except Exception as e:
                QMessageBox.critical(self, "Import", str(e))

    def _open_tps_wizard(self):
        p = self.parent()
        while p is not None and not hasattr(p, "live"):
            p = p.parent()
        live_g = (lambda: getattr(p, "live", {})) if p else (lambda: {})
        send = getattr(p, "_tx", None) if p else None
        TpsCalWizardDialog(live_g, send_fn=send, parent=self).exec()

    def _live_trig(self, v: int):
        self.trig_lbl.setText(f"{v} °")
        p = self.parent()
        while p is not None and not hasattr(p, "_tx"):
            p = p.parent()
        if p is not None and getattr(p, "connected", False):
            p._tx("SET:TRIG,%d\n" % int(v))
            if hasattr(p, "engine"):
                p.engine["trig_angle"] = int(v)
            if hasattr(p, "trig_slider"):
                p.trig_slider.blockSignals(True)
                p.trig_slider.setValue(int(v))
                p.trig_slider.blockSignals(False)
                if hasattr(p, "trig_val"):
                    p.trig_val.setText(f"{int(v)}°")

    def _refill_fire_orders(self, cyl: int, prefer: str | None = None):
        orders = list(FIRING_ORDERS_BY_CYL.get(int(cyl), FIRING_ORDERS_BY_CYL.get(4, ["1-3-4-2"])))
        cur = prefer or (self.fire.currentText() if hasattr(self, "fire") else None)
        self.fire.blockSignals(True)
        self.fire.clear()
        self.fire.addItems(orders)
        if cur and cur in orders:
            self.fire.setCurrentText(cur)
        elif orders:
            self.fire.setCurrentIndex(0)
        self.fire.blockSignals(False)


    def apply_to(self, settings: dict) -> None:
        settings["fp_prime_ms"] = self.fp_prime.value()
        settings["start_prime_ms"] = self.inj_prime.value()
        settings["start_prime_enable"] = self.inj_prime_en.isChecked()
        settings["rpm_limit"] = self.rpm_lim.value()
        settings["rpm_cut_mode"] = self.rpm_cut.currentText()
        if hasattr(self, "max_adv"):
            settings["max_advance"] = int(self.max_adv.value())
            settings["max_retard"] = int(self.max_ret.value())
        settings["fan_enable"] = self.fan_en.isChecked()
        settings["fan_c"] = self.fan.value()
        settings["map_kpa_max"] = self.map_kpa_max.value()
        cyl_n = int(self.cyl.value())
        if cyl_n in (5, 6, 8):
            settings["inj_mode"] = "Batch"
        else:
            settings["inj_mode"] = self.inj_mode.currentText()
        settings["batch_above_rpm"] = self.batch_rpm.value()
        if hasattr(self, "inj_flow"):
            settings["inj_flow_cc"] = float(self.inj_flow.value())
            settings["req_fuel_ms"] = float(self.req_fuel.value())
            settings["ve_mode"] = bool(self.ve_mode_cb.isChecked())
            if hasattr(self, "max_inj"):
                settings["max_inj_ms"] = float(self.max_inj.value())
            if hasattr(self, "fuel_press"):
                settings["fuel_pressure_bar"] = float(self.fuel_press.value())
                settings["fuel_pressure_rated_bar"] = float(self.fuel_press_rated.value())
        settings["load_mode"] = self.load_mode.currentText()
        settings["throttle_type"] = self.thr_type.currentText()
        settings["idle_control"] = self.idle.currentText()
        settings["o2_mode"] = self.o2_mode.currentText()
        if hasattr(self, "boost_en") and not self.boost_en.isChecked():
            settings["boost_mode"] = "OFF"
        else:
            settings["boost_mode"] = self.boost_mode.currentText()
        settings["vvt_mode"] = self.vvt_mode.currentText()
        settings["cam_home"] = bool(getattr(self, "cam_home", None) and self.cam_home.isChecked())
        settings["cam1_vvt_cl"] = bool(getattr(self, "cam1_vvt", None) and self.cam1_vvt.isChecked())
        settings["cam2_vvt_cl"] = bool(getattr(self, "cam2_vvt", None) and self.cam2_vvt.isChecked())
        settings["coil_type"] = self.coil.currentText()
        settings["firing_order"] = self.fire.currentText()
        if hasattr(self, "dfco_en"):
            settings["dfco_enable"] = bool(self.dfco_en.isChecked())
            settings["dfco_enter_rpm"] = int(self.dfco_enter.value())
            settings["dfco_exit_rpm"] = int(self.dfco_exit.value())
            settings["dfco_max_tps"] = float(self.dfco_tps.value())
            settings["dfco_min_ect"] = float(self.dfco_ect.value())
            settings["dfco_delay_ms"] = int(self.dfco_delay.value())
        settings["wheel_id"] = int(self.wheel.currentData())
        sens = settings.setdefault("sensors", {})
        sens.setdefault("ect", {})["enabled"] = self.ect_en.isChecked()
        sens.setdefault("iat", {})["enabled"] = self.iat_en.isChecked()
        sens.setdefault("o2", {})["enabled"] = self.o2_mode.currentText() != "Disabled"
        sens.setdefault("o2", {})["mode"] = self.o2_mode.currentText()
        if hasattr(self, "_sensor_cal"):
            self._sensor_cal.apply_to(settings)
        if settings["load_mode"] == "MAP":
            settings["map_bins"] = make_map_bins(int(settings.get("map_kpa_max") or 240))
        elif settings["load_mode"] == "TPS":
            from strix_v2.constants import make_tps_bins
            settings["tps_bins"] = make_tps_bins()
        if self._rpm > 0:
            return
        settings["cylinders"] = self.cyl.value()
        if settings["cylinders"] in (5, 6, 8):
            settings["inj_mode"] = "Batch" if settings.get("inj_mode") == "Sequential" else settings.get("inj_mode") or "Batch"
        wid = int(self.wheel.currentData())
        for w_id, _n, teeth, missing in WHEEL_PROFILES:
            if w_id == wid:
                settings["teeth"] = teeth
                settings["missing"] = missing
                break
        else:
            settings["teeth"] = int(getattr(self, "teeth", None).value() if hasattr(self, "teeth") else 36)
            settings["missing"] = int(getattr(self, "missing", None).value() if hasattr(self, "missing") else 1)
        settings["trig_angle"] = int(self.trig.value())  # slider or spin




class SetupWizardDialog(QDialog):
    """Revived multi-page setup wizard (V1 flow + V2 primes/hybrid)."""

    def __init__(self, settings: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Setup wizard")
        self.setMinimumSize(560, 520)
        self.settings = dict(settings)
        root = QVBoxLayout(self)
        self.stack = QTabWidget()  # tabs act as pages; user can jump
        root.addWidget(self.stack, 1)

        # Page 1 — Trigger & ignition
        p1 = QWidget(); l1 = QFormLayout(p1)
        self.w_wheel = QComboBox()
        for wid, name in EngineSettingsDialog.WHEELS:
            self.w_wheel.addItem(name, wid)
        wi = int(settings.get("wheel_id") or 0)
        i = self.w_wheel.findData(wi)
        if i >= 0:
            self.w_wheel.setCurrentIndex(i)
        self.w_cyl = QSpinBox(); self.w_cyl.setRange(1, 8)
        self.w_cyl.setValue(int(settings.get("cylinders") or 4))
        self.w_coil = QComboBox(); self.w_coil.addItems(["Smart", "Dumb", "Distributor"])
        ct = settings.get("coil_type") or "Smart"
        if ct in ("Smart", "Dumb", "Distributor"):
            self.w_coil.setCurrentText(ct)
        self.w_fire = QComboBox(); self.w_fire.addItems(list(ALL_FIRING_ORDERS))
        fo = settings.get("firing_order") or "1-3-4-2"
        if fo in EngineSettingsDialog.FIRE_ORDERS:
            self.w_fire.setCurrentText(fo)
        self.w_inj = QComboBox(); self.w_inj.addItems(["Sequential", "Batch", "Batch above RPM"])
        im = settings.get("inj_mode") or "Sequential"
        if im in ("Sequential", "Batch", "Batch above RPM"):
            self.w_inj.setCurrentText(im)
        self.w_rpm = QSpinBox(); self.w_rpm.setRange(2000, 12000)
        self.w_rpm.setValue(int(settings.get("rpm_limit") or 7000))
        self.w_cut = QComboBox(); self.w_cut.addItems(["Hard", "Soft"])
        self.w_cut.setCurrentText(settings.get("rpm_cut_mode") or "Hard")
        l1.addRow("Crank wheel", self.w_wheel)
        l1.addRow("Cylinders", self.w_cyl)
        l1.addRow("Coil type", self.w_coil)
        l1.addRow("Firing order", self.w_fire)
        l1.addRow("Injection mode", self.w_inj)
        l1.addRow("RPM limit", self.w_rpm)
        l1.addRow("Limiter type", self.w_cut)
        self.stack.addTab(p1, "1 · Trigger")

        # Page 2 — Load & throttle
        p2 = QWidget(); l2 = QFormLayout(p2)
        self.w_load = QComboBox(); self.w_load.addItems(["MAP", "TPS", "HYBRID"])
        lm = (settings.get("load_mode") or "MAP").upper()
        if lm not in ("MAP", "TPS", "HYBRID"):
            lm = "MAP"
        self.w_load.setCurrentText(lm)
        self.w_thr = QComboBox(); self.w_thr.addItems(["Cable", "DBW"])
        self.w_thr.setCurrentText(settings.get("throttle_type") or "Cable")
        self.w_idle = QComboBox(); self.w_idle.addItems(["Disabled", "Single wire PWM", "Dual wire"])
        self.w_idle.setCurrentText(settings.get("idle_control") or "Disabled")
        self.w_idle_rpm = QSpinBox(); self.w_idle_rpm.setRange(500, 2000)
        self.w_idle_rpm.setValue(int(settings.get("idle_target_rpm") or 850))
        self.w_idle_en = QCheckBox("Closed-loop idle enable")
        self.w_idle_en.setChecked(bool(settings.get("idle_enable", True)))
        self.w_fp = QSpinBox(); self.w_fp.setRange(0, 15000); self.w_fp.setValue(int(settings.get("fp_prime_ms") or 2000))
        self.w_fp.setSuffix(" ms")
        self.w_ip = QSpinBox(); self.w_ip.setRange(0, 500); self.w_ip.setValue(int(settings.get("start_prime_ms") or 50))
        self.w_ip.setSuffix(" ms")
        l2.addRow("Load axis", self.w_load)
        l2.addRow("Throttle", self.w_thr)
        l2.addRow("Idle actuator", self.w_idle)
        l2.addRow(self.w_idle_en)
        l2.addRow("Idle target RPM", self.w_idle_rpm)
        l2.addRow("Fuel pump prime", self.w_fp)
        l2.addRow("Start inj. prime", self.w_ip)
        self.stack.addTab(p2, "2 · Load / Idle")

        # Page 3 — I/O
        p3 = QWidget(); l3 = QFormLayout(p3)
        sens = settings.get("sensors") or {}
        self.w_ect = QCheckBox("ECT enabled"); self.w_ect.setChecked(bool(sens.get("ect", {}).get("enabled", True)))
        self.w_iat = QCheckBox("IAT enabled"); self.w_iat.setChecked(bool(sens.get("iat", {}).get("enabled", True)))
        self.w_o2 = QComboBox(); self.w_o2.addItems(["Disabled", "Narrowband", "Wideband"])
        self.w_o2.setCurrentText(settings.get("o2_mode") or "Disabled")
        self.w_bst = QComboBox(); self.w_bst.addItems(["OFF", "Single value", "Closed-loop", "Open-loop"])
        self.w_bst.setCurrentText(settings.get("boost_mode") or "OFF")
        self.w_vvt = QComboBox(); self.w_vvt.addItems(["Disabled", "Intake", "Exhaust", "Intake & Exhaust"])
        self.w_vvt.setCurrentText(settings.get("vvt_mode") or "Disabled")
        self.w_fan = QCheckBox("Fan control"); self.w_fan.setChecked(bool(settings.get("fan_enable", True)))
        self.w_fanc = QSpinBox(); self.w_fanc.setRange(60, 130); self.w_fanc.setValue(int(settings.get("fan_c") or 95))
        l3.addRow(self.w_ect)
        l3.addRow(self.w_iat)
        l3.addRow("O2", self.w_o2)
        l3.addRow("Boost", self.w_bst)
        l3.addRow("VVT", self.w_vvt)
        l3.addRow(self.w_fan)
        l3.addRow("Fan °C", self.w_fanc)
        self.stack.addTab(p3, "3 · I/O")

        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.button(QDialogButtonBox.Ok).setText("Finish & Apply")
        bb.accepted.connect(self.accept)
        bb.rejected.connect(self.reject)
        root.addWidget(bb)

    def apply_to(self, settings: dict) -> None:
        settings["wheel_id"] = int(self.w_wheel.currentData())
        settings["cylinders"] = self.w_cyl.value()
        settings["coil_type"] = self.w_coil.currentText()
        settings["firing_order"] = self.w_fire.currentText()
        settings["inj_mode"] = self.w_inj.currentText()
        settings["rpm_limit"] = self.w_rpm.value()
        settings["rpm_cut_mode"] = self.w_cut.currentText()
        settings["load_mode"] = self.w_load.currentText()
        settings["throttle_type"] = self.w_thr.currentText()
        settings["idle_control"] = self.w_idle.currentText()
        settings["idle_enable"] = self.w_idle_en.isChecked()
        settings["idle_target_rpm"] = self.w_idle_rpm.value()
        settings["fp_prime_ms"] = self.w_fp.value()
        settings["start_prime_ms"] = self.w_ip.value()
        settings["start_prime_enable"] = self.w_ip.value() > 0
        settings["o2_mode"] = self.w_o2.currentText()
        settings["boost_mode"] = self.w_bst.currentText()
        settings["vvt_mode"] = self.w_vvt.currentText()
        settings["fan_enable"] = self.w_fan.isChecked()
        settings["fan_c"] = self.w_fanc.value()
        sens = settings.setdefault("sensors", {})
        sens.setdefault("ect", {})["enabled"] = self.w_ect.isChecked()
        sens.setdefault("iat", {})["enabled"] = self.w_iat.isChecked()
        sens.setdefault("o2", {})["mode"] = self.w_o2.currentText()
        sens.setdefault("o2", {})["enabled"] = self.w_o2.currentText() != "Disabled"



class TpsCalWizardDialog(QDialog):
    """Cable or DBW throttle position calibration — store closed/open ADC, SET:TPS."""

    def __init__(self, live_getter, send_fn=None, parent=None):
        super().__init__(parent)
        self._live = live_getter
        self._send = send_fn
        self.setWindowTitle("TPS calibration wizard")
        self.setMinimumWidth(420)
        self.closed = 0
        self.open_adc = 0
        lay = QVBoxLayout(self)
        lay.addWidget(QLabel(
            "1. Key on, engine stopped\n"
            "2. Leave throttle closed → Store CLOSED\n"
            "3. Hold WOT → Store OPEN + Save\n"
            "Live ADC is read from the ECU (TADC)."
        ))
        self.live_lbl = QLabel("Live ADC: —")
        self.live_lbl.setStyleSheet("font-size:18px;font-weight:700;font-family:Consolas,monospace;")
        self.closed_lbl = QLabel("Closed ADC: —")
        self.open_lbl = QLabel("Open ADC: —")
        lay.addWidget(self.live_lbl)
        lay.addWidget(self.closed_lbl)
        lay.addWidget(self.open_lbl)
        b1 = QPushButton("1. Store CLOSED throttle")
        b2 = QPushButton("2. Store FULL throttle + Save")
        b1.clicked.connect(self._store_closed)
        b2.clicked.connect(self._store_open)
        lay.addWidget(b1)
        lay.addWidget(b2)
        tip = QLabel("Span must be > 50 ADC counts. Values are sent as SET:TPS,<closed>,<open>.")
        tip.setStyleSheet("color:#8899aa;font-size:11px;")
        lay.addWidget(tip)
        bb = QDialogButtonBox(QDialogButtonBox.Close)
        bb.rejected.connect(self.reject)
        bb.accepted.connect(self.accept)
        bb.button(QDialogButtonBox.Close).clicked.connect(self.close)
        lay.addWidget(bb)
        self._t = QTimer(self)
        self._t.timeout.connect(self._tick)
        self._t.start(150)

    def _adc(self) -> int:
        live = self._live() if callable(self._live) else (self._live or {})
        for k in ("tadc", "TADC", "tps_adc", "adc_tps"):
            if live.get(k) is not None:
                try:
                    return int(float(live[k]))
                except (TypeError, ValueError):
                    pass
        # fallback: rough from TPS % if only that is present
        try:
            return int(float(live.get("tps") or 0) * 40.95)  # 0–100% → 0–4095 scale guess
        except (TypeError, ValueError):
            return 0

    def _tick(self):
        self.live_lbl.setText(f"Live ADC: {self._adc()}")

    def _store_closed(self):
        self.closed = self._adc()
        self.closed_lbl.setText(f"Closed ADC: {self.closed}")

    def _store_open(self):
        if not self.closed:
            QMessageBox.warning(self, "TPS", "Store CLOSED first.")
            return
        self.open_adc = self._adc()
        span = abs(self.open_adc - self.closed)
        if span < 50:
            QMessageBox.warning(
                self, "TPS",
                f"Span only {span} counts — open throttle fully or check sensor wiring.")
            return
        self.open_lbl.setText(f"Open ADC: {self.open_adc}")
        lo, hi = (self.closed, self.open_adc) if self.closed < self.open_adc else (self.open_adc, self.closed)
        cmd = f"SET:TPS,{self.closed},{self.open_adc}\n"
        if self._send:
            self._send(cmd)
            self._send("SAVE\n")
        QMessageBox.information(
            self, "TPS saved",
            f"Closed={self.closed}  Open={self.open_adc}  (span {span})\n"
            f"Sent to ECU: {cmd.strip()}")
        self.accept()


class TriggerWizardDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Trigger wizard")
        self.setMinimumSize(400, 320)
        lay = QVBoxLayout(self)
        lay.addWidget(QLabel(
            "Crank engine (RPM will show pulses). Wizard samples tooth gaps "
            "to suggest 36-1, 60-2, etc.\nOnly available when RPM was 0 at open."
        ))
        self.log = QListWidget()
        lay.addWidget(self.log)
        self.result = QLabel("Suggestion: —")
        lay.addWidget(self.result)
        row = QHBoxLayout()
        self.btn_sample = QPushButton("Sample 3 s")
        self.btn_sample.clicked.connect(self._sample)
        row.addWidget(self.btn_sample)
        row.addStretch(1)
        lay.addLayout(row)
        bb = QDialogButtonBox(QDialogButtonBox.Close)
        bb.rejected.connect(self.reject)
        bb.clicked.connect(self.accept)
        lay.addWidget(bb)
        self._parent = parent
        self.suggested = None

    def _sample(self):
        self.log.clear()
        self.log.addItem("Listening for crank edges (use live TOOTH/GAP if available)…")
        # Best-effort: read recent live rpm / tooth from parent
        live = getattr(self._parent, "live", {}) or {}
        rpm = int(live.get("rpm") or 0)
        tooth = live.get("tooth", "—")
        self.log.addItem(f"RPM={rpm}  tooth={tooth}")
        # Heuristic placeholders until firmware streams gap histogram
        if rpm <= 0:
            self.result.setText("Suggestion: no crank activity — crank the engine")
            self.suggested = None
        elif rpm < 400:
            self.result.setText("Suggestion: 36-1 (common) — confirm with scope")
            self.suggested = (36, 1)
        else:
            self.result.setText("Suggestion: 60-2 or 36-1 — verify missing-tooth pattern")
            self.suggested = (36, 1)



class MotorsportDialog(QDialog):
    """V1 motorsport features: Launch / ALS / Flat-foot shift / clutch."""

    def __init__(self, settings: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Motorsport")
        self.setMinimumWidth(480)
        self.settings = settings
        root = QVBoxLayout(self)
        tabs = QTabWidget(); root.addWidget(tabs)

        def spin(lo, hi, val, suf=""):
            s = QSpinBox(); s.setRange(lo, hi); s.setValue(int(val))
            if suf: s.setSuffix(suf)
            return s

        # Launch
        w = QWidget(); f = QFormLayout(w)
        self.lc_en = QCheckBox("Launch control enable")
        self.lc_en.setChecked(bool(settings.get("launch_enable", False)))
        self.lc_rpm = spin(2000, 9000, settings.get("launch_rpm") or 4500, " RPM")
        self.lc_tps = spin(50, 100, settings.get("launch_tps") or 80, " %")
        self.lc_bst = spin(0, 250, settings.get("launch_boost_kpa") or 0, " kPa")
        f.addRow(self.lc_en); f.addRow("Launch RPM", self.lc_rpm)
        f.addRow("Min TPS", self.lc_tps); f.addRow("Target boost", self.lc_bst)
        tabs.addTab(w, "Launch")

        # ALS
        w = QWidget(); f = QFormLayout(w)
        self.als_en = QCheckBox("Anti-lag enable")
        self.als_en.setChecked(bool(settings.get("als_enable", False)))
        self.als_ret = spin(0, 40, settings.get("als_retard") or 15, " °")
        self.als_fuel = spin(0, 50, settings.get("als_fuel_pct") or 10, " %")
        self.als_ms = spin(500, 15000, settings.get("als_max_ms") or 4000, " ms")
        f.addRow(self.als_en); f.addRow("Retard", self.als_ret)
        f.addRow("Fuel add", self.als_fuel); f.addRow("Max duration", self.als_ms)
        tabs.addTab(w, "ALS")

        # FFS
        w = QWidget(); f = QFormLayout(w)
        self.ffs_en = QCheckBox("Flat-foot shift enable")
        self.ffs_en.setChecked(bool(settings.get("ffs_enable", False)))
        self.ffs_tps = spin(50, 100, settings.get("ffs_tps") or 70, " %")
        self.ffs_ret = spin(0, 30, settings.get("ffs_retard") or 10, " °")
        f.addRow(self.ffs_en); f.addRow("Min TPS", self.ffs_tps); f.addRow("Retard", self.ffs_ret)
        tabs.addTab(w, "Flat-foot")

        # Clutch
        w = QWidget(); f = QFormLayout(w)
        self.clutch_en = QCheckBox("Clutch switch input enable")
        self.clutch_en.setChecked(bool(settings.get("clutch_enable", True)))
        f.addRow(self.clutch_en)
        f.addRow(QLabel("Clutch used by Launch + FFS (PB13)."))
        tabs.addTab(w, "Clutch")

        # Optional buttons when shown as a dialog; hidden when embedded in a tab
        self._bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self._bb.accepted.connect(self.accept)
        self._bb.rejected.connect(self.reject)
        root.addWidget(self._bb)

    def set_embedded(self, embedded: bool = True):
        if hasattr(self, "_bb"):
            self._bb.setVisible(not embedded)

    def apply_to(self, settings: dict):
        settings["launch_enable"] = self.lc_en.isChecked()
        settings["launch_rpm"] = self.lc_rpm.value()
        settings["launch_tps"] = self.lc_tps.value()
        settings["launch_boost_kpa"] = self.lc_bst.value()
        settings["als_enable"] = self.als_en.isChecked()
        settings["als_retard"] = self.als_ret.value()
        settings["als_fuel_pct"] = self.als_fuel.value()
        settings["als_max_ms"] = self.als_ms.value()
        settings["ffs_enable"] = self.ffs_en.isChecked()
        settings["ffs_tps"] = self.ffs_tps.value()
        settings["ffs_retard"] = self.ffs_ret.value()
        settings["clutch_enable"] = self.clutch_en.isChecked()
