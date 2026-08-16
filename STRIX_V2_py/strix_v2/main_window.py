"""STRIX V2 main window."""
from __future__ import annotations

import json
import time
import csv
from collections import deque
from pathlib import Path

from PySide6.QtCore import QThread, Qt, QTimer, Signal
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QComboBox,
    QLabel, QTabWidget, QStatusBar, QMessageBox, QApplication, QFileDialog,
    QTableWidget, QTableWidgetItem, QSpinBox, QProgressBar, QFrame, QSizePolicy,
    QSlider, QDialog, QListWidget, QDialogButtonBox,
)

from strix_v2.constants import (
    BAUD, ROWS, COLS, DARK_STYLE, SETTINGS_FILE, make_map_bins, make_tps_bins,
    suggested_ve_map, suggested_adv_map, suggested_inj_ms_map,
    suggested_vvt_map, suggested_boost_map,
)
from strix_v2.serial_worker import SerialWorker, list_serial_ports
from strix_v2.protocol import default_live, parse_line
from strix_v2.widgets.live_strip import LiveStrip
from strix_v2.widgets.map_view import MapView
from strix_v2.widgets.map3d import Map3DView
from strix_v2.widgets.dashboard import DashboardDialog
from strix_v2.device_id import get_or_create_device_id, load_device_meta, save_device_meta
from strix_v2.tcal import default_engine_settings, save_tcal, load_tcal
from strix_v2.dialogs import (
    ProgramSettingsDialog, SensorCalDialog, EngineSettingsDialog,
    TriggerWizardDialog, SetupWizardDialog, MotorsportDialog, TpsCalWizardDialog,
)

# Firmware injection modes: 1=batch, 2=sequential, 3=sequential below batch RPM
INJ_MODE_TO_ECU = {"Sequential": 2, "Batch": 1, "Batch above RPM": 3}
INJ_MODE_FROM_ECU = {0: "Sequential", 1: "Batch", 2: "Sequential", 3: "Batch above RPM"}
COIL_TYPE_TO_ECU = {"Smart": 0, "Dumb": 1, "Distributor": 2}
COIL_TYPE_FROM_ECU = {v: k for k, v in COIL_TYPE_TO_ECU.items()}

# WHEEL_PROFILES ids are a different namespace from the firmware table in
# ecu_wheels.c, so they must be translated in both directions. Tuner "Custom"
# and firmware profiles without a tuner equivalent are simply not mapped.
WHEEL_TO_ECU = {1: 0, 2: 2, 6: 4, 28: 5, 7: 7, 3: 8, 4: 9, 5: 9}
WHEEL_FROM_ECU = {0: 1, 1: 1, 2: 2, 4: 6, 5: 28, 6: 6, 7: 7, 8: 3, 9: 4, 10: 4}


def _settings_path() -> Path:
    return Path.home() / ".strix_v2" / SETTINGS_FILE


class FlashSaveWorker(QThread):
    done = Signal(bool, str)

    def __init__(self, send_fn, parent=None):
        super().__init__(parent)
        self._send = send_fn

    def run(self):
        try:
            ok = bool(self._send("SAVE\n"))
            self.done.emit(ok, "OK" if ok else "TX failed")
        except Exception as e:
            self.done.emit(False, str(e))


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("STRIX V2")
        self.resize(1280, 800)
        self.setStyleSheet(DARK_STYLE)

        self.engine = default_engine_settings()
        self.strip_optional: set[str] = {"ign", "pw", "bat", "afr", "load", "cam", "dwell"}
        self.meta = load_device_meta()
        self.device_id = get_or_create_device_id()
        self.live = default_live()
        self._cfg_pending = False
        self._rx_window: list[float] = []
        self._flash_state = "idle"
        self._last_cfg_summary = "—"
        self._pending_cfg_parts = None
        self.connected = False
        self._map_dl_mode = None
        self._offline_queue: list[str] = []
        self._log: deque = deque(maxlen=600)
        self._rx_count = 0
        self._auto_tried = False

        self.worker = SerialWorker()
        self.worker.line_received.connect(self._on_line)
        self.worker.connected_changed.connect(self._on_conn)
        self.worker.status.connect(self._on_status)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 6)
        root.setSpacing(6)

        # ── Row 1: link + trigger ──
        row_link = QHBoxLayout()
        row_link.setSpacing(6)
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(120)
        self.port_combo.setMaximumWidth(200)
        self.btn_refresh = QPushButton("↻")
        self.btn_refresh.setFixedWidth(32)
        self.btn_refresh.setToolTip("Refresh ports")
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect = QPushButton("Connect")
        self.btn_connect.clicked.connect(self.toggle_connect)
        self.btn_save = QPushButton("Flash")
        self.btn_save.clicked.connect(self._start_flash)
        self.btn_save.setEnabled(False)
        self.health_led = QLabel("●")
        self.health_led.setStyleSheet("color:#666;font-size:18px;")
        self.trig_slider = QSlider(Qt.Horizontal)
        self.trig_slider.setRange(0, 360)
        self.trig_slider.setFixedWidth(110)
        self.trig_slider.setValue(int(self.engine.get("trig_angle") or 30))
        self.trig_slider.setToolTip("Trigger angle — live while running")
        self.trig_val = QLabel(f"{self.trig_slider.value()}°")
        self.trig_val.setMinimumWidth(36)
        self.trig_slider.valueChanged.connect(self._on_trig_slider)
        for w in (QLabel("Port"), self.port_combo, self.btn_refresh, self.btn_connect,
                  self.btn_save, self.health_led):
            row_link.addWidget(w)
        row_link.addSpacing(10)
        row_link.addWidget(QLabel("Trig°"))
        row_link.addWidget(self.trig_slider)
        row_link.addWidget(self.trig_val)
        row_link.addStretch(1)
        root.addLayout(row_link)

        # ── Row 2: tools ──
        row_tools = QHBoxLayout()
        row_tools.setSpacing(4)
        self.btn_log = QPushButton("Log")
        self.btn_log.clicked.connect(self.save_csv_log)
        self.btn_settings = QPushButton("Program")
        self.btn_settings.clicked.connect(self.open_program_settings)
        self.btn_engine = QPushButton("Engine")
        self.btn_engine.clicked.connect(self.open_engine_settings)
        self.btn_interp = QPushButton("Interp")
        self.btn_interp.clicked.connect(self.interpolate_maps)
        self.btn_smooth = QPushButton("Smooth")
        self.btn_smooth.clicked.connect(self.smooth_maps)
        self.btn_ve = QPushButton("VE")
        self.btn_ve.setCheckable(True)
        self.btn_ve.setChecked(bool(self.engine.get("ve_mode", True)))
        self.btn_ve.toggled.connect(self._ve_mode_toggled)
        self.btn_fill_rec = QPushButton("Fill rec.")
        self.btn_fill_rec.clicked.connect(self._fill_recommended_fuel)
        self.btn_dash = QPushButton("Dash")
        self.btn_dash.clicked.connect(self.open_dashboard)
        self.btn_export = QPushButton("Export")
        self.btn_export.clicked.connect(self._export_pack)
        self.btn_import = QPushButton("Import")
        self.btn_import.clicked.connect(self._import_pack)
        for b in (self.btn_log, self.btn_settings, self.btn_engine, self.btn_interp,
                  self.btn_smooth, self.btn_ve, self.btn_fill_rec, self.btn_dash,
                  self.btn_export, self.btn_import):
            b.setMaximumHeight(28)
            row_tools.addWidget(b)
        row_tools.addStretch(1)
        root.addLayout(row_tools)

        self.conn_strip = QLabel("Port: —  ·  RX: 0/s  ·  CFG: —  ·  Flash: idle")
        self.conn_strip.setStyleSheet(
            "background:#12161e;color:#9ab;padding:4px 10px;font-family:Consolas,monospace;font-size:12px;"
        )
        self.mismatch_bar = QFrame()
        mb = QHBoxLayout(self.mismatch_bar)
        mb.setContentsMargins(8, 4, 8, 4)
        self.mismatch_lbl = QLabel("ECU data differs from tuner")
        self.mismatch_lbl.setStyleSheet("color:#ffcc66;font-weight:700;")
        btn_apply_ecu = QPushButton("Apply ECU")
        btn_keep = QPushButton("Keep local")
        btn_apply_ecu.clicked.connect(self._banner_apply_ecu)
        btn_keep.clicked.connect(lambda: self.mismatch_bar.hide())
        mb.addWidget(self.mismatch_lbl, 1)
        mb.addWidget(btn_apply_ecu)
        mb.addWidget(btn_keep)
        self.mismatch_bar.setStyleSheet("background:#3a2a10;border:1px solid #886622;")
        self.mismatch_bar.hide()
        self.flash_bar = QProgressBar()
        self.flash_bar.setRange(0, 0)
        self.flash_bar.setFormat("Writing flash…")
        self.flash_bar.setMaximumHeight(16)
        self.flash_bar.hide()
        root.addWidget(self.conn_strip)
        root.addWidget(self.mismatch_bar)
        root.addWidget(self.flash_bar)

        self.strip = LiveStrip()
        self.strip.set_optional(self.strip_optional)
        root.addWidget(self.strip)

        self.tabs = QTabWidget()
        root.addWidget(self.tabs, 1)

        self.map_ign = MapView("Ignition (° BTDC)", is_ign=True)
        self.map_inj = MapView("VE (%)", is_ign=False, kind="inj", vmax=120.0)
        self.map_ign.set_table(suggested_adv_map())
        if bool(self.engine.get("ve_mode", True)):
            self.map_inj.set_table(suggested_ve_map())
            self.map_inj.hdr.setText("VE (%)")
            self.map_inj.vmax = 120.0
        else:
            self.map_inj.set_table(suggested_inj_ms_map(
                req_fuel_ms=float(self.engine.get("req_fuel_ms") or 2.5),
                flow_cc=float(self.engine.get("inj_flow_cc") or 220),
                fuel_pressure_bar=float(self.engine.get("fuel_pressure_bar") or 3.0),
                fuel_pressure_rated_bar=float(self.engine.get("fuel_pressure_rated_bar") or 3.0),
            ))
            self.map_inj.hdr.setText("Injection (ms)")
            self.map_inj.vmax = 40.0
        self.map_ign.cell_changed.connect(lambda r, c, v: self._cell_tx(0, r, c, v))
        self.map_inj.cell_changed.connect(lambda r, c, v: self._cell_tx(1, r, c, v))
        self.map_ign.dirty_changed.connect(self._update_dirty_status)
        self.map_inj.dirty_changed.connect(self._update_dirty_status)
        self.tabs.addTab(self.map_ign, "Ignition")
        self.tabs.addTab(self.map_inj, "Injection [VE %]")

        vvt_rpm = [500, 1000, 1500, 2000, 3000, 4000, 5000, 6000]
        vvt_load = [0, 10, 20, 30, 40, 50, 70, 100]
        self.vvt_page = QWidget()
        vvt_lay = QHBoxLayout(self.vvt_page)
        self.map_vvt_in = MapView("VVT Intake (°)", is_ign=True, rows=8, cols=8,
                                  kind="vvt", vmax=50, rpm_bins=vvt_rpm)
        self.map_vvt_ex = MapView("VVT Exhaust (°)", is_ign=True, rows=8, cols=8,
                                  kind="vvt", vmax=50, rpm_bins=vvt_rpm)
        self.map_vvt_in.set_load_bins(vvt_load, "TPS")
        self.map_vvt_ex.set_load_bins(vvt_load, "TPS")
        self.map_vvt_in.set_table(suggested_vvt_map(8, 8, exhaust=False))
        self.map_vvt_ex.set_table(suggested_vvt_map(8, 8, exhaust=True))
        vvt_lay.addWidget(self.map_vvt_in)
        vvt_lay.addWidget(self.map_vvt_ex)
        self._vvt_tab_idx = self.tabs.addTab(self.vvt_page, "VVT")

        self.boost_page = QWidget()
        bl = QVBoxLayout(self.boost_page)
        row = QHBoxLayout()
        self.boost_map_type = QComboBox()
        self.boost_map_type.addItems(["Closed-loop target (kPa)", "Open-loop duty %"])
        self.boost_map_type.currentIndexChanged.connect(self._boost_type_changed)
        row.addWidget(self.boost_map_type)
        row.addStretch(1)
        bl.addLayout(row)
        self.map_boost = MapView("Boost", is_ign=False, rows=8, cols=8,
                                 kind="boost", vmax=250, rpm_bins=vvt_rpm)
        self.map_boost.set_load_bins(vvt_load, "TPS")
        cl = (self.engine.get("boost_mode") or "Closed-loop") != "Open-loop"
        self.map_boost.set_table(suggested_boost_map(8, 8, closed_loop=cl))
        self.map_boost.vmax = 250.0 if cl else 100.0
        bl.addWidget(self.map_boost)
        self._boost_tab_idx = self.tabs.addTab(self.boost_page, "Boost")

        self.page_ms = QWidget()
        ms_outer = QVBoxLayout(self.page_ms)
        self._ms_form = MotorsportDialog(self.engine, parent=self)
        self._ms_form.setWindowFlags(Qt.Widget)
        if hasattr(self._ms_form, "set_embedded"):
            self._ms_form.set_embedded(True)
        ms_outer.addWidget(self._ms_form)
        btn_ms = QPushButton("Apply motorsport settings")
        btn_ms.clicked.connect(self._apply_motorsport_tab)
        ms_outer.addWidget(btn_ms)
        self._ms_tab_idx = self.tabs.addTab(self.page_ms, "Motorsport")

        self.page_3d = QWidget()
        lay3 = QVBoxLayout(self.page_3d)
        row3 = QHBoxLayout()
        row3.addWidget(QLabel("View"))
        self.combo_3d = QComboBox()
        self.combo_3d.addItems(["Ignition", "Fuel / VE", "Boost", "VVT Intake", "VVT Exhaust"])
        self.combo_3d.currentIndexChanged.connect(self._refresh_3d)
        row3.addWidget(self.combo_3d)
        row3.addStretch(1)
        lay3.addLayout(row3)
        self.map3d = Map3DView()
        lay3.addWidget(self.map3d, 1)
        self._3d_tab_idx = self.tabs.addTab(self.page_3d, "3D")

        self.status = QStatusBar()
        self.setStatusBar(self.status)
        self.conn_lbl = QLabel("Disconnected")
        self.status.addPermanentWidget(self.conn_lbl)

        self._ui_timer = QTimer(self)
        self._ui_timer.timeout.connect(self._refresh_ui)
        self._ui_timer.start(50)

        QShortcut(QKeySequence("F1"), self, activated=self._show_help_overlay)
        QShortcut(QKeySequence("Ctrl+S"), self, activated=self._start_flash)

        self._load_local_settings()
        self._apply_load_bins()
        self._update_feature_tabs()
        self._refresh_fuel_tab_title()
        self._restore_geometry()
        self.refresh_ports()
        QTimer.singleShot(400, self._auto_connect_tick)

    # ── settings / geometry ──
    def _load_local_settings(self):
        try:
            p = _settings_path()
            if p.exists():
                d = json.loads(p.read_text())
                if "engine" in d:
                    self.engine.update(d["engine"])
                if "strip_optional" in d:
                    self.strip_optional = set(d["strip_optional"])
                    self.strip.set_optional(self.strip_optional)
        except Exception:
            pass

    def _save_local_settings(self):
        try:
            p = _settings_path()
            p.parent.mkdir(parents=True, exist_ok=True)
            d = {}
            if p.exists():
                try:
                    d = json.loads(p.read_text())
                except Exception:
                    d = {}
            d["engine"] = self.engine
            d["strip_optional"] = list(self.strip_optional)
            p.write_text(json.dumps(d, indent=2))
        except Exception:
            pass

    def _restore_geometry(self):
        try:
            d = json.loads(_settings_path().read_text())
            if d.get("geometry"):
                self.restoreGeometry(bytes.fromhex(d["geometry"]))
            ti = d.get("last_tab")
            if ti is not None and 0 <= int(ti) < self.tabs.count():
                self.tabs.setCurrentIndex(int(ti))
        except Exception:
            pass

    def closeEvent(self, e):
        try:
            path = _settings_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            d = {}
            if path.exists():
                try:
                    d = json.loads(path.read_text())
                except Exception:
                    d = {}
            d["geometry"] = self.saveGeometry().data().hex()
            d["last_tab"] = self.tabs.currentIndex()
            d["engine"] = self.engine
            path.write_text(json.dumps(d, indent=2))
        except Exception:
            pass
        if self.connected:
            self.worker.disconnect()
        super().closeEvent(e)

    def _apply_load_bins(self):
        lm = (self.engine.get("load_mode") or "MAP").upper()
        if lm in ("TPS", "ALPHA-N", "ALPHA_N"):
            bins = self.engine.get("tps_bins") or make_tps_bins()
            lab = "TPS"
        else:
            bins = self.engine.get("map_bins") or make_map_bins(int(self.engine.get("map_kpa_max") or 240))
            lab = "MAP"
        self.map_ign.set_load_bins(bins, lab)
        self.map_inj.set_load_bins(bins, lab)

    def _update_feature_tabs(self):
        bm = self.engine.get("boost_mode") or "OFF"
        show_boost = bm not in ("OFF", "", None)
        vm = self.engine.get("vvt_mode") or "Disabled"
        show_vvt = vm not in ("Disabled", "", None)
        for idx, show in ((self._vvt_tab_idx, show_vvt), (self._boost_tab_idx, show_boost)):
            try:
                self.tabs.setTabVisible(idx, show)
            except Exception:
                pass

    # ── serial ──
    def refresh_ports(self):
        cur = self.port_combo.currentText()
        self.port_combo.clear()
        ports = list_serial_ports()
        self.port_combo.addItems(ports or ["(no ports)"])
        if cur in ports:
            self.port_combo.setCurrentText(cur)

    def _auto_connect_tick(self):
        if self.connected or self._auto_tried:
            return
        last = self.meta.get("last_port")
        ports = list_serial_ports()
        if last and last in ports:
            self._auto_tried = True
            self.port_combo.setCurrentText(last)
            self.worker.connect_port(last)

    def toggle_connect(self):
        if self.connected:
            self.worker.disconnect()
            return
        port = self.port_combo.currentText()
        if not port or port.startswith("("):
            QMessageBox.warning(self, "Port", "Select a serial port")
            return
        ok, err = self.worker.connect_port(port)
        if not ok:
            QMessageBox.critical(self, "Connect failed", err)

    def _on_conn(self, on: bool):
        self.connected = on
        self.btn_connect.setText("Disconnect" if on else "Connect")
        self.btn_save.setEnabled(on)
        self.conn_lbl.setText("Connected" if on else "Disconnected")
        self.conn_lbl.setStyleSheet("color:#44ff88;" if on else "color:#ff8866;")
        if on:
            self._cfg_pending = True
            self._tx("GETCFG\n")
            self._tx("GETWHEEL\n")
            port = self.port_combo.currentText()
            save_device_meta(last_port=port)
            self.meta["last_port"] = port
            QTimer.singleShot(200, self._post_connect)
        self._update_conn_strip()

    def _post_connect(self):
        self._rx_count = 0
        self._tx("GETUART")
        self._tx("GETPROTO")
        QTimer.singleShot(800, self._post_connect_maps)

    def _post_connect_maps(self):
        if getattr(self, "_rx_count", 0) == 0:
            self.status.showMessage("No RX from ECU — check USB CDC", 10000)
            return
        self._tx("GETCFG")
        self._tx("GETMAP")
        for cmd in list(self._offline_queue):
            self._tx(cmd)
        self._offline_queue.clear()
        self.status.showMessage("Connected — reading maps…")

    def _on_status(self, msg: str):
        self.status.showMessage(msg, 5000)

    def _tx(self, cmd: str) -> bool:
        if not cmd.endswith("\n"):
            cmd += "\n"
        if not self.connected:
            s = cmd.strip()
            if s.startswith("SET:") or s.startswith("CFG:"):
                self._offline_queue.append(s)
            return False
        return self.worker.send(cmd)

    def _on_line(self, line: str):
        up = line.strip().upper()
        self._rx_count = getattr(self, "_rx_count", 0) + 1
        self._rx_window.append(time.time())
        if self._rx_count % 5 == 0:
            self._update_conn_strip()
        self._log.append((time.time(), line.strip()))

        if up.startswith("RPM:"):
            parse_line(line, self.live)
            return
        if up.startswith("CFG:"):
            self._apply_ecu_cfg_line(line.strip())
            return
        if up.startswith("MAP:ADV"):
            self._map_dl_mode = "ADV"
            self._map_dl_row = 0
            self._map_dl_adv = [[0] * COLS for _ in range(ROWS)]
            self._map_dl_t0 = time.time()
            return
        if up.startswith("MAP:INJ"):
            self._map_dl_mode = "INJ"
            self._map_dl_row = 0
            self._map_dl_inj = [[0.0] * COLS for _ in range(ROWS)]
            self._map_dl_t0 = time.time()
            return
        if up.startswith("MAP:END"):
            self._finish_map_dl()
            return
        if self._map_dl_mode in ("ADV", "INJ"):
            if time.time() - getattr(self, "_map_dl_t0", time.time()) > 8.0:
                self._map_dl_mode = None
            else:
                self._ingest_map_row(line)
                return
        if up.startswith("OK:SAVE") or up.startswith("ERR:SAVE"):
            self._flash_state = "OK" if up.startswith("OK:") else "fail"
            self._update_conn_strip()
        parse_line(line, self.live)

    def _ingest_map_row(self, line: str):
        parts = [p.strip() for p in line.strip().split(",") if p.strip() != ""]
        try:
            vals = [float(p) for p in parts]
        except ValueError:
            return
        r = getattr(self, "_map_dl_row", 0)
        if self._map_dl_mode == "ADV" and r < ROWS:
            for c in range(min(COLS, len(vals))):
                self._map_dl_adv[r][c] = int(round(vals[c]))
            self._map_dl_row = r + 1
        elif self._map_dl_mode == "INJ" and r < ROWS:
            for c in range(min(COLS, len(vals))):
                self._map_dl_inj[r][c] = round(vals[c], 1)
            self._map_dl_row = r + 1

    def _finish_map_dl(self):
        if hasattr(self, "_map_dl_adv"):
            self.map_ign.set_table(self._map_dl_adv)
            self.map_ign.mark_clean()
        if hasattr(self, "_map_dl_inj"):
            self.map_inj.set_table(self._map_dl_inj)
            self.map_inj.mark_clean()
        self._map_dl_mode = None
        self._update_dirty_status()
        self.status.showMessage("Maps loaded from ECU")

    def _refresh_ui(self):
        self.strip.update_live(self.live)
        rpm = float(self.live.get("rpm") or 0)
        load = float(self.live.get("load") or self.live.get("map") or 0)
        self.map_ign.set_live(rpm, load)
        self.map_inj.set_live(rpm, load)
        if hasattr(self, "page_3d") and self.tabs.currentWidget() is self.page_3d:
            pass  # 3D is static until combo change
        age = time.time() - (self._rx_window[-1] if self._rx_window else 0)
        if not self.connected:
            self.health_led.setStyleSheet("color:#666;font-size:18px;")
        elif age < 0.5:
            self.health_led.setStyleSheet("color:#44ff88;font-size:18px;")
        elif age < 2.0:
            self.health_led.setStyleSheet("color:#ffcc44;font-size:18px;")
        else:
            self.health_led.setStyleSheet("color:#ff5566;font-size:18px;")

    def _cell_tx(self, which: int, r: int, c: int, v: float):
        if which == 0:
            mx = int(self.engine.get("max_advance") or 40)
            mn = -int(self.engine.get("max_retard") or 10)
            v = max(mn, min(mx, int(round(v))))
            self.map_ign.table[r][c] = v
            self._tx(f"SET:A,{r},{c},{v}\n")
        else:
            mx = float(self.engine.get("max_inj_ms") or 15.0)
            v = max(0.4, min(mx, float(v)))
            self.map_inj.table[r][c] = round(v, 1)
            self._tx(f"SET:I,{r},{c},{v:.1f}\n")

    # ── CFG from ECU ──
    def _apply_ecu_cfg_line(self, line: str):
        if not line.startswith("CFG:"):
            return
        body = line[4:].strip()
        parts: dict[str, str] = {}
        nums: list[float] = []
        for tok in body.replace(";", ",").split(","):
            tok = tok.strip()
            if ":" in tok:
                k, v = tok.split(":", 1)
                parts[k.upper()] = v
            else:
                try:
                    nums.append(float(tok))
                except ValueError:
                    pass
        if len(nums) >= 3:
            parts.setdefault("TEETH", str(int(nums[0])))
            parts.setdefault("MISSING", str(int(nums[1])))
            parts.setdefault("ANGLE", str(int(nums[2])))
        inj_names = dict(INJ_MODE_FROM_ECU)
        boost_names = {0: "OFF", 1: "Closed-loop", 2: "Open-loop"}
        summary = []
        def add(label, key, transform=None):
            if key not in parts:
                return
            val = parts[key]
            if transform:
                try:
                    val = transform(val)
                except Exception:
                    pass
            summary.append((label, str(val)))
        add("Crank teeth", "TEETH")
        add("Missing", "MISSING")
        add("Trigger angle", "ANGLE")
        add("Cylinders", "CYL")
        add("Inj mode", "INJMODE", lambda v: inj_names.get(int(float(v)), v))
        add("Ign mode", "IGNMODE", lambda v: "Sequential" if int(float(v)) else "Wasted spark")
        add("Cam home", "CAMMODE", lambda v: "ON" if int(float(v)) else "OFF")
        add("Coil", "COILTYPE", lambda v: COIL_TYPE_FROM_ECU.get(int(float(v)), v))
        add("Batch above", "BATCHRPM")
        add("VE mode", "VEMODE", lambda v: "ON" if int(float(v)) else "OFF")
        add("Req fuel", "REQFUEL")
        add("Flow cc", "FLOW")
        self._pending_cfg_parts = parts
        self._last_cfg_summary = ",".join(f"{a}={b}" for a, b in summary[:4]) or "ok"
        self._update_conn_strip()
        if not getattr(self, "_cfg_pending", False):
            self._merge_cfg_parts(parts)
            return
        self._cfg_pending = False
        self.mismatch_lbl.setText("ECU settings differ from local — Apply ECU or Keep local")
        self.mismatch_bar.show()
        dlg = QDialog(self)
        dlg.setWindowTitle("ECU settings on connect")
        lay = QVBoxLayout(dlg)
        lay.addWidget(QLabel("Settings read from the microcontroller:"))
        lst = QListWidget()
        for lab, val in summary:
            lst.addItem(f"{lab}:  {val}")
        lay.addWidget(lst)
        bb = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        bb.button(QDialogButtonBox.Ok).setText("Apply to tuner")
        bb.button(QDialogButtonBox.Cancel).setText("Keep local")
        bb.accepted.connect(dlg.accept)
        bb.rejected.connect(dlg.reject)
        lay.addWidget(bb)
        if dlg.exec():
            self._merge_cfg_parts(parts)
            self._save_local_settings()
            self.mismatch_bar.hide()
            self.status.showMessage("Engine settings updated from ECU")

    def _merge_cfg_parts(self, parts: dict):
        if "TEETH" in parts:
            self.engine["teeth"] = int(float(parts["TEETH"]))
        if "MISSING" in parts:
            self.engine["missing"] = int(float(parts["MISSING"]))
        if "ANGLE" in parts:
            self.engine["trig_angle"] = int(float(parts["ANGLE"]))
            self.trig_slider.blockSignals(True)
            self.trig_slider.setValue(int(self.engine["trig_angle"]))
            self.trig_slider.blockSignals(False)
            self.trig_val.setText(f"{int(self.engine['trig_angle'])}°")
        if "CYL" in parts:
            c = int(float(parts["CYL"]))
            self.engine["cylinders"] = c
            if c in (5, 6, 8):
                self.engine["inj_mode"] = "Batch"
        if "INJMODE" in parts:
            im = int(float(parts["INJMODE"]))
            self.engine["inj_mode"] = INJ_MODE_FROM_ECU.get(im, "Sequential")
        if "IGNMODE" in parts:
            self.engine["ign_sequential"] = bool(int(float(parts["IGNMODE"])))
        if "CAMMODE" in parts:
            self.engine["cam_home"] = bool(int(float(parts["CAMMODE"])))
        if "COILTYPE" in parts:
            ct = int(float(parts["COILTYPE"]))
            if ct in COIL_TYPE_FROM_ECU:
                self.engine["coil_type"] = COIL_TYPE_FROM_ECU[ct]
        if "BATCHRPM" in parts:
            self.engine["batch_above_rpm"] = int(float(parts["BATCHRPM"]))
        if "VEMODE" in parts:
            self.engine["ve_mode"] = bool(int(float(parts["VEMODE"])))
            self.btn_ve.blockSignals(True)
            self.btn_ve.setChecked(bool(self.engine["ve_mode"]))
            self.btn_ve.blockSignals(False)
            self._refresh_fuel_tab_title()
        if "REQFUEL" in parts:
            self.engine["req_fuel_ms"] = float(parts["REQFUEL"])
        if "FLOW" in parts:
            self.engine["inj_flow_cc"] = float(parts["FLOW"])
        if "WHEEL" in parts:
            w = WHEEL_FROM_ECU.get(int(float(parts["WHEEL"])))
            if w is not None:
                self.engine["wheel_id"] = w
        if "BOOST" in parts:
            b = int(float(parts["BOOST"]))
            self.engine["boost_mode"] = {0: "OFF", 1: "Closed-loop", 2: "Open-loop"}.get(b, "OFF")
        self._update_feature_tabs()

    def _banner_apply_ecu(self):
        if self._pending_cfg_parts:
            self._merge_cfg_parts(self._pending_cfg_parts)
            self._save_local_settings()
        self.mismatch_bar.hide()

    # ── tools ──
    def _on_trig_slider(self, v: int):
        self.trig_val.setText(f"{v}°")
        self.engine["trig_angle"] = int(v)
        if self.connected:
            self._tx("SET:TRIG,%d\n" % int(v))

    def _refresh_fuel_tab_title(self):
        ve = bool(self.engine.get("ve_mode", True))
        badge = "VE %" if ve else "ms"
        for i in range(self.tabs.count()):
            if self.tabs.tabText(i).startswith("Injection"):
                self.tabs.setTabText(i, f"Injection [{badge}]")
                break

    def _ve_mode_toggled(self, on: bool):
        self.engine["ve_mode"] = bool(on)
        flow = float(self.engine.get("inj_flow_cc") or 220)
        req = float(self.engine.get("req_fuel_ms") or 2.5)
        p_act = float(self.engine.get("fuel_pressure_bar") or 3.0)
        p_rat = float(self.engine.get("fuel_pressure_rated_bar") or 3.0)
        if on:
            self.map_inj.hdr.setText("VE (%)")
            self.map_inj.vmax = 120.0
            self.map_inj.set_table(suggested_ve_map())
        else:
            self.map_inj.hdr.setText("Injection (ms)")
            self.map_inj.vmax = 40.0
            self.map_inj.set_table(suggested_inj_ms_map(
                req_fuel_ms=req, flow_cc=flow,
                fuel_pressure_bar=p_act, fuel_pressure_rated_bar=p_rat))
        if self.connected:
            self._tx("SET:VEMODE,%d\n" % (1 if on else 0))
            self._tx("SET:REQFUEL,%.2f,%.1f,%.2f,%.2f\n" % (req, flow, p_act, p_rat))
            for r in range(ROWS):
                for c in range(COLS):
                    self._cell_tx(1, r, c, float(self.map_inj.table[r][c]))
        self._refresh_fuel_tab_title()
        self._save_local_settings()
        self._refresh_3d()

    def _fill_recommended_fuel(self):
        if self.engine.get("ve_mode", True):
            self.map_inj.set_table(suggested_ve_map())
        else:
            self.map_inj.set_table(suggested_inj_ms_map(
                req_fuel_ms=float(self.engine.get("req_fuel_ms") or 2.5),
                flow_cc=float(self.engine.get("inj_flow_cc") or 220),
                fuel_pressure_bar=float(self.engine.get("fuel_pressure_bar") or 3.0),
                fuel_pressure_rated_bar=float(self.engine.get("fuel_pressure_rated_bar") or 3.0),
            ))
        if self.connected:
            for r in range(ROWS):
                for c in range(COLS):
                    self._cell_tx(1, r, c, float(self.map_inj.table[r][c]))
        self.status.showMessage("Fuel map filled with recommended values")

    def _boost_type_changed(self, idx: int):
        cl = idx == 0
        self.map_boost.set_table(suggested_boost_map(8, 8, closed_loop=cl))
        self.map_boost.vmax = 250.0 if cl else 100.0
        self.map_boost.hdr.setText("Boost target (kPa)" if cl else "Boost duty %")
        self._refresh_3d()

    def _refresh_3d(self, *_):
        if not hasattr(self, "map3d"):
            return
        idx = self.combo_3d.currentIndex() if hasattr(self, "combo_3d") else 0

        def axes(mv, zlab):
            return dict(
                rpm_bins=getattr(mv, "rpm_bins", None),
                load_bins=getattr(mv, "load_bins", None),
                x_label="RPM",
                y_label=getattr(mv, "load_label", "LOAD") or "LOAD",
                z_label=zlab,
            )

        if idx == 0:
            self.map3d.set_table(self.map_ign.table, "Ignition 3D", **axes(self.map_ign, "ADV °"))
        elif idx == 1:
            z = "VE %" if self.engine.get("ve_mode", True) else "INJ ms"
            self.map3d.set_table(self.map_inj.table, "Fuel 3D", **axes(self.map_inj, z))
        elif idx == 2 and hasattr(self, "map_boost"):
            self.map3d.set_table(self.map_boost.table, "Boost 3D", **axes(self.map_boost, "BOOST"))
        elif idx == 3 and hasattr(self, "map_vvt_in"):
            self.map3d.set_table(self.map_vvt_in.table, "VVT In 3D", **axes(self.map_vvt_in, "VVT °"))
        elif idx == 4 and hasattr(self, "map_vvt_ex"):
            self.map3d.set_table(self.map_vvt_ex.table, "VVT Ex 3D", **axes(self.map_vvt_ex, "VVT °"))

    def _update_dirty_status(self):
        n = len(self.map_ign.dirty) + len(self.map_inj.dirty)
        if n:
            self.status.showMessage(f"{n} cells unsaved — Flash to write ECU")
        self._update_conn_strip()

    def _update_conn_strip(self):
        port = self.port_combo.currentText()
        now = time.time()
        self._rx_window = [t for t in self._rx_window if now - t < 1.0]
        rx = len(self._rx_window)
        self.conn_strip.setText(
            f"Port: {port}  ·  RX: {rx}/s  ·  CFG: {self._last_cfg_summary}  ·  Flash: {self._flash_state}"
        )

    def _start_flash(self):
        if not self.connected:
            return
        self._flash_state = "saving"
        self.flash_bar.show()
        self._update_conn_strip()
        self._flash_worker = FlashSaveWorker(self._tx, self)
        self._flash_worker.done.connect(self._on_flash_done)
        self._flash_worker.start()

    def _on_flash_done(self, ok: bool, msg: str):
        self._flash_state = "OK" if ok else "fail"
        self.flash_bar.hide()
        if ok:
            self.map_ign.mark_clean()
            self.map_inj.mark_clean()
        self._update_conn_strip()
        self.status.showMessage("Flash OK" if ok else f"Flash failed: {msg}")

    def open_dashboard(self):
        DashboardDialog(lambda: self.live, parent=self).exec()

    def open_program_settings(self):
        dlg = ProgramSettingsDialog(self.strip_optional, parent=self)
        if dlg.exec():
            self.strip_optional = dlg.selected_keys()
            self.strip.set_optional(self.strip_optional)
            self._save_local_settings()

    def open_engine_settings(self):
        rpm = int(self.live.get("rpm") or 0)
        dlg = EngineSettingsDialog(self.engine, rpm=rpm, parent=self)
        if dlg.exec():
            dlg.apply_to(self.engine)
            self._apply_load_bins()
            self._update_feature_tabs()
            ve_on = bool(self.engine.get("ve_mode", True))
            self.btn_ve.blockSignals(True)
            self.btn_ve.setChecked(ve_on)
            self.btn_ve.blockSignals(False)
            self._refresh_fuel_tab_title()
            self.trig_slider.blockSignals(True)
            self.trig_slider.setValue(int(self.engine.get("trig_angle") or 30))
            self.trig_slider.blockSignals(False)
            self.trig_val.setText(f"{int(self.engine.get('trig_angle') or 30)}°")
            self._save_local_settings()
            if rpm == 0:
                # queued while offline, flushed on connect
                self._push_engine_config()
            if self.connected:
                self._tx("SET:TRIG,%d\n" % int(self.engine.get("trig_angle") or 30))
                self._tx("SET:REQFUEL,%.2f,%.1f,%.2f,%.2f\n" % (
                    float(self.engine.get("req_fuel_ms") or 2.5),
                    float(self.engine.get("inj_flow_cc") or 220),
                    float(self.engine.get("fuel_pressure_bar") or 3.0),
                    float(self.engine.get("fuel_pressure_rated_bar") or 3.0),
                ))
                self._tx("SET:VEMODE,%d\n" % (1 if ve_on else 0))
                self._tx("SET:IGNLIM,%d,%d\n" % (
                    int(self.engine.get("max_advance") or 40),
                    int(self.engine.get("max_retard") or 10),
                ))
                self._tx("SET:INJMAX,%.1f\n" % float(self.engine.get("max_inj_ms") or 15.0))
                self._tx("SET:DFCO,%d,%d,%d,%.1f,%.0f,%d\n" % (
                    1 if self.engine.get("dfco_enable", True) else 0,
                    int(self.engine.get("dfco_enter_rpm") or 1600),
                    int(self.engine.get("dfco_exit_rpm") or 1200),
                    float(self.engine.get("dfco_max_tps") or 3.0),
                    float(self.engine.get("dfco_min_ect") or 50.0),
                    int(self.engine.get("dfco_delay_ms") or 200),
                ))
                self.map_ign.vmax = float(self.engine.get("max_advance") or 40)
                self.map_inj.vmax = float(self.engine.get("max_inj_ms") or 15.0)
                # Settings only survive a power cycle once written to flash,
                # and flashing halts capture/scheduling, so never while turning
                if rpm == 0:
                    self._start_flash()

    def _push_engine_config(self):
        """Send the full crank/cam/sequential configuration to the ECU.

        Order matters: the wheel profile rewrites teeth/missing/cam mode and
        CFG clears sync, so both go before the sequential settings.
        """
        eng = self.engine
        cyl = int(eng.get("cylinders") or 4)
        cam_home = bool(eng.get("cam_home", True))
        coil = COIL_TYPE_TO_ECU.get(eng.get("coil_type") or "Smart", 0)
        inj_mode = eng.get("inj_mode") or "Sequential"
        inj_code = INJ_MODE_TO_ECU.get(inj_mode, INJ_MODE_TO_ECU["Batch"])
        if not cam_home:
            inj_code = INJ_MODE_TO_ECU["Batch"]
        # Keep whatever the ECU reported; a distributor drives a single output,
        # so its spark can never be sequential
        ign_seq = 1 if bool(eng.get("ign_sequential", cam_home)) else 0
        if coil == 2 or not cam_home:
            ign_seq = 0

        wheel = WHEEL_TO_ECU.get(int(eng.get("wheel_id") or 0))
        if wheel is not None:
            self._tx("SET:WHEEL,%d\n" % wheel)
        self._tx("CFG:%d,%d,%d\n" % (
            int(eng.get("teeth") or 36),
            int(eng.get("missing") or 1),
            int(eng.get("trig_angle") or 30),
        ))
        self._tx("SET:CYL,%d\n" % cyl)
        self._tx("SET:SENS:CAMMODE,%d\n" % (1 if cam_home else 0))
        self._tx("SET:COILTYPE,%d\n" % coil)
        self._tx("SET:IGNMODE,%d\n" % ign_seq)
        self._tx("SET:INJMODE,%d\n" % inj_code)
        self._tx("SET:BATCHRPM,%d\n" % int(eng.get("batch_above_rpm") or 3000))
        self._tx("GETCFG\n")

    def _apply_motorsport_tab(self):
        if hasattr(self, "_ms_form"):
            self._ms_form.apply_to(self.engine)
            self._save_local_settings()
            self.status.showMessage("Motorsport settings applied")

    def interpolate_maps(self):
        self.map_ign.smooth_selected(1)
        self.map_inj.smooth_selected(1)
        self.status.showMessage("Maps smoothed (selected / all)")

    def smooth_maps(self):
        self.interpolate_maps()

    def save_csv_log(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save log", "strix_log.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "line"])
            for t, line in self._log:
                w.writerow([f"{t:.3f}", line])
        self.status.showMessage(f"Saved {path}")

    def _export_pack(self):
        path, _ = QFileDialog.getSaveFileName(self, "Export", "strix_export.tcal", "TCAL (*.tcal);;CSV (*.csv)")
        if not path:
            return
        if path.lower().endswith(".csv"):
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["map", "r", "c", "value"])
                for name, mv in (("ign", self.map_ign), ("inj", self.map_inj)):
                    for r in range(mv.rows):
                        for c in range(mv.cols):
                            w.writerow([name, r, c, mv.table[r][c]])
        else:
            save_tcal(path, self.engine)
        self.status.showMessage(f"Exported {path}")

    def _import_pack(self):
        path, _ = QFileDialog.getOpenFileName(self, "Import", "", "TCAL (*.tcal)")
        if not path:
            return
        self.engine = load_tcal(path)
        self._apply_load_bins()
        self._update_feature_tabs()
        self._refresh_fuel_tab_title()
        self.status.showMessage(f"Imported {path}")

    def _show_help_overlay(self):
        QMessageBox.information(
            self, "Keyboard",
            "Arrows — move cell\nPgUp/+ increase · PgDn/− decrease\n"
            "Shift — ×5 step\nCtrl+P — % change\nCtrl+C/V — copy/paste\n"
            "Ctrl+S — flash\nF1 — help\n3D: LMB orbit · RMB pan · wheel zoom · R reset",
        )
