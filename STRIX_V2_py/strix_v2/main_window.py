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
    suggested_vvt_map, suggested_boost_map, suggested_afr_map,
    suggested_idle_fuel_map, suggested_idle_ign_map,
)
from strix_v2.serial_worker import SerialWorker, list_serial_ports
from strix_v2.protocol import default_live, parse_line
from strix_v2.widgets.live_strip import LiveStrip
from strix_v2.widgets.map_view import MapView
from strix_v2.widgets.map3d import Map3DView
from strix_v2.widgets.dashboard import DashboardDialog
from strix_v2.widgets.cyl_trim import CylTrimPage
from strix_v2.widgets.startup_page import StartupPage
from strix_v2.widgets.log_viewer import LogViewer
from strix_v2.widgets.runtime_cluster import RuntimeCluster
from strix_v2.widgets.curve import CurvePage
from strix_v2.widgets.warmup import WarmupWizardDialog
from strix_v2.datalog import DataLogger
from strix_v2.inc_lookup import parse_inc, linear_wb_span, downsample_wb
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

# Same IDs as firmware ecu_wheels.c
WHEEL_TO_ECU = {0: 0, 1: 1, 2: 2, 3: 3, 4: 4, 5: 5, 6: 6, 7: 7, 8: 8, 9: 9, 10: 10, 11: 11}
WHEEL_FROM_ECU = {v: k for k, v in WHEEL_TO_ECU.items()}


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
        self._tx_queue: deque = deque()
        self._log: deque = deque(maxlen=600)
        self.logger = DataLogger()
        self._rx_count = 0
        self._auto_tried = False

        self.worker = SerialWorker()
        self.worker.line_received.connect(self._on_line)
        self.worker.connected_changed.connect(self._on_conn)
        self.worker.status.connect(self._on_status)
        self._tx_timer = QTimer(self)
        self._tx_timer.setInterval(25)
        self._tx_timer.timeout.connect(self._drain_tx)

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
        self.btn_read = QPushButton("Read ECU")
        self.btn_read.setToolTip("GETCFG + GETMAP from controller")
        self.btn_read.clicked.connect(self._read_from_ecu)
        self.btn_read.setEnabled(False)
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
                  self.btn_read, self.btn_save, self.health_led):
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
        self.btn_rec = QPushButton("Log")
        self.btn_rec.setCheckable(True)
        self.btn_rec.clicked.connect(self._toggle_record)
        self.btn_log = QPushButton("Save RX")
        self.btn_log.clicked.connect(self.save_csv_log)
        self.btn_offline = QPushButton("Offline")
        self.btn_offline.setCheckable(True)
        self.btn_offline.setToolTip("Stay disconnected — queue SET/CFG locally")
        self.btn_offline.toggled.connect(self._offline_toggled)
        self.btn_engine = QPushButton("Engine Settings")
        self.btn_engine.clicked.connect(self.open_engine_settings)
        self.btn_interp = QPushButton("Interpolate")
        self.btn_interp.clicked.connect(self.interpolate_maps)
        self.btn_smooth = QPushButton("Smooth")
        self.btn_smooth.clicked.connect(self.smooth_maps)
        self.btn_fill_rec = QPushButton("Fill rec.")
        self.btn_fill_rec.clicked.connect(self._fill_recommended_fuel)
        self.btn_dash = QPushButton("Dash")
        self.btn_dash.clicked.connect(self.open_dashboard)
        self.btn_export = QPushButton("Export")
        self.btn_export.clicked.connect(self._export_pack)
        self.btn_import = QPushButton("Import")
        self.btn_import.clicked.connect(self._import_pack)
        # keep VE state without toolbar button (Engine Settings → Fuel)
        self.btn_ve = QPushButton("VE")
        self.btn_ve.setCheckable(True)
        self.btn_ve.setChecked(bool(self.engine.get("ve_mode", True)))
        self.btn_ve.toggled.connect(self._ve_mode_toggled)
        self.btn_ve.hide()
        for b in (self.btn_rec, self.btn_log, self.btn_offline,
                  self.btn_engine, self.btn_interp,
                  self.btn_smooth, self.btn_fill_rec, self.btn_dash,
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

        strip_row = QHBoxLayout()
        strip_row.setSpacing(4)
        self.strip = LiveStrip()
        self.strip.set_optional(self.strip_optional)
        strip_row.addWidget(self.strip, 1)
        self.btn_strip_info = QPushButton("Strip Info")
        self.btn_strip_info.setMaximumHeight(28)
        self.btn_strip_info.setToolTip("Choose optional live-strip channels")
        self.btn_strip_info.clicked.connect(self.open_program_settings)
        strip_row.addWidget(self.btn_strip_info, 0, Qt.AlignTop)
        root.addLayout(strip_row)

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

        self.cyl_trim = CylTrimPage(int(self.engine.get("cylinders") or 4))
        self.cyl_trim.trim_changed.connect(self._on_cyl_trim)
        self.cyl_trim.disable_changed.connect(self._on_inj_disable)
        self._cyl_tab_idx = self.tabs.addTab(self.cyl_trim, "Cyl trim")

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

        self.afr_page = QWidget()
        al = QVBoxLayout(self.afr_page)
        al.addWidget(QLabel("AFR target — active only with Wideband O2"))
        self.map_afr = MapView("AFR target", is_ign=False, kind="afr", vmax=16.0)
        self.map_afr.set_table(suggested_afr_map())
        self.map_afr.cell_changed.connect(lambda r, c, v: self._cell_tx(2, r, c, v))
        al.addWidget(self.map_afr)
        self._afr_tab_idx = self.tabs.addTab(self.afr_page, "AFR target")

        # Idle 5×5 fuel + ign correction
        self.idle_page = QWidget()
        il = QHBoxLayout(self.idle_page)
        idle_rpm = [600, 750, 900, 1050, 1200]
        idle_ect = [-10, 20, 40, 60, 80]
        self.map_idle_fuel = MapView("Idle fuel corr (%)", is_ign=False, rows=5, cols=5,
                                     kind="inj", vmax=30.0, rpm_bins=idle_rpm)
        self.map_idle_fuel.set_load_bins(idle_ect, "ECT")
        self.map_idle_fuel.set_table(suggested_idle_fuel_map())
        self.map_idle_fuel.cell_changed.connect(lambda r, c, v: self._idle_tx("FUEL", r, c, v))
        self.map_idle_ign = MapView("Idle ign corr (°)", is_ign=True, rows=5, cols=5,
                                    kind="ign", vmax=20.0, rpm_bins=idle_rpm)
        self.map_idle_ign.set_load_bins(idle_ect, "ECT")
        self.map_idle_ign.set_table(suggested_idle_ign_map())
        self.map_idle_ign.cell_changed.connect(lambda r, c, v: self._idle_tx("IGN", r, c, v))
        il.addWidget(self.map_idle_fuel)
        il.addWidget(self.map_idle_ign)
        self._idle_tab_idx = self.tabs.addTab(self.idle_page, "Idle")

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
        self.map3d.cell_changed.connect(self._on_3d_cell)
        lay3.addWidget(self.map3d, 1)
        self._3d_tab_idx = self.tabs.addTab(self.page_3d, "3D")

        self.runtime = RuntimeCluster()
        self._rt_tab_idx = self.tabs.addTab(self.runtime, "Runtime")

        self.curves = CurvePage(self.engine)
        self.curves.changed.connect(self._on_curve_changed)
        self._curve_tab_idx = self.tabs.addTab(self.curves, "Curves")

        self.log_view = LogViewer()
        self._log_tab_idx = self.tabs.addTab(self.log_view, "Logs")

        self.startup = StartupPage(self.engine)
        self.startup.apply_requested.connect(self._apply_startup)
        self.startup.open_wue.connect(lambda: self._goto_curve("wue"))
        self.startup.open_ase.connect(lambda: self._goto_curve("ase"))
        self._startup_tab_idx = self.tabs.addTab(self.startup, "Startup")

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
            bins = self.engine.get("map_bins") or make_map_bins(int(self.engine.get("map_kpa_max") or 240), int(self.engine.get("map_kpa_min") or 0))
            lab = "MAP"
        self.map_ign.set_load_bins(bins, lab)
        self.map_inj.set_load_bins(bins, lab)
        if hasattr(self, "map_afr"):
            self.map_afr.set_load_bins(bins, lab)

    def _update_feature_tabs(self):
        bm = self.engine.get("boost_mode") or "OFF"
        show_boost = bm not in ("OFF", "", None)
        vm = self.engine.get("vvt_mode") or "Disabled"
        show_vvt = vm not in ("Disabled", "", None)
        o2 = (self.engine.get("o2_mode") or "Disabled")
        show_afr = o2 == "Wideband"
        pairs = [
            (self._vvt_tab_idx, show_vvt),
            (self._boost_tab_idx, show_boost),
        ]
        if hasattr(self, "_afr_tab_idx"):
            pairs.append((self._afr_tab_idx, show_afr))
        if show_afr and self.connected:
            self._tx("SET:AFRMAPEN,1\n")
        elif self.connected:
            self._tx("SET:AFRMAPEN,0\n")
        for idx, show in pairs:
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
        if self.connected or self._auto_tried or self.btn_offline.isChecked():
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
        if self.btn_offline.isChecked():
            self.status.showMessage("Offline mode — uncheck Offline to connect")
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
        self.btn_read.setEnabled(on)
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
        # Read only — never push local maps/settings over ECU on connect
        self._tx("GETCFG")
        self._tx("GETMAP")
        for cmd in list(self._offline_queue):
            self._tx(cmd)
        self._offline_queue.clear()
        self.status.showMessage("Connected — reading ECU maps & settings…")

    def _on_status(self, msg: str):
        self.status.showMessage(msg, 4000)
        low = (msg or "").lower()
        if any(s in low for s in ("clearcomm", "permission", "command 22", "access is denied")):
            if self.connected and not getattr(self, "_reopening", False):
                self._reopening = True
                QTimer.singleShot(1200, self._try_reopen_port)

    def _try_reopen_port(self):
        self._reopening = False
        port = ""
        if hasattr(self, "port_combo"):
            port = self.port_combo.currentText().strip()
        if not port:
            return
        ok, err = self.worker.connect_port(port)
        if ok:
            self.connected = True
            self.status.showMessage("Serial reopened after USB hiccup", 3000)
        else:
            self.connected = False
            self.status.showMessage(f"Reopen failed: {err}", 5000)

    def _tx(self, cmd: str) -> bool:
        if not cmd.endswith("\n"):
            cmd += "\n"
        if not self.connected:
            s = cmd.strip()
            if s.startswith("SET:") or s.startswith("CFG:"):
                self._offline_queue.append(s)
            return False
        self._tx_queue.append(cmd)
        if not self._tx_timer.isActive():
            self._tx_timer.start()
        return True

    def _drain_tx(self):
        if not self.connected:
            self._tx_queue.clear()
            self._tx_timer.stop()
            return
        if not self._tx_queue:
            self._tx_timer.stop()
            return
        cmd = self._tx_queue.popleft()
        self.worker.send(cmd)

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
            queued = "QUEUED" in up
            self._flash_state = "queued" if queued else ("OK" if up.startswith("OK:") else "fail")
            if up.startswith("OK:"):
                self.map_ign.mark_clean()
                self.map_inj.mark_clean()
            self._update_conn_strip()
            if queued:
                self.status.showMessage("Flash queued — writes when RPM = 0")
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
            # Heatmap scale follows ECU fuel mode (VE % vs injector duty ms)
            self._apply_fuel_mode_heatmap(fill_suggested=False)
            self.map_inj.set_table(self._map_dl_inj)
            self.map_inj.mark_clean()
        self._map_dl_mode = None
        self._refresh_3d()
        self._update_dirty_status()
        self.status.showMessage("Maps loaded from ECU", 4000)

    def _live_load_for_maps(self) -> float:
        """Load in same units as displayed map axis (MAP kPa or TPS %).

        Always use live MAP/TPS telemetry — not firmware LOAD or MCELL —
        so the crosshair lands on the row whose label matches the strip value.
        """
        lab = ""
        if hasattr(self, "map_inj"):
            lab = (getattr(self.map_inj, "load_label", "") or "").upper()
        if "TPS" in lab:
            return float(self.live.get("tps") or 0)
        return float(self.live.get("map") or 0)

    def _refresh_ui(self):
        self.strip.update_live(
            self.live,
            map_kpa_max=self.engine.get("map_kpa_max") or 240,
            load_mode=self.engine.get("load_mode") or "MAP",
        )
        rpm = float(self.live.get("rpm") or 0)
        load = self._live_load_for_maps()
        self.map_ign.set_live(rpm, load)
        self.map_inj.set_live(rpm, load)
        if hasattr(self, "map_afr") and self.tabs.isTabVisible(self._afr_tab_idx):
            self.map_afr.set_live(rpm, load)
        if hasattr(self, "map_idle_fuel"):
            ect = float(self.live.get("ect") or 0)
            self.map_idle_fuel.set_live(rpm, ect)
            self.map_idle_ign.set_live(rpm, ect)
        if hasattr(self, "runtime"):
            self.runtime.update_live(self.live)
        if hasattr(self, "curves"):
            self.curves.set_live(self.live)
        if self.logger.active:
            self.logger.append(self.live)
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
        elif which == 2:
            v = max(8.0, min(22.0, float(v)))
            self.map_afr.table[r][c] = round(v, 1)
            self._tx(f"SET:AFR,{r},{c},{v:.1f}\n")
        else:
            if bool(self.engine.get("ve_mode", True)):
                v = max(20.0, min(120.0, float(v)))
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
        add("EOI BTDC", "EOI")
        add("Cylinders", "CYL")
        add("Inj mode", "INJMODE", lambda v: inj_names.get(int(float(v)), v))
        add("Ign mode", "IGNMODE", lambda v: "Sequential" if int(float(v)) else "Wasted spark")
        add("Cam home", "CAMMODE", lambda v: "ON" if int(float(v)) else "OFF")
        add("Coil", "COILTYPE", lambda v: COIL_TYPE_FROM_ECU.get(int(float(v)), v))
        add("Batch above", "BATCHRPM")
        add("VE mode", "VEMODE", lambda v: "ON" if int(float(v)) else "OFF")
        add("Req fuel", "REQFUEL")
        add("Flow cc", "FLOW")
        add("MAP scale", "MAPSCALE")
        self._pending_cfg_parts = parts
        self._last_cfg_summary = ",".join(f"{a}={b}" for a, b in summary[:4]) or "ok"
        self._update_conn_strip()
        # Always take ECU as source of truth — no prompt dialog
        self._cfg_pending = False
        self._merge_cfg_parts(parts)
        self._save_local_settings()
        self.mismatch_bar.hide()
        ve = "VE %" if self.engine.get("ve_mode", True) else "Injector Duty"
        self.status.showMessage(f"ECU settings applied — fuel mode: {ve}", 5000)

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
        if "EOI" in parts:
            e = int(float(parts["EOI"]))
            if e < 10:
                e = 10
            if e > 540:
                e = 540
            self.engine["eoi_btdc"] = e
        if "CYL" in parts:
            c = int(float(parts["CYL"]))
            self.engine["cylinders"] = c
            if c in (5, 6, 8):
                self.engine["inj_mode"] = "Batch"
        if "INJMODE" in parts:
            im = int(float(parts["INJMODE"]))
            self.engine["inj_mode"] = INJ_MODE_FROM_ECU.get(im, "Batch")
        if "IGNMODE" in parts:
            ign = int(float(parts["IGNMODE"]))
            self.engine["ign_mode"] = "Sequential" if ign == 1 else "Wasted Spark"
        if "IGNMODE" in parts or "INJMODE" in parts:
            ign_s = str(self.engine.get("ign_mode") or "")
            inj_s = str(self.engine.get("inj_mode") or "")
            if ign_s.startswith("Seq") and inj_s.startswith("Seq"):
                self.engine["run_mode"] = "Sequential"
            else:
                self.engine["run_mode"] = "Batch"
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
            # Heatmap scale only — table values come from GETMAP
            self._apply_fuel_mode_heatmap(fill_suggested=False)
        if "REQFUEL" in parts:
            self.engine["req_fuel_ms"] = float(parts["REQFUEL"])
        if "FLOW" in parts:
            self.engine["inj_flow_cc"] = float(parts["FLOW"])
        if "MAPSCALE" in parts:
            # "min:max" from CFG:...MAPSCALE:0:500
            raw = parts["MAPSCALE"].strip()
            try:
                if ":" in raw:
                    a, b = raw.split(":", 1)
                    mn, mx = float(a), float(b)
                else:
                    mn, mx = 0.0, float(raw)
                if mx >= 20:
                    self.engine["map_kpa_min"] = int(round(mn))
                    self.engine["map_kpa_max"] = int(round(mx))
                    from strix_v2.constants import make_map_bins
                    self.engine["map_bins"] = make_map_bins(
                        int(self.engine["map_kpa_max"]),
                        int(self.engine["map_kpa_min"]),
                    )
                    self._apply_load_bins()
            except (ValueError, TypeError):
                pass
        if "WHEEL" in parts:
            w = WHEEL_FROM_ECU.get(int(float(parts["WHEEL"])))
            if w is not None:
                self.engine["wheel_id"] = w
                from strix_v2.constants import WHEEL_PROFILES
                for wid, name, teeth, miss in WHEEL_PROFILES:
                    if wid == w:
                        self.engine["teeth"] = teeth
                        self.engine["missing"] = miss
                        if "cam" in name.lower():
                            self.engine["cam_home"] = True
                        break
        if "CAM" in parts and "CAMMODE" not in parts:
            parts["CAMMODE"] = parts["CAM"]
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

    def _apply_fuel_mode_heatmap(self, fill_suggested: bool = True):
        """Update fuel map header, vmax (heatmap scale), and optionally fill suggested table."""
        ve = bool(self.engine.get("ve_mode", True))
        flow = float(self.engine.get("inj_flow_cc") or 220)
        req = float(self.engine.get("req_fuel_ms") or 2.5)
        p_act = float(self.engine.get("fuel_pressure_bar") or 3.0)
        p_rat = float(self.engine.get("fuel_pressure_rated_bar") or 3.0)
        if ve:
            self.map_inj.hdr.setText("VE (%)")
            self.map_inj.vmax = 120.0
            if fill_suggested:
                self.map_inj.set_table(suggested_ve_map())
        else:
            self.map_inj.hdr.setText("Injection (ms)")
            self.map_inj.vmax = float(self.engine.get("max_inj_ms") or 15.0)
            if self.map_inj.vmax < 10.0:
                self.map_inj.vmax = 15.0
            if fill_suggested:
                self.map_inj.set_table(suggested_inj_ms_map(
                    req_fuel_ms=req, flow_cc=flow,
                    fuel_pressure_bar=p_act, fuel_pressure_rated_bar=p_rat))
        # Force heatmap repaint even when table not replaced
        if hasattr(self.map_inj, "legend"):
            self.map_inj.legend.setText(
                f"Scale 0 → {self.map_inj.vmax:g}  |  cyan=live  |  trail=last 50  |  "
                f"Arrows select  |  +/- or PgUp/Dn  |  Shift×5  |  Ctrl+C/V  |  Ctrl+P %"
            )
        if hasattr(self.map_inj, "_canvas"):
            self.map_inj._canvas.update()
        self._refresh_fuel_tab_title()

    def _ve_mode_toggled(self, on: bool):
        self.engine["ve_mode"] = bool(on)
        self._apply_fuel_mode_heatmap(fill_suggested=True)
        flow = float(self.engine.get("inj_flow_cc") or 220)
        req = float(self.engine.get("req_fuel_ms") or 2.5)
        p_act = float(self.engine.get("fuel_pressure_bar") or 3.0)
        p_rat = float(self.engine.get("fuel_pressure_rated_bar") or 3.0)
        if self.connected:
            self._tx("SET:VEMODE,%d\n" % (1 if on else 0))
            self._tx("SET:REQFUEL,%.2f,%.1f,%.2f,%.2f\n" % (req, flow, p_act, p_rat))
            for r in range(ROWS):
                for c in range(COLS):
                    self._cell_tx(1, r, c, float(self.map_inj.table[r][c]))
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
        self.status.showMessage("VE map filled with NA recommended values")

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
        mode = "ONLINE" if self.connected else f"OFFLINE q={len(self._offline_queue)}"
        rec = f"  ·  LOG: {self.logger.rows}" if self.logger.active else ""
        self.conn_strip.setText(
            f"{mode}  ·  Port: {port}  ·  RX: {rx}/s  ·  CFG: {self._last_cfg_summary}  ·  Flash: {self._flash_state}{rec}"
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

    def _read_from_ecu(self):
        if not self.connected:
            return
        self._cfg_pending = True
        self._tx("GETCFG\n")
        self._tx("GETWHEEL\n")
        self._tx("GETMAP\n")
        self.status.showMessage("Reading maps & settings from ECU…")

    def reset_ecu_defaults(self):
        """Reset tuner + (if connected) ECU RAM to factory defaults. Requires confirm."""
        rpm = int(self.live.get("rpm") or 0)
        if rpm > 50:
            QMessageBox.warning(
                self, "Reset defaults",
                "Stop the engine (RPM must be near 0) before resetting defaults.")
            return
        r = QMessageBox.warning(
            self, "Reset ECU to defaults",
            "This will replace ALL local engine settings and maps with factory defaults.\n\n"
            "If connected, defaults are written to ECU RAM — press Flash at RPM 0 to store NVM.\n\n"
            "This cannot be undone (export a .tcal first if needed).\n\n"
            "Continue?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if r != QMessageBox.Yes:
            return
        # Fresh defaults
        defaults = default_engine_settings()
        self.engine.clear()
        self.engine.update(defaults)
        self.map_ign.set_table(suggested_adv_map())
        self.map_ign.mark_clean()
        self._apply_fuel_mode_heatmap(fill_suggested=True)
        self.map_inj.mark_clean()
        if hasattr(self, "map_afr"):
            self.map_afr.set_table(suggested_afr_map())
            self.map_afr.mark_clean()
        if hasattr(self, "map_idle_fuel"):
            self.map_idle_fuel.set_table(suggested_idle_fuel_map())
            self.map_idle_fuel.mark_clean()
        if hasattr(self, "map_idle_ign"):
            self.map_idle_ign.set_table(suggested_idle_ign_map())
            self.map_idle_ign.mark_clean()
        if hasattr(self, "map_boost"):
            self.map_boost.set_table(suggested_boost_map(8, 8, closed_loop=True))
            self.map_boost.mark_clean()
        self.btn_ve.blockSignals(True)
        self.btn_ve.setChecked(True)
        self.btn_ve.blockSignals(False)
        self.trig_slider.blockSignals(True)
        self.trig_slider.setValue(int(self.engine.get("trig_angle") or 30))
        self.trig_slider.blockSignals(False)
        self.trig_val.setText(f"{int(self.engine.get('trig_angle') or 30)}°")
        self._apply_load_bins()
        self._update_feature_tabs()
        self._refresh_3d()
        self._save_local_settings()
        if self.connected:
            self._push_engine_config(live_only=False)
            self._tx("SET:VEMODE,1\n")
            self._tx("SET:REQFUEL,%.2f,%.1f,%.2f,%.2f\n" % (
                float(self.engine.get("req_fuel_ms") or 2.5),
                float(self.engine.get("inj_flow_cc") or 220),
                float(self.engine.get("fuel_pressure_bar") or 3.0),
                float(self.engine.get("fuel_pressure_rated_bar") or 3.0),
            ))
            self._tx("SET:EOI,%d\n" % int(self.engine.get("eoi_btdc") or 340))
            self._tx("SET:AE,1,20,1.5,40,400\n")
            # Upload default maps cell-by-cell
            for r in range(ROWS):
                for c in range(COLS):
                    self._cell_tx(0, r, c, float(self.map_ign.table[r][c]))
                    self._cell_tx(1, r, c, float(self.map_inj.table[r][c]))
            self.status.showMessage(
                "Defaults loaded in RAM — press Flash at RPM 0 to store NVM", 10000)
        else:
            self.status.showMessage("Local defaults restored (offline)", 5000)

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
            # Mode change from Engine Settings → refresh fuel heatmap scale
            # (keep existing cell values; user can Fill rec. for suggested maps)
            prev_ve = getattr(self, "_last_applied_ve_mode", ve_on)
            if prev_ve != ve_on:
                self._apply_fuel_mode_heatmap(fill_suggested=True)
            else:
                self._apply_fuel_mode_heatmap(fill_suggested=False)
            self._last_applied_ve_mode = ve_on
            self.trig_slider.blockSignals(True)
            self.trig_slider.setValue(int(self.engine.get("trig_angle") or 30))
            self.trig_slider.blockSignals(False)
            self.trig_val.setText(f"{int(self.engine.get('trig_angle') or 30)}°")
            self._save_local_settings()
            if self.connected:
                self._push_engine_config(live_only=False)
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
                self._tx("SET:IDLEEN,%d\n" % (1 if self.engine.get("idle_enable", True) else 0))
                ect_b = self.engine.get("idle_ect_bins") or [-10, 20, 40, 60, 90]
                rpm_b = self.engine.get("idle_target_rpm_tbl") or [1400, 1100, 950, 850, 850]
                for i in range(min(5, len(ect_b), len(rpm_b))):
                    self._tx("SET:IDLETGT,%d,%.1f,%.0f\n" % (i, float(ect_b[i]), float(rpm_b[i])))
                if rpm_b:
                    self._tx("SET:IDLERPM,%.0f\n" % float(rpm_b[-1]))
                # Acceleration enrichment
                self._tx("SET:AE,%d,%.0f,%.1f,%.0f,%d\n" % (
                    1 if self.engine.get("ae_enable", True) else 0,
                    float(self.engine.get("ae_tps_dot_thresh") or 20.0),
                    float(self.engine.get("ae_gain") or 1.5),
                    float(self.engine.get("ae_max_pct") or 40.0),
                    int(self.engine.get("ae_decay_ms") or 400),
                ))
                self.map_ign.vmax = float(self.engine.get("max_advance") or 40)
                if not ve_on:
                    self.map_inj.vmax = float(self.engine.get("max_inj_ms") or 15.0)
                self.status.showMessage(
                    "Settings in RAM — press Flash at RPM 0 to store NVM"
                )
            else:
                self._push_engine_config()

    def _push_engine_config(self, live_only: bool = False):
        """Send crank/cam/sequential configuration to the ECU.

        live_only: skip wheel/CFG/cyl while the engine is spinning.
        Ignition and injection are independent (SET:IGNMODE / SET:INJMODE).
        """
        eng = self.engine
        cyl = int(eng.get("cylinders") or 4)
        cam_home = bool(eng.get("cam_home", True))
        coil = COIL_TYPE_TO_ECU.get(eng.get("coil_type") or "Smart", 0)
        ign_seq = str(eng.get("ign_mode") or "").startswith("Seq") and cam_home and coil != 2
        inj_seq = str(eng.get("inj_mode") or "").startswith("Seq")
        if cyl in (5, 6, 8):
            inj_seq = False
        if ign_seq or inj_seq:
            cam_home = True

        wheel = WHEEL_TO_ECU.get(int(eng.get("wheel_id") if eng.get("wheel_id") is not None else 9), 9)
        self._tx("SET:WHEEL,%d\n" % int(wheel))
        self._tx("CFG:%d,%d,%d\n" % (
            int(eng.get("teeth") or 60),
            int(eng.get("missing") or 2),
            int(eng.get("trig_angle") or 30),
        ))
        if not live_only:
            self._tx("SET:CYL,%d\n" % cyl)
        self._tx("SET:SENS:CAMMODE,%d\n" % (1 if cam_home else 0))
        self._tx("SET:COILTYPE,%d\n" % coil)
        if hasattr(self, "engine") and eng.get("coil_charge_mode"):
            cm = 0 if str(eng.get("coil_charge_mode")).startswith("Constant Duty") else 1
            if str(eng.get("coil_charge_mode")) in ("0", "Constant duty", "Duty"):
                cm = 0
            self._tx("SET:COILMODE,%d\n" % cm)
        self._tx("SET:IGNMODE,%d\n" % (1 if ign_seq else 0))
        self._tx("SET:INJMODE,%d\n" % (2 if inj_seq else 1))
        eoi = int(eng.get("eoi_btdc") or 340)
        if eoi < 10:
            eoi = 10
        if eoi > 540:
            eoi = 540
        self._tx("SET:EOI,%d\n" % eoi)
        self._tx("SET:BATCHRPM,%d\n" % int(eng.get("batch_above_rpm") or 3000))
        fan_en = 1 if eng.get("fan_enable", True) else 0
        self._tx("SET:SENS:FANEN,%d\n" % fan_en)
        if fan_en:
            self._tx("SET:FAN,%d\n" % int(eng.get("fan_c") or 95))
        self._tx("SET:SENS:TACHO,%d,%d\n" % (
            1 if eng.get("tacho_enable") else 0,
            int(eng.get("tacho_ppr") or 2),
        ))
        # MAP sensor linear scale: ADC0=min, ADC4095=max kPa
        map_min = int(eng.get("map_kpa_min") or 0)
        map_max = int(eng.get("map_kpa_max") or 240)
        if map_max < map_min + 20:
            map_max = map_min + 20
        self._tx("SET:MAPSCALE,%d,%d\n" % (map_min, map_max))
        self.engine["map_bins"] = make_map_bins(map_max, map_min)
        self._apply_load_bins()
        # RAM only — explicit Flash/Save writes NVM (SAVE here drops STM32 CDC)
        self._tx("GETCFG\n")

    def _apply_motorsport_tab(self):
        if hasattr(self, "_ms_form"):
            self._ms_form.apply_to(self.engine)
            self._save_local_settings()
            if self.connected:
                self._tx("SET:FLEX,%d,%d,%d,%.1f,%.1f\n" % (
                    1 if self.engine.get("flex_enable") else 0,
                    int(self.engine.get("flex_adc_e0") or 410),
                    int(self.engine.get("flex_adc_e100") or 3686),
                    float(self.engine.get("flex_fuel_pct_per10") or 4.7),
                    float(self.engine.get("flex_ign_deg_per10") or 0.8),
                ))
                self._tx("SET:VSS,%d,%d\n" % (
                    1 if self.engine.get("vss_enable") else 0,
                    int(self.engine.get("vss_pulses_per_km") or 8000),
                ))
                self._tx("SET:LCDECAY,%d\n" % (
                    1 if self.engine.get("launch_decay_enable") else 0,
                ))
                vss_b = self.engine.get("launch_vss_bins") or [0, 20, 40, 60, 80, 100, 130, 160]
                fuel_b = self.engine.get("launch_fuel_tbl") or [25, 20, 14, 8, 4, 2, 0, 0]
                ret_b = self.engine.get("launch_retard_tbl") or [15, 12, 8, 5, 2, 0, 0, 0]
                for i in range(min(8, len(vss_b))):
                    self._tx("SET:LCFUEL,%d,%.1f,%.1f\n" % (
                        i, float(vss_b[i]), float(fuel_b[i] if i < len(fuel_b) else 0)))
                    self._tx("SET:LCRET,%d,%.1f,%.1f\n" % (
                        i, float(vss_b[i]), float(ret_b[i] if i < len(ret_b) else 0)))
            self.status.showMessage("Motorsport / VSS / launch decay in RAM — Flash at RPM 0")

    def interpolate_maps(self):
        self.map_ign.smooth_selected(1)
        self.map_inj.smooth_selected(1)
        self.status.showMessage("Maps smoothed (selected / all)")

    def smooth_maps(self):
        self.interpolate_maps()

    def save_csv_log(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save RX dump", "strix_rx.csv", "CSV (*.csv)")
        if not path:
            return
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t", "line"])
            for t, line in self._log:
                w.writerow([f"{t:.3f}", line])
        self.status.showMessage(f"Saved {path}")

    def _toggle_record(self, on: bool):
        if on:
            default = str(Path.home() / ".strix_v2" / "logs" / time.strftime("strix_%Y%m%d_%H%M%S.csv"))
            path, _ = QFileDialog.getSaveFileName(self, "Record datalog", default, "CSV (*.csv)")
            if not path:
                self.btn_rec.setChecked(False)
                return
            self.logger.start(path)
            self.btn_rec.setText("Stop")
            self.status.showMessage(f"Recording {path}")
        else:
            p = self.logger.stop()
            self.btn_rec.setText("Log")
            if p:
                self.log_view.load_csv(p)
                self.tabs.setCurrentWidget(self.log_view)
                self.status.showMessage(f"Log saved {p} ({self.log_view.lbl.text()})")

    def _offline_toggled(self, on: bool):
        if on and self.connected:
            self.worker.disconnect()
        self._update_conn_strip()

    def open_warmup(self):
        dlg = WarmupWizardDialog(self.engine, lambda: self.live, parent=self)
        if dlg.exec():
            self._save_local_settings()
            if hasattr(self, "curves"):
                self.curves._reload()
            self._push_curves()
            self.status.showMessage("Warm-up / ASE written to RAM — Flash at RPM 0")

    def import_afr_inc(self):
        start = str(Path(__file__).resolve().parent / "lookups")
        path, _ = QFileDialog.getOpenFileName(self, "AFR lookup .inc", start, "INC (*.inc);;All (*)")
        if not path:
            return
        try:
            parsed = parse_inc(path)
        except Exception as e:
            QMessageBox.warning(self, "AFR .inc", str(e))
            return
        a0, a1, vmax = linear_wb_span(parsed)
        table = downsample_wb(parsed)
        sens = self.engine.setdefault("sensors", {}).setdefault("o2", {})
        sens["enabled"] = True
        sens["mode"] = "Wideband"
        sens["wb_table"] = table
        sens["inc_name"] = parsed.get("name")
        self.engine["o2_mode"] = "Wideband"
        if self.connected or True:
            self._tx("SET:O2MODE,2\n")
            self._tx("SET:WB,%.2f,%.2f,%.2f\n" % (a0, a1, min(5.0, vmax)))
        QMessageBox.information(
            self, "AFR .inc",
            f"{parsed.get('name')}  {parsed['n']} pts\n"
            f"AFR {a0:.1f} … {a1:.1f} over 0–{vmax:.1f} V\n"
            f"ECU uses linear SET:WB; full table stored in tcal."
        )
        self._save_local_settings()

    def _on_curve_changed(self, key: str):
        self._save_local_settings()
        self._push_curves(key)

    def _push_curves(self, only: str | None = None):
        curves = self.engine.get("curves") or {}
        if only in (None, "wue"):
            rec = curves.get("wue") or {}
            xs, ys = rec.get("xs") or [], rec.get("ys") or []
            for i, (x, y) in enumerate(zip(xs, ys)):
                self._tx("SET:WUE,%d,%.1f,%.1f\n" % (i, float(x), float(y)))
        if only in (None, "ase"):
            self._tx("SET:ASE,%.1f,%.1f,%.1f\n" % (
                float(self.engine.get("ase_initial_pct") or 35.0),
                float(self.engine.get("ase_decay_sec") or 8.0),
                float(self.engine.get("ase_min_ect") or 60.0),
            ))
        if only in (None, "iat"):
            rec = curves.get("iat") or {}
            for i, (x, y) in enumerate(zip(rec.get("xs") or [], rec.get("ys") or [])):
                self._tx("SET:IATCOMP,%d,%.1f,%.1f\n" % (i, float(x), float(y)))
        if only in (None, "bat"):
            rec = curves.get("bat") or {}
            for i, (x, y) in enumerate(zip(rec.get("xs") or [], rec.get("ys") or [])):
                self._tx("SET:BATCOMP,%d,%.1f,%.1f\n" % (i, float(x), float(y)))

    def _on_3d_cell(self, r: int, c: int, v: float):
        idx = self.combo_3d.currentIndex() if hasattr(self, "combo_3d") else 0
        if idx == 0:
            self.map_ign.table[r][c] = int(round(v))
            self._cell_tx(0, r, c, v)
        elif idx == 1:
            self.map_inj.table[r][c] = float(v)
            self._cell_tx(1, r, c, v)
        elif idx == 2 and hasattr(self, "map_boost"):
            self.map_boost.table[r][c] = float(v)
        elif idx == 3 and hasattr(self, "map_vvt_in"):
            self.map_vvt_in.table[r][c] = float(v)
        elif idx == 4 and hasattr(self, "map_vvt_ex"):
            self.map_vvt_ex.table[r][c] = float(v)
        self._refresh_3d()

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


    def _on_cyl_trim(self, cyl: int, pct: float):
        self._tx("SET:CYLTRIM,%d,%.1f\n" % (int(cyl), float(pct)))

    def _on_inj_disable(self, mask: int):
        self._tx("SET:INJDIS,%d\n" % int(mask))
        self.status.showMessage("Injector disable mask 0x%02X" % (mask & 0xFF), 3000)

    def _apply_startup(self):
        self.startup.apply_to(self.engine)
        self._save_local_settings()
        self._tx("SET:FPPRIME,%d\n" % int(self.engine.get("fp_prime_ms") or 2000))
        self._tx("SET:INJPRIMEEN,%d\n" % (1 if self.engine.get("start_prime_enable", True) else 0))
        self._tx("SET:INJPRIME,%d\n" % int(self.engine.get("start_prime_ms") or 50))
        self._tx("SET:CRANKADV,%d,%.1f,%d\n" % (
            1 if self.engine.get("crank_adv_enable", True) else 0,
            float(self.engine.get("crank_adv_deg") or 10.0),
            int(self.engine.get("crank_adv_rpm") or 400),
        ))
        self._tx("SET:FLOOD,%d,%.0f\n" % (
            1 if self.engine.get("flood_clear_enable", True) else 0,
            float(self.engine.get("flood_clear_tps") or 85.0),
        ))
        self.status.showMessage("Startup settings in RAM — Flash at RPM 0")

    def _goto_curve(self, key: str):
        if hasattr(self, "curves"):
            self.tabs.setCurrentWidget(self.curves)
            # select curve in combo if present
            try:
                for i in range(self.curves.combo.count()):
                    if self.curves.combo.itemData(i) == key:
                        self.curves.combo.setCurrentIndex(i)
                        break
            except Exception:
                pass

    def _idle_tx(self, which: str, r: int, c: int, v: float):
        if which == "FUEL":
            v = max(-20.0, min(40.0, float(v)))
            self.map_idle_fuel.table[r][c] = round(v, 1)
            self._tx("SET:IDLEFUEL,%d,%d,%.1f\n" % (r, c, v))
        else:
            v = max(-10.0, min(20.0, float(v)))
            self.map_idle_ign.table[r][c] = round(v, 1)
            self._tx("SET:IDLEIGN,%d,%d,%.1f\n" % (r, c, v))

    def _show_help_overlay(self):
        QMessageBox.information(
            self, "Keyboard",
            "Maps (Ignition / Injection / AFR / Idle):\n"
            "  Arrows — move cell   Shift+Arrows — extend selection\n"
            "  Home/End — first/last RPM col\n"
            "  + / PgUp  or  − / PgDn — nudge   Shift = ×5   [ ] = ×10\n"
            "  Ctrl+A select all · Ctrl+C/V copy/paste · Ctrl+P % change\n"
            "  Cyan = current cell · blue trail = last 50 cells\n"
            "Ctrl+S — flash · F1 — help\n"
            "3D: Shift+click cell · wheel nudge · double-click edit",
        )
