# ecu_tuner_aligned.py
import sys
import json
import time
import threading
from collections import deque
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QComboBox, QCheckBox, QLabel, QDialog,
    QGraphicsView, QGraphicsScene, QGraphicsRectItem, QGraphicsTextItem,
    QTabWidget, QTableWidget, QTableWidgetItem, QSlider, QSpinBox, QGroupBox,
    QProgressBar, QFrame, QDoubleSpinBox, QStackedWidget
)
from PySide6.QtCore import Qt, QTimer, Signal, QObject, QUrl
from PySide6.QtGui import QPainter, QColor, QFont, QPen, QBrush, QTransform
import serial
from pathlib import Path
import serial.tools.list_ports

# Optional Qt WebEngine (WebGL dashboard) — install: pip install PySide6-WebEngine
try:
    from PySide6.QtWebEngineWidgets import QWebEngineView
    from PySide6.QtWebEngineCore import QWebEngineSettings
    HAS_WEBENGINE = True
except Exception:
    QWebEngineView = None  # type: ignore
    QWebEngineSettings = None  # type: ignore
    HAS_WEBENGINE = False


# ──────────────────────────────────────────────
#  CONSTANTS
# ──────────────────────────────────────────────
ADV_MAX = 45
INJ_MAX = 20
ADV_MIN = -10
INJ_MIN = 0.0
RPM_MAX = 8000
ROWS    = 15
COLS    = 22
CAL_COLS = 15   # MAP/TPS/sensor cal rows match load axis
ETB_ROWS = 16   # throttle map RPM bands
ETB_COLS = 17   # throttle map pedal points
BREAKPOINT_FILE = "torqueefi_breakpoints.json"
UI_CONFIG_FILE  = "torqueefi_ui.json"
SCOPE_SECONDS   = 30
SCOPE_HZ        = 10
BAUD    = 115200

# (id, name, teeth, missing) — matches ecu_wheels.h
WHEEL_PROFILES = [
    (6,  "36-1", 36, 1),
    (3,  "60-2", 60, 2),
    (4,  "60-2 + cam", 60, 2),
    (5,  "60-2 + halfmoon", 60, 2),
    (7,  "24-1", 24, 1),
    (9,  "8-1", 8, 1),
    (12, "40-1 Ford V10", 40, 1),
    (16, "12-3", 12, 3),
    (8,  "4-1 + cam", 4, 1),
    (10, "6-1 + cam", 6, 1),
    (11, "12-1 + cam", 12, 1),
    (28, "36-1 + 2nd trig", 36, 1),
    (35, "24-2 + 2nd trig", 24, 2),
    (48, "Miata 99-05", 36, 1),
    (49, "12 even + cam", 12, 0),
    (50, "24 even + cam", 24, 0),
    (25, "GM 58x + 4x cam", 60, 2),
    (63, "BMW N20 58x", 60, 2),
    (65, "36-2 + 1 cam", 36, 2),
    (66, "GM 40 OSS", 40, 0),
    (0,  "Dizzy 4-cyl", 2, 0),
    (1,  "Dizzy 6-cyl", 3, 0),
    (2,  "Dizzy 8-cyl", 4, 0),
]


# Map edit steps: +/− = small, PageUp/PageDown = large
ADV_STEP_SMALL = 1
ADV_STEP_LARGE = 5
INJ_STEP_SMALL = 0.1
INJ_STEP_LARGE = 0.5

# Serial protocol limits / timeouts
SERIAL_READ_TIMEOUT   = 0.1     # seconds
SERIAL_WRITE_TIMEOUT  = 2.0
SERIAL_MAX_LINE_LEN   = 2048    # telemetry + map rows
SERIAL_ERROR_WINDOW   = 5.0     # seconds for error rate limiting
SERIAL_MAX_ERRORS     = 8       # consecutive errors before forced disconnect
SERIAL_RECONNECT_COOLDOWN = 2.0

# Crank-sync debounce / quality
SYNC_LOCK_FRAMES   = 4      # consecutive SYNC=1 frames to declare LOCKED
SYNC_UNLOCK_FRAMES = 12
CAM_LOCK_FRAMES    = 2
CAM_UNLOCK_FRAMES  = 15  # sticky cam indicator in UI      # consecutive SYNC=0 frames to declare LOST
SYNC_SEARCH_RPM    = 80    # RPM above which "searching" is meaningful

DARK_STYLE = """
    QMainWindow, QWidget, QDialog {
        background-color: #1a1e26; color: #d0d8e8;
    }
    QPushButton {
        background-color: #2a3a5a; color: #c8d8ff;
        border: 1px solid #3a5a8a; border-radius: 5px;
        padding: 6px 14px; font-size: 13px;
    }
    QPushButton:hover   { background-color: #3a5a8a; border: 1px solid #5a8acc; }
    QPushButton:pressed { background-color: #1a2a4a; }
    QPushButton:checked {
        background-color: #1a4a2a; color: #44ff88;
        border: 1px solid #44ff88;
    }
    QComboBox {
        background-color: #222a38; color: #c8d8ff;
        border: 1px solid #3a5a8a; border-radius: 4px; padding: 4px 8px;
    }
    QComboBox QAbstractItemView {
        background-color: #1e2736; selection-background-color: #3a5a8a; color: #c8d8ff;
    }
    QCheckBox { color: #a0b8d8; spacing: 6px; }
    QCheckBox::indicator {
        width: 16px; height: 16px;
        border: 1px solid #3a5a8a; border-radius: 3px; background: #222a38;
    }
    QCheckBox::indicator:checked { background: #3a7acc; border-color: #5a9aff; }
    QTabWidget::pane { border: 1px solid #3a5a8a; background-color: #1a1e26; }
    QTabBar::tab {
        background: #222a38; color: #8090b0;
        padding: 6px 18px; border: 1px solid #2a3a58; border-bottom: none;
    }
    QTabBar::tab:selected {
        background: #1a1e26; color: #a8d0ff; border-top: 2px solid #4a8acc;
    }
    QTableWidget {
        background-color: #1e2430; gridline-color: #2a3a58;
        color: #c8d8e8; border: 1px solid #2a3a58;
    }
    QHeaderView::section {
        background-color: #222a3a; color: #7090b8;
        border: none; padding: 4px; font-weight: bold;
    }
    QGroupBox {
        font-size: 14px; font-weight: bold; color: #a0c8ff;
        border: 1px solid #2a4a6a; border-radius: 8px;
        margin-top: 10px; padding: 10px 6px 6px 6px;
    }
    QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }
    QSlider::groove:horizontal {
        height: 6px; background: #2a3a58; border-radius: 3px;
    }
    QSlider::handle:horizontal {
        background: #4a8acc; border: 1px solid #6aaaf0;
        width: 14px; height: 14px; margin: -4px 0; border-radius: 7px;
    }
    QSlider::sub-page:horizontal { background: #2a6aaa; border-radius: 3px; }
    QSpinBox {
        background-color: #222a38; color: #c8d8ff;
        border: 1px solid #3a5a8a; border-radius: 4px; padding: 4px;
    }
    QLabel { color: #c0cce0; }
    QProgressBar {
        background: #1e2430; border: 1px solid #3a5a7a; border-radius: 4px;
    }
    QProgressBar::chunk {
        background: qlineargradient(x1:0,y1:0,x2:1,y2:0,
            stop:0 #cc7700, stop:1 #ffaa00);
        border-radius: 3px;
    }
"""

# ──────────────────────────────────────────────
#  UI HELPERS
# ──────────────────────────────────────────────
def vsep():
    s = QFrame(); s.setFrameShape(QFrame.VLine)
    s.setStyleSheet("color:#2a4a6a; max-width:2px;"); return s

def hsep():
    s = QFrame(); s.setFrameShape(QFrame.HLine)
    s.setStyleSheet("color:#2a4a6a;"); return s

def lbl(text, size=14, bold=False, color="#c0cce0"):
    w = QLabel(text)
    w.setFont(QFont("Segoe UI", size, QFont.Bold if bold else QFont.Normal))
    w.setStyleSheet(f"color:{color};")
    return w

def make_slider(lo, hi):
    s = QSlider(Qt.Horizontal); s.setRange(lo, hi); s.setValue(lo); return s

def make_pbar(lo, hi, height=12, rpm_style=False):
    chunk = (
        "background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
        "stop:0 #44cc44,stop:0.6 #ffcc00,stop:1.0 #ff3333);"
        if rpm_style else
        "background:qlineargradient(x1:0,y1:0,x2:1,y2:0,"
        "stop:0 #cc7700,stop:1 #ffaa00);"
    )
    pb = QProgressBar()
    pb.setRange(lo, hi); pb.setTextVisible(False); pb.setFixedHeight(height)
    pb.setStyleSheet(
        f"QProgressBar{{background:#1a2030;border:none;border-radius:4px;}}"
        f"QProgressBar::chunk{{{chunk}border-radius:4px;}}"
    )
    return pb

def status_dot(color="#444444"):
    w = QLabel("●"); w.setStyleSheet(f"color:{color}; font-size:16px;"); return w

def hmap_color(value, max_val):
    t = max(0.0, min(1.0, value / max_val)) if max_val else 0.0
    if   t < 0.25: r, g, b = 0,               int(t*4*160),        210
    elif t < 0.50: r, g, b = 0,               160,                 int((0.5-t)*4*210)
    elif t < 0.75: r, g, b = int((t-0.5)*4*210), 160,              0
    else:          r, g, b = 210,             int((1.0-t)*4*160),  0
    return QColor(r, g, b, 210)

# ──────────────────────────────────────────────
#  SERIAL WORKER  (with protocol error handling)
# ──────────────────────────────────────────────
class SerialWorker(QObject):
    """Background serial I/O with robust error reporting."""

    data_received = Signal(str)          # well-formed telemetry line
    error_occurred = Signal(str)         # human-readable error message
    connection_lost = Signal(str)        # port gone / fatal I/O
    status_changed = Signal(str)         # informational status

    def __init__(self):
        super().__init__()
        self.ser = None
        self.running = True
        self._lock = threading.Lock()
        self._port_name = ""
        self._consec_errors = 0
        self._last_error_ts = 0.0
        self._error_log = deque(maxlen=20)

    # ── connection ───────────────────────────
    @staticmethod
    def _normalize_port(port: str) -> str:
        """Strip UI labels; fix Windows COM10+ path."""
        if not port:
            return ""
        port = port.strip().split()[0].strip()
        import re
        m = re.match(r"^(COM)(\d+)$", port, re.I)
        if m and int(m.group(2)) >= 10:
            port = "\\\\.\\" + m.group(1).upper() + m.group(2)
        return port

    def connect(self, port: str) -> tuple[bool, str]:
        """Open port. Returns (ok, message)."""
        with self._lock:
            self._safe_close()
            port = self._normalize_port(port or "")
            self._port_name = port
            if not port or port.startswith("("):
                return False, "No serial port selected – click ↻ and pick STM32 CDC"
            try:
                self.ser = serial.Serial(
                    port=port,
                    baudrate=BAUD,
                    timeout=SERIAL_READ_TIMEOUT,
                    write_timeout=SERIAL_WRITE_TIMEOUT,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    rtscts=False,
                    dsrdtr=False,
                )
                try:
                    self.ser.dtr = True
                    self.ser.rts = False
                except Exception:
                    pass
                time.sleep(0.25)
                try:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()
                except Exception:
                    pass
                if not self.ser.is_open:
                    self.ser = None
                    return False, f"Port {port} opened then closed immediately"
                self._consec_errors = 0
                self._ready_ts = time.monotonic() + 0.6  # CDC settle
                self.status_changed.emit(f"Opened {port} @ {BAUD}")
                return True, f"Connected to {port}"
            except serial.SerialException as e:
                self.ser = None
                msg = self._format_serial_error(e, port)
                self._log_error(msg)
                return False, msg
            except OSError as e:
                self.ser = None
                msg = f"OS error opening {port}: {e}"
                self._log_error(msg)
                return False, msg
            except Exception as e:
                self.ser = None
                msg = f"Unexpected error opening {port}: {e}"
                self._log_error(msg)
                return False, msg

    def disconnect(self):
        with self._lock:
            self._safe_close()
        self.status_changed.emit("Offline")

    def _safe_close(self):
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None

    def is_open(self) -> bool:
        with self._lock:
            return bool(self.ser and self.ser.is_open)

    # ── transmit ─────────────────────────────
    def send(self, data: str) -> bool:
        """Write a string (caller supplies newline). Returns success."""
        if not data:
            return False
        payload = data if isinstance(data, (bytes, bytearray)) else data.encode("ascii", errors="replace")

        # Wait until CDC settle time after open
        ready = getattr(self, "_ready_ts", 0)
        while time.monotonic() < ready:
            time.sleep(0.05)

        with self._lock:
            if not self.ser or not self.ser.is_open:
                if self._port_name:
                    try:
                        self.ser = serial.Serial(
                            port=self._port_name,
                            baudrate=BAUD,
                            timeout=SERIAL_READ_TIMEOUT,
                            write_timeout=SERIAL_WRITE_TIMEOUT,
                            rtscts=False,
                            dsrdtr=False,
                        )
                        try:
                            self.ser.dtr = True
                            self.ser.rts = False
                        except Exception:
                            pass
                        self._ready_ts = time.monotonic() + 0.6
                        time.sleep(0.3)
                    except Exception as e:
                        self.ser = None
                        self.error_occurred.emit(f"TX reopen failed: {e}")
                        return False
                if not self.ser or not self.ser.is_open:
                    self.error_occurred.emit(
                        "TX failed: not connected – select STM32 CDC + Connect")
                    return False
            try:
                n = self.ser.write(payload)
                try:
                    self.ser.flush()
                except Exception:
                    pass
                if n is not None and n != len(payload):
                    self.error_occurred.emit(f"TX short write {n}/{len(payload)}")
                    return False
                return True
            except serial.SerialTimeoutException:
                self.error_occurred.emit(
                    "TX timeout on write – board not accepting USB OUT "
                    "(check CDC_Receive_FS / USB 48MHz / cable)")
                return False
            except serial.SerialException as e:
                self.error_occurred.emit(f"TX serial error: {e}")
                self.ser = None
                return False
            except OSError as e:
                self.error_occurred.emit(f"TX OS error: {e}")
                self.ser = None
                return False
            except Exception as e:
                self.error_occurred.emit(f"TX unexpected: {e}")
                return False

    # ── receive loop ─────────────────────────
    def run(self):
        buf = bytearray()
        while self.running:
            try:
                line = self._read_line(buf)
                if line is not None:
                    self.data_received.emit(line)
                    self._consec_errors = 0
            except _FatalSerialError as e:
                self.connection_lost.emit(str(e))
                with self._lock:
                    self._safe_close()
            except Exception:
                # Non-fatal: keep looping
                pass
            time.sleep(0.01)

    def _read_line(self, buf: bytearray):
        """Read until newline (\n or \r). Returns decoded line or None."""
        with self._lock:
            ser = self.ser
            if not ser or not ser.is_open:
                return None
            try:
                waiting = ser.in_waiting
            except (serial.SerialException, OSError) as e:
                raise _FatalSerialError(f"Port lost while reading: {e}") from e

            if waiting > 0:
                try:
                    # Cap per-read to avoid huge spikes
                    chunk = ser.read(min(waiting, 4096))
                except serial.SerialException as e:
                    raise _FatalSerialError(f"Read error: {e}") from e
                except OSError as e:
                    raise _FatalSerialError(f"Read OS error: {e}") from e
                if chunk:
                    buf.extend(chunk)

            # Soft cap – keep tail only, no scary spam every time
            if len(buf) > SERIAL_MAX_LINE_LEN * 16:
                del buf[:-512]

            # Prefer \n; also treat \r as line end (CDC / terminal)
            nl = buf.find(b"\n")
            cr = buf.find(b"\r")
            cut = -1
            if nl >= 0 and cr >= 0:
                cut = min(nl, cr)
            elif nl >= 0:
                cut = nl
            elif cr >= 0:
                cut = cr

            if cut < 0:
                if len(buf) > SERIAL_MAX_LINE_LEN:
                    # Drop orphan fragment (binary / missing newline)
                    del buf[:len(buf) - 64]
                return None

            raw = bytes(buf[:cut])
            # Consume terminator(s) \r, \n, or \r\n
            drop = cut + 1
            if drop < len(buf) and buf[cut] == 13 and buf[drop] == 10:
                drop += 1
            elif drop < len(buf) and buf[cut] == 10 and buf[drop] == 13:
                drop += 1
            del buf[:drop]

            if not raw:
                return None
            try:
                return raw.decode("ascii", errors="replace").strip("\0")
            except Exception:
                return None


    def _bump_error(self, fatal: bool = False):
        now = time.monotonic()
        if now - self._last_error_ts > SERIAL_ERROR_WINDOW:
            self._consec_errors = 0
        self._consec_errors += 1
        self._last_error_ts = now
        if fatal or self._consec_errors >= SERIAL_MAX_ERRORS:
            with self._lock:
                self._safe_close()
            self.connection_lost.emit(
                f"Too many serial errors ({self._consec_errors}) – disconnected")

    def _log_error(self, msg: str):
        self._error_log.append((time.time(), msg))
        self.error_occurred.emit(msg)

    @staticmethod
    def _format_serial_error(exc: Exception, port: str) -> str:
        text = str(exc).lower()
        if "permission" in text or "access" in text:
            return f"Permission denied on {port} – check user groups / close other apps"
        if "no such file" in text or "cannot find" in text or "not found" in text:
            return f"Port {port} not found – device unplugged?"
        if "busy" in text or "in use" in text:
            return f"Port {port} is busy – close other serial tools"
        if "timeout" in text:
            return f"Timeout opening {port}"
        return f"Cannot open {port}: {exc}"


class _FatalSerialError(Exception):
    """Raised inside worker when the port is gone and must be closed."""


# ──────────────────────────────────────────────
#  MAP GRAPHICS VIEW
# ──────────────────────────────────────────────


class SparklineWidget(QWidget):
    """Tiny 30 s trend strip for RPM or ECT."""
    def __init__(self, color="#ffcc00", parent=None):
        super().__init__(parent)
        self._color = color
        self._data = []
        self.setFixedSize(72, 22)
        self.setToolTip("30 s trend")

    def set_data(self, values):
        self._data = list(values)[-120:]
        self.update()

    def paintEvent(self, _e):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(20, 26, 34))
        if len(self._data) < 2:
            p.setPen(QColor(60, 70, 80))
            p.drawText(self.rect(), Qt.AlignCenter, "—")
            return
        mn = min(self._data)
        mx = max(self._data)
        if mx <= mn:
            mx = mn + 1.0
        w, h = self.width(), self.height()
        pts = []
        n = len(self._data)
        for i, v in enumerate(self._data):
            x = i * (w - 2) / max(1, n - 1) + 1
            y = h - 2 - (v - mn) / (mx - mn) * (h - 4)
            pts.append((x, y))
        pen = QPen(QColor(self._color))
        pen.setWidth(1)
        p.setPen(pen)
        for i in range(1, len(pts)):
            p.drawLine(int(pts[i-1][0]), int(pts[i-1][1]),
                       int(pts[i][0]), int(pts[i][1]))


class ScopeWidget(QWidget):
    """Simple scrolling plot of RPM, MAP, SYNC over ~30 s."""
    def __init__(self, parent):
        super().__init__(parent)
        self.p = parent
        self.setMinimumHeight(220)
        self.setStyleSheet("background:#10141c;")

    def paintEvent(self, _ev):
        from PySide6.QtGui import QPainterPath
        p = QPainter(self)
        p.fillRect(self.rect(), QColor(16, 20, 28))
        buf = list(getattr(self.p, "_scope_buf", []))
        if len(buf) < 2:
            p.setPen(QColor(100, 120, 140))
            p.drawText(self.rect(), Qt.AlignCenter, "Waiting for live data…")
            return
        w, h = self.width(), self.height()
        margin = 36
        n = len(buf)
        # grid
        p.setPen(QPen(QColor(40, 50, 70), 1))
        for i in range(5):
            y = margin + i * (h - 2 * margin) / 4
            p.drawLine(margin, int(y), w - 10, int(y))
        # RPM (yellow)
        def x_at(i):
            return margin + i * (w - margin - 10) / max(1, n - 1)
        rpms = [b[0] for b in buf]
        maps = [b[1] for b in buf]
        syncs = [b[2] for b in buf]
        rmax = max(rpms) if max(rpms) > 100 else 8000
        mmax = max(maps) if max(maps) > 10 else 100
        path_r = QPainterPath()
        path_m = QPainterPath()
        for i, (r, m, s) in enumerate(buf):
            x = x_at(i)
            yr = h - margin - (r / rmax) * (h - 2 * margin)
            ym = h - margin - (m / mmax) * (h - 2 * margin)
            if i == 0:
                path_r.moveTo(x, yr); path_m.moveTo(x, ym)
            else:
                path_r.lineTo(x, yr); path_m.lineTo(x, ym)
            if s:
                p.fillRect(int(x) - 1, margin, 2, h - 2 * margin, QColor(40, 120, 60, 40))
        p.setPen(QPen(QColor(255, 200, 0), 2))
        p.drawPath(path_r)
        p.setPen(QPen(QColor(80, 180, 255), 2))
        p.drawPath(path_m)
        p.setPen(QColor(180, 190, 200))
        p.drawText(8, 16, f"RPM max {rmax:.0f}")
        p.drawText(8, 32, f"MAP max {mmax:.0f}")
        p.drawText(8, h - 8, "Green band = SYNC locked")


class HeatMapView(QGraphicsView):
    """Generic RPM×Load heat map for ETB / VVT (not the main ign/inj tables)."""
    GX, GY = 48, 36

    def __init__(self, parent, rows, cols, get_table, set_cell, row_labels, col_labels,
                 vmax=100, title="", cell_min=22):
        super().__init__(parent)
        self.p = parent
        self.rows = rows
        self.cols = cols
        self.get_table = get_table
        self.set_cell = set_cell
        self.row_labels = row_labels
        self.col_labels = col_labels
        self.vmax = vmax
        self.title = title
        self.cell_min = cell_min
        self.CELL = 36
        self.sel_r = 0
        self.sel_c = 0
        self._scene = QGraphicsScene()
        self.setScene(self._scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setStyleSheet("background-color:#12151c; border:1px solid #2a3a58;")
        self.build_grid()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        vw = max(120, self.viewport().width() - 8)
        vh = max(100, self.viewport().height() - 8)
        cw = max(self.cell_min, int((vw - self.GX - 8) / max(1, self.cols)))
        ch = max(16, int((vh - self.GY - 8) / max(1, self.rows)))
        cell = max(self.cell_min, min(56, min(cw, max(ch, int(cw * 0.8)))))
        if abs(cell - self.CELL) >= 2:
            self.CELL = cell
            self.build_grid()

    def build_grid(self):
        GX, GY, C = self.GX, self.GY, self.CELL
        sc = self._scene
        sc.clear()
        tbl = self.get_table()
        t = QGraphicsTextItem(self.title)
        t.setDefaultTextColor(QColor(140, 180, 220))
        t.setPos(GX, 4)
        sc.addItem(t)
        pen_def = QPen(QColor(25, 25, 25))
        pen_sel = QPen(QColor(255, 220, 40), 2)
        for r in range(self.rows):
            rl = QGraphicsTextItem(str(self.row_labels[r]))
            rl.setDefaultTextColor(QColor(120, 160, 210))
            rl.setPos(4, GY + r * C + 4)
            sc.addItem(rl)
            for c in range(self.cols):
                try:
                    val = float(tbl[r][c])
                except Exception:
                    val = 0.0
                x, y = GX + c * C, GY + r * C
                rect = QGraphicsRectItem(x, y, C, C)
                rect.setBrush(QBrush(hmap_color(val, self.vmax)))
                if r == self.sel_r and c == self.sel_c:
                    rect.setPen(pen_sel)
                else:
                    rect.setPen(pen_def)
                rect.setData(0, (r, c))
                sc.addItem(rect)
                txt = QGraphicsTextItem(f"{val:g}" if val != int(val) else f"{int(val)}")
                txt.setDefaultTextColor(QColor(255, 255, 255))
                txt.setPos(x + 2, y + max(0, C // 2 - 10))
                sc.addItem(txt)
        for c in range(self.cols):
            cl = QGraphicsTextItem(str(self.col_labels[c]))
            cl.setDefaultTextColor(QColor(120, 160, 210))
            cl.setPos(GX + c * C + 2, GY - 18)
            sc.addItem(cl)
        sc.setSceneRect(0, 0, GX + self.cols * C + 20, GY + self.rows * C + 20)

    def _rc_at(self, qpt):
        item = self._scene.itemAt(self.mapToScene(qpt.toPoint()), QTransform())
        return item.data(0) if isinstance(item, QGraphicsRectItem) else None

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.setFocus(Qt.MouseFocusReason)
            rc = self._rc_at(e.position())
            if rc:
                self.sel_r, self.sel_c = rc
                self.build_grid()
                e.accept()
                return
        super().mousePressEvent(e)

    def wheelEvent(self, e):
        delta = e.angleDelta().y()
        if delta == 0:
            return
        step = 1 if delta > 0 else -1
        if e.modifiers() & Qt.ShiftModifier:
            step *= 5
        tbl = self.get_table()
        r, c = self.sel_r, self.sel_c
        try:
            val = float(tbl[r][c]) + step
        except Exception:
            return
        val = max(0, min(self.vmax, val))
        self.set_cell(r, c, val)
        self.build_grid()
        e.accept()

    def keyPressEvent(self, e):
        k = e.key()
        if k == Qt.Key_Up:
            self.sel_r = max(0, self.sel_r - 1); self.build_grid()
        elif k == Qt.Key_Down:
            self.sel_r = min(self.rows - 1, self.sel_r + 1); self.build_grid()
        elif k == Qt.Key_Left:
            self.sel_c = max(0, self.sel_c - 1); self.build_grid()
        elif k == Qt.Key_Right:
            self.sel_c = min(self.cols - 1, self.sel_c + 1); self.build_grid()
        elif k in (Qt.Key_Plus, Qt.Key_Equal):
            self.wheelEvent(type("E", (), {"angleDelta": lambda: type("D", (), {"y": lambda: 120})(), "modifiers": lambda: Qt.NoModifier})())
        elif k == Qt.Key_Minus:
            tbl = self.get_table()
            r, c = self.sel_r, self.sel_c
            val = max(0, float(tbl[r][c]) - 1)
            self.set_cell(r, c, val); self.build_grid()
        else:
            super().keyPressEvent(e)



# ──────────────────────────────────────────────
#  SETUP WIZARD (3 pages)
# ──────────────────────────────────────────────
class SetupWizard(QDialog):
    """Guided engine setup — trigger → load → I/O; saves JSON + pushes ECU."""

    def __init__(self, parent):
        super().__init__(parent)
        self.p = parent
        self.setWindowTitle("Setup Wizard")
        self.setMinimumSize(560, 520)
        self.resize(600, 560)
        root = QVBoxLayout(self)

        self.stack = QStackedWidget()
        root.addWidget(self.stack, 1)

        self._build_page_trigger()
        self._build_page_load()
        self._build_page_io()

        nav = QHBoxLayout()
        self.btn_back = QPushButton("← Back")
        self.btn_next = QPushButton("Next →")
        self.btn_finish = QPushButton("Finish & Apply")
        self.btn_back.clicked.connect(self._back)
        self.btn_next.clicked.connect(self._next)
        self.btn_finish.clicked.connect(self._finish)
        nav.addWidget(self.btn_back)
        nav.addStretch()
        nav.addWidget(self.btn_next)
        nav.addWidget(self.btn_finish)
        root.addLayout(nav)
        self._sync_nav()

    def _sync_nav(self):
        i = self.stack.currentIndex()
        self.btn_back.setEnabled(i > 0)
        self.btn_next.setVisible(i < self.stack.count() - 1)
        self.btn_finish.setVisible(i == self.stack.count() - 1)

    def _back(self):
        self.stack.setCurrentIndex(max(0, self.stack.currentIndex() - 1))
        self._sync_nav()

    def _next(self):
        self.stack.setCurrentIndex(min(self.stack.count() - 1, self.stack.currentIndex() + 1))
        self._sync_nav()

    def _build_page_trigger(self):
        w = QWidget(); lay = QVBoxLayout(w)
        lay.addWidget(lbl("1 — Trigger & ignition", 14, True))
        lay.addWidget(lbl("Crank / cam wheel", 12, True))
        self.w_wheel = QComboBox()
        for wid, wname, wt, wm in WHEEL_PROFILES:
            self.w_wheel.addItem(f"{wname}", wid)
        if hasattr(self.p, "wheel_combo") and self.p.wheel_combo.count():
            self.w_wheel.setCurrentIndex(self.p.wheel_combo.currentIndex())
        lay.addWidget(self.w_wheel)

        lay.addWidget(lbl("Cylinders (1–4)", 12, True))
        self.w_cyl = QSpinBox(); self.w_cyl.setRange(1, 4)
        self.w_cyl.setValue(min(4, int(getattr(self.p, "cylinders", 4))))
        lay.addWidget(self.w_cyl)

        lay.addWidget(lbl("Ignition coil type", 12, True))
        self.w_coil = QComboBox()
        self.w_coil.addItem("Smart coil (logic-level)", 1)
        self.w_coil.addItem("Dumb coil (dwell)", 0)
        self.w_coil.addItem("Distributor (dumb)", 2)
        idx = 0 if getattr(self.p, "coil_smart", True) else 1
        self.w_coil.setCurrentIndex(idx)
        lay.addWidget(self.w_coil)

        lay.addWidget(lbl("Firing order", 12, True))
        self.w_fire = QComboBox()
        for name, val in (("1-3-4-2", 0), ("1-2-4-3", 1), ("1-3-2-4", 2), ("1-2-3-4", 3)):
            self.w_fire.addItem(name, val)
        self.w_fire.setCurrentIndex(int(getattr(self.p, "fire_order", 0)))
        lay.addWidget(self.w_fire)

        lay.addWidget(lbl("Injection mode", 12, True))
        self.w_inj = QComboBox()
        for name, val in (
            ("AUTO (seq if CAM)", 0),
            ("BATCH / semi-seq", 1),
            ("SEQUENTIAL (needs CAM)", 2),
            ("HYBRID: seq → batch @ RPM", 3),
        ):
            self.w_inj.addItem(name, val)
        lay.addWidget(self.w_inj)

        lay.addWidget(lbl("RPM limiter", 12, True))
        hr = QHBoxLayout()
        self.w_rpmlim = QSpinBox(); self.w_rpmlim.setRange(2000, 12000)
        self.w_rpmlim.setValue(int(getattr(self.p, "rpm_limit", 7000)))
        self.w_rpmlim.setSuffix(" RPM")
        self.w_cut = QComboBox()
        self.w_cut.addItem("Hard cut", 0)
        self.w_cut.addItem("Soft cut", 1)
        hr.addWidget(self.w_rpmlim); hr.addWidget(self.w_cut)
        lay.addLayout(hr)
        lay.addStretch()
        self.stack.addWidget(w)

    def _build_page_load(self):
        w = QWidget(); lay = QVBoxLayout(w)
        lay.addWidget(lbl("2 — Load sensing & throttle", 14, True))

        lay.addWidget(lbl("Throttle type", 12, True))
        self.w_thr = QComboBox()
        self.w_thr.addItem("Cable throttle", 0)
        self.w_thr.addItem("Drive-by-wire (electronic)", 1)
        self.w_thr.setCurrentIndex(1 if getattr(self.p, "dbw_enable", True) else 0)
        lay.addWidget(self.w_thr)

        def on_thr(_=None):
            dbw = self.w_thr.currentData() == 1
            self.btn_ped.setEnabled(dbw)
            self.w_idle.setEnabled(not dbw)
            if dbw:
                from PySide6.QtWidgets import QMessageBox
                QMessageBox.warning(
                    self, "Drive-by-wire warning",
                    "WARNING: Incorrect DBW calibration can cause a runaway engine.\n\n"
                    "Calibrate pedal and throttle fully before starting the engine.\n"
                    "Use at your own risk.")
        self.w_thr.currentIndexChanged.connect(on_thr)

        self.btn_ped = QPushButton("Calibrate Pedal Position Wizard…")
        self.btn_ped.setEnabled(self.w_thr.currentData() == 1)
        self.btn_ped.clicked.connect(lambda: self.p.open_cal_window())
        lay.addWidget(self.btn_ped)

        self.btn_tps = QPushButton("Calibrate Throttle Position (TPS) Wizard…")
        self.btn_tps.clicked.connect(lambda: self.p.open_cal_window() if hasattr(self.p, "open_cal_window") else None)
        lay.addWidget(self.btn_tps)

        lay.addWidget(lbl("Load type", 12, True))
        self.w_load = QComboBox()
        self.w_load.addItem("Manifold pressure (MAP / speed-density)", 0)
        self.w_load.addItem("Throttle position (TPS / Alpha-N)", 1)
        self.w_load.setCurrentIndex(1 if getattr(self.p, "use_tps", False) else 0)
        lay.addWidget(self.w_load)

        self.btn_map = QPushButton("Calibrate MAP Sensor Wizard…")
        self.btn_map.clicked.connect(lambda: self.p.open_map_wizard() if hasattr(self.p, "open_map_wizard") else None)
        lay.addWidget(self.btn_map)

        lay.addWidget(lbl("Idle control (cable throttle only)", 12, True))
        self.w_idle = QComboBox()
        self.w_idle.addItem("Disabled", -1)
        self.w_idle.addItem("Single wire PWM", 1)
        self.w_idle.addItem("Dual wire (H-bridge)", 0)
        self.w_idle.setCurrentIndex(0)
        self.w_idle.setEnabled(self.w_thr.currentData() == 0)
        lay.addWidget(self.w_idle)
        lay.addStretch()
        self.stack.addWidget(w)

    def _build_page_io(self):
        w = QWidget(); lay = QVBoxLayout(w)
        lay.addWidget(lbl("3 — Inputs / outputs", 14, True))

        self.chk_iat = QCheckBox("Intake Air Temperature (IAT) sensor enabled")
        self.chk_iat.setChecked(True)
        lay.addWidget(self.chk_iat)
        lay.addWidget(lbl("If unticked: IAT compensations forced to 0, live IAT hidden.", 10, color="#8090b0"))

        self.chk_ect = QCheckBox("Engine Coolant Temperature (ECT) sensor enabled")
        self.chk_ect.setChecked(True)
        lay.addWidget(self.chk_ect)
        lay.addWidget(lbl("If unticked: ECT compensations forced to 0, live ECT hidden.", 10, color="#8090b0"))

        lay.addWidget(lbl("Oxygen sensor", 12, True))
        self.w_o2 = QComboBox()
        self.w_o2.addItem("Disabled", 0)
        self.w_o2.addItem("Narrowband", 1)
        self.w_o2.addItem("Wideband", 2)
        self.w_o2.setCurrentIndex(1)
        lay.addWidget(self.w_o2)
        lay.addWidget(lbl("Disabled → O2 closed-loop and fuel trims off.", 10, color="#8090b0"))

        lay.addWidget(lbl("Boost control", 12, True))
        self.w_boost = QComboBox()
        self.w_boost.addItem("OFF", 0)
        self.w_boost.addItem("Single value target", 1)
        self.w_boost.addItem("Closed-loop map", 2)
        self.w_boost.addItem("Open-loop duty map", 3)
        lay.addWidget(self.w_boost)

        self.chk_fan = QCheckBox("Radiator fan control")
        self.chk_fan.setChecked(True)
        lay.addWidget(self.chk_fan)
        fr = QHBoxLayout()
        self.w_fan = QSpinBox(); self.w_fan.setRange(60, 130)
        self.w_fan.setValue(int(getattr(self.p, "fan_setpoint", 95)))
        self.w_fan.setSuffix(" °C")
        self.w_fan.setEnabled(True)
        self.chk_fan.toggled.connect(self.w_fan.setEnabled)
        fr.addWidget(lbl("Fan on temperature", 11)); fr.addWidget(self.w_fan)
        lay.addLayout(fr)

        lay.addWidget(lbl("VVT", 12, True))
        self.w_vvt = QComboBox()
        self.w_vvt.addItem("Disabled", 0)
        self.w_vvt.addItem("Intake only", 1)
        self.w_vvt.addItem("Exhaust only", 2)
        self.w_vvt.addItem("Intake & Exhaust", 3)
        lay.addWidget(self.w_vvt)
        lay.addWidget(lbl("When enabled, use main VVT Maps tab for closed-loop targets.", 10, color="#8090b0"))
        lay.addStretch()
        self.stack.addWidget(w)

    def _collect(self):
        return {
            "wheel_id": self.w_wheel.currentData(),
            "cylinders": int(self.w_cyl.value()),
            "coil_type": int(self.w_coil.currentData()),  # 1 smart 0 dumb 2 dist
            "fire_order": int(self.w_fire.currentData()),
            "inj_mode": int(self.w_inj.currentData()),
            "rpm_limit": int(self.w_rpmlim.value()),
            "rpm_cut_mode": int(self.w_cut.currentData()),
            "dbw": bool(self.w_thr.currentData() == 1),
            "use_tps": bool(self.w_load.currentData() == 1),
            "idle_out": int(self.w_idle.currentData()),
            "iat_en": self.chk_iat.isChecked(),
            "ect_en": self.chk_ect.isChecked(),
            "o2_mode": int(self.w_o2.currentData()),
            "boost_mode": int(self.w_boost.currentData()),
            "fan_en": self.chk_fan.isChecked(),
            "fan_c": int(self.w_fan.value()),
            "vvt_mode": int(self.w_vvt.currentData()),
        }

    def _finish(self):
        cfg = self._collect()
        p = self.p
        # Apply to parent state
        p.cylinders = cfg["cylinders"]
        p.fire_order = cfg["fire_order"]
        p.coil_smart = cfg["coil_type"] == 1
        p.dbw_enable = cfg["dbw"]
        p.use_tps = cfg["use_tps"]
        p.rpm_limit = cfg["rpm_limit"]
        p.fan_setpoint = cfg["fan_c"]
        p.idle_out_mode = max(0, cfg["idle_out"]) if cfg["idle_out"] >= 0 else 0
        p.wizard_cfg = cfg

        if cfg["idle_out"] < 0 and not cfg["dbw"]:
            p.idle_out_mode = 0  # disabled handled via flag
        p.sensor_iat_en = cfg["iat_en"]
        p.sensor_ect_en = cfg["ect_en"]
        p.o2_mode = cfg["o2_mode"]
        p.boost_mode_wiz = cfg["boost_mode"]
        p.vvt_mode = cfg["vvt_mode"]
        p.fan_enable = cfg["fan_en"]

        # Zero compensations if sensors off
        if not cfg["iat_en"]:
            for i in range(len(p.iatC)):
                p.iatC[i] = 0.0
        if not cfg["ect_en"]:
            for i in range(len(p.tempC)):
                p.tempC[i] = 0.0

        # Save JSON
        try:
            from pathlib import Path
            import json
            path = Path(__file__).resolve().parent / "torqueefi_setup_wizard.json"
            path.write_text(json.dumps(cfg, indent=2), encoding="utf-8")
        except Exception as e:
            p.status_label.setText(f"Wizard save file failed: {e}")

        # Push ECU
        if p.connected:
            # Wheel
            p.wheel_combo = self.w_wheel
            try:
                p._apply_wheel_profile()
            except Exception:
                pass
            p._tx(f"SET:CYL,{cfg['cylinders']}\n")
            p._tx(f"SET:FIRE,{cfg['fire_order']}\n")
            p._tx(f"SET:COIL,{1 if cfg['coil_type']==1 else 0}\n")
            p._tx(f"SET:INJMODE,{cfg['inj_mode']}\n")
            p._tx(f"SET:Q,0,0,{cfg['rpm_limit']}\n")
            p._tx(f"SET:N,0,0,{cfg['fan_c'] if cfg['fan_en'] else 200}\n")  # high = effectively off
            p._tx(f"SET:L,0,0,{1 if cfg['use_tps'] else 0}\n")
            p._tx(f"SET:DBW,{1 if cfg['dbw'] else 0}\n")
            if cfg["idle_out"] >= 0 and not cfg["dbw"]:
                p._tx(f"SET:IDLEOUT,{cfg['idle_out']}\n")
            p._tx(f"SET:O2CL,{1 if cfg['o2_mode']==1 else 0}\n")
            if cfg["boost_mode"] == 0:
                p._tx("SET:BSTEN,0\n")
                p._tx("SET:BOOST,0\n")
            elif cfg["boost_mode"] == 1:
                p._tx("SET:BSTEN,0\n")
            elif cfg["boost_mode"] == 2:
                p._tx("SET:BSTMODE,0\n")
                p._tx("SET:BSTEN,1\n")
            elif cfg["boost_mode"] == 3:
                p._tx("SET:BSTMODE,1\n")
                p._tx("SET:BSTEN,1\n")
            p._tx(f"SET:VVTCL,{1 if cfg['vvt_mode'] else 0}\n")
            # Soft/hard cut if protocol supports
            p._tx(f"SET:RPMCUT,{cfg['rpm_cut_mode']}\n")

        # UI toggles for live labels
        if hasattr(p, "ect_label"):
            p.ect_label.setVisible(cfg["ect_en"])
        if hasattr(p, "iat_label"):
            p.iat_label.setVisible(cfg["iat_en"])
        if hasattr(p, "chk_tps"):
            p.chk_tps.setChecked(cfg["use_tps"])

        p.status_label.setText("Setup wizard applied & saved")
        p.status_label.setStyleSheet("color:#44ff88; font-size:12px;")
        self.accept()


class MapGraphicsView(QGraphicsView):
    GX, GY, CELL = 48, 44, 54  # wider cells, moderate height for 15×22

    def __init__(self, parent, fixed_view=None):
        super().__init__(parent)
        self.p = parent
        self.fixed_view = fixed_view  # None = follow parent.view; 0=ign 1=inj
        self._scene = QGraphicsScene()
        self.setScene(self._scene)
        self.setRenderHint(QPainter.Antialiasing)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setStyleSheet("background-color:#12151c; border:1px solid #2a3a58;")
        self._items   = [[None]*COLS for _ in range(ROWS)]
        self._ch_h    = None
        self._ch_v    = None
        self._ch_dot  = None
        self.zone_drag = False
        self.zr0 = self.zc0 = self.zr1 = self.zc1 = -1
        self.build_grid()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._autosize_cells()

    def _autosize_cells(self):
        """Fit 15×22 grid into current viewport."""
        vw = max(200, self.viewport().width() - 8)
        vh = max(160, self.viewport().height() - 8)
        # margins for axis labels
        usable_w = vw - self.GX - 12
        usable_h = vh - self.GY - 12
        cw = max(28, int(usable_w / max(1, COLS)))
        ch = max(18, int(usable_h / max(1, ROWS)))
        # Use square-ish but prefer filling width
        cell = min(cw, max(ch, int(cw * 0.75)))
        cell = max(26, min(64, cell))
        if abs(cell - self.CELL) >= 2:
            type(self).CELL = cell  # class attr used by build_grid
            self.CELL = cell
            self.build_grid()

    @property
    def _view(self):
        return self.p.view if self.fixed_view is None else self.fixed_view
    @property
    def _table(self): return self.p.adv if self._view == 0 else self.p.inj
    @property
    def _max(self):   return ADV_MAX if self._view == 0 else INJ_MAX
    @property
    def _bins(self):
        """Y-axis = ECU load axis (0.10..1.08). Both MAP and TPS modes."""
        return self.p.map_bins

    def _clamp(self, v):
        return int(max(ADV_MIN, min(ADV_MAX, round(v)))) if self._view == 0 \
               else round(max(INJ_MIN, min(INJ_MAX, v)), 1)

    def build_grid(self):
        GX, GY, C = self.GX, self.GY, self.CELL
        sc  = self._scene
        sc.clear()
        self._items = [[None]*COLS for _ in range(ROWS)]
        tbl = self._table

        title = QGraphicsTextItem(
            f"{'Ignition (°)' if self._view==0 else 'Injector (ms)'}"
            f" [{'TPS' if self.p.use_tps else 'MAP'}]"
        )
        title.setPos(180, 10)
        title.setFont(QFont("Segoe UI", 11, QFont.Bold))
        title.setDefaultTextColor(QColor(160, 200, 255))
        sc.addItem(title)

        pen_default   = QPen(QColor(25, 25, 25))
        pen_selected  = QPen(QColor(255, 220, 40), 3)  # keyboard focus
        pen_zone      = QPen(QColor(255, 200, 0), 1.5)
        pen_track     = QPen(QColor(0, 255, 180), 2)

        zr_lo = min(self.zr0, self.zr1); zr_hi = max(self.zr0, self.zr1)
        zc_lo = min(self.zc0, self.zc1); zc_hi = max(self.zc0, self.zc1)

        for r in range(ROWS):
            row_lbl = QGraphicsTextItem(f"{self._bins[r]:.1f}")
            row_lbl.setPos(GX - 45, GY + r*C + 10)
            row_lbl.setDefaultTextColor(QColor(120, 160, 210))
            sc.addItem(row_lbl)

            for c in range(COLS):
                x, y = GX + c*C, GY + r*C
                rect = QGraphicsRectItem(x, y, C, C)
                rect.setBrush(QBrush(hmap_color(tbl[r][c], self._max)))

                is_sel   = (r == self.p.sel_r and c == self.p.sel_c)
                in_zone  = (self.p.zone_active
                            and zr_lo<=r<=zr_hi and zc_lo<=c<=zc_hi)

                if is_sel and self.p.crosshair_track:
                    rect.setPen(pen_track)
                elif is_sel:
                    rect.setPen(pen_selected)
                elif in_zone:
                    rect.setPen(pen_zone)
                else:
                    rect.setPen(pen_default)

                rect.setData(0, (r, c))
                sc.addItem(rect)
                self._items[r][c] = rect

                val_str = f"{tbl[r][c]:.1f}" if self._view == 1 \
                          else f"{int(tbl[r][c])}"
                txt = QGraphicsTextItem(val_str)
                txt.setPos(x + 5, y + 11)
                txt.setDefaultTextColor(QColor(255, 255, 255))
                sc.addItem(txt)

        for c in range(COLS):
            clbl = QGraphicsTextItem(str(int(self.p.rpm_bins[c])))
            clbl.setPos(GX + c*C + 4, GY - 22)
            clbl.setDefaultTextColor(QColor(120, 160, 210))
            sc.addItem(clbl)

        cy_pen       = QPen(QColor(255, 255, 40, 240), 2.5)
        self._ch_h   = sc.addLine(0, 0, 0, 0, cy_pen)
        self._ch_v   = sc.addLine(0, 0, 0, 0, cy_pen)
        self._ch_dot = sc.addEllipse(0, 0, 12, 12,
                                     QPen(QColor(255, 60, 40), 2),
                                     QBrush(QColor(255, 80, 40, 200)))
        self._ch_h.hide(); self._ch_v.hide(); self._ch_dot.hide()

    def update_crosshair(self):
        if None in (self._ch_h, self._ch_v, self._ch_dot):
            return
        if not self.p.connected:
            self._ch_h.hide(); self._ch_v.hide(); self._ch_dot.hide()
            return

        GX, GY, C = self.GX, self.GY, self.CELL
        # Prefer ECU MCELL (firmware lookup corner)
        try:
            mr = int(self.p.live.get("mcell_r", -1))
            mc = int(self.p.live.get("mcell_c", -1))
            if 0 <= mr < ROWS and 0 <= mc < COLS:
                li = float(mr)
                ri = float(mc)
            else:
                raise ValueError("no mcell")
        except (TypeError, ValueError):
            rpm  = self.p.live["rpm"]
            load = float(self.p.live.get("load") or 0.0)
            if load <= 0.0:
                raw = self.p.live["tps"] if self.p.use_tps else self.p.live["map"]
                load = float(raw) * (0.01 if float(raw) > 1.5 else 1.0)
            bins = self._bins
            rpm_range  = self.p.rpm_bins[-1] - self.p.rpm_bins[0]
            load_range = bins[-1] - bins[0]
            ri = max(0.0, min(COLS-1, (rpm  - self.p.rpm_bins[0]) / rpm_range  * (COLS-1))) if rpm_range else 0.0
            li = max(0.0, min(ROWS-1, (load - bins[0]) / load_range * (ROWS-1))) if load_range else 0.0

        cx = GX + ri*C + C/2
        cy = GY + li*C + C/2

        self._ch_h.setLine(GX, cy, GX + COLS*C, cy)
        self._ch_v.setLine(cx, GY, cx, GY + ROWS*C)
        self._ch_dot.setRect(cx - 4, cy - 4, 9, 9)
        self._ch_h.show(); self._ch_v.show(); self._ch_dot.show()

    def live_cell(self):
        # Prefer ECU-reported cell (exact lookup index)
        try:
            mr = int(self.p.live.get("mcell_r", -1))
            mc = int(self.p.live.get("mcell_c", -1))
            if 0 <= mr < ROWS and 0 <= mc < COLS:
                return mr, mc
        except (TypeError, ValueError):
            pass
        rpm  = self.p.live["rpm"]
        load = float(self.p.live.get("load") or 0.0)
        if load <= 0.0:
            raw = self.p.live["tps"] if self.p.use_tps else self.p.live["map"]
            load = float(raw) * (0.01 if float(raw) > 1.5 else 1.0)
        bins = self._bins

        rpm_range  = self.p.rpm_bins[-1] - self.p.rpm_bins[0]
        load_range = bins[-1] - bins[0]

        ci = int(round(max(0, min(COLS-1,
             (rpm  - self.p.rpm_bins[0]) / rpm_range  * (COLS-1))))) if rpm_range  else 0
        ri = int(round(max(0, min(ROWS-1,
             (load - bins[0])            / load_range * (ROWS-1))))) if load_range else 0
        return ri, ci

    def wheelEvent(self, e):
        f = 1.15 if e.angleDelta().y() > 0 else 1/1.15
        self.scale(f, f)

    def _rc_at(self, qpt):
        item = self._scene.itemAt(self.mapToScene(qpt.toPoint()), QTransform())
        return item.data(0) if isinstance(item, QGraphicsRectItem) else None

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.setFocus(Qt.MouseFocusReason)
            rc = self._rc_at(e.position())
            if rc:
                r, c = rc
                if e.modifiers() & Qt.ShiftModifier:
                    # Zone select – disable pan so drag selects cells
                    self.setDragMode(QGraphicsView.NoDrag)
                    self.zr0 = self.zr1 = r
                    self.zc0 = self.zc1 = c
                    self.zone_drag = True
                    self.p.zone_active = True
                    self.p.sel_r, self.p.sel_c = r, c
                    self.p.map_view = self
                    self.p.view = self._view
                else:
                    self.p.sel_r, self.p.sel_c = r, c
                    self.p.zone_active = False
                    self.zr0 = self.zc0 = self.zr1 = self.zc1 = -1
                self.build_grid()
                self.p._update_cell_hud()
                e.accept()
                return
        super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        if self.zone_drag:
            rc = self._rc_at(e.position())
            if rc:
                self.zr1, self.zc1 = rc
                self.build_grid()
            e.accept()
            return
        super().mouseMoveEvent(e)

    def mouseReleaseEvent(self, e):
        if self.zone_drag:
            self.zone_drag = False
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            bounds = self._zone_bounds()
            if bounds is not None:
                r0, r1, c0, c1 = bounds
                if r0 != r1 or c0 != c1:
                    self._interpolate_zone()
            self.p._update_cell_hud()
            e.accept()
            return
        super().mouseReleaseEvent(e)

    def _zone_bounds(self):
        """Return (r0,r1,c0,c1) for active zone, or None."""
        if not self.p.zone_active:
            return None
        if self.zr0 < 0 or self.zc0 < 0:
            return None
        r0, r1 = min(self.zr0, self.zr1), max(self.zr0, self.zr1)
        c0, c1 = min(self.zc0, self.zc1), max(self.zc0, self.zc1)
        return r0, r1, c0, c1

    def _interpolate_zone(self):
        """Bilinear interpolation across selected zone corners → fill cells."""
        bounds = self._zone_bounds()
        if bounds is None:
            return
        r0, r1, c0, c1 = bounds
        if r0 == r1 and c0 == c1:
            return
        tbl = self._table
        v00, v10 = tbl[r0][c0], tbl[r0][c1]
        v01, v11 = tbl[r1][c0], tbl[r1][c1]
        rs, cs = r1 - r0, c1 - c0
        for r in range(r0, r1 + 1):
            for c in range(c0, c1 + 1):
                yf = 0.0 if rs == 0 else (r - r0) / rs
                xf = 0.0 if cs == 0 else (c - c0) / cs
                v = ((1 - xf) * (1 - yf) * v00 + xf * (1 - yf) * v10
                     + (1 - xf) * yf * v01 + xf * yf * v11)
                tbl[r][c] = self._clamp(v)
                self.p.send_map_cell(r, c)
                time.sleep(0.018)
        self.p.status_label.setText(
            f"Zone {r1 - r0 + 1}×{c1 - c0 + 1} bilinear-interpolated")
        self.p.status_label.setStyleSheet("color:#8090b0; font-size:12px;")
        self.build_grid()

    def _nudge_cells(self, delta):
        # snapshot for undo
        try:
            if self.p.sel_r >= 0:
                r, c = self.p.sel_r, self.p.sel_c
                tbl = self._table
                self.p._push_undo(self._view, r, c, tbl[r][c])
        except Exception:
            pass
        """Apply delta to selected cell, or to every cell in active zone."""
        bounds = self._zone_bounds()
        if bounds is not None:
            r0, r1, c0, c1 = bounds
            for r in range(r0, r1 + 1):
                for c in range(c0, c1 + 1):
                    self._table[r][c] = self._clamp(self._table[r][c] + delta)
                    self.p.send_map_cell(r, c)
            self.p.status_label.setText(
                f"Zone {r1 - r0 + 1}×{c1 - c0 + 1} adjusted by {delta:+g}")
        elif self.p.sel_r >= 0:
            sr, sc = self.p.sel_r, self.p.sel_c
            self._table[sr][sc] = self._clamp(self._table[sr][sc] + delta)
            self.p.send_map_cell(sr, sc)
        else:
            return
        self.build_grid()
        self.p._update_cell_hud()

    def _step_for_key(self, key, text=""):
        """Small step for +/−, large for PageUp/PageDown. Sign included."""
        if self._view == 0:  # ignition
            small, large = ADV_STEP_SMALL, ADV_STEP_LARGE
        else:                 # injector
            small, large = INJ_STEP_SMALL, INJ_STEP_LARGE
        # Physical key codes (main + keypad)
        mapping = {
            Qt.Key_Plus:      +small,
            Qt.Key_Equal:     +small,
            Qt.Key_Minus:     -small,
            Qt.Key_Underscore: -small,
            Qt.Key_PageUp:    +large,
            Qt.Key_PageDown:  -large,
        }
        if key in mapping:
            return mapping[key]
        # Fallback: character from keyboard layout
        if text in ("+", "="):
            return +small
        if text in ("-", "_"):
            return -small
        return None

    def keyPressEvent(self, e):
        key = e.key()
        text = e.text() or ""
        mods = e.modifiers()

        # Trigger-angle mode is owned by main window (Shift+B / ← →)
        if key == Qt.Key_B and (mods & Qt.ShiftModifier):
            self.p.keyPressEvent(e)
            return
        if getattr(self.p, "trig_adjust_mode", False) and key in (
                Qt.Key_Left, Qt.Key_Right, Qt.Key_Escape):
            self.p.keyPressEvent(e)
            return

        # Arrow keys – move selection
        arrows = {
            Qt.Key_Left:  (0, -1),
            Qt.Key_Right: (0,  1),
            Qt.Key_Up:    (-1, 0),
            Qt.Key_Down:  (1,  0),
        }
        if key in arrows:
            if self.p.sel_r < 0:
                self.p.sel_r, self.p.sel_c = 0, 0
            else:
                dr, dc = arrows[key]
                self.p.sel_r = max(0, min(ROWS - 1, self.p.sel_r + dr))
                self.p.sel_c = max(0, min(COLS - 1, self.p.sel_c + dc))
            self.p.zone_active = False
            self.zr0 = self.zc0 = self.zr1 = self.zc1 = -1
            self.build_grid()
            self.p._update_cell_hud()
            e.accept()
            return

        # Value nudge: + / − small, PgUp / PgDn large
        delta = self._step_for_key(key, text)
        if delta is not None:
            if self.p.sel_r < 0 and not self.p.zone_active:
                e.ignore()
                return
            self._nudge_cells(delta)
            e.accept()
            return

        # I = re-run interpolation on current zone (corners fixed)
        if key in (Qt.Key_I,) and self.p.zone_active:
            self._interpolate_zone()
            e.accept()
            return

        super().keyPressEvent(e)


# ──────────────────────────────────────────────
#  MAIN WINDOW
# ──────────────────────────────────────────────
class ECUTuner(QMainWindow):

    _SERIAL_MAP = {
        "RPM":"rpm","MAP":"map","TPS":"tps","TMP":"ect","LOAD":"load",  # normalised 0..1.2

        "IAT":"iat","BAT":"bat",
        "EADC":"eadc","TADC":"tadc","BADC":"badc","IADC":"iadc","MADC":"madc",
        "SYNC":"sync","CAM":"cam","FAN":"fan","FP":"fp",
        "IGN":"ign","PW":"pw","INJ":"pw","PWUS":"pwus","INJMODE":"injmode","SEQ":"seq","BATCHRPM":"batchrpm","IDLE":"idle","IRPM":"irpm","ITHR":"ithr","DASH":"dash","DFCO":"dfco","OFC":"dfco","VVT1":"vvt1","VVT2":"vvt2","C1PH":"c1ph","C2PH":"c2ph","ASE":"ase","CLTCH":"clutch","LC":"lc","ALS":"als","ALSTO":"alsto","ALSF":"alsf","FFS":"ffs",
        "GAP":"gap","TERR":"terr","SERR":"terr","LOST":"lost",
        "TOOTH":"tooth","DEG":"tdeg","ANG":"tdeg",
        # STM32 sequential / closed-loop extensions
        "DWELL":"dwell","CYL":"cyl",
        "MCELL":"mcell","BASEIGN":"baseign","BASEINJ":"baseinj","TRET":"tret","LAM":"lam","O2":"o2","KNK":"knock","KRET":"kret","STFT":"stft","LTFT":"ltft","TTRIM":"ttrim","CL":"o2cl",
    }

    _KNOWN_TX_PREFIXES = (
        "SET:", "UPLOAD:", "SAVE", "CFG:", "GETCFG", "GETMAP",
        "GET:TPSCAL", "GETTPSCAL", "TRIMRESET", "RESET", "REBOOT", "SET:RESET", "SET:O2CL", "SET:LTFT",
        "SET:BOOST", "SET:TPS", "SET:PED", "SET:VVT",
    )

    def __init__(self):
        super().__init__()
        self.setWindowTitle("TorquEFI Basic – STM32 / USB CDC Tuner")
        self.setGeometry(40, 40, 1280, 800)
        self.setMinimumSize(900, 600)

        self.ser_worker     = SerialWorker()
        self.connected      = False
        self.use_tps        = False
        self.dbw_enable     = True
        self.coil_smart     = True
        self.idle_out_mode  = 0  # 0=2wire 1=1wire 2=stepper
        self.fire_order     = 0  # 0=1-3-4-2
        self.cylinders      = 4
        self.view           = 0
        self.sel_r          = self.sel_c = -1
        self.zone_active    = False
        self.crosshair_track = False
        self.snap_to_live = False
        self.high_contrast = False
        self._alarm_flash = False
        self._link_state = "off"  # off|ok|stale|err
        self._spark_rpm = []
        self._spark_ect = []
        self.trig_adjust_mode = False  # Shift+B then ←/→ adjusts trigger angle
        # Map download from ECU (GETMAP): None | "ADV" | "INJ"
        self._map_dl_mode = None
        self._map_dl_row = 0
        self._map_dl_started = 0.0
        self._map_dl_adv = None
        self._map_dl_inj = None

        self.adv      = [[int(10 + r*.8 + c) for c in range(COLS)] for r in range(ROWS)]
        self.inj      = [[round(2 + r*.4 + c*.5, 1) for c in range(COLS)] for r in range(ROWS)]
        self.rpm_bins = [250 + c*375          for c in range(COLS)]   # 250–8125 RPM, 22 pts
        self.map_bins = [round(0.10 + r*(0.98/14), 3) for r in range(ROWS)]  # 0.10–1.08, 15 pts
        self.tps_load = [round(r*100/14, 1)   for r in range(ROWS)]   # 0–100 %, 15 pts

        self.tempB   = [-40 + i * (160.0 / (CAL_COLS - 1)) for i in range(CAL_COLS)]
        self.tempC   = [1.0 for _ in range(CAL_COLS)]
        self.ectAdc  = [100 + i * (int(3800 / max(1, CAL_COLS - 1))) for i in range(CAL_COLS)]
        self.tpsB    = [i * (1023.0 / (CAL_COLS - 1)) for i in range(CAL_COLS)]
        # BAT compensation rows: voltage (V), scale factor, ADC @ that voltage
        self.batB    = [round(9.0 + i * (7.0 / max(1, CAL_COLS - 1)), 2) for i in range(CAL_COLS)]
        self.batC    = [1.0 for _ in range(CAL_COLS)]
        # ~11:1 divider, 3.3V/4096 ADC → ADC ≈ Vbat / 11 / 3.3 * 4096
        self.batAdc  = [int(round(v / 11.0 / 3.3 * 4096)) for v in self.batB]
        self.iatB    = [-20 + i * (120.0 / max(1, CAL_COLS - 1)) for i in range(CAL_COLS)]
        self.iatC    = [12 - i * 0.5 for i in range(CAL_COLS)]
        self.iatAdc  = [900 - i * (int(800 / max(1, CAL_COLS - 1))) for i in range(CAL_COLS)]
        self._load_suggested_sensor_tables()
        # Cold-start enrichment: ECT °C → extra fuel %
        self.cse_temp = [-20, 0, 10, 20, 30, 40, 50, 60, 70, 80]
        self.cse_pct  = [80, 55, 40, 28, 18, 12, 7, 3, 0, 0]
        # 17×16 pedal→throttle target (%), default linear in pedal
        self.etb_map = [
            [round(c * 100.0 / (ETB_COLS - 1), 1) for c in range(ETB_COLS)]
            for _ in range(ETB_ROWS)
        ]
        self.etb_pedal_bins = [round(c * 100.0 / (ETB_COLS - 1), 1) for c in range(ETB_COLS)]
        self.vvt_in_map = [[(10 + max(0, c-2)*3 if 2 <= c <= 5 else 0) // (2 if r >= 5 else 1)
                           for c in range(8)] for r in range(8)]
        self.vvt_ex_map = [[max(0, self.vvt_in_map[r][c] // 2) for c in range(8)] for r in range(8)]
        self.vvt_rpm_lbl = ["800","1200","1800","2500","3500","4500","5500","6500"]
        self.vvt_load_lbl = ["10%","20%","30%","45%","55%","70%","85%","100%"]
        self.bst_rpm_lbl = ["1500","2000","2500","3000","3500","4000","5000","6000"]
        self.bst_tps_lbl = ["20%","30%","40%","50%","60%","70%","80%","100%"]
        self.bst_map = [[round(20 + c*8 * (0.5 if r < 2 else (0.85 if r > 5 else 1.0)), 0)
                         for c in range(8)] for r in range(8)]
        self.bst_open_loop = False
        self.bst_map_duty = [[round(min(100, 10 + c*10 + r*2), 0)
                              for c in range(8)] for r in range(8)]
        self.etb_rpm_bins = [int(500 + r * (7500 / max(1, ETB_ROWS - 1))) for r in range(ETB_ROWS)]
        self.mapCalB = self.map_bins[:]
        self.mapCalC = [0.0]*ROWS
        self.mapAdc  = [100 + i*100    for i in range(ROWS)]

        self.live = {
            "rpm":0, "map":0.0, "tps":0, "ect":25.0, "iat":25.0, "bat":14.0,
            "eadc":512, "tadc":512, "badc":512, "iadc":512, "madc":512,
            "sync":0, "cam":0, "fan":0, "fp":0,
            "ign":0.0, "pw":0.0, "pwus":0, "mcell_r":-1, "mcell_c":-1, "tret":0.0, "lam":0.0, "injmode":0, "seq":0, "dwell":0.0, "cyl":4,
            "gap":0.0, "terr":0, "lost":0, "tooth":0, "tdeg":0.0,
            "o2":0.45, "knock":0.0, "stft":0.0, "ltft":0.0, "o2cl":0,
        }

        # Crank wheel config (applied via Engine Setup → CFG:)
        self.crank_teeth   = 36
        self.crank_missing = 1
        self.trigger_angle = 30   # deg BTDC at tooth #1 after gap
        self.cylinders     = 4
        self.rpm_limit     = 7000
        self.fan_setpoint  = 95
        self.fan_hyst_c    = 5
        self.ign_max_adv   = 45
        self.ign_min_adv   = -15
        self.strip_vis = {
            'rpm': True, 'tps': True, 'map': True,  # locked
            'load': True, 'ign': True, 'inj': True,
            'ect': True, 'fan': True, 'fp': True,
            'bat': True, 'afr': True, 'trim': True,
            'sync': True, 'cam': True,
        }
        self._load_engine_profile_json()

        # Debounced sync state machine
        #   "idle" | "searching" | "locked" | "lost"
        self.sync_state        = "idle"
        self._save_result      = None
        self._save_msg         = ""
        self.sync_detail       = None  # set in UI build; guarded on use
        self.sync_lbl          = None
        self.sync_cfg_lbl      = None
        self._sync_raw_streak  = 0      # consecutive frames with same raw bit
        self._sync_raw_last    = 0
        self._sync_lock_ts     = 0.0    # when we entered LOCKED
        self._sync_lost_ts     = 0.0
        self.sync_loss_count   = 0      # UI counter (local)
        self._sync_rpm_samples = deque(maxlen=12)

        # Protocol stats (UI feedback)
        self._parse_ok = 0
        self._parse_bad = 0
        self._last_rx_ts = 0.0

        self._init_ui()
        self._start_serial_thread()

    def _load_engine_profile_json(self):
        """Apply defaults from Engine Setup Tool (engine_profiles.json)."""
        import json
        from pathlib import Path
        path = Path(__file__).resolve().parent / "engine_profiles.json"
        if not path.is_file():
            return
        try:
            d = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            return
        self.cylinders     = int(d.get("cylinders", self.cylinders))
        self.crank_teeth   = int(d.get("teeth", self.crank_teeth))
        self.crank_missing = int(d.get("missing", self.crank_missing))
        self.trigger_angle = int(d.get("trig_angle", self.trigger_angle))
        self.rpm_limit     = int(d.get("rpm_limit", self.rpm_limit))
        self.fan_setpoint  = int(d.get("fan_c", self.fan_setpoint))
        self.use_tps       = bool(d.get("load_alpha_n", self.use_tps))

    # ── UI BUILD ────────────────────────────────────────────────
    def _init_ui(self):
        central = QWidget(); self.setCentralWidget(central)
        main = QVBoxLayout(central)
        main.setSpacing(4); main.setContentsMargins(6, 6, 6, 6)

        main.addLayout(self._build_top_bar())
        main.addWidget(self._build_live_strip())

        self.tabs = QTabWidget()
        self.map_view_ign = MapGraphicsView(self, fixed_view=0)
        self.map_view_inj = MapGraphicsView(self, fixed_view=1)
        self.map_view = self.map_view_ign

        ign_page = QWidget(); igl = QVBoxLayout(ign_page); igl.setContentsMargins(2,2,2,2)
        igl.addWidget(self.map_view_ign)
        inj_page = QWidget(); ijl = QVBoxLayout(inj_page); ijl.setContentsMargins(2,2,2,2)
        ijl.addWidget(self.map_view_inj)

        self.tabs.addTab(ign_page, "Ignition")
        self.tabs.addTab(inj_page, "Injection")
        self.tabs.addTab(self._build_vvt_tab(), "VVT Maps")
        self.tabs.addTab(self._build_throttle_tab(), "Throttle Map")
        self.tabs.addTab(self._build_boost_tab(), "Boost")
        self.tabs.addTab(self._build_motorsport_tab(), "Motorsport")
        self.tabs.currentChanged.connect(self._on_main_tab)
        main.addWidget(self.tabs, 1)

        main.addLayout(self._build_bottom_bar())

        try:
            self.load_breakpoints()
        except Exception:
            pass
        self._load_ui_config()
        try:
            self._apply_strip_vis()
        except Exception:
            pass

        self._undo_stack = []  # list of (view, r, c, old_val)
        self._scope_buf = deque(maxlen=SCOPE_SECONDS * SCOPE_HZ)

        self._tmr_fast = self._make_timer(50,   self._update_fast)
        self._tmr_ports = self._make_timer(2000, self._refresh_ports)
        self._tmr_slow = self._make_timer(2000, self._update_slow)

    def _build_top_bar(self):
        top = QHBoxLayout(); top.setSpacing(6)

        self.port_combo = QComboBox()
        self._refresh_ports()
        top.addWidget(self.port_combo)

        btn_refresh = QPushButton("↻", clicked=self._refresh_ports)
        btn_refresh.setFixedWidth(32)
        btn_refresh.setToolTip("Refresh serial port list")
        top.addWidget(btn_refresh)

        btn_con = QPushButton("Connect",    clicked=self.connect_serial)
        btn_wiz = QPushButton("Connect Wizard", clicked=self.connection_wizard)
        btn_dis = QPushButton("Disconnect", clicked=self.disconnect_serial)
        top.addWidget(btn_con)
        top.addWidget(btn_wiz)
        top.addWidget(btn_dis)
        top.addWidget(vsep())

        self.chk_tps = QCheckBox("Use TPS Load")
        self.chk_tps.setChecked(self.use_tps)
        self.chk_tps.stateChanged.connect(self.toggle_load)
        top.addWidget(self.chk_tps)
        top.addWidget(vsep())

        self.btn_track = QPushButton("🎯 Live Tune")
        self.btn_track.setCheckable(True)
        self.btn_track.setToolTip(
            "Track live RPM/Load cell – use +/− or Page Up/Down to adjust on the fly")
        self.btn_track.toggled.connect(self._on_track_toggled)
        top.addWidget(self.btn_track)
        self.chk_snap = QCheckBox("Snap cell")
        self.chk_snap.setToolTip("Selection follows live map cell")
        self.chk_snap.toggled.connect(lambda on: setattr(self, "snap_to_live", bool(on)))
        top.addWidget(self.chk_snap)
        top.addWidget(vsep())

        top.addWidget(QPushButton("Engine Settings", clicked=self.open_engine_settings))
        top.addWidget(QPushButton("Setup Wizard", clicked=self.open_setup_wizard))
        top.addWidget(QPushButton("Help", clicked=self.open_help))
        top.addWidget(vsep())

        self.conn_dot = QLabel("●")
        self.conn_dot.setStyleSheet("color:#444; font-size:18px;")
        top.addWidget(self.conn_dot)
        self.status_label = QLabel("Offline")
        self.status_label.setStyleSheet("color:#8090b0; font-size:12px;")
        top.addWidget(self.status_label)
        top.addStretch()
        return top



    def _pill(self, text, tip=""):
        w = QLabel(text)
        w.setAlignment(Qt.AlignCenter)
        w.setFixedHeight(26)
        w.setMinimumWidth(52)
        w.setStyleSheet(
            "background:#2a3344; color:#9aa8bc; border:1px solid #3a4a5a;"
            "border-radius:13px; font-weight:700; font-size:11px; padding:0 8px;")
        if tip:
            w.setToolTip(tip)
        return w

    def _build_live_strip(self):
        """Two-row live strip: primary values + status tray / secondary."""
        wrap = QFrame()
        wrap.setObjectName("liveStrip")
        wrap.setStyleSheet(
            "QFrame#liveStrip{background:#121820;border:1px solid #2a4a6a;border-radius:8px;}")
        root = QVBoxLayout(wrap)
        root.setContentsMargins(8, 6, 8, 6)
        root.setSpacing(4)

        # ── Row 1: primary ───────────────────────────────────
        row1 = QHBoxLayout(); row1.setSpacing(12)
        self.conn_led = QLabel("●")
        self.conn_led.setStyleSheet("color:#555; font-size:16px;")
        self.conn_led.setToolTip("USB link: off")
        row1.addWidget(self.conn_led)

        self.rpm_label = lbl("RPM —", 22, True, "#ffcc00")
        self.rpm_label.setToolTip("Engine speed")
        row1.addWidget(self.rpm_label)
        self.spark_rpm = SparklineWidget("#ffcc00")
        row1.addWidget(self.spark_rpm)

        self.strip_tps = lbl("TPS —", 14, True, "#aaffaa")
        self.strip_map = lbl("MAP —", 14, True, "#88ccff")
        self.strip_load = lbl("LOAD —", 14, True, "#ffcc66")
        self.tps_label = self.strip_tps
        self.map_label = self.strip_map
        self.load_label = self.strip_load
        for w, tip in (
            (self.strip_tps, "Throttle position %"),
            (self.strip_map, "Manifold pressure kPa"),
            (self.strip_load, "Engine load (TPS or MAP based)"),
        ):
            w.setToolTip(tip)
            row1.addWidget(w)

        self.timing_label = lbl("IGN —", 13, True, "#64c8ff")
        self.pulse_label = lbl("INJ —", 13, True, "#64ff9c")
        self.timing_label.setToolTip("Ignition advance °")
        self.pulse_label.setToolTip("Injector pulse width ms")
        row1.addWidget(self.timing_label)
        row1.addWidget(self.pulse_label)

        # Status tray — equal pills
        tray = QHBoxLayout(); tray.setSpacing(6)
        self.sync_box = self._pill("SYNC", "Crank sync lock")
        self.cam_box = self._pill("CAM", "Cam phase lock")
        self.fan_box = self._pill("FAN",
            f"Radiator fan — on at {getattr(self, "fan_setpoint", 95)}°C, hyst {getattr(self, "fan_hyst_c", 5)}°C")
        self.fp_box = self._pill("FP", "Fuel pump (run-on ~3s after stall)")
        for w in (self.sync_box, self.cam_box, self.fan_box, self.fp_box):
            tray.addWidget(w)
        row1.addLayout(tray)

        self.sync_dot = self.sync_box
        self.cam_dot = self.cam_box
        self.sync_lbl = QLabel(""); self.sync_lbl.setVisible(False)
        self.sync_detail = QLabel(""); self.sync_detail.setVisible(False)
        self.sync_cfg_lbl = QLabel(""); self.sync_cfg_lbl.setVisible(False)
        self.fan_dot = self.fan_box
        self.fp_dot = self.fp_box
        self.sync_lbl = QLabel(""); self.cam_lbl = QLabel("")
        self.fan_lbl = QLabel(""); self.fp_lbl = QLabel("")
        for w in (self.sync_lbl, self.cam_lbl, self.fan_lbl, self.fp_lbl):
            w.setVisible(False)

        row1.addStretch()
        root.addLayout(row1)

        # ── Row 2: secondary ─────────────────────────────────
        row2 = QHBoxLayout(); row2.setSpacing(12)
        self.ect_label = lbl("ECT —", 13, True, "#99ffcc")
        self.ect_label.setToolTip("Coolant temperature")
        row2.addWidget(self.ect_label)
        self.spark_ect = SparklineWidget("#99ffcc")
        row2.addWidget(self.spark_ect)

        self.bat_label = lbl("BAT —", 13, True, "#99ccff")
        self.afr_label = lbl("AFR —", 13, True, "#ffaa66")
        self.trim_label = lbl("TRIM —", 12, True, "#cc99ff")
        self.bat_label.setToolTip("Battery voltage")
        self.afr_label.setToolTip("AFR (from O2 if available)")
        self.trim_label.setToolTip("STFT + LTFT total fuel trim")
        row2.addWidget(self.bat_label)
        row2.addWidget(self.afr_label)
        row2.addWidget(self.trim_label)

        self.iat_label = lbl("", 1); self.iat_label.setVisible(False)
        self.rpm_bar = self.tps_bar = self.load_bar = None
        self.strip_rpm = self.rpm_label

        row2.addStretch()
        # Presets + contrast + customize
        for name, slot in (
            ("Street", lambda: self._apply_strip_preset("street")),
            ("Dyno", lambda: self._apply_strip_preset("dyno")),
            ("Debug", lambda: self._apply_strip_preset("debug")),
        ):
            b = QPushButton(name)
            b.setFixedHeight(24)
            b.setStyleSheet("padding:2px 8px; font-size:11px;")
            b.clicked.connect(slot)
            row2.addWidget(b)
        btn_hc = QPushButton("HC")
        btn_hc.setCheckable(True)
        btn_hc.setToolTip("High-contrast / sunlight mode")
        btn_hc.setFixedHeight(24)
        btn_hc.toggled.connect(self._toggle_high_contrast)
        row2.addWidget(btn_hc)
        self._btn_hc = btn_hc
        btn_cfg = QPushButton("⋯")
        btn_cfg.setFixedWidth(28)
        btn_cfg.setToolTip("Customize live strip")
        btn_cfg.clicked.connect(self._customize_live_strip)
        row2.addWidget(btn_cfg)
        root.addLayout(row2)

        self._live_strip_frame = wrap
        return wrap


    def _build_scope_page(self):
        page = QWidget()
        v = QVBoxLayout(page)
        v.addWidget(lbl("Last 30 s — RPM / MAP / SYNC (for unlock diagnosis)", 12, color="#8090b0"))
        self.scope_view = ScopeWidget(self)
        v.addWidget(self.scope_view, 1)
        return page

    def _build_live_panel(self):
        box = QGroupBox("LIVE ENGINE")
        lay = QVBoxLayout(box); lay.setSpacing(5)

        self.rpm_label = lbl("RPM: –––", 32, True, "#ffcc00")
        self.rpm_bar   = make_pbar(0, RPM_MAX, 10, rpm_style=True)
        lay.addWidget(self.rpm_label)
        lay.addWidget(self.rpm_bar)

        self.timing_label = lbl("IGN:  –°",       18, True, "#64c8ff")
        self.pulse_label  = lbl("INJ:  –.– ms",   18, True, "#64ff9c")
        lay.addWidget(self.timing_label)
        lay.addWidget(self.pulse_label)
        lay.addWidget(hsep())

        self.map_label = lbl("MAP:  –.–– kPa", 15)
        lay.addWidget(self.map_label)

        tps_row = QHBoxLayout()
        self.tps_label = lbl("TPS:  – %", 15)
        self.tps_bar   = make_pbar(0, 100, 12)
        tps_row.addWidget(self.tps_label)
        tps_row.addWidget(self.tps_bar, 1)
        lay.addLayout(tps_row)
        load_row = QHBoxLayout()
        self.load_label = lbl("LOAD: –.–", 15, color="#ffcc66")
        self.load_bar = make_pbar(0, 120, 12)
        load_row.addWidget(self.load_label)
        load_row.addWidget(self.load_bar, 1)
        lay.addLayout(load_row)
        lay.addWidget(hsep())

        self.ect_label = lbl("ECT:  –.– °C", color="#99ffcc")
        self.iat_label = lbl("IAT:  –.– °C", color="#ffcc99")
        self.bat_label = lbl("BAT:  –.–– V",  color="#99ccff")
        lay.addWidget(self.ect_label)
        lay.addWidget(self.iat_label)
        lay.addWidget(self.bat_label)
        self.o2_label = lbl("O2:   –.–– V  CL:–", color="#ff99cc")
        self.trim_label = lbl("STFT: –.–  LTFT: –.–", color="#ccff99")
        self.knock_label = lbl("KNOCK: –", color="#ffaa66")
        self.cam_label = lbl("CAM: –  CYL: –", color="#aaccff")
        lay.addWidget(self.o2_label)
        lay.addWidget(self.trim_label)
        btn_strip_cfg = QPushButton("⋯")
        btn_strip_cfg.setFixedWidth(28)
        btn_strip_cfg.setToolTip("Customize live strip")
        btn_strip_cfg.clicked.connect(self._customize_live_strip)
        lay.addWidget(btn_strip_cfg)
        lay.addWidget(self.knock_label)
        lay.addWidget(self.cam_label)
        lay.addWidget(hsep())

        status_grid = QGroupBox("STATUS")
        sg = QVBoxLayout(status_grid)
        sg.setSpacing(3)
        sg.setContentsMargins(8, 10, 8, 6)

        def status_row(dot_attr, txt_attr, label_text):
            row = QHBoxLayout()
            dot = status_dot()
            txt = lbl(label_text, 11)
            setattr(self, dot_attr, dot)
            setattr(self, txt_attr, txt)
            row.addWidget(dot)
            row.addWidget(txt)
            row.addStretch()
            return row

        sg.addLayout(status_row("sync_dot", "sync_lbl", "Sync"))
        sg.addLayout(status_row("cam_dot", "cam_lbl", "Cam"))
        sg.addLayout(status_row("fan_dot", "fan_lbl", "Fan"))
        sg.addLayout(status_row("fp_dot", "fp_lbl", "Pump"))
        # Keep attrs used elsewhere (minimal hidden labels)
        self.sync_detail = lbl("", 1, color="#607080")
        self.sync_detail.setVisible(False)
        self.sync_cfg_lbl = lbl("", 1, color="#708090")
        self.sync_cfg_lbl.setVisible(False)
        self.trig_mode_lbl = lbl("Shift+B: trig", 10, color="#607080")
        sg.addWidget(self.trig_mode_lbl)
        # Wheel selector lives in Engine Settings
        self.wheel_combo = QComboBox()
        for wid, wname, wt, wm in WHEEL_PROFILES:
            self.wheel_combo.addItem(f"{wname}", wid)

        lay.addWidget(status_grid)
        self._refresh_sync_cfg_lbl()

        # Cell / RX / map keys moved to Help — clean live panel
        lay.addStretch()
        return box


    def _apply_strip_preset(self, name: str):
        """Street / Dyno / Debug strip visibility presets."""
        presets = {
            "street": {
                "rpm": True, "tps": True, "map": True, "load": True,
                "ign": True, "inj": True, "ect": True, "fan": True, "fp": True,
                "bat": True, "afr": True, "trim": False, "sync": True, "cam": True,
            },
            "dyno": {
                "rpm": True, "tps": True, "map": True, "load": True,
                "ign": True, "inj": True, "ect": True, "fan": False, "fp": True,
                "bat": True, "afr": True, "trim": True, "sync": True, "cam": True,
            },
            "debug": {
                "rpm": True, "tps": True, "map": True, "load": True,
                "ign": True, "inj": True, "ect": True, "fan": True, "fp": True,
                "bat": True, "afr": True, "trim": True, "sync": True, "cam": True,
            },
        }
        vis = presets.get(name, presets["street"])
        self.strip_vis.update(vis)
        self.strip_vis["rpm"] = self.strip_vis["tps"] = self.strip_vis["map"] = True
        self._apply_strip_vis()
        try:
            path = self._ui_config_path()
            data = {}
            if path.is_file():
                data = json.loads(path.read_text(encoding="utf-8"))
            data["strip_vis"] = self.strip_vis
            data["strip_preset"] = name
            path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        except Exception:
            pass
        self.status_label.setText(f"Live strip preset: {name}")

    def _toggle_high_contrast(self, on: bool):
        self.high_contrast = bool(on)
        fs = 18 if on else 13
        big = 28 if on else 22
        for w, size in (
            (getattr(self, "rpm_label", None), big),
            (getattr(self, "strip_tps", None), fs),
            (getattr(self, "strip_map", None), fs),
            (getattr(self, "strip_load", None), fs),
            (getattr(self, "ect_label", None), fs),
            (getattr(self, "bat_label", None), fs),
            (getattr(self, "afr_label", None), fs),
        ):
            if w is None:
                continue
            try:
                w.setStyleSheet(w.styleSheet() + f"; font-size:{size}px;")
            except Exception:
                pass
        # Hide secondary in HC unless needed
        if on:
            for key in ("trim", "fan"):
                self.strip_vis[key] = False
            self._apply_strip_vis()
        self.status_label.setText("High-contrast ON" if on else "High-contrast OFF")

    def _customize_live_strip(self):
        """Choose optional live-strip gauges; RPM/TPS/MAP always on."""
        d = QDialog(self)
        d.setWindowTitle("Live strip telemetry")
        d.setMinimumWidth(360)
        lay = QVBoxLayout(d)
        lay.addWidget(lbl("Always shown: RPM · TPS · MAP", 11, color="#8090b0"))
        opts = [
            ("load", "LOAD"), ("ign", "IGN"), ("inj", "INJ"),
            ("ect", "ECT"), ("fan", "FAN"), ("fp", "Fuel pump"),
            ("bat", "BAT"), ("afr", "AFR"), ("trim", "TRIM"),
            ("sync", "SYNC"), ("cam", "CAM"),
        ]
        checks = {}
        for key, title in opts:
            cb = QCheckBox(title)
            cb.setChecked(bool(self.strip_vis.get(key, True)))
            checks[key] = cb
            lay.addWidget(cb)
        def apply():
            for k, cb in checks.items():
                self.strip_vis[k] = cb.isChecked()
            self.strip_vis["rpm"] = self.strip_vis["tps"] = self.strip_vis["map"] = True
            self._apply_strip_vis()
            try:
                path = self._ui_config_path()
                data = {}
                if path.is_file():
                    data = json.loads(path.read_text(encoding="utf-8"))
                data["strip_vis"] = self.strip_vis
                path.write_text(json.dumps(data, indent=2), encoding="utf-8")
            except Exception:
                pass
            d.accept()
        lay.addWidget(QPushButton("Apply", clicked=apply))
        d.exec()

    def _apply_strip_vis(self):
        """Show/hide optional strip widgets; RPM/TPS/MAP forced visible."""
        mapping = {
            "load": getattr(self, "strip_load", None) or getattr(self, "load_label", None),
            "ign": getattr(self, "timing_label", None),
            "inj": getattr(self, "pulse_label", None),
            "ect": getattr(self, "ect_label", None),
            "fan": getattr(self, "fan_box", None) or getattr(self, "fan_dot", None),
            "fp": getattr(self, "fp_box", None) or getattr(self, "fp_dot", None),
            "spark_rpm": getattr(self, "spark_rpm", None),
            "spark_ect": getattr(self, "spark_ect", None),
            "bat": getattr(self, "bat_label", None),
            "afr": getattr(self, "afr_label", None),
            "trim": getattr(self, "trim_label", None),
            "sync": getattr(self, "sync_box", None),
            "cam": getattr(self, "cam_box", None),
        }
        for k, w in mapping.items():
            if w is None:
                continue
            on = True if k in ("rpm", "tps", "map") else bool(self.strip_vis.get(k, True))
            try:
                w.setVisible(on)
            except Exception:
                pass
        for w in (getattr(self, "strip_tps", None), getattr(self, "strip_map", None),
                  getattr(self, "rpm_label", None)):
            if w is not None:
                w.setVisible(True)

    def _build_bottom_bar(self):
        bottom = QHBoxLayout(); bottom.setSpacing(6)
        for txt, slot in [
            ("Read from ECU", self._request_ecu_tables),
            ("Upload Maps",   self.upload_maps),
            ("Save Flash",    self.save_eeprom),
            ("Export Tune",   self.export_tune),
            ("Dashboard",     self.open_dashboard),
            ("Undo",          self.undo_edit),
        ]:
            bottom.addWidget(QPushButton(txt, clicked=slot))
        bottom.addStretch()
        q = QPushButton("Quit", clicked=self.close)
        q.setStyleSheet(
            "background:#5a2020;color:#ffaaaa;"
            "border:1px solid #8a3030;border-radius:5px;padding:5px 16px;")
        bottom.addWidget(q)
        return bottom

    @staticmethod
    def _make_timer(ms, slot):
        t = QTimer(); t.timeout.connect(slot); t.start(ms); return t

    # ── LIVE TRACK TOGGLE ───────────────────────────────────────
    def _on_track_toggled(self, checked):
        self.crosshair_track = checked
        if checked:
            self.btn_track.setStyleSheet(
                "background:#1a4a2a;color:#44ff88;"
                "border:1px solid #44ff88;border-radius:5px;padding:5px 12px;")
            self.status_label.setText(
                "Live Tune ON – +/− small, PgUp/Dn large")
        else:
            self.btn_track.setStyleSheet("")
            self.status_label.setText("Live Tune OFF")
        if hasattr(self, "map_view_ign"):
            self.map_view_ign.build_grid()
            self.map_view_inj.build_grid()
        elif hasattr(self, "map_view") and self.map_view:
            self.map_view.build_grid()
        self._update_cell_hud()

    # ── SERIAL ──────────────────────────────────────────────────
    def _refresh_ports(self):
        """List all serial ports; prefer STM32 CDC but never hide others."""
        current = self.port_combo.currentText()
        self.port_combo.clear()
        try:
            ports = list(serial.tools.list_ports.comports())
        except Exception:
            ports = []
        # Sort: STM/CDC first, then by device name
        def score(p):
            d = f"{p.device} {p.description} {p.manufacturer or ''} {p.hwid or ''}".lower()
            s = 0
            if any(x in d for x in ("stm", "stmicro", "cdc", "acm", "usbmodem", "serial")):
                s -= 10
            if "bluetooth" in d or "debug" in d:
                s += 5
            return (s, p.device)
        ports = sorted(ports, key=score)
        labels = []
        for p in ports:
            desc = (p.description or "").strip()
            labels.append(f"{p.device}  {desc}" if desc else p.device)
        self.port_combo.addItems(labels if labels else ["(no ports — check USB / DFU)"])
        # restore selection
        if current:
            for i in range(self.port_combo.count()):
                if current.split()[0] in self.port_combo.itemText(i):
                    self.port_combo.setCurrentIndex(i)
                    break
        # auto-pick first real port
        if self.port_combo.count() and "(no ports" not in self.port_combo.currentText():
            pass
        elif self.port_combo.count() > 0 and "(no ports" not in self.port_combo.itemText(0):
            self.port_combo.setCurrentIndex(0)


    def _set_conn(self, ok):
        self.connected = ok
        self.conn_dot.setStyleSheet(
            f"color:{'#44ff88' if ok else '#ff4444'}; font-size:18px;")


    def showEvent(self, event):
        """Keep window inside the usable screen area."""
        super().showEvent(event)
        try:
            from PySide6.QtGui import QGuiApplication
            screen = QGuiApplication.primaryScreen()
            if screen is not None:
                ag = screen.availableGeometry()
                g = self.frameGeometry()
                w = min(g.width(), ag.width() - 20)
                h = min(g.height(), ag.height() - 20)
                self.resize(max(900, w), max(560, h))
                # Clamp position on-screen
                x = max(ag.left(), min(self.x(), ag.right() - self.width()))
                y = max(ag.top(), min(self.y(), ag.bottom() - self.height()))
                self.move(x, y)
        except Exception:
            pass


    def _apply_inj_mode(self):
        if not self.connected:
            return
        m = self.inj_mode_combo.currentData()
        if m is None:
            m = self.inj_mode_combo.currentIndex()
        self.send(f"SET:INJMODE,{int(m)}\n")
        if int(m) == 3:
            self.send("SET:INJBATCHRPM,3000\n")
        self.status_label.setText(f"Injection mode → {self.inj_mode_combo.currentText()}")

    def _apply_wheel_profile(self):
        """Apply wheel to firmware and SAVE permanently to flash."""
        wid = self.wheel_combo.currentData()
        if wid is None:
            return
        for w in WHEEL_PROFILES:
            if w[0] == int(wid):
                self.crank_teeth = w[2]
                self.crank_missing = w[3]
                break
        if hasattr(self, "_refresh_sync_cfg_lbl"):
            self._refresh_sync_cfg_lbl()
        if not self.connected:
            self.status_label.setText("Wheel set locally – connect to save on ECU")
            self.status_label.setStyleSheet("color:#ffaa66; font-size:12px;")
            return
        ok = self._tx(f"SET:WHEEL,{int(wid)}\n")
        ok = self._tx(
            f"CFG:{self.crank_teeth},{self.crank_missing},{self.trigger_angle}\n") and ok
        ok = self._tx("SAVE\n") and ok
        name = self.wheel_combo.currentText()
        if ok:
            # Reset MCU so crank decoder restarts with new teeth/missing
            time.sleep(0.15)
            self._tx("RESET\n")
            self.status_label.setText(
                f"Wheel {name} saved – MCU resetting… re-connect if needed")
            self.status_label.setStyleSheet("color:#44ff88; font-size:12px;")
            # USB CDC often drops during NVIC reset
            try:
                if self.ser_worker:
                    self.ser_worker.disconnect()
            except Exception:
                pass
            self.connected = False
            try:
                self.conn_btn.setText("Connect")
            except Exception:
                pass
        else:
            self.status_label.setText("Wheel TX failed – check connection")
            self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")


    def connect_serial(self):

        raw = self.port_combo.currentText().strip()
        port = SerialWorker._normalize_port(raw)
        if not port or port.startswith("("):
            self.status_label.setText("Select port + Connect")
            self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
            self._set_conn(False)
            return
        ok, msg = self.ser_worker.connect(port)
        open_ok = ok and self.ser_worker.is_open()
        self._set_conn(open_ok)
        if open_ok:
            self._parse_ok = 0
            self._parse_bad = 0
            self.sync_state = "idle"
            self._sync_raw_streak = 0
            self._sync_raw_last = 0
            self._sync_lock_ts = 0.0
            self._sync_lost_ts = 0.0
            self._sync_rpm_samples.clear()
            self._map_dl_mode = None
            self._map_dl_row = 0
            self._paint_sync_ui()
            self.status_label.setText(
                f"Connected  {port}")
            self.status_label.setStyleSheet("color:#44ff88; font-size:12px;")
        else:
            self.status_label.setText(msg)
            self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")

    def disconnect_serial(self):
        self.ser_worker.disconnect()
        self._set_conn(False)
        self.sync_state = "idle"
        self.status_label.setText("Offline")
        self.status_label.setStyleSheet("color:#8090b0; font-size:12px;")
        self._paint_sync_ui()
        if getattr(self, "sync_lbl", None) is not None:
            self.sync_lbl.setText("Sync")
        if getattr(self, "sync_detail", None) is not None:
            if getattr(self, "sync_detail", None) is not None:
                self.sync_detail.setText("—")
        for dot in (self.fan_dot, self.fp_dot):
            dot.setStyleSheet("color:#444; font-size:16px;")

    def _start_serial_thread(self):
        w = self.ser_worker
        w.data_received.connect(self._parse_serial)
        w.error_occurred.connect(self._on_serial_error)
        w.connection_lost.connect(self._on_connection_lost)
        w.status_changed.connect(self._on_serial_status)
        threading.Thread(target=w.run, daemon=True).start()

    def _on_serial_error(self, msg: str):
        # Soft errors – show briefly, do not tear down UI state fully
        self.status_label.setText(msg)
        self.status_label.setStyleSheet("color:#ffaa66; font-size:12px;")

    def _on_connection_lost(self, msg: str):
        self._set_conn(False)
        self.status_label.setText(msg)
        self.status_label.setStyleSheet("color:#ff6666; font-size:12px;")
        for dot in (self.sync_dot, getattr(self, 'cam_dot', None), self.fan_dot, self.fp_dot):
            if dot is None:
                continue
            dot.setStyleSheet("color:#444; font-size:16px;")

    def _on_serial_status(self, msg: str):
        self.status_label.setText(msg)
        self.status_label.setStyleSheet("color:#8090b0; font-size:12px;")

    def _request_ecu_tables(self):
        """Ask MCU for config + maps. Requires an open CDC port."""
        if not self.ser_worker.is_open():
            # Try reconnect once using last port
            port = getattr(self.ser_worker, "_port_name", "") or ""
            if port:
                ok, msg = self.ser_worker.connect(port)
                self._set_conn(ok)
                if not ok:
                    self.status_label.setText(f"Read failed: {msg}")
                    self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
                    return
            else:
                self.status_label.setText("Connect first")
                self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
                self._set_conn(False)
                return

        self.connected = True
        self._map_dl_mode = None
        self._map_dl_row = 0
        self._map_dl_started = time.monotonic()
        self._map_dl_adv = [[0] * COLS for _ in range(ROWS)]
        self._map_dl_inj = [[0.0] * COLS for _ in range(ROWS)]
        self.status_label.setText("Reading ECU…")
        self.status_label.setStyleSheet("color:#ffcc44; font-size:12px;")

        ok1 = False
        last_err = ""
        for attempt in range(1, 4):
            ok1 = self.ser_worker.send("GETCFG\n")
            try:
                self.ser_worker.send("GET:IGNLIM\n")
                self.ser_worker.send("GET:FAN\n")
            except Exception:
                pass
            if ok1:
                break
            last_err = f"attempt {attempt}"
            time.sleep(0.35)
        if not ok1:
            self.status_label.setText(
                "Transmit failed on GETCFG – board not accepting USB commands. "
                "In PuTTY type GETCFG and Enter; if no reply, fix firmware CDC RX.")
            self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
            return

        def _send_getmap():
            if not self.ser_worker.is_open():
                # one reconnect try
                port = getattr(self.ser_worker, "_port_name", "") or ""
                if port:
                    ok, _msg = self.ser_worker.connect(port)
                    self._set_conn(ok)
                if not self.ser_worker.is_open():
                    self.status_label.setText("Port closed before GETMAP")
                    self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
                    return
            ok2 = False
            for attempt in range(1, 4):
                ok2 = self.ser_worker.send("GETMAP\n")
                if ok2:
                    break
                time.sleep(0.4)
            if ok2:
                self._map_dl_started = time.monotonic()
                self.status_label.setText("Loading maps…")
                self.status_label.setStyleSheet("color:#ffcc44; font-size:12px;")
            else:
                self.status_label.setText(
                    "TX failed on GETMAP – wait 1s, Read again "
                    "(or type GETMAP in PuTTY)")
                self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")

        # Let CFG reply and USB settle before flooding with GETMAP
        QTimer.singleShot(800, _send_getmap)

    def _parse_serial(self, line: str):
        """Parse telemetry, CFG:, or map dump rows from GETMAP."""
        if not line:
            return

        self._last_rx_ts = time.monotonic()
        up = line.upper()

        # Non-telemetry ACK / cal dumps from STM32
        if up.startswith("WHEEL:"):
            # WHEEL:id,teeth,missing,cam,name
            try:
                body = line.split(":", 1)[1]
                parts = body.split(",")
                wid = int(float(parts[0]))
                if len(parts) > 1:
                    self.crank_teeth = int(float(parts[1]))
                if len(parts) > 2:
                    self.crank_missing = int(float(parts[2]))
                if hasattr(self, "wheel_combo"):
                    for i in range(self.wheel_combo.count()):
                        if int(self.wheel_combo.itemData(i)) == wid:
                            self.wheel_combo.setCurrentIndex(i)
                            break
                self._refresh_sync_cfg_lbl()
                self.status_label.setText(f"Wheel {parts[-1] if len(parts)>4 else wid}")
            except (ValueError, TypeError, IndexError):
                pass
            self._parse_ok += 1
            return
        if up.startswith("OK:") or up.startswith("ERR:") or up.startswith("TPSCAL:"):
            self.status_label.setText(line[:80])
            self.status_label.setStyleSheet(
                "color:#44ff88; font-size:12px;" if up.startswith("OK") or up.startswith("TPS")
                else "color:#ff8866; font-size:12px;")
            self._parse_ok += 1
            return

        # Config dump: CFG:teeth,missing,trig,...
        if up.startswith("ERR:SAVE"):
            self._save_result = False
            self._save_msg = line.strip()
            detail = "flash write failed"
            if "VERIFY" in up:
                detail = "flash VERIFY failed after retry"
            elif "ERASE" in up:
                detail = "flash ERASE failed"
            elif "PROGRAM" in up:
                detail = "flash PROGRAM failed"
            self.status_label.setText("Save failed: " + detail)
            self.status_label.setStyleSheet("color:#ff6666; font-size:12px;")
            return
        if up.startswith("OK:CRC"):
            if getattr(self, "_save_result", None) is None:
                self._save_result = True
                self._save_msg = "Flash present " + line.strip()
            self.status_label.setText(line.strip())
            self.status_label.setStyleSheet("color:#44ff88; font-size:12px;")
            return
        if up.startswith("BUSY:SAVE"):
            self.status_label.setText("ECU writing flash…")
            self.status_label.setStyleSheet("color:#ffcc44; font-size:12px;")
            return
        if up.startswith("MAPSUM:"):
            self.status_label.setText(line.strip()[:80])
            return
        if up.startswith("OK:SAVE"):
            crc = ""
            if "CRC:" in up:
                crc = up.split("CRC:")[-1].split(",")[0].strip()
            msg = "Flash save OK — maps + LTFT + cal"
            if crc:
                msg += f"  {crc}"
            self.status_label.setText(msg)
            self.status_label.setStyleSheet("color:#44ff88; font-size:12px;")
            return
        if up.startswith("OK:IGNLIM"):
            try:
                body = line.split(":", 1)[1] if ":" in line else ""
                # OK:IGNLIM,min,max
                parts = line.replace("OK:IGNLIM", "").strip(", ").split(",")
                if len(parts) >= 2:
                    self.ign_min_adv = int(float(parts[0].split(",")[-1] if False else parts[0]))
                    self.ign_max_adv = int(float(parts[1]))
            except Exception:
                pass
            return
        if up.startswith("OK:FAN"):
            try:
                parts = [p for p in line.replace("OK:FAN", "").strip(", ").split(",") if p]
                # might be OK:FAN,95,90
                nums = []
                for p in line.split(","):
                    try: nums.append(float("".join(c for c in p if c in "0123456789.-")))
                    except Exception: pass
                if len(nums) >= 1:
                    self.fan_setpoint = int(nums[0])
                if len(nums) >= 2 and nums[0] > nums[1]:
                    self.fan_hyst_c = int(nums[0] - nums[1])
            except Exception:
                pass
            return
        if up.startswith("CFG:"):
            if self._apply_cfg_line(line[4:]):
                self._parse_ok += 1
            else:
                self._parse_bad += 1
            return

        # Map dump framing
        if up.startswith("MAP:ADV"):
            self._map_dl_mode = "ADV"
            self._map_dl_row = 0
            self._map_dl_adv_rows = 0
            self._map_dl_adv = [[0] * COLS for _ in range(ROWS)]
            self._parse_ok += 1
            return
        if up.startswith("MAP:INJ"):
            self._map_dl_mode = "INJ"
            self._map_dl_row = 0
            self._map_dl_inj_rows = 0
            self._map_dl_inj = [[0.0] * COLS for _ in range(ROWS)]
            self._parse_ok += 1
            return
        if up.startswith("MAP:END"):
            self._finish_map_download()
            self._parse_ok += 1
            return

        # Map data rows while downloading (escape if telemetry arrives)
        if self._map_dl_mode in ("ADV", "INJ"):
            if "RPM:" in up or up.startswith("RPM:"):
                # Skip telemetry line; keep download mode
                self._parse_ok += 1
                return
            else:
                if self._ingest_map_row(line):
                    self._parse_ok += 1
                else:
                    # Empty/broken INJ row: pad as zeros and advance so MAP:END still works
                    if self._map_dl_mode == "INJ" and self._map_dl_row < ROWS:
                        if self._map_dl_inj is None:
                            self._map_dl_inj = [[0.0] * COLS for _ in range(ROWS)]
                        r = self._map_dl_row
                        for c in range(COLS):
                            self._map_dl_inj[r][c] = 0.0
                        self._map_dl_row = r + 1
                        self._map_dl_inj_rows = self._map_dl_row
                        self._parse_ok += 1
                    else:
                        self._parse_bad += 1
                        if self._parse_bad > 40:
                            self._map_dl_mode = None
                            self._map_dl_row = 0
                            self.status_label.setText("Map download aborted – showing live data")
                if self._map_dl_mode in ("ADV", "INJ"):
                    return

        parts = line.split(",")
        accepted = 0
        rejected = 0

        for part in parts:
            part = part.strip()
            if not part:
                continue
            if ":" not in part:
                rejected += 1
                continue
            k, _, v = part.partition(":")
            key = self._SERIAL_MAP.get(k.strip().upper())
            if key is None:
                continue
            v = v.strip()
            try:
                if key in ("rpm", "sync", "cam", "fan", "fp",
                           "eadc", "tadc", "badc", "iadc", "madc",
                           "terr", "lost", "tooth", "cyl", "o2cl", "pwus"):
                    self.live[key] = int(float(v))
                elif key in ("tps", "map", "load", "ect", "iat"):
                    self.live[key] = float(v)
                else:
                    self.live[key] = float(v)
                accepted += 1
            except (ValueError, TypeError, OverflowError):
                rejected += 1

        if accepted:
            # MCELL may arrive as "r:c" string from ECU
            mc = self.live.get("mcell")
            if mc is not None and mc != "":
                try:
                    if isinstance(mc, str) and ":" in mc:
                        a, b = mc.split(":", 1)
                        self.live["mcell_r"] = int(float(a))
                        self.live["mcell_c"] = int(float(b))
                    else:
                        # sometimes two keys — leave mcell_r/c if already set
                        pass
                except (ValueError, TypeError):
                    pass
            self._parse_ok += 1
            if self.live.get("pwus"):
                try:
                    self.live["pw"] = float(self.live["pwus"]) * 0.001
                except Exception:
                    pass
            self._update_sync_state()
        if rejected and not accepted:
            self._parse_bad += 1

    def _ingest_map_row(self, line: str) -> bool:
        """Parse one comma-separated map row during GETMAP download.

        Tolerates empty INJ rows (nano printf float gaps), short rows (pad),
        and blank lines (skip without aborting the download).
        """
        mode = self._map_dl_mode
        row = self._map_dl_row
        if row >= ROWS:
            return False
        try:
            raw_line = (line or "").strip()
            # Blank / framing noise — ignore, stay on same row index
            if not raw_line or raw_line.startswith("MAP:"):
                return True

            parts = [p.strip() for p in raw_line.split(",")]
            # Drop trailing empty token from trailing comma
            while parts and parts[-1] == "":
                parts.pop()

            # Completely empty cells (e.g. ",,,") — treat as zero row for INJ
            nonempty = [p for p in parts if p != ""]
            if not nonempty:
                parts = ["0"] * COLS

            # Pad short rows with last value or 0
            if len(parts) < COLS:
                fill = parts[-1] if parts else "0"
                parts = parts + [fill] * (COLS - len(parts))
            parts = parts[:COLS]

            if mode == "ADV":
                if self._map_dl_adv is None:
                    self._map_dl_adv = [[0] * COLS for _ in range(ROWS)]
                for c in range(COLS):
                    cell = parts[c] if parts[c] != "" else "0"
                    self._map_dl_adv[row][c] = int(round(float(cell)))
                self._map_dl_adv_rows = row + 1
            else:
                if self._map_dl_inj is None:
                    self._map_dl_inj = [[0.0] * COLS for _ in range(ROWS)]
                # ECU: integer tenths (25 = 2.5 ms). Legacy: "2.5" as ms.
                for c in range(COLS):
                    cell = parts[c] if parts[c] != "" else "0"
                    try:
                        raw = float(cell)
                    except ValueError:
                        raw = 0.0
                    if "." not in cell:
                        raw = raw / 10.0  # tenths → ms
                    self._map_dl_inj[row][c] = raw
                self._map_dl_inj_rows = row + 1
            self._map_dl_row = row + 1
            return True
        except (ValueError, TypeError, IndexError):
            return False

    def _finish_map_download(self):
        """Apply buffered GETMAP tables into live grids.

        ADV: apply if complete.
        INJ: apply complete table; if only empty/zero rows arrived, keep local
        inj map instead of wiping UI to zeros.
        """
        try:
            adv_rows = int(getattr(self, "_map_dl_adv_rows", 0) or 0)
            inj_rows = int(getattr(self, "_map_dl_inj_rows", 0) or 0)
            applied = []

            if self._map_dl_adv is not None and adv_rows >= ROWS:
                for r in range(ROWS):
                    for c in range(COLS):
                        self.adv[r][c] = int(self._map_dl_adv[r][c])
                applied.append(f"ADV:{adv_rows}")
            elif adv_rows:
                applied.append(f"ADV:partial:{adv_rows}/{ROWS}")

            inj_sum = 0.0
            if self._map_dl_inj is not None and inj_rows > 0:
                for r in range(min(inj_rows, ROWS)):
                    for c in range(COLS):
                        inj_sum += float(self._map_dl_inj[r][c])

            if self._map_dl_inj is not None and inj_rows >= ROWS and inj_sum > 0.01:
                for r in range(ROWS):
                    for c in range(COLS):
                        self.inj[r][c] = float(self._map_dl_inj[r][c])
                applied.append(f"INJ:{inj_rows}")
            elif self._map_dl_inj is not None and inj_rows >= ROWS and inj_sum <= 0.01:
                # Full set of empty/zero rows — keep local table
                applied.append("INJ:empty (kept local)")
            elif inj_rows:
                # Partial — fill received rows only, leave rest of local map
                for r in range(min(inj_rows, ROWS)):
                    for c in range(COLS):
                        self.inj[r][c] = float(self._map_dl_inj[r][c])
                applied.append(f"INJ:partial:{inj_rows}/{ROWS}")
            else:
                applied.append("INJ:none (kept local)")

            for name in ("map_view_ign", "map_view_inj", "map_view"):
                v = getattr(self, name, None)
                if v is not None and hasattr(v, "build_grid"):
                    v.build_grid()

            ok = adv_rows >= ROWS and (
                (inj_rows >= ROWS and inj_sum > 0.01) or "kept local" in ",".join(applied)
            )
            # Still success if ADV ok and we deliberately kept local INJ
            if adv_rows >= ROWS and inj_rows == 0:
                ok = True
            msg = "Maps loaded from ECU (" + ", ".join(applied) + ")"
            if not ok:
                msg = "Map read incomplete — " + ", ".join(applied)
            self.status_label.setText(msg)
            self.status_label.setStyleSheet(
                "color:#44ff88; font-size:12px;" if ok
                else "color:#ffaa44; font-size:12px;")
        except Exception as ex:
            self.status_label.setText(f"Map load apply error: {ex}")
            self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
        self._map_dl_mode = None
        self._map_dl_row = 0


    def _apply_cfg_line(self, body: str) -> bool:
        """Parse teeth,missing,trig[,cyl,rpmLim,fan,useTps] from ECU."""
        try:
            fields = [p.strip() for p in body.split(",")]
            if len(fields) < 3:
                return False
            teeth = int(float(fields[0]))
            missing = int(float(fields[1]))
            trig = int(float(fields[2]))
            if teeth < 12 or teeth > 60:
                return False
            if missing < 0 or missing >= teeth or (teeth - missing) < 3:
                return False
            if trig < 0 or trig > 90:
                return False
            self.crank_teeth = teeth
            self.crank_missing = missing
            self.trigger_angle = trig
            # STM32: CFG:teeth,missing,trig,CYL:n,SEQ:n  or plain numeric extras
            for f in fields[3:]:
                fu = f.upper()
                if fu.startswith("CYL:"):
                    self.cylinders = max(1, min(8, int(float(fu.split(":")[1]))))
                elif fu.startswith("SEQ:"):
                    pass  # sequential flag informational
                else:
                    try:
                        n = int(float(f))
                        if 1 <= n <= 8 and "CYL" not in "".join(fields[3:]).upper():
                            self.cylinders = n
                    except ValueError:
                        pass
            self._refresh_sync_cfg_lbl()
            self._update_trig_mode_lbl()
            self.status_label.setText(
                f"ECU config: {teeth}-{missing} trig {trig}°  "
                f"{self.cylinders} cyl  limit {self.rpm_limit}"
            )
            self.status_label.setStyleSheet("color:#8090b0; font-size:12px;")
            return True
        except (ValueError, TypeError, IndexError):
            return False

    def _tx(self, data: str) -> bool:
        """Send with connection guard and basic command shape check."""
        if not self.connected or not self.ser_worker.is_open():
            self._set_conn(False)
            self.status_label.setText("Not connected")
            self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
            return False
        if not any(data.startswith(p) for p in self._KNOWN_TX_PREFIXES):
            # Soft warning only – still send (allows future commands)
            pass
        ok = self.ser_worker.send(data)
        if not ok:
            self.status_label.setText("TX failed")
            self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
        return ok

    # ── CRANK SYNC STATE MACHINE ────────────────────────────────
    def _update_sync_state(self):
        """Debounce raw SYNC bit and track lock quality."""
        if not self.connected:
            return
        raw = 1 if self.live.get("sync") else 0
        rpm = int(self.live.get("rpm") or 0)
        now = time.monotonic()

        if raw == self._sync_raw_last:
            self._sync_raw_streak += 1
        else:
            self._sync_raw_last = raw
            self._sync_raw_streak = 1

        prev = self.sync_state

        if rpm < SYNC_SEARCH_RPM and not raw:
            # Engine stopped / not turning – idle
            if self.sync_state not in ("idle",):
                self.sync_state = "idle"
        elif raw and self._sync_raw_streak >= SYNC_LOCK_FRAMES:
            if self.sync_state != "locked":
                self.sync_state = "locked"
                self._sync_lock_ts = now
                self._sync_rpm_samples.clear()
        elif not raw and self._sync_raw_streak >= SYNC_UNLOCK_FRAMES:
            if self.sync_state == "locked":
                self.sync_state = "lost"
                self._sync_lost_ts = now
                self.sync_loss_count += 1
            elif self.sync_state in ("idle", "lost") and rpm >= SYNC_SEARCH_RPM:
                self.sync_state = "searching"
            elif self.sync_state != "lost":
                self.sync_state = "searching" if rpm >= SYNC_SEARCH_RPM else "idle"
        elif not raw and self.sync_state == "locked":
            # brief dropout – stay locked until unlock threshold
            pass
        elif raw and self.sync_state != "locked":
            self.sync_state = "searching"

        if self.sync_state == "locked" and rpm > 0:
            self._sync_rpm_samples.append(rpm)

        # ECU cumulative LOST field (if present) wins as lower bound
        ecu_lost = int(self.live.get("lost") or 0)
        if ecu_lost > self.sync_loss_count:
            self.sync_loss_count = ecu_lost

        if prev != self.sync_state:
            self._paint_sync_ui()

    def _rpm_jitter_pct(self) -> float:
        """Peak-to-peak RPM variation % while locked (sync quality proxy)."""
        s = list(self._sync_rpm_samples)
        if len(s) < 4:
            return 0.0
        avg = sum(s) / len(s)
        if avg <= 0:
            return 0.0
        return (max(s) - min(s)) / avg * 100.0

    def _paint_sync_ui(self):
        """Solid SYNC / CAM boxes — green locked, amber searching, red lost."""
        if not hasattr(self, "sync_box"):
            return
        st = getattr(self, "sync_state", "idle")
        if st == "locked":
            self.sync_box.setStyleSheet(
                "background:#1a5c2a; color:#44ff88; border:1px solid #44ff88;"
                " border-radius:4px; font-weight:bold;")
            self.sync_box.setText(" SYNC ")
        elif st == "searching":
            self.sync_box.setStyleSheet(
                "background:#5c4a1a; color:#ffcc44; border:1px solid #ffcc44;"
                " border-radius:4px; font-weight:bold;")
            self.sync_box.setText(" SYNC ")
        elif st == "lost":
            self.sync_box.setStyleSheet(
                "background:#5c1a1a; color:#ff6666; border:1px solid #ff6666;"
                " border-radius:4px; font-weight:bold;")
            self.sync_box.setText(" SYNC ")
        else:
            self.sync_box.setStyleSheet(
                "background:#333; color:#888; border:1px solid #555;"
                " border-radius:4px; font-weight:bold;")
            self.sync_box.setText(" SYNC ")
        cam_on = bool(self.live.get("cam") or self.live.get("sync") == 1 and self.live.get("cam"))
        try:
            cam_on = int(float(self.live.get("cam", 0))) != 0
        except Exception:
            cam_on = False
        if cam_on:
            self.cam_box.setStyleSheet(
                "background:#1a3a5c; color:#66ccff; border:1px solid #66ccff;"
                " border-radius:4px; font-weight:bold;")
            self.cam_box.setText(" CAM ")
        else:
            self.cam_box.setStyleSheet(
                "background:#333; color:#888; border:1px solid #555;"
                " border-radius:4px; font-weight:bold;")
            self.cam_box.setText(" CAM ")


    def _refresh_sync_cfg_lbl(self):
        if not hasattr(self, "sync_cfg_lbl"):
            return
        phys = max(1, self.crank_teeth - self.crank_missing)
        deg = 360.0 / self.crank_teeth if self.crank_teeth else 0
        self.sync_cfg_lbl.setText(
            f"{self.crank_teeth}-{self.crank_missing}  "
            f"trig {self.trigger_angle}°  "
            f"({phys} teeth · {deg:.1f}°/t)"
        )
        self._update_trig_mode_lbl()

    def _update_trig_mode_lbl(self):
        if not hasattr(self, "trig_mode_lbl"):
            return
        if self.trig_adjust_mode:
            self.trig_mode_lbl.setText(f"Trig {self.trigger_angle}°  ←/→")
            self.trig_mode_lbl.setStyleSheet(
                "color:#ffcc44; font-weight:bold; font-size:11px;")
        else:
            self.trig_mode_lbl.setText(f"Trig {self.trigger_angle}°")
            self.trig_mode_lbl.setStyleSheet("color:#607080; font-size:10px;")

    def _nudge_trigger_angle(self, delta: int):
        ang = max(0, min(90, int(self.trigger_angle) + int(delta)))
        if ang == self.trigger_angle:
            return
        self.trigger_angle = ang
        self._refresh_sync_cfg_lbl()
        if self.connected:
            ok = self._tx(
                f"CFG:{self.crank_teeth},{self.crank_missing},{self.trigger_angle}\n")
            self.status_label.setText(
                f"Trigger → {self.trigger_angle}° BTDC"
                + ("" if ok else " (TX error)"))
            self.status_label.setStyleSheet(
                "color:#44ff88; font-size:12px;" if ok
                else "color:#ff8866; font-size:12px;")
        else:
            self.status_label.setText(
                f"Trigger {self.trigger_angle}° local – connect to send")
            self.status_label.setStyleSheet("color:#ffaa66; font-size:12px;")

    # ── FAST UPDATE (80 ms) ─────────────────────────────────────
    def _update_fast(self):
        # Cap UI refresh so gauges are readable
        now = time.monotonic()
        if getattr(self, "_ui_last_paint", 0) and (now - self._ui_last_paint) < 0.04:
            return
        self._ui_last_paint = now

        live = self.live
        rpm  = int(live.get("rpm") or 0)
        # Scope buffer (~30 s at SCOPE_HZ via 50 ms timer → every 2nd frame)
        if not hasattr(self, "_scope_div"):
            self._scope_div = 0
        self._scope_div += 1
        if self._scope_div >= 2 and hasattr(self, "_scope_buf"):
            self._scope_div = 0
            try:
                self._scope_buf.append((
                    float(rpm),
                    float(live.get("map") or 0),
                    1 if live.get("sync") else 0,
                ))
            except Exception:
                pass
            if hasattr(self, "scope_view"):
                self.scope_view.update()

        self.rpm_label.setText(f"RPM: {rpm}")
        if self.rpm_bar is not None:
            self.rpm_bar.setValue(min(rpm, RPM_MAX))

        try:
            ign = float(live.get("ign") or 0)
        except Exception:
            ign = 0.0
        # Prefer integer microseconds (PWUS) — survives nano-printf float gaps
        pw = 0.0
        try:
            if live.get("pwus"):
                pw = float(live["pwus"]) * 0.001
            elif live.get("pw") is not None:
                pw = float(live.get("pw") or 0)
            elif live.get("inj") is not None:
                pw = float(live.get("inj") or 0)
        except Exception:
            pw = 0.0
        if hasattr(self, "timing_label") and self.timing_label is not None:
            self.timing_label.setText(f"IGN: {ign:.0f}°")
        if hasattr(self, "pulse_label") and self.pulse_label is not None:
            self.pulse_label.setText(f"INJ: {pw:.2f} ms")

        # ECT / Fan / Fuel pump
        try:
            ect = float(live.get("ect") or 0)
        except Exception:
            ect = 0.0
        if hasattr(self, "ect_label") and self.ect_label is not None:
            if ect < 40:
                ec = "#66aacc"
            elif ect < 95:
                ec = "#99ffcc"
            elif ect < 105:
                ec = "#ffcc44"
            else:
                ec = "#ff6644"
            self.ect_label.setText(f"ECT {ect:.0f}°C")
            self.ect_label.setStyleSheet(f"color:{ec}; font-size:13px; font-weight:700;")
        fan_on = int(live.get("fan") or 0)
        fp_on = int(live.get("fp") or 0)
        if hasattr(self, "fan_box") and self.fan_box is not None:
            if fan_on:
                self.fan_box.setText(" FAN ON ")
                self.fan_box.setStyleSheet(
                    "background:#1a4a6a; color:#66ddff; border:1px solid #44aacc;"
                    "border-radius:4px; font-weight:bold;")
            else:
                self.fan_box.setText(" FAN ")
                self.fan_box.setStyleSheet(
                    "background:#333; color:#888; border:1px solid #555;"
                    "border-radius:4px; font-weight:bold;")
        if hasattr(self, "fp_box") and self.fp_box is not None:
            if fp_on:
                self.fp_box.setText(" FP ON ")
                self.fp_box.setStyleSheet(
                    "background:#1a4a2a; color:#66ff99; border:1px solid #44cc66;"
                    "border-radius:4px; font-weight:bold;")
            else:
                self.fp_box.setText(" FP ")
                self.fp_box.setStyleSheet(
                    "background:#333; color:#888; border:1px solid #555;"
                    "border-radius:4px; font-weight:bold;")


        # Sparklines (reuse ~30 s buffers)
        try:
            self._spark_rpm.append(float(rpm))
            if len(self._spark_rpm) > 120:
                self._spark_rpm = self._spark_rpm[-120:]
            if hasattr(self, "spark_rpm"):
                self.spark_rpm.set_data(self._spark_rpm)
        except Exception:
            pass
        try:
            self._spark_ect.append(float(live.get("ect") or 0))
            if len(self._spark_ect) > 120:
                self._spark_ect = self._spark_ect[-120:]
            if hasattr(self, "spark_ect"):
                self.spark_ect.set_data(self._spark_ect)
        except Exception:
            pass

        # Alarms: flash ECT over-temp or FP off while spinning
        self._alarm_flash = not self._alarm_flash
        ect_alarm = ect > 105.0
        fp_alarm = (rpm > 400) and (fp_on == 0)
        if hasattr(self, "ect_label") and self.ect_label is not None and ect_alarm:
            self.ect_label.setStyleSheet(
                f"color:{'#ffffff' if self._alarm_flash else '#ff2244'};"
                f"background:{'#aa0000' if self._alarm_flash else 'transparent'};"
                "font-size:13px; font-weight:800; border-radius:4px;")
        if hasattr(self, "fp_box") and self.fp_box is not None and fp_alarm:
            self.fp_box.setText(" FP OFF! ")
            self.fp_box.setStyleSheet(
                "background:%s; color:#fff; border:1px solid #ff4444;"
                "border-radius:13px; font-weight:bold;" % (
                    "#aa0000" if self._alarm_flash else "#552222"))

        # Connection LED
        if hasattr(self, "conn_led"):
            if not self.connected:
                col, tip = "#555", "USB link: disconnected"
            else:
                age = time.monotonic() - float(getattr(self, "_last_rx_ts", 0) or 0)
                if age < 1.5:
                    col, tip = "#33ee88", "USB link: OK"
                elif age < 5.0:
                    col, tip = "#ffcc33", "USB link: stale telemetry"
                else:
                    col, tip = "#ff4444", "USB link: no data"
            self.conn_led.setStyleSheet(f"color:{col}; font-size:16px;")
            self.conn_led.setToolTip(tip)
            if hasattr(self, "conn_dot"):
                self.conn_dot.setStyleSheet(f"color:{col}; font-size:18px;")
                self.conn_dot.setToolTip(tip)

        # Status tray pill colours for SYNC/CAM (when updated elsewhere may overwrite)
        sync_on = int(live.get("sync") or 0)
        cam_on = int(live.get("cam") or 0)
        if hasattr(self, "sync_box"):
            self.sync_box.setStyleSheet(
                "background:%s; color:%s; border:1px solid %s;"
                "border-radius:13px; font-weight:700; font-size:11px; padding:0 8px;" % (
                    ("#1a4a2a", "#66ff99", "#44cc66") if sync_on else ("#2a3344", "#9aa8bc", "#3a4a5a")))
        if hasattr(self, "cam_box"):
            self.cam_box.setStyleSheet(
                "background:%s; color:%s; border:1px solid %s;"
                "border-radius:13px; font-weight:700; font-size:11px; padding:0 8px;" % (
                    ("#1a3a4a", "#66ccff", "#4499cc") if cam_on else ("#2a3344", "#9aa8bc", "#3a4a5a")))


        # Live MAP / TPS / LOAD every frame
        try:
            map_v = float(live.get("map") or 0)
        except Exception:
            map_v = 0.0
        for w in (getattr(self, "strip_map", None), getattr(self, "map_label", None)):
            if w is not None:
                try: w.setText(f"MAP {map_v:.1f}")
                except Exception: pass

        try:
            tps_v = float(live.get("tps") or 0)
        except Exception:
            tps_v = 0.0
        tps = int(min(100, max(0, tps_v)))
        for w in (getattr(self, "strip_tps", None), getattr(self, "tps_label", None)):
            if w is not None:
                try: w.setText(f"TPS {tps}%")
                except Exception: pass
        if self.tps_bar is not None:
            self.tps_bar.setValue(tps)

        try:
            load = float(live.get("load") or 0)
        except Exception:
            load = 0.0
        if load <= 0.0:
            load = (tps_v / 100.0) if self.use_tps else (map_v / 100.0)
        live["load"] = load
        if hasattr(self, "load_label"):
            for w in (getattr(self, "strip_load", None), getattr(self, "load_label", None)):
                if w is not None:
                    try: w.setText(f"LOAD {load:.2f}")
                    except Exception: pass
            if self.load_bar is not None:
                self.load_bar.setValue(int(max(0, min(120, load * 100))))


        # ECU BASEIGN/BASEINJ = bilinear map lookup (pre-retard/trim).
        # Live IGN/PW include retards, trims, min-PW clamp.
        try:
            live_ign = float(self.live.get("ign") or 0)
            live_pw  = float(self.live.get("pw") or 0)
            tret = float(self.live.get("tret") or 0)
            # BASEINJ from ECU is tenths; BASEIGN is degrees
            base_ign = self.live.get("baseign")
            base_inj = self.live.get("baseinj")
            if base_ign is not None and base_ign != "":
                bi = int(float(base_ign))
            else:
                bi = None
            if base_inj is not None and base_inj != "":
                bj = float(base_inj)
                if bj > 25:  # tenths
                    bj = bj / 10.0
            else:
                bj = None
            # Fallback: local cell (corner only — not interpolated)
            mr = int(self.live.get("mcell_r", -1))
            mc = int(self.live.get("mcell_c", -1))
            if bi is None and 0 <= mr < ROWS and 0 <= mc < COLS:
                bi = int(self.adv[mr][mc])
            if bj is None and 0 <= mr < ROWS and 0 <= mc < COLS:
                bj = float(self.inj[mr][mc])
            if hasattr(self, "ign_label") and self.ign_label:
                if bi is not None:
                    self.ign_label.setText(
                        f"IGN {live_ign:.0f}°  base {bi}°  ret {tret:.1f}")
                else:
                    self.ign_label.setText(f"IGN {live_ign:.0f}°")
            if hasattr(self, "pulse_label") and self.pulse_label:
                if bj is not None:
                    self.pulse_label.setText(
                        f"INJ {live_pw:.2f} ms  base {bj:.1f}")
                else:
                    self.pulse_label.setText(f"INJ {live_pw:.2f} ms")
        except Exception:
            pass

        # Live crosshair on ignition AND injection maps
        if hasattr(self, "map_view_ign"):
            self.map_view_ign.update_crosshair()
            self.map_view_inj.update_crosshair()
        elif getattr(self, "map_view", None):
            self.map_view.update_crosshair()

        if (self.crosshair_track or self.snap_to_live) and self.connected:
            src = getattr(self, "map_view_ign", None) or getattr(self, "map_view", None)
            if src is not None:
                try:
                    nr, nc = src.live_cell()
                    if nr != self.sel_r or nc != self.sel_c:
                        self.sel_r, self.sel_c = nr, nc
                        if self.snap_to_live and hasattr(src, "build_grid"):
                            src.build_grid()
                except Exception:
                    pass
                    if hasattr(self, "map_view_ign"):
                        self.map_view_ign.build_grid()
                        self.map_view_inj.build_grid()
                    elif getattr(self, "map_view", None):
                        self.map_view.build_grid()
                    self._update_cell_hud()

        # Refresh sync detail timers (hold time etc.) while locked/lost
        if self.connected and self.sync_state in ("locked", "lost", "searching"):
            self._paint_sync_ui()

        # Abort stuck GETMAP so live telemetry is not blocked
        if self._map_dl_mode in ("ADV", "INJ") and getattr(self, "_map_dl_started", 0):
            if time.monotonic() - self._map_dl_started > 3.0:
                self._map_dl_mode = None
                self._map_dl_row = 0
                self.status_label.setText("Map download timeout – live data active")
                self.status_label.setStyleSheet("color:#ffaa66; font-size:12px;")

        # Protocol health
        stale = ""
        if self.connected and self._last_rx_ts:
            age = time.monotonic() - self._last_rx_ts
            if age > 3.0:
                stale = "  STALE"
        # RX stats only in status bar when needed
        if self.connected and stale:
            pass

    # ── SLOW UPDATE (2 s) ───────────────────────────────────────

        # STM32 closed-loop / sequential
        if hasattr(self, "o2_label"):
            cl = "ON" if self.live.get("o2cl") else "off"
            self.o2_label.setText(f"O2:  {self.live.get('o2', 0):.2f} V  CL:{cl}")
            self.trim_label.setText(
                f"STFT: {self.live.get('stft', 0):+.1f}%  LTFT: {self.live.get('ltft', 0):+.1f}%")
            self.knock_label.setText(f"KNOCK: {self.live.get('knock', 0):.0f}")
            self.cam_label.setText(
                f"CAM: {'LOCK' if self.live.get('cam') else '—'}  "
                f"CYL: {int(self.live.get('cyl') or self.cylinders)}")
            if hasattr(self, "cam_dot"):
                self.cam_dot.setStyleSheet(
                    "color:#44ff88; font-size:16px;" if self.live.get("cam")
                    else "color:#444; font-size:16px;")


    def _update_slow(self):
        live = self.live

        # MAP refreshed here (not every 80 ms)
        self.map_label.setText(f"MAP:  {live['map']:.2f} kPa")

        ect = live["ect"]
        eadc = int(live.get("eadc") or 0)
        iadc = int(live.get("iadc") or 0)
        ec  = "#ff3333" if ect > 95 else "#ff9944" if ect > 85 else "#99ffcc"
        self.ect_label.setText(f"ECT:  {ect:.1f} °C  (ADC {eadc})")
        self.ect_label.setStyleSheet(f"color:{ec};")

        self.iat_label.setText(f"IAT:  {live['iat']:.1f} °C  (ADC {iadc})")

        bat = live["bat"]
        bc  = "#ff3333" if bat < 11.5 else "#ffaa44" if bat < 12.5 else "#99ccff"
        if hasattr(self, "bat_label") and self.bat_label.isVisible():
            self.bat_label.setText(f"BAT {bat:.1f}V")
        o2v = float(self.live.get("o2") or 0)
        # NB approx AFR: 14.7 when ~0.45V; crude display from O2 voltage
        afr = 14.7 + (0.45 - o2v) * 8.0
        if afr < 10: afr = 10.0
        if afr > 20: afr = 20.0
        if hasattr(self, "afr_label"):
            self.afr_label.setText(f"AFR {afr:.1f}")
        self.bat_label.setStyleSheet(f"color:{bc};")

        if self.connected:
            # Sync UI driven by state machine; keep fan/fp here
            if live["fan"]:
                self.fan_dot.setStyleSheet("color:#44ccff; font-size:16px;")
                self.fan_lbl.setText("COOLING FAN  ON")
            else:
                self.fan_dot.setStyleSheet("color:#446688; font-size:16px;")
                self.fan_lbl.setText("COOLING FAN  off")

            if live["fp"]:
                self.fp_dot.setStyleSheet("color:#ffaa33; font-size:16px;")
                self.fp_lbl.setText("FUEL PUMP  ON")
            else:
                self.fp_dot.setStyleSheet("color:#554422; font-size:16px;")
                self.fp_lbl.setText("FUEL PUMP  off")

            # RPM present but never locked → hint wheel config
            rpm = int(live.get("rpm") or 0)
            if rpm >= SYNC_SEARCH_RPM and self.sync_state in ("searching", "lost"):
                self.status_label.setText(
                    "Crank turning but not locked – check teeth/gap/sensor")
                self.status_label.setStyleSheet("color:#ffaa66; font-size:12px;")

            # Auto-detect dead link (no frames for a while)
            if self._last_rx_ts and (time.monotonic() - self._last_rx_ts) > 8.0:
                if self.ser_worker.is_open():
                    self.status_label.setText("No telemetry for 8s – check ECU")
                    self.status_label.setStyleSheet("color:#ffaa66; font-size:12px;")
        else:
            self.sync_state = "idle"
            self._paint_sync_ui()
            if getattr(self, "sync_lbl", None) is not None:
                self.sync_lbl.setText("Sync")
            if getattr(self, "sync_detail", None) is not None:
                if getattr(self, "sync_detail", None) is not None:
                    self.sync_detail.setText("—")
            for dot, txt, base in [
                (self.fan_dot,  self.fan_lbl,  "COOLING FAN"),
                (self.fp_dot,   self.fp_lbl,   "FUEL PUMP"),
            ]:
                dot.setStyleSheet("color:#444; font-size:16px;")
                txt.setText(base)

    # ── MAP CONTROL ─────────────────────────────────────────────
    def toggle_load(self):
        self.use_tps = self.chk_tps.isChecked()
        if self.connected:
            self._tx(f"SET:L,0,0,{int(self.use_tps)}\n")
        if hasattr(self, "map_view_ign"):
            self.map_view_ign.build_grid()
            self.map_view_inj.build_grid()
        elif hasattr(self, "map_view") and self.map_view:
            self.map_view.build_grid()

    def set_view(self, v):
        self.view = v
        if hasattr(self, "map_view_ign"):
            self.map_view_ign.build_grid()
            self.map_view_inj.build_grid()
        elif hasattr(self, "map_view") and self.map_view:
            self.map_view.build_grid()
        self._refresh_keys_step_lbl()
        self._update_cell_hud()

    def _refresh_keys_step_lbl(self):
        return
        if not hasattr(self, "keys_step_lbl"):
            return
        if self.view == 0:
            self.keys_step_lbl.setText(
                f"Steps: ±{ADV_STEP_SMALL}°  /  ±{ADV_STEP_LARGE}°")
        else:
            self.keys_step_lbl.setText(
                f"Steps: ±{INJ_STEP_SMALL} ms  /  ±{INJ_STEP_LARGE} ms")

    def _update_cell_hud(self):
        return  # cell HUD removed from live panel (see Help)
        if not hasattr(self, "cell_hud"):
            return
        if self.sel_r < 0:
            self.cell_hud.setText("Cell: –   Val: –")
            return
        r, c = self.sel_r, self.sel_c
        tbl = self.adv if self.view == 0 else self.inj
        val = tbl[r][c]
        unit = "°" if self.view == 0 else " ms"
        val_s = f"{int(val)}" if self.view == 0 else f"{val:.1f}"
        zone = ""
        if self.zone_active:
            b = self.map_view._zone_bounds()
            if b:
                r0, r1, c0, c1 = b
                zone = f"  zone {r1-r0+1}×{c1-c0+1}"
        self.cell_hud.setText(f"Cell: [{r},{c}]  Val: {val_s}{unit}{zone}")

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_F1:
            self.open_key_cheatsheet()
            e.accept()
            return
        key = e.key()
        mods = e.modifiers()

        # Shift+B – toggle trigger-angle adjust mode
        if key == Qt.Key_B and (mods & Qt.ShiftModifier):
            self.trig_adjust_mode = not self.trig_adjust_mode
            self._update_trig_mode_lbl()
            self.status_label.setText(
                "Trigger angle mode ON – use ← / →"
                if self.trig_adjust_mode else "Trigger angle mode OFF")
            self.status_label.setStyleSheet(
                "color:#ffcc44; font-size:12px;" if self.trig_adjust_mode
                else "color:#8090b0; font-size:12px;")
            e.accept()
            return

        # In trig mode, left/right change angle (do not move map cell)
        if self.trig_adjust_mode and key in (Qt.Key_Left, Qt.Key_Right):
            self._nudge_trigger_angle(-1 if key == Qt.Key_Left else +1)
            e.accept()
            return

        # Escape exits trig mode
        if self.trig_adjust_mode and key == Qt.Key_Escape:
            self.trig_adjust_mode = False
            self._update_trig_mode_lbl()
            self.status_label.setText("Trigger angle mode OFF")
            self.status_label.setStyleSheet("color:#8090b0; font-size:12px;")
            e.accept()
            return

        # Forward edit keys to map view when it does not have focus
        if self.map_view:
            self.map_view.setFocus(Qt.OtherFocusReason)
            self.map_view.keyPressEvent(e)
            if e.isAccepted():
                return
        super().keyPressEvent(e)

    def send_map_cell(self, r, c, which=None):
        """Live-write one cell to MCU RAM (immediate). which: None=current view, 0=adv, 1=inj."""
        if not self.connected:
            return False
        v = self.view if which is None else which
        if v == 0:
            val = float(self.adv[r][c])
            ok = self._tx(f"SET:A,{r},{c},{val:.2f}\n")
            tag, unit = "ADV", "°"
        else:
            val = float(self.inj[r][c])
            ok = self._tx(f"SET:I,{r},{c},{val:.2f}\n")
            tag, unit = "INJ", " ms"
        if ok:
            self.status_label.setText(
                f"Live → ECU  {tag}[{r},{c}] = {val:g}{unit}")
            self.status_label.setStyleSheet("color:#66cc99; font-size:12px;")
        return ok


    def upload_maps(self):
        """Push maps via bulk UPLOAD:ADV / UPLOAD:INJ (avoids RX overflow)."""
        if not self.connected:
            self.status_label.setText("Not connected – cannot upload")
            return False
        try:
            if self.ser_worker.ser:
                self.ser_worker.ser.reset_input_buffer()
        except Exception:
            pass
        self._tx("ABORT\n")
        time.sleep(0.08)

        for tag, table, fmt in (
            ("ADV", self.adv, lambda v: str(int(v))),
            ("INJ", self.inj, lambda v: f"{float(v):.1f}"),
        ):
            self.status_label.setText(f"Uploading {tag} map…")
            QApplication.processEvents()
            if not self._tx(f"UPLOAD:{tag}\n"):
                self.status_label.setText(f"UPLOAD:{tag} TX failed")
                return False
            time.sleep(0.05)
            for r in range(ROWS):
                line = ",".join(fmt(table[r][c]) for c in range(COLS)) + "\n"
                if not self._tx(line):
                    self.status_label.setText(f"{tag} row {r} TX failed")
                    return False
                time.sleep(0.025)
                QApplication.processEvents()
            time.sleep(0.12)

        time.sleep(0.1)
        # Stamp fingerprint cell so SAVE / GETFLASH can prove persistence
        try:
            a00 = int(self.adv[0][0])
            i00 = float(self.inj[0][0])
            self._tx(f"SET:A,0,0,{a00}\n")
            time.sleep(0.04)
            self._tx(f"SET:I,0,0,{i00:.1f}\n")
            time.sleep(0.04)
        except Exception:
            pass
        self.status_label.setText(
            "Maps in ECU RAM — click Save Flash to keep after power-cycle")
        self.status_label.setStyleSheet("color:#66cc99; font-size:12px;")
        return True


    def save_maps_to_flash(self):
        """Upload maps then SAVE; wait for OK:SAVE (same as Save Flash)."""
        self.save_eeprom()
        return getattr(self, "_save_result", None) is True


    def _push_engine_cfg(self) -> bool:
        ok = True
        ok = self._tx(f"SET:Y,0,0,{self.cylinders}\n") and ok
        ok = self._tx(
            f"CFG:{self.crank_teeth},{self.crank_missing},{self.trigger_angle}\n") and ok
        ok = self._tx(f"SET:Q,0,0,{self.rpm_limit}\n") and ok
        ok = self._tx(f"SET:N,0,0,{self.fan_setpoint}\n") and ok
        ok = self._tx(f"SET:L,0,0,{int(self.use_tps)}\n") and ok
        return ok

    def _push_sensor_cal(self) -> bool:
        ok = True
        for i in range(CAL_COLS):
            ok = self._tx(f"SET:B,{i},0,{self.tempB[i]:.1f}\n") and ok
            time.sleep(0.015)
            ok = self._tx(f"SET:E,{i},0,{float(self.ectAdc[i]):.0f}\n") and ok
            time.sleep(0.015)
        for i in range(CAL_COLS):
            ok = self._tx(f"SET:J,{i},0,{self.iatB[i]:.1f}\n") and ok
            time.sleep(0.015)
            ok = self._tx(f"SET:K,{i},0,{float(self.iatAdc[i]):.0f}\n") and ok
            time.sleep(0.015)
            ok = self._tx(f"SET:P,{i},0,{self.tpsB[i]:.1f}\n") and ok
            time.sleep(0.015)
        return ok




    def open_setup_wizard(self):
        SetupWizard(self).exec()


    def open_key_cheatsheet(self):
        d = QDialog(self)
        d.setWindowTitle("Keyboard shortcuts")
        d.setMinimumWidth(420)
        lay = QVBoxLayout(d)
        text = (
            "<b>Map editing</b><br>"
            "Arrows — move cell<br>"
            "+ / − — small step &nbsp;&nbsp; PgUp / PgDn — large step<br>"
            "I — interpolate zone<br>"
            "<b>Trigger</b><br>"
            "Shift+B — adjust trigger angle mode, ← → change, Esc exit<br>"
            "<b>General</b><br>"
            "F1 — this cheat-sheet<br>"
            "Live Tune — crosshair follows engine<br>"
            "Snap cell — selection follows live cell<br>"
            "Street / Dyno / Debug — live strip presets<br>"
            "HC — high-contrast mode"
        )
        lab = QLabel(text)
        lab.setTextFormat(Qt.RichText)
        lay.addWidget(lab)
        lay.addWidget(QPushButton("Close", clicked=d.accept))
        d.exec()

    def open_help(self):
        d = QDialog(self)
        d.setMinimumWidth(420)
        lay = QVBoxLayout(d)
        text = (
            "MAP KEYS\n"
            "────────\n"
            "  Arrow keys     Move selected cell\n"
            "  + / -          Small step (ignition / fuel)\n"
            "  Page Up / Down Large step\n"
            "  Live Tune ON   Edits send to ECU immediately\n"
            "  Upload Maps    Push full tables to RAM\n"
            "  Save Flash     Permanent store on MCU\n"
            "\n"
            "ENGINE SETTINGS\n"
            "────────\n"
            "  Wheel profile  Applies + saves to flash\n"
            "  RPM limiter    Hard or soft cut\n"
            "  Throttle map   Pedal → target TPS\n"
            "  Breakpoints    RPM/load axes\n"
            "\n"
            "SYNC\n"
            "────────\n"
            "  Correct wheel type is required for lock.\n"
            "  Cam needed for full sequential 720°.\n"
        )
        lab = QLabel(text)
        lab.setStyleSheet("font-family: Consolas, monospace; color:#c0d0e8;")
        lab.setWordWrap(True)
        lay.addWidget(lab)
        lay.addWidget(QPushButton("Close", clicked=d.accept))
        d.exec()



    def _on_main_tab(self, idx):
        if idx == 0:
            self.map_view = self.map_view_ign
            self.view_mode = 0
        elif idx == 1:
            self.map_view = self.map_view_inj
            self.view_mode = 1




    def _build_motorsport_tab(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.addWidget(lbl("Launch · Anti-lag · Flat-foot  (clutch PB13 active-low)", 11, color="#8090b0"))

        v.addWidget(lbl("Launch control", 12, True))
        lr = QHBoxLayout()
        self.lc_en = QCheckBox("Enable")
        self.lc_rpm = QSpinBox(); self.lc_rpm.setRange(1500, 10000); self.lc_rpm.setValue(4000); self.lc_rpm.setSuffix(" RPM")
        self.lc_tps = QSpinBox(); self.lc_tps.setRange(20, 100); self.lc_tps.setValue(80); self.lc_tps.setSuffix(" % TPS")
        self.lc_bst = QSpinBox(); self.lc_bst.setRange(0, 250); self.lc_bst.setValue(50); self.lc_bst.setSuffix(" kPa")
        lr.addWidget(self.lc_en); lr.addWidget(lbl("Hold", 11)); lr.addWidget(self.lc_rpm)
        lr.addWidget(lbl("Min TPS", 11)); lr.addWidget(self.lc_tps)
        lr.addWidget(lbl("Boost", 11)); lr.addWidget(self.lc_bst)
        v.addLayout(lr)
        v.addWidget(QPushButton("Apply Launch", clicked=lambda: self._tx(
            f"SET:LAUNCH,{1 if self.lc_en.isChecked() else 0},"
            f"{self.lc_rpm.value()},{self.lc_tps.value()},{self.lc_bst.value()}\n")))

        v.addWidget(lbl("Anti-lag", 12, True))
        ar = QHBoxLayout()
        self.als_en = QCheckBox("Enable")
        self.als_ret = QSpinBox(); self.als_ret.setRange(0, 40); self.als_ret.setValue(15); self.als_ret.setSuffix(" °")
        self.als_ex = QCheckBox("Ex VVT"); self.als_ex.setChecked(True)
        self.als_max = QDoubleSpinBox(); self.als_max.setRange(0.5, 30); self.als_max.setValue(3.0); self.als_max.setSuffix(" s")
        self.als_cool = QDoubleSpinBox(); self.als_cool.setRange(0, 60); self.als_cool.setValue(5.0); self.als_cool.setSuffix(" cool")
        self.als_fuel = QDoubleSpinBox(); self.als_fuel.setRange(0, 100); self.als_fuel.setValue(40); self.als_fuel.setSuffix(" %fuel")
        ar.addWidget(self.als_en); ar.addWidget(self.als_ret); ar.addWidget(self.als_ex)
        ar.addWidget(self.als_max); ar.addWidget(self.als_cool); ar.addWidget(self.als_fuel)
        v.addLayout(ar)
        def apply_als():
            self._tx(
                f"SET:ALS,{1 if self.als_en.isChecked() else 0},"
                f"{self.als_ret.value()},{1 if self.als_ex.isChecked() else 0},"
                f"{self.als_max.value():.1f},{self.als_cool.value():.1f}\n")
            self._tx(f"SET:ALSFUEL,{self.als_fuel.value():.0f}\n")
        v.addWidget(QPushButton("Apply Anti-lag", clicked=apply_als))

        v.addWidget(lbl("Flat-foot shift", 12, True))
        fr = QHBoxLayout()
        self.ffs_en = QCheckBox("Enable")
        self.ffs_tps = QSpinBox(); self.ffs_tps.setRange(20, 100); self.ffs_tps.setValue(70); self.ffs_tps.setSuffix(" %")
        self.ffs_ret = QSpinBox(); self.ffs_ret.setRange(0, 40); self.ffs_ret.setValue(20); self.ffs_ret.setSuffix(" °")
        fr.addWidget(self.ffs_en); fr.addWidget(self.ffs_tps); fr.addWidget(self.ffs_ret)
        v.addLayout(fr)
        v.addWidget(QPushButton("Apply FFS", clicked=lambda: self._tx(
            f"SET:FFS,{1 if self.ffs_en.isChecked() else 0},"
            f"{self.ffs_tps.value()},{self.ffs_ret.value()}\n")))
        v.addWidget(lbl("Knock control → Engine Settings → Ignition", 10, color="#8090b0"))
        v.addStretch()
        return w



    def _build_boost_tab(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.setContentsMargins(2, 2, 2, 2)
        self._bst_hint = lbl(
            "Closed-loop: cells = target boost (gauge kPa).  Open-loop: cells = solenoid duty %.",
            11, color="#8090b0")
        v.addWidget(self._bst_hint)
        mode_row = QHBoxLayout()
        mode_row.addWidget(lbl("Control mode", 11))
        self.bst_mode_combo = QComboBox()
        self.bst_mode_combo.addItem("Closed-loop (target kPa)", 0)
        self.bst_mode_combo.addItem("Open-loop (duty %)", 1)
        self.bst_mode_combo.setCurrentIndex(1 if getattr(self, "bst_open_loop", False) else 0)
        mode_row.addWidget(self.bst_mode_combo, 1)
        v.addLayout(mode_row)

        def active_table():
            return self.bst_map_duty if self.bst_open_loop else self.bst_map

        def set_bst(r, c, val):
            val = float(val)
            if self.bst_open_loop:
                val = max(0.0, min(100.0, val))
                self.bst_map_duty[r][c] = val
            else:
                val = max(0.0, min(300.0, val))
                self.bst_map[r][c] = val
            if self.connected:
                self._tx(f"SET:BSTMAP,{r},{c},{val:.0f}\n")

        self.bst_view = HeatMapView(
            self, 8, 8,
            get_table=active_table,
            set_cell=set_bst,
            row_labels=self.bst_tps_lbl,
            col_labels=self.bst_rpm_lbl,
            vmax=200, title="Boost target kPa (gauge)", cell_min=28)
        v.addWidget(self.bst_view, 1)

        def apply_mode():
            ol = bool(self.bst_mode_combo.currentData())
            self.bst_open_loop = ol
            if ol:
                self.bst_view.vmax = 100
                self.bst_view.title = "Wastegate duty %"
                self._bst_hint.setText(
                    "Open-loop: map = solenoid duty % (0–100). No MAP feedback.")
            else:
                self.bst_view.vmax = 200
                self.bst_view.title = "Boost target kPa (gauge)"
                self._bst_hint.setText(
                    "Closed-loop: map = target gauge kPa. PID uses MAP feedback.")
            if self.connected:
                self._tx(f"SET:BSTMODE,{1 if ol else 0}\n")
                self._tx("SET:BSTEN,1\n")
            self.bst_view.build_grid()
            self.status_label.setText(
                "Boost mode: OPEN-LOOP duty" if ol else "Boost mode: CLOSED-LOOP target")

        self.bst_mode_combo.currentIndexChanged.connect(lambda _=None: apply_mode())
        hr = QHBoxLayout()
        hr.addWidget(QPushButton("Apply mode", clicked=apply_mode))
        hr.addWidget(QPushButton("Enable map", clicked=lambda: self._tx("SET:BSTEN,1\n")))
        hr.addWidget(QPushButton("Disable map", clicked=lambda: self._tx("SET:BSTEN,0\n")))
        v.addLayout(hr)
        return w

    def _build_vvt_tab(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.setContentsMargins(2, 2, 2, 2)
        v.addWidget(lbl(
            "VVT closed-loop targets (°) — Intake | Exhaust   mouse wheel / ± to edit, live TX when connected",
            11, color="#8090b0"))
        row = QHBoxLayout()
        def set_in(r, c, val):
            self.vvt_in_map[r][c] = int(val)
            if self.connected:
                self._tx(f"SET:VVTIN,{r},{c},{int(val)}\n")
        def set_ex(r, c, val):
            self.vvt_ex_map[r][c] = int(val)
            if self.connected:
                self._tx(f"SET:VVTEX,{r},{c},{int(val)}\n")
        self.vvt_view_in = HeatMapView(
            self, 8, 8,
            get_table=lambda: self.vvt_in_map,
            set_cell=set_in,
            row_labels=self.vvt_load_lbl,
            col_labels=self.vvt_rpm_lbl,
            vmax=50, title="Intake °", cell_min=28)
        self.vvt_view_ex = HeatMapView(
            self, 8, 8,
            get_table=lambda: self.vvt_ex_map,
            set_cell=set_ex,
            row_labels=self.vvt_load_lbl,
            col_labels=self.vvt_rpm_lbl,
            vmax=50, title="Exhaust °", cell_min=28)
        row.addWidget(self.vvt_view_in, 1)
        row.addWidget(self.vvt_view_ex, 1)
        v.addLayout(row, 1)
        hr = QHBoxLayout()
        hr.addWidget(QPushButton("Enable VVT CL", clicked=lambda: self._tx("SET:VVTCL,1\n")))
        hr.addWidget(QPushButton("Disable VVT CL", clicked=lambda: self._tx("SET:VVTCL,0\n")))
        v.addLayout(hr)
        return w

    def _build_throttle_tab(self):
        w = QWidget(); v = QVBoxLayout(w)
        v.setContentsMargins(2, 2, 2, 2)
        self._throttle_tab_hint = lbl(
            "Throttle map — pedal % (X) × RPM (Y) → target TPS %   (DBW)",
            11, color="#8090b0")
        v.addWidget(self._throttle_tab_hint)
        def set_etb(r, c, val):
            self.etb_map[r][c] = round(float(val), 1)
            if self.connected:
                self._tx(f"SET:ETB,{r},{c},{self.etb_map[r][c]:.1f}\n")
        rpm_lbl = [str(x) for x in self.etb_rpm_bins]
        ped_lbl = [f"{x:g}" for x in self.etb_pedal_bins]
        self.etb_view = HeatMapView(
            self, ETB_ROWS, ETB_COLS,
            get_table=lambda: self.etb_map,
            set_cell=set_etb,
            row_labels=rpm_lbl,
            col_labels=ped_lbl,
            vmax=100, title="Target throttle %", cell_min=20)
        self.etb_view.setEnabled(bool(getattr(self, "dbw_enable", True)))
        v.addWidget(self.etb_view, 1)
        if not getattr(self, "dbw_enable", True):
            self._throttle_tab_hint.setText(
                "DBW disabled — throttle map inactive (idle actuator only)")
        return w

    def _on_map_tab(self, idx):
        """Focus keyboard edits on the visible map."""
        if idx == 0:
            self.map_view = self.map_view_ign
            self.view_mode = 0
        else:
            self.map_view = self.map_view_inj
            self.view_mode = 1

    def open_vvt_maps(self):

        """Edit 8×8 intake / exhaust cam target maps (°), push cells live."""
        d = QDialog(self)
        d.setWindowTitle("VVT Closed-Loop Targets — 8×8 (RPM × Load)")
        d.resize(720, 520)
        lay = QVBoxLayout(d)
        tabs = QTabWidget()
        def make_tab(name, cmd_prefix):
            w = QWidget(); v = QVBoxLayout(w)
            tbl = QTableWidget(8, 8)
            rpm_h = ["800","1200","1800","2500","3500","4500","5500","6500"]
            ld_h = ["10%","20%","30%","45%","55%","70%","85%","100%"]
            tbl.setHorizontalHeaderLabels(rpm_h)
            tbl.setVerticalHeaderLabels(ld_h)
            for r in range(8):
                for c in range(8):
                    base = 10 + max(0, c-2)*3 if 2 <= c <= 5 else 0
                    if r >= 5: base //= 2
                    if name.startswith("Ex"): base //= 2
                    tbl.setItem(r, c, QTableWidgetItem(str(base)))
                tbl.setRowHeight(r, 28)
            def on_change(item):
                if not self.connected: return
                try:
                    val = int(float(item.text()))
                except Exception:
                    return
                self._tx(f"{cmd_prefix},{item.row()},{item.column()},{val}\n")
            tbl.itemChanged.connect(on_change)
            v.addWidget(tbl)
            return w
        tabs.addTab(make_tab("Intake", "SET:VVTIN"), "Intake °")
        tabs.addTab(make_tab("Exhaust", "SET:VVTEX"), "Exhaust °")
        lay.addWidget(tabs)
        hr = QHBoxLayout()
        hr.addWidget(QPushButton("Enable CL", clicked=lambda: self._tx("SET:VVTCL,1\n")))
        hr.addWidget(QPushButton("Disable CL", clicked=lambda: self._tx("SET:VVTCL,0\n")))
        hr.addWidget(QPushButton("Close", clicked=d.accept))
        lay.addLayout(hr)
        d.exec()



    def _load_suggested_sensor_tables(self):
        """Always-on suggested ECT / IAT / BAT tables + compensation %."""
        ect_t = [-40,-20,0,20,40,60,80,90,100,110,120,130,140,150,160]
        ect_a = [3900,3600,3200,2700,2200,1700,1250,1000,800,650,520,420,340,280,230]
        # ECT fuel/ign bias % (positive = more fuel / retard hint stored as comp)
        ect_c = [15,12,8,4,2,0,0,0,-1,-2,-3,-4,-5,-6,-8]
        iat_t = [-40,-20,0,20,40,60,80,90,100,110,120,130,140,150,160]
        iat_a = [3850,3550,3150,2650,2150,1650,1220,980,780,640,510,410,330,270,220]
        # IAT density-style compensation %
        iat_c = [12,10,7,4,2,0,-2,-3,-5,-7,-9,-11,-12,-14,-15]
        for i in range(min(len(self.tempB), len(ect_t))):
            self.tempB[i] = float(ect_t[i])
            self.ectAdc[i] = float(ect_a[i])
            self.tempC[i] = float(ect_c[i]) if i < len(ect_c) else 0.0
        for i in range(min(len(self.iatB), len(iat_t))):
            self.iatB[i] = float(iat_t[i])
            self.iatAdc[i] = float(iat_a[i])
            self.iatC[i] = float(iat_c[i]) if i < len(iat_c) else 0.0
        n = len(self.batB)
        for i in range(n):
            v = 9.0 + i * (7.0 / max(1, n - 1))
            self.batB[i] = round(v, 2)
            # BAT: slight injector dead-time bias at low voltage (comp %)
            self.batC[i] = round(max(0.0, (12.0 - v) * 2.0), 1) if v < 12.0 else 0.0
            self.batAdc[i] = float(int(round(v / 11.0 / 3.3 * 4096)))
        # MAP compensation % mild default
        if hasattr(self, "mapCalC"):
            for i in range(len(self.mapCalC)):
                self.mapCalC[i] = 0.0

    def open_engine_settings(self):
        """Unified Engine Settings — hardware, wheel, compensations, STM32 controls."""
        d = QDialog(self)
        d.setWindowTitle("Engine Settings")
        d.setMinimumSize(640, 560)
        d.resize(720, 640)
        root = QVBoxLayout(d)
        tabs = QTabWidget()
        root.addWidget(tabs, 1)

        def section(title):
            return lbl(title, 12, True)

        # ── Tab: Hardware ─────────────────────────────────────
        hw = QWidget(); hwl = QVBoxLayout(hw)
        hwl.addWidget(section("Cylinders & firing order"))
        er = QHBoxLayout()
        cyl = QSpinBox(); cyl.setRange(1, 6); cyl.setValue(int(getattr(self, "cylinders", 4)))
        fire = QComboBox()
        for name, val in (("1-3-4-2", 0), ("1-2-4-3", 1), ("1-3-2-4", 2)):
            fire.addItem(name, val)
        fire.setCurrentIndex(int(getattr(self, "fire_order", 0)))
        er.addWidget(lbl("Cylinders", 11)); er.addWidget(cyl)
        er.addWidget(lbl("Firing", 11)); er.addWidget(fire, 1)
        hwl.addLayout(er)

        hwl.addWidget(section("Ignition coils"))
        coil = QComboBox()
        coil.addItem("Smart coil (logic-level, default)", 1)
        coil.addItem("Dumb coil (dwell control)", 0)
        coil.setCurrentIndex(0 if getattr(self, "coil_smart", True) else 1)
        hwl.addWidget(coil)

        hwl.addWidget(section("Throttle / idle actuator"))
        dbw = QCheckBox("Drive-by-wire (electronic throttle)")
        dbw.setChecked(bool(getattr(self, "dbw_enable", True)))
        idle_mode = QComboBox()
        idle_mode.addItem("2-wire PWM (H-bridge / motor)", 0)
        idle_mode.addItem("1-wire PWM (solenoid / valve)", 1)
        idle_mode.addItem("Stepper (4-wire)", 2)
        idle_mode.setCurrentIndex(int(getattr(self, "idle_out_mode", 0)))
        idle_mode.setEnabled(not dbw.isChecked())
        dbw.toggled.connect(lambda on: idle_mode.setEnabled(not on))
        hwl.addWidget(dbw)
        hwl.addWidget(lbl("When DBW is off — idle control output:", 11, color="#8090b0"))
        hwl.addWidget(idle_mode)
        hwl.addWidget(lbl("Idle engages only when TPS < 5%.", 10, color="#607080"))

        hwl.addWidget(section("Crank wheel profile"))
        wr = QHBoxLayout()
        wcombo = QComboBox()
        for wid, wname, wt, wm in WHEEL_PROFILES:
            wcombo.addItem(f"{wname}", wid)
        if hasattr(self, "wheel_combo") and self.wheel_combo.count():
            wcombo.setCurrentIndex(self.wheel_combo.currentIndex())
        def apply_w():
            self.wheel_combo = wcombo
            self._apply_wheel_profile()
        wr.addWidget(wcombo, 1)
        wr.addWidget(QPushButton("Apply", clicked=apply_w))
        wcombo.activated.connect(lambda _=None: apply_w())
        hwl.addLayout(wr)

        def apply_hw():
            self.cylinders = int(cyl.value())
            self.fire_order = int(fire.currentData())
            self.coil_smart = bool(coil.currentData())
            self.dbw_enable = dbw.isChecked()
            self.idle_out_mode = int(idle_mode.currentData())
            if self.connected:
                self._tx(f"SET:CYL,{self.cylinders}\n")
                self._tx(f"SET:FIRE,{self.fire_order}\n")
                self._tx(f"SET:COIL,{1 if self.coil_smart else 0}\n")
                self._tx(f"SET:DBW,{1 if self.dbw_enable else 0}\n")
                self._tx(f"SET:IDLEOUT,{self.idle_out_mode}\n")
            self.status_label.setText("Hardware settings applied")
            if hasattr(self, "etb_view"):
                self.etb_view.setEnabled(self.dbw_enable)
        hwl.addWidget(QPushButton("Apply hardware", clicked=apply_hw))
        hwl.addStretch()
        tabs.addTab(hw, "Hardware")


        # ── Tab: Fuel ─────────────────────────────────────────
        wf = QWidget(); wfl = QVBoxLayout(wf)
        wfl.addWidget(section("Injection mode"))
        imr = QHBoxLayout()
        icombo = QComboBox()
        for name, val in (
            ("AUTO (seq if CAM)", 0),
            ("BATCH / semi-seq", 1),
            ("SEQUENTIAL (needs CAM)", 2),
            ("HYBRID: seq → batch @ RPM", 3),
        ):
            icombo.addItem(name, val)
        rpm_spin = QSpinBox(); rpm_spin.setRange(500, 12000); rpm_spin.setValue(3000); rpm_spin.setSuffix(" RPM")
        try:
            rpm_spin.setValue(int(self.live.get("batchrpm") or 3000))
        except Exception:
            pass
        rpm_spin.setEnabled(False)
        icombo.currentIndexChanged.connect(lambda: rpm_spin.setEnabled(icombo.currentData() == 3))
        def apply_inj():
            m = int(icombo.currentData())
            self._tx(f"SET:INJMODE,{m}\n")
            if m == 3:
                self._tx(f"SET:INJBATCHRPM,{int(rpm_spin.value())}\n")
            self.status_label.setText(f"Injection mode {m}")
        imr.addWidget(icombo, 1); imr.addWidget(rpm_spin)
        imr.addWidget(QPushButton("Apply", clicked=apply_inj))
        wfl.addLayout(imr)

        wfl.addWidget(section("Narrowband O2 closed-loop"))
        o2r = QHBoxLayout()
        o2r.addWidget(QPushButton("O2 CL ON", clicked=lambda: self._tx("SET:O2CL,1\n")))
        o2r.addWidget(QPushButton("O2 CL OFF", clicked=lambda: self._tx("SET:O2CL,0\n")))
        o2r.addWidget(QPushButton("Trim Reset", clicked=lambda: self._tx("TRIMRESET\n")))
        wfl.addLayout(o2r)
        lt = QDoubleSpinBox(); lt.setRange(-25, 25); lt.setValue(0)
        st = QDoubleSpinBox(); st.setRange(-25, 25); st.setValue(0)
        ltr = QHBoxLayout()
        ltr.addWidget(lbl("LTFT %", 11)); ltr.addWidget(lt)
        ltr.addWidget(QPushButton("Set LTFT", clicked=lambda: self._tx(f"SET:LTFT,{lt.value():.1f}\n")))
        ltr.addWidget(lbl("STFT %", 11)); ltr.addWidget(st)
        ltr.addWidget(QPushButton("Set STFT", clicked=lambda: self._tx(f"SET:STFT,{st.value():.1f}\n")))
        wfl.addLayout(ltr)
        wfl.addWidget(QPushButton("Read trims (GET:TRIM)", clicked=lambda: self._tx("GET:TRIM\n")))
        wfl.addWidget(lbl("Fuel path: map × LTFT × STFT(if CL) × cold × after-start", 10, color="#8090b0"))

        wfl.addWidget(section("Overrun / decel fuel cut"))
        dfr = QHBoxLayout()
        den = QSpinBox(); den.setRange(0, 1); den.setValue(1)
        dent = QSpinBox(); dent.setRange(800, 6000); dent.setValue(1600); dent.setSuffix(" enter")
        dex = QSpinBox(); dex.setRange(600, 5000); dex.setValue(1200); dex.setSuffix(" exit")
        dfr.addWidget(den); dfr.addWidget(dent); dfr.addWidget(dex)
        dfr.addWidget(QPushButton("Apply OFC", clicked=lambda: self._tx(
            f"SET:OVERRUN,{den.value()},{dent.value()},{dex.value()}\n")))
        wfl.addLayout(dfr)
        wfl.addWidget(lbl(
            "Fuel off when coasting above enter RPM; restore below exit or on throttle.",
            10, color="#8090b0"))
        wfl.addStretch()
        tabs.addTab(wf, "Fuel")


        # ── Tab: Compensations (always suggested) + cold start ─
        cp = QWidget(); cpl = QVBoxLayout(cp)
        self._load_suggested_sensor_tables()
        cpl.addWidget(section("Sensor compensation (always suggested defaults)"))
        cpl.addWidget(lbl(
            "ECT / IAT / BAT tables always use suggested NTC and battery values.\n"
            "Open Calibrate Sensors to capture live ADC or tweak, then Save Calibration.",
            10, color="#8090b0"))
        ref = QLabel(
            "ECT/IAT: −40°C≈3900 · 20°C≈2700 · 80°C≈1250 · 100°C≈800 ADC\n"
            "BAT (~11:1): 12.0V≈1360 · 13.8V≈1560 · 14.4V≈1630 ADC"
        )
        ref.setStyleSheet("color:#a0b0c8; font-family:Consolas,monospace; font-size:11px;")
        ref.setWordWrap(True)
        cpl.addWidget(ref)
        cpl.addWidget(QPushButton("Open Calibrate Sensors…",
            clicked=lambda: self.open_cal_window()))

        cpl.addWidget(hsep())
        cpl.addWidget(section("Cold-start fuel enrichment (% extra fuel vs ECT)"))
        cpl.addWidget(lbl(
            "Extra fuel when cold. 0% above ~70–80°C. Edit → Apply to ECU.",
            10, color="#8090b0"))
        cse_tbl = QTableWidget(len(self.cse_temp), 2)
        cse_tbl.setHorizontalHeaderLabels(["ECT (°C)", "Extra fuel %"])
        for i in range(len(self.cse_temp)):
            cse_tbl.setItem(i, 0, QTableWidgetItem(str(self.cse_temp[i])))
            cse_tbl.setItem(i, 1, QTableWidgetItem(str(self.cse_pct[i])))
            cse_tbl.setRowHeight(i, 26)
        cse_tbl.setMaximumHeight(280)
        cpl.addWidget(cse_tbl)
        def apply_cse():
            for i in range(cse_tbl.rowCount()):
                try:
                    te = float(cse_tbl.item(i, 0).text())
                    pct = float(cse_tbl.item(i, 1).text())
                except Exception:
                    continue
                self.cse_temp[i] = te
                self.cse_pct[i] = max(0.0, min(150.0, pct))
                if self.connected:
                    self._tx(f"SET:CSE,{i},{te:.1f},{pct:.1f}\n")
            self.status_label.setText("Cold-start enrichment applied")
        cpl.addWidget(QPushButton("Apply cold-start enrichment to ECU", clicked=apply_cse))
        cpl.addWidget(hsep())
        cpl.addWidget(section("After-start enrichment decay"))
        cpl.addWidget(lbl(
            "Extra fuel from the moment the engine starts, decaying to 0 over time (only if ECT below threshold).",
            10, color="#8090b0"))
        ase_row = QHBoxLayout()
        ase_pct = QDoubleSpinBox(); ase_pct.setRange(0, 150); ase_pct.setValue(35); ase_pct.setSuffix(" %")
        ase_sec = QDoubleSpinBox(); ase_sec.setRange(1, 120); ase_sec.setValue(25); ase_sec.setSuffix(" s")
        ase_ect = QDoubleSpinBox(); ase_ect.setRange(0, 90); ase_ect.setValue(60); ase_ect.setSuffix(" °C")
        ase_row.addWidget(lbl("Initial", 11)); ase_row.addWidget(ase_pct)
        ase_row.addWidget(lbl("Decay", 11)); ase_row.addWidget(ase_sec)
        ase_row.addWidget(lbl("Max ECT", 11)); ase_row.addWidget(ase_ect)
        cpl.addLayout(ase_row)
        cpl.addWidget(QPushButton("Apply after-start decay",
            clicked=lambda: self._tx(
                f"SET:ASE,{ase_pct.value():.0f},{ase_sec.value():.0f},{ase_ect.value():.0f}\n")))
        cpl.addStretch()
        tabs.addTab(cp, "Compensations")

        # ── Tab: Ignition ─────────────────────────────────────
        ig = QWidget(); igl = QVBoxLayout(ig)
        igl.addWidget(section("Advance limits (signed timing)"))
        limr = QHBoxLayout()
        self.es_ign_max = QSpinBox(); self.es_ign_max.setRange(0, 60)
        self.es_ign_max.setValue(int(getattr(self, "ign_max_adv", 45))); self.es_ign_max.setSuffix(" ° max")
        self.es_ign_min = QSpinBox(); self.es_ign_min.setRange(-30, 0)
        self.es_ign_min.setValue(int(getattr(self, "ign_min_adv", -15))); self.es_ign_min.setSuffix(" ° min")
        limr.addWidget(lbl("Max advance", 11)); limr.addWidget(self.es_ign_max)
        limr.addWidget(lbl("Max retard (min °)", 11)); limr.addWidget(self.es_ign_min)
        igl.addLayout(limr)
        igl.addWidget(QPushButton("Apply limits (live)", clicked=lambda: (
            setattr(self, "ign_max_adv", int(self.es_ign_max.value())),
            setattr(self, "ign_min_adv", int(self.es_ign_min.value())),
            self._tx(f"SET:IGNLIM,{self.es_ign_min.value()},{self.es_ign_max.value()}\n"),
            self.status_label.setText("Ignition limits sent"))))

        igl.addWidget(section("Knock control (Goertzel)"))
        kr = QHBoxLayout()
        self.knk_en = QCheckBox("Enable"); self.knk_en.setChecked(True)
        self.knk_thr = QDoubleSpinBox(); self.knk_thr.setRange(1, 500); self.knk_thr.setValue(50)
        self.knk_step = QDoubleSpinBox(); self.knk_step.setRange(0.5, 5); self.knk_step.setValue(2)
        self.knk_max = QDoubleSpinBox(); self.knk_max.setRange(0, 20); self.knk_max.setValue(12)
        kr.addWidget(self.knk_en); kr.addWidget(lbl("Thr", 10)); kr.addWidget(self.knk_thr)
        kr.addWidget(lbl("Step°", 10)); kr.addWidget(self.knk_step)
        kr.addWidget(lbl("Max°", 10)); kr.addWidget(self.knk_max)
        igl.addLayout(kr)
        igl.addWidget(QPushButton("Apply Knock (live)", clicked=lambda: self._tx(
            f"SET:KNK,{1 if self.knk_en.isChecked() else 0},"
            f"{self.knk_thr.value():.1f},{self.knk_step.value():.1f},{self.knk_max.value():.1f}\n")))
        rpm_h = ["1500","2000","2500","3000","4000","5000","6000","7000"]
        self.knk_thr_tbl = QTableWidget(1, 8)
        self.knk_thr_tbl.setHorizontalHeaderLabels(rpm_h)
        self.knk_thr_tbl.setVerticalHeaderLabels(["Knock thr"])
        for c, v0 in enumerate([40,45,50,55,60,70,80,90]):
            self.knk_thr_tbl.setItem(0, c, QTableWidgetItem(str(v0)))
        self.knk_max_tbl = QTableWidget(1, 8)
        self.knk_max_tbl.setHorizontalHeaderLabels(rpm_h)
        self.knk_max_tbl.setVerticalHeaderLabels(["Knock max °"])
        for c, v0 in enumerate([8,10,12,12,14,14,12,10]):
            self.knk_max_tbl.setItem(0, c, QTableWidgetItem(str(v0)))
        igl.addWidget(self.knk_thr_tbl)
        igl.addWidget(self.knk_max_tbl)
        def push_knk_tbl():
            for c in range(8):
                try:
                    vt = float(self.knk_thr_tbl.item(0, c).text())
                    vm = float(self.knk_max_tbl.item(0, c).text())
                    self._tx(f"SET:KNKTHR,{c},{vt:.0f}\n")
                    self._tx(f"SET:KNKMAX,{c},{vm:.1f}\n")
                except Exception:
                    pass
            self.status_label.setText("Knock tables sent (live)")
        igl.addWidget(QPushButton("Apply knock RPM tables (live)", clicked=push_knk_tbl))
        igl.addStretch()
        tabs.addTab(ig, "Ignition")


        # ── Tab: Controls (was STM32 Control) ─────────────────
        ct = QWidget(); ctl = QVBoxLayout(ct)
        scroll = None
        try:
            from PySide6.QtWidgets import QScrollArea
            scroll = QScrollArea()
            scroll.setWidgetResizable(True)
            inner = QWidget(); ctl = QVBoxLayout(inner)
            scroll.setWidget(inner)
        except Exception:
            inner = ct

        ctl.addWidget(section("RPM limiter / fan"))
        lim = QHBoxLayout()
        rpm_lim = QSpinBox(); rpm_lim.setRange(2000, 12000)
        rpm_lim.setValue(int(getattr(self, "rpm_limit", 7000)))
        fan_sp = QSpinBox(); fan_sp.setRange(60, 120)
        fan_sp.setValue(int(getattr(self, "fan_setpoint", 95))); fan_sp.setSuffix(" °C")
        lim.addWidget(lbl("RPM limit", 11)); lim.addWidget(rpm_lim)
        lim.addWidget(lbl("Fan on", 11)); lim.addWidget(fan_sp)
        fan_hyst = QSpinBox(); fan_hyst.setRange(1, 20)
        fan_hyst.setValue(int(getattr(self, "fan_hyst_c", 5))); fan_hyst.setSuffix(" °C hyst")
        lim.addWidget(fan_hyst)
        ctl.addLayout(lim)
        def apply_lim():
            self.rpm_limit = int(rpm_lim.value())
            self.fan_setpoint = int(fan_sp.value())
            self.fan_hyst_c = int(fan_hyst.value())
            if self.connected:
                self._tx(f"SET:Q,0,0,{self.rpm_limit}\n")
                self._tx(f"SET:FAN,{self.fan_setpoint},{self.fan_hyst_c}\n")
            self.status_label.setText("Limiter / fan (+hysteresis) applied")
        ctl.addWidget(QPushButton("Apply limiter / fan", clicked=apply_lim))

        ctl.addWidget(section("Boost (single target overrides map if higher)"))
        boost_sp = QDoubleSpinBox(); boost_sp.setRange(0, 250); boost_sp.setValue(0)
        br = QHBoxLayout(); br.addWidget(boost_sp)
        br.addWidget(QPushButton("Set Boost kPa", clicked=lambda: self._tx(f"SET:BOOST,{boost_sp.value():.1f}\n")))
        ctl.addLayout(br)
        ctl.addWidget(lbl("Main map: Boost tab (8×8 closed-loop)", 10, color="#8090b0"))


        ctl.addWidget(section("Idle control (DBW / actuator)"))
        idr = QHBoxLayout()
        isp = QSpinBox(); isp.setRange(500, 2000); isp.setValue(850); isp.setSuffix(" RPM")
        idr.addWidget(isp)
        idr.addWidget(QPushButton("Target", clicked=lambda: self._tx(f"SET:IDLE,{isp.value()}\n")))
        idr.addWidget(QPushButton("Enable", clicked=lambda: self._tx("SET:IDLEEN,1\n")))
        idr.addWidget(QPushButton("Disable", clicked=lambda: self._tx("SET:IDLEEN,0\n")))
        ctl.addLayout(idr)

        ctl.addWidget(section("Dashpot"))
        dpr = QHBoxLayout()
        gsp = QDoubleSpinBox(); gsp.setRange(0.05, 2.0); gsp.setValue(0.35)
        dsp = QDoubleSpinBox(); dsp.setRange(0.50, 0.995); dsp.setDecimals(3); dsp.setValue(0.92)
        msp = QDoubleSpinBox(); msp.setRange(5, 40); msp.setValue(25)
        dpr.addWidget(lbl("G", 10)); dpr.addWidget(gsp)
        dpr.addWidget(lbl("D", 10)); dpr.addWidget(dsp)
        dpr.addWidget(lbl("Max", 10)); dpr.addWidget(msp)
        dpr.addWidget(QPushButton("Apply", clicked=lambda: self._tx(
            f"SET:DASHPOT,{gsp.value():.2f},{dsp.value():.3f},{msp.value():.0f}\n")))
        ctl.addLayout(dpr)


        ctl.addWidget(section("TPS endpoints (always)"))
        tpr = QHBoxLayout()
        tpr.addWidget(QPushButton("TPS Closed", clicked=lambda: self._tx("SET:TPS,CLOSED\n")))
        tpr.addWidget(QPushButton("TPS Open", clicked=lambda: self._tx("SET:TPS,OPEN\n")))
        ctl.addLayout(tpr)
        ctl.addWidget(section("Pedal endpoints (DBW only)"))
        pbr = QHBoxLayout()
        pb1 = QPushButton("Pedal Closed", clicked=lambda: self._tx("SET:PED,CLOSED\n"))
        pb2 = QPushButton("Pedal Open", clicked=lambda: self._tx("SET:PED,OPEN\n"))
        pb1.setEnabled(bool(getattr(self, "dbw_enable", True)))
        pb2.setEnabled(bool(getattr(self, "dbw_enable", True)))
        pbr.addWidget(pb1); pbr.addWidget(pb2)
        ctl.addLayout(pbr)

        ctl.addWidget(hsep())
        ctl.addWidget(QPushButton("SAVE flash (maps + LTFT + cal)", clicked=lambda: self._tx("SAVE\n")))
        ctl.addStretch()
        if scroll is not None:
            tabs.addTab(scroll, "Controls")
        else:
            tabs.addTab(ct, "Controls")

        # ── Tab: Tools ────────────────────────────────────────
        tl = QWidget(); tll = QVBoxLayout(tl)
        tll.addWidget(QPushButton("Calibrate Sensors…", clicked=lambda: self.open_cal_window()))
        tll.addWidget(QPushButton("MAP Sensor Wizard…", clicked=lambda: self.open_map_wizard()))
        tll.addWidget(QPushButton("Save breakpoints to PC + ECU", clicked=lambda: self.save_breakpoints()))
        tll.addWidget(QPushButton("Load breakpoints from PC", clicked=lambda: self.load_breakpoints()))
        tll.addWidget(lbl("VVT maps → main VVT tab   ·   Throttle map → main Throttle tab", 10, color="#8090b0"))
        tll.addStretch()
        tabs.addTab(tl, "Tools")

        root.addWidget(QPushButton("Close", clicked=d.accept))
        d.exec()



    def open_map_wizard(self):
        """MAP sensor wizard: atmosphere + sensor min/max → fill scale & ADC table."""
        d = QDialog(self)
        d.setWindowTitle("MAP Sensor Calibration Wizard")
        d.setMinimumWidth(480)
        lay = QVBoxLayout(d)
        lay.addWidget(lbl(
            "Key-on, engine off. Enter atmospheric pressure and sensor range.\n"
            "Capture live ADC at atmosphere — table is filled min→max automatically.",
            11, color="#8090b0"))

        # Preset sensors
        lay.addWidget(lbl("Sensor type", 12, True))
        preset = QComboBox()
        # name, kPa_min, kPa_max
        presets = [
            ("1 bar absolute (10–105 kPa)", 10.0, 105.0),
            ("2 bar absolute (20–200 kPa)", 20.0, 200.0),
            ("2.5 bar absolute (20–250 kPa)", 20.0, 250.0),
            ("3 bar absolute (20–300 kPa)", 20.0, 300.0),
            ("Custom", 10.0, 250.0),
        ]
        for name, lo, hi in presets:
            preset.addItem(name, (lo, hi))
        lay.addWidget(preset)

        gr = QHBoxLayout()
        kmin = QDoubleSpinBox(); kmin.setRange(0, 100); kmin.setValue(10); kmin.setSuffix(" kPa min")
        kmax = QDoubleSpinBox(); kmax.setRange(50, 500); kmax.setValue(105); kmax.setSuffix(" kPa max")
        gr.addWidget(kmin); gr.addWidget(kmax)
        lay.addLayout(gr)

        def on_preset(_=None):
            data = preset.currentData()
            if data and preset.currentText() != "Custom":
                kmin.setValue(data[0]); kmax.setValue(data[1])
                kmin.setEnabled(False); kmax.setEnabled(False)
            else:
                kmin.setEnabled(True); kmax.setEnabled(True)
        preset.currentIndexChanged.connect(on_preset)
        on_preset()

        lay.addWidget(lbl("Atmosphere", 12, True))
        ar = QHBoxLayout()
        atm = QDoubleSpinBox(); atm.setRange(70, 110); atm.setDecimals(1)
        atm.setValue(101.3); atm.setSuffix(" kPa")
        ar.addWidget(lbl("Pressure", 11)); ar.addWidget(atm, 1)
        lay.addLayout(ar)

        live_adc = lbl(f"Live MAP ADC: {int(self.live.get('madc') or 0)}", 14, True, "#ffcc44")
        lay.addWidget(live_adc)
        captured = {"adc": None}

        def tick():
            live_adc.setText(f"Live MAP ADC: {int(self.live.get('madc') or 0)}")
        tmr = QTimer(d); tmr.timeout.connect(tick); tmr.start(200)

        def capture():
            captured["adc"] = float(self.live.get("madc") or 0)
            live_adc.setText(f"Captured ADC @ atm: {int(captured['adc'])}")
            self.status_label.setText(f"MAP atm ADC = {int(captured['adc'])}")
        lay.addWidget(QPushButton("1. Capture ADC at atmosphere", clicked=capture))

        preview = QLabel("")
        preview.setStyleSheet("color:#a0b0c8; font-family:Consolas,monospace; font-size:11px;")
        preview.setWordWrap(True)
        lay.addWidget(preview)

        def build_tables():
            lo = float(kmin.value()); hi = float(kmax.value())
            if hi <= lo + 5:
                preview.setText("Max must be greater than min.")
                return False
            patm = float(atm.value())
            if patm < lo or patm > hi:
                preview.setText("Atmosphere should be within sensor range.")
                return False
            adc_atm = captured["adc"]
            if adc_atm is None:
                adc_atm = float(self.live.get("madc") or 0)
                captured["adc"] = adc_atm
            # Typical transfer: ~0.5V–4.5V of 5V sensor → ~620–3720 on 12-bit 3.3V direct
            # Use linear kPa↔ADC; anchor so ADC(patm) = measured
            adc_lo_typ = 500.0
            adc_hi_typ = 3800.0
            # Ideal ADC at atmosphere from typical curve
            ideal_atm = adc_lo_typ + (patm - lo) / (hi - lo) * (adc_hi_typ - adc_lo_typ)
            offset = adc_atm - ideal_atm
            n = ROWS
            lines = []
            for i in range(n):
                kpa = lo + i * (hi - lo) / max(1, n - 1)
                adc = adc_lo_typ + (kpa - lo) / (hi - lo) * (adc_hi_typ - adc_lo_typ) + offset
                adc = max(0.0, min(4095.0, adc))
                if i < len(self.mapCalB):
                    self.mapCalB[i] = round(kpa, 2)
                if i < len(self.mapAdc):
                    self.mapAdc[i] = round(adc, 0)
                if i < len(self.mapCalC):
                    self.mapCalC[i] = 0.0
                # also align load bins roughly to absolute pressure / 100 for SD
                if i < len(self.map_bins):
                    self.map_bins[i] = round(kpa / 100.0, 3)
                lines.append(f"  [{i:2d}]  {kpa:6.1f} kPa   ADC {adc:5.0f}")
            preview.setText(
                f"Filled {n} rows  (offset {offset:+.0f} ADC from typical curve)\n" +
                "\n".join(lines[:5] + ["  …"] + lines[-3:])
            )
            return True

        def apply_local():
            if build_tables():
                if hasattr(self, "map_view_ign"):
                    try:
                        self.map_view_ign.build_grid()
                        self.map_view_inj.build_grid()
                    except Exception:
                        pass
                self.status_label.setText("MAP scale populated from wizard (local)")

        def apply_ecu():
            if not build_tables():
                return
            if not self.connected:
                self.status_label.setText("MAP table filled locally — connect to send to ECU")
                return
            ok = True
            for i in range(ROWS):
                # SET:MAPCAL,i,kPa,adc  (firmware may ignore unknown — still store local)
                if not self._tx(f"SET:MAPCAL,{i},{self.mapCalB[i]:.2f},{self.mapAdc[i]:.0f}\n"):
                    ok = False
                time.sleep(0.02)
            self.status_label.setText(
                "MAP cal sent to ECU" if ok else "MAP table local; some TX failed")

        br = QHBoxLayout()
        br.addWidget(QPushButton("2. Build scale table", clicked=apply_local))
        br.addWidget(QPushButton("3. Apply to ECU", clicked=apply_ecu))
        br.addWidget(QPushButton("Close", clicked=d.accept))
        lay.addLayout(br)
        d.exec()
        tmr.stop()

    def open_etb_map(self):
        """Edit non-linear pedal→throttle target map (17 pedal × 16 RPM)."""
        d = QDialog(self)
        d.setWindowTitle("Throttle Map – Pedal → Target TPS %")
        d.resize(900, 520)
        lay = QVBoxLayout(d)
        lay.addWidget(lbl(
            "Rows = RPM bands, Cols = pedal %  → cell = target throttle %", 11, color="#8090b0"))
        tbl = QTableWidget(ETB_ROWS, ETB_COLS)
        tbl.setHorizontalHeaderLabels([f"{p:g}%" for p in self.etb_pedal_bins])
        tbl.setVerticalHeaderLabels([f"{r}" for r in self.etb_rpm_bins])
        for r in range(ETB_ROWS):
            for c in range(ETB_COLS):
                tbl.setItem(r, c, QTableWidgetItem(f"{self.etb_map[r][c]:g}"))
            tbl.setRowHeight(r, 22)
        for c in range(ETB_COLS):
            tbl.setColumnWidth(c, 48)
        lay.addWidget(tbl)

        def apply_local():
            for r in range(ETB_ROWS):
                for c in range(ETB_COLS):
                    try:
                        self.etb_map[r][c] = float(tbl.item(r, c).text())
                    except Exception:
                        pass
            self.status_label.setText("Throttle map updated (local)")

        def send_ecu():
            apply_local()
            if not self.connected:
                self.status_label.setText("Not connected")
                return
            for r in range(ETB_ROWS):
                for c in range(ETB_COLS):
                    self._tx(f"SET:ETB,{r},{c},{self.etb_map[r][c]:.1f}\n")
            self.status_label.setText("Throttle map sent to ECU")

        br = QHBoxLayout()
        br.addWidget(QPushButton("Apply", clicked=apply_local))
        br.addWidget(QPushButton("Send to ECU", clicked=send_ecu))
        br.addWidget(QPushButton("Close", clicked=d.accept))
        lay.addLayout(br)
        d.exec()

    def save_breakpoints(self):
        """Persist RPM/load axis to JSON and optionally ECU."""
        data = {
            "rpm_bins": list(self.rpm_bins),
            "map_bins": list(self.map_bins),
            "tps_load": list(self.tps_load),
            "etb_map": self.etb_map,
        }
        try:
            path = BREAKPOINT_FILE
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            self.status_label.setText(f"Breakpoints saved → {path}")
            self.status_label.setStyleSheet("color:#44ff88; font-size:12px;")
        except OSError as e:
            self.status_label.setText(f"Save failed: {e}")
        self.push_breakpoints_to_ecu()

    def load_breakpoints(self):
        try:
            with open(BREAKPOINT_FILE) as f:
                data = json.load(f)
            if "rpm_bins" in data and len(data["rpm_bins"]) == COLS:
                self.rpm_bins = [float(x) for x in data["rpm_bins"]]
            if "map_bins" in data and len(data["map_bins"]) == ROWS:
                self.map_bins = [float(x) for x in data["map_bins"]]
            if "tps_load" in data and len(data["tps_load"]) == ROWS:
                self.tps_load = [float(x) for x in data["tps_load"]]
            if "etb_map" in data:
                self.etb_map = data["etb_map"]
            if hasattr(self, "map_view_ign"):
                self.map_view_ign.build_grid()
                self.map_view_inj.build_grid()
            elif hasattr(self, "map_view") and self.map_view:
                self.map_view.build_grid()
            self.status_label.setText("Breakpoints loaded from PC")
        except Exception as e:
            self.status_label.setText(f"Load failed: {e}")

    def push_breakpoints_to_ecu(self):
        if not self.connected:
            return False
        ok = True
        for i, v in enumerate(self.rpm_bins):
            ok = self._tx(f"SET:RPMB,{i},{float(v):.1f}\n") and ok
        for i, v in enumerate(self.map_bins):
            ok = self._tx(f"SET:MAPB,{i},{float(v):.4f}\n") and ok
        self.status_label.setText(
            "Breakpoints pushed to ECU" if ok else "Breakpoint push had TX errors")
        return ok



    def _push_undo(self, view, r, c, old_val):
        self._undo_stack.append((view, r, c, old_val))
        if len(self._undo_stack) > 200:
            self._undo_stack = self._undo_stack[-200:]


    def undo_edit(self):
        if not getattr(self, "_undo_stack", None):
            self.status_label.setText("Nothing to undo")
            return
        view, r, c, old = self._undo_stack.pop()
        tbl = self.adv if view == 0 else self.inj
        tbl[r][c] = old
        self.view = view
        if hasattr(self, "map_view_ign"):
            self.map_view_ign.build_grid()
            self.map_view_inj.build_grid()
        elif getattr(self, "map_view", None):
            self.map_view.build_grid()
        if self.connected:
            if view == 0:
                self._tx(f"SET:A,{r},{c},{int(old)}\n")
            else:
                self._tx(f"SET:I,{r},{c},{float(old):.2f}\n")
        self.status_label.setText(f"Undo [{r},{c}] → {old}")





    def open_dashboard(self):
        """Full-screen dash with WebGL gauges (QWebEngine) or Qt fallback."""
        html_path = Path(__file__).resolve().parent / "dashboard_webgl.html"
        if not html_path.is_file():
            alt = Path(__file__).resolve().parent.parent / "dashboard_webgl.html"
            if alt.is_file():
                html_path = alt

        has_web = bool(HAS_WEBENGINE and QWebEngineView is not None)

        d = QDialog(self)
        d.setWindowTitle("TorquEFI Dashboard")
        d.setWindowState(Qt.WindowFullScreen)

        if has_web and html_path.is_file():
            lay = QVBoxLayout(d)
            lay.setContentsMargins(0, 0, 0, 0)
            lay.setSpacing(0)
            view = QWebEngineView()
            try:
                if QWebEngineSettings is not None:
                    s = view.settings()
                    s.setAttribute(QWebEngineSettings.WebAttribute.WebGLEnabled, True)
                    s.setAttribute(
                        QWebEngineSettings.WebAttribute.Accelerated2dCanvasEnabled, True)
                    s.setAttribute(
                        QWebEngineSettings.WebAttribute.LocalContentCanAccessFileUrls, True)
            except Exception:
                pass

            timer = QTimer(d)
            timer.setInterval(50)

            def tick():
                live = self.live
                inj = float(live.get("pw") or live.get("inj") or 0)
                payload = {
                    "rpm": int(live.get("rpm") or 0),
                    "tps": int(live.get("tps") or 0),
                    "map": float(live.get("map") or 0),
                    "ign": float(live.get("ign") or 0),
                    "o2": float(live.get("o2") or 0),
                    "pw": inj,
                    "vvt1": float(live.get("vvt1") or 0),
                    "vvt2": float(live.get("vvt2") or 0),
                    "sync": int(live.get("sync") or 0),
                    "cam": int(live.get("cam") or 0),
                }
                js = "try{updateDash(" + json.dumps(payload) + ");}catch(e){}"
                view.page().runJavaScript(js)

            def on_loaded(ok):
                if not ok:
                    # Fall back if page failed to load
                    timer.stop()
                    self.status_label.setText("Dashboard HTML failed to load — Qt gauges")
                    # Replace web view content by closing and using fallback
                    d.close()
                    self._open_dashboard_qt_fallback()
                    return
                timer.timeout.connect(tick)
                timer.start()
                tick()

            view.loadFinished.connect(on_loaded)
            view.setUrl(QUrl.fromLocalFile(str(html_path.resolve())))
            lay.addWidget(view, 1)

            bar = QHBoxLayout()
            btn_close = QPushButton("Close  ·  Esc")
            btn_close.setStyleSheet(
                "background:#152030;color:#b0c4e0;border:1px solid #2a4060;"
                "border-radius:8px;padding:8px 16px;")
            btn_close.clicked.connect(d.accept)
            bar.addStretch()
            bar.addWidget(btn_close)
            foot = QWidget()
            foot.setStyleSheet("background:#0a0e14;")
            fl = QVBoxLayout(foot)
            fl.setContentsMargins(8, 4, 8, 8)
            fl.addLayout(bar)
            lay.addWidget(foot)

            d.exec()
            timer.stop()
            return

        self._open_dashboard_qt_fallback(d)

    def _open_dashboard_qt_fallback(self, d=None):
        """Qt-only gauges when PySide6-WebEngine is not installed."""
        from PySide6.QtWidgets import QGridLayout, QSizePolicy

        if d is None:
            d = QDialog(self)
            d.setWindowTitle("TorquEFI Dashboard")
            d.setWindowState(Qt.WindowFullScreen)
        d.setStyleSheet(
            "QDialog{background:#080b12;color:#e8eef8;}"
            "QLabel#gaugeTitle{color:#5a6e88;font-size:11px;font-weight:700;letter-spacing:3px;}"
            "QFrame#card{background:#101620;border:1px solid #243044;border-radius:12px;}"
            "QFrame#rpmCard{background:#121018;border:2px solid #2a3820;border-radius:16px;}"
            "QPushButton{background:#152030;color:#b0c4e0;border:1px solid #2a4060;"
            "border-radius:8px;padding:8px 16px;}")
        root = QVBoxLayout(d)
        root.setContentsMargins(12, 10, 12, 10)
        values = {}

        def card(title, color, big=False):
            f = QFrame()
            f.setObjectName("rpmCard" if big else "card")
            f.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            cl = QVBoxLayout(f)
            t = QLabel(title)
            t.setObjectName("gaugeTitle")
            t.setAlignment(Qt.AlignCenter)
            v = QLabel("—")
            v.setAlignment(Qt.AlignCenter)
            v.setStyleSheet(
                f"color:{color};font-size:{64 if big else 36}px;font-weight:800;")
            cl.addWidget(t)
            cl.addWidget(v, 1)
            values[title] = v
            return f

        root.addWidget(card("RPM", "#ffd000", True), 3)
        g = QGridLayout()
        g.setSpacing(8)
        for i, (name, col) in enumerate([
            ("TPS", "#6dffb0"), ("MAP", "#5ec0ff"), ("IGN", "#5eb8ff"), ("AFR", "#ff8fd0"),
            ("INJ", "#5aff9a"), ("VVT IN", "#c0a0ff"), ("VVT EX", "#90b8ff"),
        ]):
            g.addWidget(card(name, col), i // 4, i % 4)
        root.addLayout(g, 4)
        root.addWidget(lbl(
            "For WebGL gauges: pip install PySide6-WebEngine", 11, color="#8090b0"))
        btn = QPushButton("Close")
        btn.clicked.connect(d.accept)
        root.addWidget(btn)

        def tick():
            live = self.live
            values["RPM"].setText(str(int(live.get("rpm") or 0)))
            values["TPS"].setText(str(int(live.get("tps") or 0)))
            values["MAP"].setText(f"{float(live.get('map') or 0):.1f}")
            values["IGN"].setText(f"{float(live.get('ign') or 0):.0f}")
            o2 = float(live.get("o2") or 0)
            afr = 18 - (o2 / 0.9) * 6 if o2 > 0.01 else 0
            values["AFR"].setText(f"{afr:.1f}" if o2 > 0.01 else "—")
            inj = float(live.get("pw") or 0)
            inj = inj / 1000 if inj > 100 else inj
            values["INJ"].setText(f"{inj:.2f}")
            values["VVT IN"].setText(f"{float(live.get('vvt1') or 0):.0f}")
            values["VVT EX"].setText(f"{float(live.get('vvt2') or 0):.0f}")

        timer = QTimer(d)
        timer.timeout.connect(tick)
        timer.start(80)
        tick()
        d.exec()


    def connection_wizard(self):
        """Auto-pick STM CDC port, open, test GETCFG."""
        self._refresh_ports()
        ports = list(serial.tools.list_ports.comports())
        pick = None
        for p in ports:
            d = (p.device + " " + (p.description or "") + " " + (p.manufacturer or "")).lower()
            if any(x in d for x in ("stm", "acm", "usbmodem", "serial", "cdc", "stmicro")):
                pick = p.device
                break
        if not pick and ports:
            pick = ports[0].device
        if not pick:
            self.status_label.setText("No serial ports found")
            self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
            return
        # select in combo
        for i in range(self.port_combo.count()):
            if pick in self.port_combo.itemText(i):
                self.port_combo.setCurrentIndex(i)
                break
        self.status_label.setText(f"Wizard: opening {pick}…")
        QApplication.processEvents()
        ok, msg = self.ser_worker.connect(pick)
        self._set_conn(ok)
        if not ok:
            self.status_label.setText(f"Wizard failed: {msg}")
            return
        time.sleep(0.8)
        if self._tx("GETCFG\n"):
            self._tx("GET:IGNLIM\n")
            self._tx("GET:FAN\n")
            self.status_label.setText(f"Wizard OK — {pick}  (GETCFG sent)")
            self.status_label.setStyleSheet("color:#44ff88; font-size:12px;")
            self._save_ui_config()
        else:
            self.status_label.setText("Port open but GETCFG TX failed")
            self.status_label.setStyleSheet("color:#ffaa66; font-size:12px;")

    def _ui_config_path(self):
        from pathlib import Path
        return Path(__file__).resolve().parent / UI_CONFIG_FILE

    def _load_ui_config(self):
        try:
            path = self._ui_config_path()
            if not path.is_file():
                return
            data = json.loads(path.read_text(encoding="utf-8"))
            g = data.get("geometry")
            if g and len(g) == 4:
                self.setGeometry(*g)
            if isinstance(data.get("strip_vis"), dict):
                self.strip_vis.update(data["strip_vis"])
                self.strip_vis["rpm"] = self.strip_vis["tps"] = self.strip_vis["map"] = True
                try:
                    self._apply_strip_vis()
                except Exception:
                    pass
            port = data.get("port")
            if port:
                for i in range(self.port_combo.count()):
                    if port in self.port_combo.itemText(i):
                        self.port_combo.setCurrentIndex(i)
                        break
        except Exception:
            pass

    def _save_ui_config(self):
        try:
            g = self.geometry()
            port = self.port_combo.currentText().split()[0] if self.port_combo.count() else ""
            data = {
                "geometry": [g.x(), g.y(), g.width(), g.height()],
                "port": port,
            }
            self._ui_config_path().write_text(json.dumps(data, indent=2), encoding="utf-8")
        except Exception:
            pass

    def closeEvent(self, e):
        try:
            self._save_ui_config()
        except Exception:
            pass
        super().closeEvent(e)


    def open_stm32_control(self):
        """Deprecated — controls live under Engine Settings."""
        self.open_engine_settings()


    def save_eeprom(self):
        """Upload maps to RAM, SAVE to flash, confirm via OK:SAVE or GETCRC."""
        if not self.connected:
            self.status_label.setText("Not connected – cannot save")
            self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
            return
        self.status_label.setText("Uploading maps…")
        self.status_label.setStyleSheet("color:#ffcc44; font-size:12px;")
        QApplication.processEvents()

        if not self.upload_maps():
            return

        time.sleep(0.3)
        self._tx("ABORT\n")
        time.sleep(0.1)
        self._save_result = None
        self._save_msg = ""
        self.status_label.setText("Saving to flash (may pause USB 1–2s)…")
        QApplication.processEvents()
        if not self._tx("SAVE\n"):
            self.status_label.setText("SAVE TX failed")
            self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
            return

        t0 = time.monotonic()
        while time.monotonic() - t0 < 12.0:
            QApplication.processEvents()
            if self._save_result is not None:
                break
            time.sleep(0.05)

        if self._save_result is True:
            # Confirm RAM fingerprint still matches what we sent
            try:
                self._tx("GETMAPSUM\n")
                time.sleep(0.15)
                QApplication.processEvents()
            except Exception:
                pass
            self.status_label.setText(self._save_msg or "Flash save OK")
            self.status_label.setStyleSheet("color:#44ff88; font-size:12px;")
            return
        if self._save_result is False:
            self.status_label.setText(self._save_msg or "Flash save FAILED")
            self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
            return

        # No OK:SAVE (USB may have stalled) — ask for CRC as second check
        self._tx("GETCRC\n")
        t1 = time.monotonic()
        while time.monotonic() - t1 < 1.5:
            QApplication.processEvents()
            if self._save_result is True:
                self.status_label.setText(
                    (self._save_msg or "Flash OK") + " (via GETCRC)")
                self.status_label.setStyleSheet("color:#44ff88; font-size:12px;")
                return
            time.sleep(0.05)

        self.status_label.setText(
            "No SAVE reply — power-cycle ECU and Read from ECU to verify")
        self.status_label.setStyleSheet("color:#ffaa66; font-size:12px;")


    def export_tune(self):
        from PySide6.QtWidgets import QFileDialog
        path, _ = QFileDialog.getSaveFileName(self, "Export Tune", "", "*.txt")
        if path:
            try:
                with open(path, "w") as f:
                    f.write("# Ignition map\n")
                    for row in self.adv:
                        f.write(",".join(f"{int(v)}" for v in row) + "\n")
                    f.write("# Injector map\n")
                    for row in self.inj:
                        f.write(",".join(f"{v:.1f}" for v in row) + "\n")
                self.status_label.setText(f"Saved {path}")
            except OSError as e:
                self.status_label.setText(f"Export failed: {e}")
                self.status_label.setStyleSheet("color:#ff8866; font-size:12px;")

    def open_cal_window(self):
        CalDialog(self).exec()


# ──────────────────────────────────────────────
#  CALIBRATION DIALOG  (TPS wizard lives on TPS tab)
# ──────────────────────────────────────────────
class CalDialog(QDialog):
    _TABS = [
        ("tempB",   "tempC",   "ectAdc", CAL_COLS, "B", "T"),
        ("tpsB",    None,      "tpsB",   CAL_COLS, "P", None),
        ("batB",    "batC",    "batAdc", CAL_COLS, "BATV", "BATC"),
        ("iatB",    "iatC",    "iatAdc", CAL_COLS, None, None),
        ("mapCalB", "mapCalC", "mapAdc",  ROWS, None, None),  # 15 load rows
    ]
    _TAB_NAMES = ["ECT","TPS","BAT","IAT","MAP"]
    _ADC_LIVE  = ["eadc","tadc","badc","iadc","madc"]

    def __init__(self, parent):
        super().__init__(parent)
        self.p = parent
        self.setWindowTitle("Sensor Calibration – Matches Arduino")
        self.setGeometry(180, 100, 860, 700)
        lay = QVBoxLayout(self)

        self.tabs   = QTabWidget()
        self.tables = []
        for i, cfg in enumerate(self._TABS):
            if self._TAB_NAMES[i] == "TPS":
                page = self._make_tps_page(i, cfg)
                self.tabs.addTab(page, "TPS")
            else:
                t = self._make_table(i, cfg)
                self.tables.append(t)
                self.tabs.addTab(t, self._TAB_NAMES[i])
        # tables index: 0=ECT, 1=BAT, 2=IAT, 3=MAP  (TPS page holds its own table)
        lay.addWidget(self.tabs)

        btns = QHBoxLayout()
        for txt, slot in [("Capture Live ADC",           self._capture),
                          ("Save Calibration to EEPROM", self._save),
                          ("Close",                      self.close)]:
            btns.addWidget(QPushButton(txt, clicked=slot))
        lay.addLayout(btns)

        self._tps_closed_adc = 0
        self._tps_tick = QTimer(self)
        self._tps_tick.timeout.connect(self._tps_live_tick)
        self._tps_tick.start(200)

    def _make_table(self, idx, cfg):
        v1, v2, adc, rows, *_ = cfg
        tbl = QTableWidget(rows, 3)
        name = self._TAB_NAMES[idx] if idx < len(self._TAB_NAMES) else "?"
        if name == "BAT":
            tbl.setHorizontalHeaderLabels(["Voltage (V)", "Comp %", "ADC"])
        elif name in ("ECT", "IAT"):
            tbl.setHorizontalHeaderLabels(["Temp (°C)", "Comp %", "ADC"])
        elif name == "MAP":
            tbl.setHorizontalHeaderLabels(["Load / kPa", "Compensation", "ADC"])
        else:
            tbl.setHorizontalHeaderLabels(["Value", "Compensation", "ADC"])
        tbl.itemChanged.connect(lambda item, i=idx: self._on_edit(item, i))
        for i in range(rows):
            fmt = f"{getattr(self.p, v1)[i]:.1f}" if name in ("ECT", "IAT", "BAT") \
                  else f"{getattr(self.p, v1)[i]:.2f}"
            tbl.setItem(i, 0, QTableWidgetItem(fmt))
            tbl.setItem(i, 1, QTableWidgetItem(
                f"{getattr(self.p, v2)[i]:.2f}" if v2 else "—"))
            tbl.setItem(i, 2, QTableWidgetItem(
                str(int(getattr(self.p, adc)[i]))))
        return tbl

    def _make_tps_page(self, idx, cfg):
        """TPS tab = calibration table + closed/full throttle wizard."""
        page = QWidget()
        v = QVBoxLayout(page)
        v.setSpacing(8)

        wiz = QGroupBox("TPS Wizard – Closed / Full throttle")
        wl = QVBoxLayout(wiz)
        self._tps_live_lbl = lbl("Live ADC: –––", 16, True, "#ffcc44")
        self._tps_closed_lbl = lbl("Closed ADC: –––", 12, color="#aaaaaa")
        self._tps_full_lbl = lbl("Full ADC: –––", 12, color="#aaaaaa")
        wl.addWidget(self._tps_live_lbl)
        wl.addWidget(self._tps_closed_lbl)
        wl.addWidget(self._tps_full_lbl)
        row = QHBoxLayout()
        row.addWidget(QPushButton("1. Store CLOSED", clicked=self._tps_store_closed))
        row.addWidget(QPushButton("2. Store FULL + apply", clicked=self._tps_store_full))
        wl.addLayout(row)
        wl.addWidget(lbl(
            "Park throttle closed → Store CLOSED.  Floor throttle → Store FULL.  "
            "Builds linear TPS ADC map (11 points).", 10, color="#8090b0"))
        v.addWidget(wiz)

        self._tps_table = self._make_table(idx, cfg)
        self.tables.insert(1, self._tps_table)  # keep index aligned: 0 ECT, 1 TPS, ...
        v.addWidget(self._tps_table)
        return page

    def _tps_live_tick(self):
        if not hasattr(self, "_tps_live_lbl"):
            return
        self._tps_live_lbl.setText(f"Live ADC: {int(self.p.live.get('tadc', 0))}")

    def _tps_store_closed(self):
        self._tps_closed_adc = float(self.p.live.get("tadc", 0))
        self._tps_closed_lbl.setText(f"Closed ADC: {int(self._tps_closed_adc)}")
        self.p.status_label.setText("Closed TPS ADC stored")

    def _tps_store_full(self):
        if not self._tps_closed_adc:
            self.p.status_label.setText("Store CLOSED throttle first")
            return
        full = float(self.p.live.get("tadc", 0))
        self._tps_full_lbl.setText(f"Full ADC: {int(full)}")
        span = full - self._tps_closed_adc
        if abs(span) < 10:
            self.p.status_label.setText("TPS span too small – check sensor wiring")
            return
        for i in range(CAL_COLS):
            self.p.tpsB[i] = self._tps_closed_adc + (i / 10.0) * span
            if self._tps_table.item(i, 0):
                self._tps_table.item(i, 0).setText(f"{self.p.tpsB[i]:.2f}")
            # ADC column mirrors value for TPS linear map
            if self._tps_table.item(i, 2):
                self._tps_table.item(i, 2).setText(str(int(self.p.tpsB[i])))
        # Send to ECU if connected
        if self.p.connected:
            sw = self.p.ser_worker
            for i in range(CAL_COLS):
                sw.send(f"SET:P,{i},0,{self.p.tpsB[i]:.1f}\n")
                time.sleep(0.03)
            sw.send("SAVE\n")
            self.p.status_label.setText("TPS map applied and saved to EEPROM")
        else:
            self.p.status_label.setText("TPS map updated locally (connect to send)")

    def _tab_data_index(self):
        """Map current UI tab → _TABS index (0..4)."""
        return self.tabs.currentIndex()

    def _on_edit(self, item, idx):
        row, col = item.row(), item.column()
        text = (item.text() or "").strip().replace(",", ".")
        try:
            val = float(text)
        except Exception:
            self.p.status_label.setText(f"Invalid number in row {row + 1}, col {col + 1}")
            self.p.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
            return
        if val != val:  # NaN
            self.p.status_label.setText("Not a valid number")
            self.p.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
            return
        v1, v2, adc, *_ = self._TABS[idx]
        name = self._TAB_NAMES[idx] if idx < len(self._TAB_NAMES) else ""
        # Range checks by column / sensor type
        if col == 2:  # ADC
            if val < 0 or val > 1023:
                self.p.status_label.setText(f"ADC must be 0–1023 (got {val:g})")
                self.p.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
                return
            val = float(int(round(val)))
        elif col == 0 and name in ("ECT", "IAT"):
            if val < -80 or val > 200:
                self.p.status_label.setText(f"Temperature out of range (−80…200 °C)")
                self.p.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
                return
        elif col == 0 and name == "TPS":
            if val < 0 or val > 1023:
                self.p.status_label.setText("TPS breakpoint should be a valid ADC-like value")
                self.p.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
                return
        if col == 0:
            getattr(self.p, v1)[row] = val
        elif col == 1 and v2:
            getattr(self.p, v2)[row] = val
        elif col == 2:
            getattr(self.p, adc)[row] = val
        self.p.status_label.setStyleSheet("color:#8090b0; font-size:12px;")

    def _capture(self):
        idx = self._tab_data_index()
        tbl = self.tables[idx] if idx < len(self.tables) else None
        if tbl is None:
            self.p.status_label.setText("No calibration table on this tab")
            return
        row = tbl.currentRow()
        if row < 0:
            self.p.status_label.setText("Select a table row before Capture Live ADC")
            self.p.status_label.setStyleSheet("color:#ffaa66; font-size:12px;")
            return
        adc_key = self._TABS[idx][2]
        live_adc = self.p.live.get(self._ADC_LIVE[idx], 0)
        try:
            live_adc = float(live_adc)
        except Exception:
            self.p.status_label.setText("Live ADC unavailable")
            return
        if live_adc < 0 or live_adc > 1023:
            self.p.status_label.setText(f"Live ADC out of range: {live_adc}")
            self.p.status_label.setStyleSheet("color:#ff8866; font-size:12px;")
            return
        getattr(self.p, adc_key)[row] = live_adc
        if tbl.item(row, 2):
            tbl.item(row, 2).setText(str(int(live_adc)))
        self.p.status_label.setText(f"Captured row {row} ADC={int(live_adc)}")
        self.p.status_label.setStyleSheet("color:#8090b0; font-size:12px;")

    def _save(self):
        if not self.p.connected:
            self.p.status_label.setText("Not connected – cannot save cal")
            return
        sw = self.p.ser_worker
        ok = True
        # ECT: temperature breakpoints + ADC (B=temp °C, E=ADC, optional T=comp)
        for i in range(CAL_COLS):
            if not sw.send(f"SET:B,{i},0,{self.p.tempB[i]:.1f}\n"):
                ok = False
                break
            time.sleep(0.02)
            if not sw.send(f"SET:E,{i},0,{self.p.ectAdc[i]:.0f}\n"):
                ok = False
                break
            time.sleep(0.02)
            if not sw.send(f"SET:T,{i},0,{self.p.tempC[i]:.1f}\n"):
                ok = False
                break
            time.sleep(0.02)
        # IAT breakpoints
        if ok:
            for i in range(CAL_COLS):
                if not sw.send(f"SET:J,{i},0,{self.p.iatB[i]:.1f}\n"):
                    ok = False
                    break
                time.sleep(0.02)
                if not sw.send(f"SET:K,{i},0,{self.p.iatAdc[i]:.0f}\n"):
                    ok = False
                    break
                time.sleep(0.02)
        # BAT compensation rows (voltage, ADC, scale)
        if ok:
            for i in range(CAL_COLS):
                if not sw.send(f"SET:BATV,{i},{self.p.batB[i]:.2f}\n"):
                    ok = False
                    break
                time.sleep(0.02)
                if not sw.send(f"SET:BATA,{i},{self.p.batAdc[i]:.0f}\n"):
                    ok = False
                    break
                time.sleep(0.02)
                if not sw.send(f"SET:BATC,{i},{self.p.batC[i]:.3f}\n"):
                    ok = False
                    break
                time.sleep(0.02)
        # TPS
        if ok:
            for i in range(CAL_COLS):
                if not sw.send(f"SET:P,{i},0,{self.p.tpsB[i]:.1f}\n"):
                    ok = False
                    break
                time.sleep(0.02)
        if ok:
            ok = sw.send("SAVE\n")
        self.p.status_label.setText(
            "Calibration saved to EEPROM" if ok else "Calibration save failed (TX error)")

    def closeEvent(self, e):
        if hasattr(self, "_tps_tick"):
            self._tps_tick.stop()
        super().closeEvent(e)


# ──────────────────────────────────────────────
#  ENTRY POINT
# ──────────────────────────────────────────────
if __name__ == "__main__":
    # Required for Qt WebEngine / WebGL (must be before QApplication)
    try:
        QApplication.setAttribute(Qt.AA_ShareOpenGLContexts, True)
    except Exception:
        pass
    # WebEngine already imported at module level when available (HAS_WEBENGINE)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(DARK_STYLE)
    window = ECUTuner()
    window.show()
    sys.exit(app.exec())
