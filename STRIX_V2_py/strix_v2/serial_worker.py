"""Background serial RX thread for STRIX V2."""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

import serial
import serial.tools.list_ports
from PySide6.QtCore import QObject, Signal


def list_serial_ports() -> list[str]:
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append(p.device)
    # Prefer STM CDC-looking names first
    def score(name: str) -> int:
        u = name.upper()
        if "STM" in u or "USB" in u or "CDC" in u:
            return 0
        if u.startswith("COM") or "TTYACM" in u or "TTYUSB" in u:
            return 1
        return 2
    return sorted(ports, key=score)


class SerialWorker(QObject):
    line_received = Signal(str)
    status = Signal(str)
    connected_changed = Signal(bool)

    def __init__(self, baud: int = 115200):
        super().__init__()
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self._rx_thread: Optional[threading.Thread] = None
        self._stop = threading.Event()
        self._lock = threading.Lock()

    @property
    def is_open(self) -> bool:
        return bool(self.ser and self.ser.is_open)

    def connect_port(self, port: str) -> tuple[bool, str]:
        self.disconnect()
        try:
            # Windows COM10+ needs \\.\ prefix
            dev = port
            if port.upper().startswith("COM"):
                try:
                    n = int(port[3:])
                    if n >= 10:
                        dev = r"\\.\\" + port
                except ValueError:
                    pass
            # Do not toggle DTR/RTS — STM32 CDC resets the MCU on DTR.
            self.ser = serial.Serial(
                dev, self.baud, timeout=0.05, write_timeout=0.4,
                dsrdtr=False, rtscts=False,
            )
            try:
                self.ser.dtr = False
                self.ser.rts = False
            except Exception:
                pass
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except Exception:
                pass
            time.sleep(0.25)
            self._stop.clear()
            self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self._rx_thread.start()
            self.connected_changed.emit(True)
            self.status.emit(f"Connected {port} @ {self.baud}")
            return True, "ok"
        except Exception as e:
            self.ser = None
            self.connected_changed.emit(False)
            return False, str(e)

    def disconnect(self) -> None:
        self._stop.set()
        if self._rx_thread and self._rx_thread.is_alive():
            self._rx_thread.join(timeout=0.5)
        self._rx_thread = None
        with self._lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None
        self.connected_changed.emit(False)

    def send(self, text: str) -> bool:
        if not self.is_open:
            return False
        data = text.encode("ascii", errors="ignore")
        if not data.endswith(b"\n"):
            data += b"\n"
        with self._lock:
            try:
                if not self.ser or not self.ser.is_open:
                    return False
                self.ser.write(data)
                return True
            except Exception as e:
                msg = str(e)
                # Windows: ClearCommError / PermissionError 13 / command 22
                # after CDC re-enum or flash erase — do not hammer the dead handle
                self.status.emit(f"TX fail: {msg}")
                if any(s in msg.lower() for s in (
                    "clearcomm", "permission", "13", "command 22",
                    "access is denied", "device", "oserror",
                )):
                    try:
                        self.ser.close()
                    except Exception:
                        pass
                    self.ser = None
                return False

    def _rx_loop(self) -> None:
        buf = bytearray()
        while not self._stop.is_set():
            try:
                if not self.ser or not self.ser.is_open:
                    time.sleep(0.05)
                    continue
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf.extend(chunk)
                while True:
                    nl = buf.find(b"\n")
                    if nl < 0:
                        break
                    raw = bytes(buf[:nl]).replace(b"\r", b"")
                    del buf[: nl + 1]
                    try:
                        line = raw.decode("ascii", errors="ignore").strip()
                    except Exception:
                        continue
                    if line:
                        self.line_received.emit(line)
                if len(buf) > 8192:
                    del buf[:-4096]
            except Exception as e:
                msg = str(e)
                self.status.emit(f"RX error: {msg}")
                if any(s in msg.lower() for s in (
                    "clearcomm", "permission", "command 22", "access is denied",
                )):
                    with self._lock:
                        try:
                            if self.ser:
                                self.ser.close()
                        except Exception:
                            pass
                        self.ser = None
                    time.sleep(0.4)
                else:
                    time.sleep(0.1)
