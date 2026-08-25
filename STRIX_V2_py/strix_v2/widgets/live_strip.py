"""Live strip — single row, 10 tiles."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget, QSizePolicy

_TAG = (
    "font-family: 'Segoe UI','Arial',sans-serif;"
    "font-size:11px;font-weight:700;letter-spacing:0.5px;"
    "color:#8a9bb0;"
)
_VAL = (
    "font-family: 'Consolas','Cascadia Mono','Courier New',monospace;"
    "font-size:18px;font-weight:700;"
    "min-width:72px;max-width:78px;min-height:28px;max-height:28px;"
    "color:#f2f6ff;"
)

# Fixed strip row — VE% + injection time (ms) side by side
STRIP_KEYS = (
    ("rpm", "RPM", "Engine speed"),
    ("tps", "TPS", "Throttle %"),
    ("map", "MAP", "Manifold kPa"),
    ("load", "LOAD", "Engine load"),
    ("ve", "VE%", "Volumetric efficiency / cell"),
    ("pw", "INJ", "Injector pulse width ms"),
    ("sync", "SYNC", "Crank lock"),
    ("cam", "CAM", "Cam home"),
    ("ect", "ECT", "Coolant °C"),
    ("iat", "IAT", "Intake air °C"),
    ("afr", "AFR", "Air–fuel ratio"),
    ("ign", "IGN", "Spark ° BTDC"),
)


class _Cell(QWidget):
    def __init__(self, tag: str, tip: str = "", parent=None):
        super().__init__(parent)
        self.setFixedWidth(78)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        if tip:
            self.setToolTip(tip)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.setSpacing(1)
        self.tag = QLabel(tag)
        self.tag.setAlignment(Qt.AlignCenter)
        self.tag.setStyleSheet(_TAG)
        self.val = QLabel("—")
        self.val.setAlignment(Qt.AlignCenter)
        self.val.setFixedHeight(28)
        self.val.setStyleSheet(_VAL)
        lay.addWidget(self.tag)
        lay.addWidget(self.val)

    def set_value(self, text: str, color: str | None = None, alarm: bool = False):
        t = text if len(text) <= 10 else text[:9] + "…"
        self.val.setText(t)
        if alarm:
            self.val.setStyleSheet(_VAL + "color:#ffe0e0;background:#7a2222;border-radius:4px;")
        elif color:
            self.val.setStyleSheet(_VAL + f"color:{color};")
        else:
            self.val.setStyleSheet(_VAL)


class LiveStrip(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("LiveStrip")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 4, 6, 2)
        outer.setSpacing(2)
        self._row = QHBoxLayout()
        self._row.setSpacing(2)
        outer.addLayout(self._row)
        self.cells: dict[str, _Cell] = {}
        self._optional_keys: set[str] = set()
        for k, lab, tip in STRIP_KEYS:
            c = _Cell(lab, tip)
            self.cells[k] = c
            self._row.addWidget(c)
        self._row.addStretch(1)
        self._alarm = QLabel("")
        self._alarm.setStyleSheet("color:#ff8888;font-weight:700;font-size:12px;")
        self._alarm.hide()
        outer.addWidget(self._alarm)

    def set_optional(self, keys: set[str] | list[str]):
        self._optional_keys = set(keys)

    def update_live(self, live: dict):
        rpm = int(live.get("rpm") or 0)
        ect = float(live.get("ect") or 0)
        sync = int(live.get("sync") or 0)
        cam_on = int(float(live.get("cam") or live.get("camsync") or 0))

        self.cells["rpm"].set_value(f"{rpm}")
        self.cells["tps"].set_value(f"{float(live.get('tps') or 0):.0f}%")
        self.cells["map"].set_value(f"{int(round(float(live.get('map') or 0)))}")
        # Firmware LOAD is normalised (0.2–2.4); show as % of ref for readability
        ld = float(live.get("load") or 0)
        if 0.0 < ld < 5.0:
            self.cells["load"].set_value(f"{ld * 100.0:.0f}%")
        else:
            self.cells["load"].set_value(f"{int(round(ld))}")
        self.cells["sync"].set_value(
            "LOCK" if sync else "—",
            "#55ff99" if sync else "#ff7777",
            alarm=(not sync and rpm > 200),
        )
        self.cells["cam"].set_value(
            "LOCK" if cam_on else "—",
            "#55ff99" if cam_on else "#888888",
        )
        self.cells["ect"].set_value("ERROR" if ect > 205 else f"{ect:.0f}°C", "#ff3333" if ect > 205 else ("#ff7777" if ect > 105 else None), alarm=ect > 205)
        afr = float(live.get("afr") or 0)
        self.cells["afr"].set_value("ERROR" if afr > 22 else f"{afr:.1f}", "#ff3333" if afr > 22 else None, alarm=afr > 22)
        iat = float(live.get("iat") or 0)
        if "iat" in self.cells:
            self.cells["iat"].set_value("ERROR" if iat > 120 else f"{iat:.0f}°C", "#ff3333" if iat > 120 else None, alarm=iat > 120)
        ve = live.get("ve")
        if ve is None:
            # approximate from baseinj / load if firmware does not send VE
            try:
                bi = float(live.get("baseinj") or 0)
                ve = bi  # often not VE; leave blank if missing
            except Exception:
                ve = 0
        if "ve" in self.cells:
            try:
                self.cells["ve"].set_value(f"{float(ve):.0f}%")
            except Exception:
                self.cells["ve"].set_value("—")
        # Injection time (ms) — prefer PW ms, else PWUS/1000
        if "pw" in self.cells:
            try:
                pw_ms = live.get("pw")
                if pw_ms is None and live.get("pwus") is not None:
                    pw_ms = float(live.get("pwus")) * 0.001
                if pw_ms is None:
                    self.cells["pw"].set_value("—")
                else:
                    self.cells["pw"].set_value(f"{float(pw_ms):.2f}")
            except Exception:
                self.cells["pw"].set_value("—")
        self.cells["ign"].set_value(f"{float(live.get('ign') or 0):.0f}°")

        alarms = []
        if ect > 205:
            alarms.append("ECT ERROR")
        elif ect > 105:
            alarms.append("ECT HIGH")
        iat = float(live.get("iat") or 0)
        if iat > 120:
            alarms.append("IAT ERROR")
        afr = float(live.get("afr") or 0)
        if afr > 22:
            alarms.append("AFR ERROR")
        if not sync and rpm > 200:
            alarms.append("SYNC LOST")
        if alarms:
            self._alarm.setText("  ·  ".join(alarms))
            self._alarm.show()
        else:
            self._alarm.hide()
