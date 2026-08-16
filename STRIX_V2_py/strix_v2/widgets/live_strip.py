"""Live strip — primary + secondary rows, alarms, readable monospace tiles."""
from __future__ import annotations

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QFrame, QHBoxLayout, QLabel, QVBoxLayout, QWidget, QSizePolicy

from strix_v2.constants import ALWAYS_STRIP, OPTIONAL_STRIP

_TAG = (
    "font-family: 'Segoe UI','Arial',sans-serif;"
    "font-size:11px;font-weight:700;letter-spacing:0.5px;"
    "color:#8a9bb0;"
)
_VAL = (
    "font-family: 'Consolas','Cascadia Mono','Courier New',monospace;"
    "font-size:18px;font-weight:700;"
    "min-width:96px;max-width:96px;min-height:28px;max-height:28px;"
    "color:#f2f6ff;"
)


class _Cell(QWidget):
    def __init__(self, tag: str, tip: str = "", parent=None):
        super().__init__(parent)
        self.setFixedWidth(100)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)
        if tip:
            self.setToolTip(tip)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(4, 2, 4, 2)
        lay.setSpacing(1)
        self.tag = QLabel(tag)
        self.tag.setObjectName("LiveTag")
        self.tag.setAlignment(Qt.AlignCenter)
        self.tag.setFixedWidth(96)
        self.tag.setStyleSheet(_TAG)
        self.val = QLabel("—")
        self.val.setObjectName("LiveValue")
        self.val.setAlignment(Qt.AlignCenter)
        self.val.setFixedWidth(96)
        self.val.setFixedHeight(28)
        self.val.setStyleSheet(_VAL)
        lay.addWidget(self.tag)
        lay.addWidget(self.val)
        self._flash = False

    def set_value(self, text: str, color: str | None = None, alarm: bool = False):
        t = text if len(text) <= 10 else text[:9] + "…"
        self.val.setText(t)
        if alarm:
            self._flash = not self._flash
            bg = "#7a2222" if self._flash else "#4a1515"
            self.val.setStyleSheet(_VAL + f"color:#ffe0e0;background:{bg};border-radius:4px;")
        elif color:
            self.val.setStyleSheet(_VAL + f"color:{color};")
        else:
            self.val.setStyleSheet(_VAL)


class LiveStrip(QFrame):
    """Primary: RPM TPS MAP LOAD SYNC. Secondary: ECT IAT BAT AFR FAN FP + optionals."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName("LiveStrip")
        outer = QVBoxLayout(self)
        outer.setContentsMargins(6, 6, 6, 4)
        outer.setSpacing(4)
        self._row1 = QHBoxLayout()
        self._row1.setSpacing(4)
        self._row2 = QHBoxLayout()
        self._row2.setSpacing(4)
        outer.addLayout(self._row1)
        outer.addLayout(self._row2)
        self.cells: dict[str, _Cell] = {}
        self._optional_keys: set[str] = set()
        self._alarm = QLabel("")
        self._alarm.setObjectName("AlarmStrip")
        self._alarm.setStyleSheet(
            "color:#ff8888;font-weight:700;font-size:13px;padding:2px 8px;"
        )
        self._alarm.hide()
        outer.addWidget(self._alarm)
        self._rebuild()

    def _clear(self, lay: QHBoxLayout):
        while lay.count():
            item = lay.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()

    def _rebuild(self):
        self._clear(self._row1)
        self._clear(self._row2)
        self.cells.clear()
        primary = [
            ("rpm", "RPM", "Engine speed"),
            ("tps", "TPS", "Throttle position %"),
            ("map", "MAP", "Manifold pressure kPa"),
            ("load", "LOAD", "Calculated engine load"),
            ("sync", "SYNC", "Crank sync lock"),
        ]
        secondary = [
            ("ect", "ECT", "Coolant — alarm > 105 °C"),
            ("iat", "IAT", "Intake air temperature"),
            ("bat", "BAT", "Battery voltage"),
            ("afr", "AFR", "Air–fuel ratio"),
            ("fan", "FAN", "Radiator fan"),
            ("fp", "FP", "Fuel pump"),
        ]
        for k, lab, tip in primary:
            c = _Cell(lab, tip)
            self.cells[k] = c
            self._row1.addWidget(c)
        self._row1.addStretch(1)
        for k, lab, tip in secondary:
            c = _Cell(lab, tip)
            self.cells[k] = c
            self._row2.addWidget(c)
        opt_tips = {
            "ign": "Ignition advance ° BTDC",
            "pw": "Injector pulse ms (final)",
            "cam": "Cam sync",
            "dwell": "Coil dwell µs",
            "stft": "Short-term fuel trim %",
            "ltft": "Long-term fuel trim %",
        }
        for k, lab in OPTIONAL_STRIP:
            if k in self._optional_keys and k not in self.cells:
                c = _Cell(lab, opt_tips.get(k, lab))
                self.cells[k] = c
                self._row2.addWidget(c)
        self._row2.addStretch(1)

    def set_optional(self, keys: set[str] | list[str]):
        self._optional_keys = set(keys)
        self._rebuild()

    def update_live(self, live: dict):
        rpm = int(live.get("rpm") or 0)
        ect = float(live.get("ect") or 0)
        sync = int(live.get("sync") or 0)
        fp = int(live.get("fp") or 0)
        fan = int(live.get("fan") or 0)

        self.cells["rpm"].set_value(f"{rpm}")
        self.cells["tps"].set_value(f"{float(live.get('tps') or 0):.0f}%")
        self.cells["map"].set_value(f"{int(round(float(live.get('map') or 0)))}")
        self.cells["load"].set_value(f"{int(round(float(live.get('load') or 0)))}")
        self.cells["sync"].set_value(
            "LOCK" if sync else "—",
            "#55ff99" if sync else "#ff7777",
            alarm=(not sync and rpm > 200),
        )
        self.cells["ect"].set_value(f"{ect:.0f}°C", "#ff7777" if ect > 105 else None, alarm=ect > 105)
        self.cells["iat"].set_value(f"{float(live.get('iat') or 0):.0f}°C")
        self.cells["bat"].set_value(f"{float(live.get('bat') or 0):.1f}V")
        self.cells["afr"].set_value(f"{float(live.get('afr') or 0):.1f}")
        self.cells["fan"].set_value("ON" if fan else "off", "#55ff99" if fan else None)
        self.cells["fp"].set_value(
            "ON" if fp else "off",
            "#55ff99" if fp else "#ff7777",
            alarm=(not fp and rpm > 400),
        )

        fmt = {
            "ign": lambda v: f"{float(v or 0):.0f}°",
            "pw": lambda v: f"{float(v or 0):.1f}",
            "baseign": lambda v: f"{float(v):.0f}°" if v is not None else "—",
            "cam": lambda v: "OK" if int(v or 0) else "—",
            "dwell": lambda v: f"{int(v or 0)}",
            "stft": lambda v: f"{float(v or 0):.1f}%",
            "ltft": lambda v: f"{float(v or 0):.1f}%",
            "lam": lambda v: f"{float(v or 0):.3f}",
        }
        for k in list(self.cells.keys()):
            if k in ("rpm", "tps", "map", "load", "sync", "ect", "iat", "bat", "afr", "fan", "fp"):
                continue
            f = fmt.get(k, lambda v: str(v if v is not None else "—"))
            self.cells[k].set_value(f(live.get(k)))

        alarms = []
        if ect > 105:
            alarms.append("ECT HIGH")
        if not fp and rpm > 400:
            alarms.append("FP OFF")
        if not sync and rpm > 200:
            alarms.append("SYNC LOST")
        if alarms:
            self._alarm.setText("  ·  ".join(alarms))
            self._alarm.show()
        else:
            self._alarm.hide()
