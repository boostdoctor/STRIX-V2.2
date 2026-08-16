import math
STRIX_VERSION = "2.1.0"
STRIX_PROTO = 2
# STRIX V2 — constants (12×22 maps)

ROWS = 12          # load axis (TPS% or MAP kPa)
COLS = 22          # RPM axis
BAUD = 115200
TELEM_HZ = 40  # target telemetry rate (firmware ~25 ms)
VE_REF_MS = 2.5  # pulse width at VE=100%, 100 kPa, 20°C

ADV_MIN, ADV_MAX = -10, 45
INJ_MIN, INJ_MAX = 0.0, 20.0

# RPM axis: 250 … 8125
RPM_BINS = [250 + c * 375 for c in range(COLS)]

# Default MAP load axis: 20 … 240 kPa (whole numbers), evenly spaced
MAP_KPA_MIN = 20
MAP_KPA_MAX_DEFAULT = 240
MAP_KPA_MAX_LIMIT = 500

def make_map_bins(kpa_max: int = MAP_KPA_MAX_DEFAULT) -> list[int]:
    kpa_max = max(MAP_KPA_MIN + ROWS, min(MAP_KPA_MAX_LIMIT, int(kpa_max)))
    if ROWS <= 1:
        return [MAP_KPA_MIN]
    step = (kpa_max - MAP_KPA_MIN) / (ROWS - 1)
    return [int(round(MAP_KPA_MIN + i * step)) for i in range(ROWS)]

def make_tps_bins() -> list[int]:
    """TPS load axis 0…100 % whole numbers."""
    if ROWS <= 1:
        return [0]
    step = 100.0 / (ROWS - 1)
    return [int(round(i * step)) for i in range(ROWS)]

MAP_SENSORS = [
    ("Custom", None),
    ("Bosch 1.1 bar (115 kPa abs)", 115),
    ("Bosch 2.5 bar (250 kPa abs)", 250),
    ("Bosch 3.0 bar (300 kPa abs)", 300),
    ("Bosch 3.5 bar (350 kPa abs)", 350),
    ("GM 1 bar (105 kPa abs)", 105),
    ("GM 2 bar (200 kPa abs)", 200),
    ("GM 3 bar (300 kPa abs)", 300),
    ("AEM 3.5 bar (350 kPa abs)", 350),
    ("Omni 4 bar (400 kPa abs)", 400),
    ("5 bar (500 kPa abs)", 500),
]

TEMP_SENSORS = [
    ("Custom", None),
    ("Bosch VW / GM IAT (std)", "bosch_iat"),
    ("Bosch CLT (std)", "bosch_clt"),
    ("GM open-air IAT", "gm_iat"),
    ("Honda ECT", "honda_ect"),
]

O2_MODES = ("Disabled", "Narrowband", "Wideband")

# Crank wheel profiles (id, name, teeth, missing) — matches STRIX V1 / ecu_wheels.h
WHEEL_PROFILES = [
    (6,  "36-1", 36, 1),
    (3,  "60-2", 60, 2),
    (4,  "60-2 + cam", 60, 2),
    (5,  "60-2 + halfmoon", 60, 2),
    (1,  "12-1", 12, 1),
    (2,  "24-1", 24, 1),
    (7,  "36-2", 36, 2),
    (28, "36-1 + 2nd trig", 36, 1),
    (0,  "Custom", 36, 1),
]

# Default ECT/IAT compensation tables: [ADC, fuel%, ign°] × 14
DEFAULT_ECT_COMP = [[int(i * 292), 20 - i, max(-2, 5 - i)] for i in range(14)]
DEFAULT_IAT_COMP = [[int(i * 292), 10 - i // 2, max(-3, 2 - i // 2)] for i in range(14)]
DEFAULT_BAT_COMP = [[80 + i * 8, 15 - i, max(-2, 4 - i // 2)] for i in range(10)]  # ADC ~8–16 V scaled


OPTIONAL_STRIP = [
    ("ign", "IGN"),
    ("pw", "INJ ms"),
    ("baseign", "BASE IGN"),
    ("bat", "BAT"),
    ("afr", "AFR"),
    ("lam", "Lambda"),
    ("fan", "FAN"),
    ("fp", "FP"),
    ("stft", "STFT"),
    ("ltft", "LTFT"),
    ("load", "LOAD"),
    ("dwell", "Dwell"),
]

ALWAYS_STRIP = ("rpm", "tps", "map", "ect", "iat", "sync")

DEVICE_ID_FILE = "strix_device_id.json"
SETTINGS_FILE = "strix_v2_settings.json"
TCAL_VERSION = 1

DARK_STYLE = """
    QPushButton { padding: 4px 8px; font-size: 11px; min-height: 24px; }
    QComboBox { padding: 2px 6px; min-height: 24px; }

QMainWindow, QWidget, QDialog {
    background-color: #12161e; color: #d0d8e8;
    font-family: "Segoe UI", "Ubuntu", sans-serif;
}
QPushButton {
    background-color: #243044; color: #c8d8ff;
    border: 1px solid #3a5a8a; border-radius: 6px;
    padding: 6px 14px; font-size: 12px;
}
QPushButton:hover   { background-color: #2e4060; border-color: #5a8acc; }
QPushButton:pressed { background-color: #1a2a40; }
QPushButton:checked {
    background-color: #1a3d2a; color: #44ff88; border-color: #44ff88;
}
QPushButton:disabled { color: #556070; border-color: #2a3040; }
QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit, QTableWidget {
    background-color: #1a2230; color: #c8d8ff;
    border: 1px solid #3a5a8a; border-radius: 4px; padding: 4px 8px;
}
QTabWidget::pane {
    border: 1px solid #2a3548; background: #151a24; border-radius: 4px;
}
QTabBar::tab {
    background: #1a2230; color: #9ab; padding: 8px 16px;
    border: 1px solid #2a3548; border-bottom: none;
    border-top-left-radius: 6px; border-top-right-radius: 6px; margin-right: 2px;
}
QTabBar::tab:selected { background: #243044; color: #e8f0ff; }
QStatusBar { background: #0e1218; color: #8a9aaa; }
QLabel#TitleLabel { color: #7eb8ff; font-size: 14px; font-weight: 700; }
QFrame#LiveStrip {
    background: #0e141c; border: 1px solid #2a3548; border-radius: 8px;
}
QLabel#LiveValue {
    color: #e8f0ff; font-size: 15px; font-weight: 700;
    min-width: 88px; max-width: 88px;
    min-height: 22px; max-height: 22px;
}
QLabel#LiveTag {
    color: #6a7a8a; font-size: 10px;
    min-width: 88px; max-width: 88px;
}
QGroupBox {
    border: 1px solid #2a3548; border-radius: 6px; margin-top: 10px;
    padding-top: 12px; color: #a0b0c0;
}
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 6px; }
QCheckBox { spacing: 8px; }
QHeaderView::section {
    background: #1a2230; color: #9ab; padding: 4px; border: 1px solid #2a3548;
}
"""


def suggested_ve_map(rows: int = ROWS, cols: int = COLS) -> list[list[float]]:
    """Reasonable VE % base map (load × RPM). Peak ~ mid-RPM / high load."""
    out = []
    for r in range(rows):
        # load fraction 0 (light) → 1 (full)
        lf = r / max(1, rows - 1)
        row = []
        for c in range(cols):
            rf = c / max(1, cols - 1)
            # base 55% idle-ish → 100% peak torque area → slight drop at high RPM
            ve = 55.0 + lf * 40.0 + rf * 12.0 - abs(rf - 0.55) * 18.0
            if lf > 0.85 and rf > 0.7:
                ve -= 5.0  # slight high-load high-rpm taper
            ve = max(40.0, min(110.0, ve))
            row.append(round(ve, 1))
        out.append(row)
    return out


def suggested_adv_map(rows: int = ROWS, cols: int = COLS) -> list[list[int]]:
    """Mild pump-gas advance table (° BTDC)."""
    out = []
    for r in range(rows):
        lf = r / max(1, rows - 1)
        row = []
        for c in range(cols):
            rf = c / max(1, cols - 1)
            # more advance at light load / mid RPM; less at high load
            adv = 12.0 + rf * 22.0 - lf * 14.0 + (0.5 - abs(rf - 0.45)) * 8.0
            adv = max(5, min(38, int(round(adv))))
            row.append(adv)
        out.append(row)
    return out


def ve_to_inj_ms(
    ve: float,
    map_kpa: float = 100.0,
    iat_c: float = 20.0,
    req_fuel_ms: float = 2.5,
    flow_cc: float = 220.0,
    fuel_pressure_bar: float = 3.0,
    fuel_pressure_rated_bar: float = 3.0,
) -> float:
    """Estimate base pulse width from VE (same model as firmware).

    Injector flow scales ~sqrt(P_rail / P_rated).
    """
    iat_k = max(250.0, iat_c + 273.15)
    dens = (max(20.0, map_kpa) / 100.0) * (293.15 / iat_k)
    base = req_fuel_ms * (max(0.0, ve) / 100.0) * dens
    p_act = max(0.5, float(fuel_pressure_bar))
    p_rat = max(0.5, float(fuel_pressure_rated_bar))
    flow_eff = max(10.0, float(flow_cc)) * math.sqrt(p_act / p_rat)
    base *= 220.0 / flow_eff
    return round(max(0.4, min(20.0, base)), 1)


def suggested_inj_ms_map(
    rows: int = ROWS,
    cols: int = COLS,
    req_fuel_ms: float = 2.5,
    flow_cc: float = 220.0,
    fuel_pressure_bar: float = 3.0,
    fuel_pressure_rated_bar: float = 3.0,
) -> list[list[float]]:
    """Default raw injector pulse-width map (ms) when VE mode is off.

    Low-load / low-RPM cells start near **2.2 ms** and scale with load & RPM.
    VE table is left unchanged; this path does not modify suggested_ve_map().
    req_fuel / flow / pressure nudge the overall scale slightly.
    """
    # flow/pressure scale relative to 220 cc @ 3 bar
    p_act = max(0.5, float(fuel_pressure_bar))
    p_rat = max(0.5, float(fuel_pressure_rated_bar))
    flow_eff = max(10.0, float(flow_cc)) * (p_act / p_rat) ** 0.5
    scale = (220.0 / flow_eff) * (float(req_fuel_ms) / 2.5)

    out = []
    for r in range(rows):
        lf = r / max(1, rows - 1)  # load 0→1
        row = []
        for c in range(cols):
            rf = c / max(1, cols - 1)  # rpm 0→1
            # idle/low ~2.2 ms → peak ~8–10 ms at high load mid RPM
            ms = 2.2 + lf * 5.5 + rf * 1.8 - abs(rf - 0.5) * 1.2
            if lf < 0.15:
                ms = 2.2 + rf * 0.6
            ms = ms * scale
            ms = round(max(2.2 * min(1.0, scale), min(20.0, ms)), 1)
            # keep absolute floor near 2.2 when scale ≈ 1
            if scale >= 0.9:
                ms = max(2.2, ms)
            row.append(ms)
        out.append(row)
    return out


def suggested_vvt_map(rows: int = 8, cols: int = 8, exhaust: bool = False) -> list[list[float]]:
    """Recommended cam target ° (intake advances with RPM; exhaust milder)."""
    out = []
    for r in range(rows):
        lf = r / max(1, rows - 1)
        row = []
        for c in range(cols):
            rf = c / max(1, cols - 1)
            if exhaust:
                # exhaust: small advance mid-RPM, less at ends
                deg = 5.0 + rf * 18.0 - abs(rf - 0.55) * 10.0 - lf * 4.0
            else:
                # intake: more advance with RPM, pull back at high load
                deg = 8.0 + rf * 28.0 - lf * 10.0 - abs(rf - 0.6) * 6.0
            deg = max(0.0, min(45.0 if not exhaust else 35.0, deg))
            row.append(round(deg, 1))
        out.append(row)
    return out


def suggested_boost_map(
    rows: int = 8,
    cols: int = 8,
    closed_loop: bool = True,
    max_kpa: float = 180.0,
) -> list[list[float]]:
    """Recommended boost target (gauge kPa) or open-loop duty %."""
    out = []
    for r in range(rows):
        lf = r / max(1, rows - 1)
        row = []
        for c in range(cols):
            rf = c / max(1, cols - 1)
            if closed_loop:
                # gauge target: low at low load/RPM, ramp to max_kpa
                tgt = 20.0 + lf * (max_kpa - 40.0) + rf * 25.0
                tgt = max(0.0, min(max_kpa, tgt))
                # floor near atmospheric gauge 0 at light load
                if lf < 0.2:
                    tgt = min(tgt, 15.0 + rf * 10.0)
                row.append(round(tgt, 1))
            else:
                duty = 15.0 + lf * 55.0 + rf * 15.0
                duty = max(0.0, min(90.0, duty))
                if lf < 0.15:
                    duty = min(duty, 20.0)
                row.append(round(duty, 1))
        out.append(row)
    return out



# Firing orders by cylinder count (common OEM / aftermarket)
FIRING_ORDERS_BY_CYL: dict[int, list[str]] = {
    1: ["1"],
    2: ["1-2", "2-1"],
    3: ["1-2-3", "1-3-2"],
    4: ["1-3-4-2", "1-2-4-3", "1-3-2-4", "1-4-3-2", "1-2-3-4"],
    5: ["1-2-4-5-3", "1-3-5-4-2"],
    6: [
        "1-5-3-6-2-4",  # common inline-6
        "1-4-2-5-3-6",
        "1-6-5-4-3-2",
        "1-2-3-4-5-6",
    ],
    8: [
        "1-8-4-3-6-5-7-2",  # SBC/BBC common
        "1-8-7-2-6-5-4-3",  # LS / many V8
        "1-5-4-8-6-3-7-2",  # Ford 5.0 HO style
        "1-6-2-4-8-3-7-5",
        "1-3-7-2-6-5-4-8",
    ],
}

ALL_FIRING_ORDERS: list[str] = []
for _cyl in sorted(FIRING_ORDERS_BY_CYL):
    for _fo in FIRING_ORDERS_BY_CYL[_cyl]:
        if _fo not in ALL_FIRING_ORDERS:
            ALL_FIRING_ORDERS.append(_fo)
