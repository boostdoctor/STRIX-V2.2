"""Engine calibration file (.tcal) import/export."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from strix_v2.constants import TCAL_VERSION, ROWS, COLS, make_map_bins, make_tps_bins


def default_engine_settings() -> dict[str, Any]:
    return {
        "tcal_version": TCAL_VERSION,
        "cylinders": 4,
        "teeth": 36,
        "missing": 1,
        "trig_angle": 30,
        "wheel_id": 0,
        "firing_order": "1-3-4-2",
        "coil_type": "Smart",  # Smart | Dumb | Distributor
        "load_mode": "MAP",  # MAP | TPS | HYBRID
        "map_kpa_max": 240,
        "map_bins": make_map_bins(240),
        "tps_bins": make_tps_bins(),
        "throttle_type": "Cable",  # Cable | DBW
        "idle_control": "Disabled",  # Disabled | Single wire PWM | Dual wire
        "inj_mode": "Sequential",  # Sequential | Batch | Batch above RPM
        "batch_above_rpm": 3000,
        "fp_prime_ms": 2000,
        "start_prime_ms": 50,
        "start_prime_enable": True,
        "inj_flow_cc": 220,   # injector flow cc/min @ rated pressure
        "fuel_pressure_bar": 3.0,       # actual rail pressure (bar)
        "fuel_pressure_rated_bar": 3.0, # pressure where flow_cc was measured
        "req_fuel_ms": 2.5,  # ms at 100% VE, 100 kPa, 20 C
        "max_inj_ms": 15.0,  # hard ceiling on injector pulse
        "max_advance": 40,  # deg BTDC clamp
        "max_retard": 10,   # deg ATDC clamp (positive number)
        "dfco_enable": True,
        "dfco_enter_rpm": 1600,
        "dfco_exit_rpm": 1200,
        "dfco_max_tps": 3.0,
        "dfco_min_ect": 50.0,
        "dfco_delay_ms": 200,
        "ve_mode": True,     # fuel map cells are VE %

        "rpm_limit": 7000,
        "rpm_cut_mode": "Hard",  # Hard | Soft
        "fan_enable": True,
        "fan_c": 95,
        "o2_mode": "Disabled",  # Disabled | Narrowband | Wideband
        "boost_mode": "OFF",  # OFF | Single value | Closed-loop | Open-loop
        "vvt_mode": "Disabled",
        "idle_enable": True,
        "idle_target_rpm": 850,  # Disabled | Intake | Exhaust | Intake & Exhaust
        "sensors": {
            "ect": {"enabled": True, "preset": "Bosch CLT (std)", "key": "bosch_clt"},
            "iat": {"enabled": True, "preset": "Bosch VW / GM IAT (std)", "key": "bosch_iat"},
            "map": {"enabled": True, "preset": "Bosch 2.5 bar (250 kPa abs)", "max_kpa": 250},
            "tps": {"enabled": True, "preset": "Custom", "closed_adc": 200, "open_adc": 3800},
            "o2": {
                "enabled": False,
                "mode": "Disabled",
                "nb_table": [[0.1, 0.1], [0.3, 0.3], [0.5, 0.5], [0.7, 0.7], [0.9, 0.9]],
                "wb_table": [
                    [10.0, 0.0], [11.0, 0.5], [12.0, 1.0], [13.0, 1.5], [14.0, 2.0],
                    [14.7, 2.5], [16.0, 3.0], [18.0, 3.5], [20.0, 4.0], [22.0, 4.5],
                ],
            },
        },
    }


def save_tcal(path: str | Path, settings: dict) -> None:
    data = dict(settings)
    data["tcal_version"] = TCAL_VERSION
    Path(path).write_text(json.dumps(data, indent=2), encoding="utf-8")


def load_tcal(path: str | Path) -> dict:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    base = default_engine_settings()
    # shallow merge top-level + sensors
    for k, v in raw.items():
        if k == "sensors" and isinstance(v, dict):
            for sk, sv in v.items():
                if sk in base["sensors"] and isinstance(sv, dict):
                    base["sensors"][sk].update(sv)
                else:
                    base["sensors"][sk] = sv
        else:
            base[k] = v
    # ensure bin lengths
    if len(base.get("map_bins") or []) != ROWS:
        base["map_bins"] = make_map_bins(int(base.get("map_kpa_max") or 240))
    if len(base.get("tps_bins") or []) != ROWS:
        base["tps_bins"] = make_tps_bins()
    return base
