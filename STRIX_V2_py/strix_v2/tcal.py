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
        "teeth": 60,
        "missing": 2,
        "trig_angle": 30,
        "eoi_btdc": 340,  # deg BTDC compression (default EOI)
        "wheel_id": 9,  # 60-2 + 1-tooth cam
        "firing_order": "1-3-4-2",
        "coil_type": "Smart",
        "coil_charge_mode": "Constant Duty",  # Smart | Dumb | Distributor
        "dwell_ms": 3.0,
        "spark_double": False,
        "load_mode": "MAP",  # MAP | TPS | HYBRID
        "map_kpa_max": 240,
        "map_bins": make_map_bins(240),
        "tps_bins": make_tps_bins(),
        "throttle_type": "Cable",  # Cable | DBW
        "idle_control": "Disabled",  # Disabled | Single wire PWM | Dual wire
        "run_mode": "Batch",  # Batch | Sequential (combined ign+inj)
        "inj_mode": "Batch",
        "ign_mode": "Wasted Spark",
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
        "ve_mode": True,     # True=VE %, False=Injector Duty (ms)
        "ae_enable": True,
        "ae_tps_dot_thresh": 20.0,
        "ae_gain": 1.5,
        "ae_max_pct": 40.0,
        "ae_decay_ms": 400,
        "flex_enable": False,
        "flex_adc_e0": 410,
        "flex_adc_e100": 3686,
        "flex_fuel_pct_per10": 4.7,
        "flex_ign_deg_per10": 0.8,

        "rpm_limit": 7000,
        "rpm_cut_mode": "Hard",  # Hard | Soft
        "fan_enable": False,
        "fan_c": 95,
        "tacho_enable": False,
        "tacho_ppr": 2,
        "cam_home": True,  # sensor present; sequential still optional
        "o2_mode": "Disabled",  # Disabled | Narrowband | Wideband
        "flood_clear_enable": True,
        "flood_clear_tps": 85.0,
        "crank_adv_enable": True,
        "crank_adv_deg": 10.0,
        "crank_adv_rpm": 400,
        "cyl_trim": [0.0, 0.0, 0.0, 0.0],
        "boost_mode": "OFF",  # OFF | Single value | Closed-loop | Open-loop
        "vvt_mode": "Disabled",
        "idle_enable": True,
        "idle_target_rpm": 850,
        "idle_ect_bins": [-10, 20, 40, 60, 90],
        "idle_target_rpm_tbl": [1400, 1100, 950, 850, 850],
        "launch_decay_enable": False,
        "launch_vss_bins": [0, 20, 40, 60, 80, 100, 130, 160],
        "launch_fuel_tbl": [25, 20, 14, 8, 4, 2, 0, 0],
        "launch_retard_tbl": [15, 12, 8, 5, 2, 0, 0, 0],
        "vss_enable": False,
        "vss_pulses_per_km": 8000,
        "ase_initial_pct": 35.0,
        "ase_decay_sec": 8.0,
        "ase_min_ect": 60.0,
        "curves": {
            "wue": {"xs": [-20, 0, 10, 20, 30, 40, 50, 60, 70, 80],
                    "ys": [80, 55, 40, 28, 18, 12, 7, 3, 0, 0]},
            "ase": {"xs": [-20, 0, 20, 40, 60, 80],
                    "ys": [60, 45, 30, 18, 8, 0]},
            "iat": {"xs": [-20, 0, 20, 40, 60, 80, 100],
                    "ys": [12, 6, 0, -4, -8, -12, -16]},
            "bat": {"xs": [8, 10, 12, 13.2, 14.4, 16],
                    "ys": [18, 10, 4, 0, -2, -4]},
        },
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
