"""Parse ECU telemetry / replies for STRIX V2."""
from __future__ import annotations

from typing import Any


KEY_MAP = {
    "RPM": "rpm",
    "MAP": "map",
    "TPS": "tps", "TADC": "tadc",
    "TMP": "ect",
    "IAT": "iat",
    "BAT": "bat",
    "LOAD": "load", "SYNCQ": "syncq",
    "SYNC": "sync",
    "CAM": "cam",
    "FAN": "fan",
    "FP": "fp",
    "IGN": "ign",
    "PW": "pw",
    "INJ": "pw",
    "PWUS": "pwus",
    "BASEIGN": "baseign",
    "BASEINJ": "baseinj",
    "MCELL": "mcell",
    "TRET": "tret",
    "AFR": "afr",
    "LAM": "lam",
    "STFT": "stft",
    "LTFT": "ltft",
    "O2": "o2",
    "ETH": "eth",
    "ETHANOL": "eth",
    "VSS": "vss",
    "LCD": "lc_decay",
    "LCF": "lc_fuel",
    "LCR": "lc_ret",
    "ASE": "ase",
    "FLOOD": "flood",
    "DFCO": "dfco",
    "VVT1": "vvt1",
    "VVT2": "vvt2",
    "CUT": "cut",
    "OFC": "ofc",
    "DWELL": "dwell",
    "CYL": "cyl",
    "UID": "ecu_uid",
    "DEVID": "ecu_uid",
}


def default_live() -> dict[str, Any]:
    return {
        "rpm": 0,
        "map": 0,
        "tps": 0, "tadc": 0.0,
        "ect": 0.0,
        "iat": 0.0,
        "bat": 0.0,
        "load": 0.0, "syncq": 0,
        "sync": 0,
        "cam": 0,
        "eth": 0.0,
        "fan": 0,
        "fp": 0,
        "ign": 0.0,
        "pw": 0.0,
        "pwus": 0,
        "baseign": None,
        "baseinj": None,
        "mcell_r": -1,
        "mcell_c": -1,
        "tret": 0.0,
        "afr": 0.0,
        "lam": 0.0,
        "stft": 0.0,
        "ltft": 0.0,
        "dwell": 0,
        "ase": 0,
        "vss": 0.0,
        "lc_decay": 0,
        "lc_fuel": 0.0,
        "lc_ret": 0.0,
        "flood": 0,
        "dfco": 0,
        "vvt1": 0.0,
        "vvt2": 0.0,
        "cut": 0,
        "cyl": 4,
        "ecu_uid": "",
    }


def parse_line(line: str, live: dict[str, Any]) -> dict[str, Any]:
    tags: dict[str, Any] = {"kind": "unknown", "raw": line}
    up = line.strip()
    if not up:
        return tags

    if up.startswith(("OK:", "ERR:", "BUSY:")):
        tags["kind"] = "ack"
        tags["body"] = up
        return tags
    # Only pure diagnostic headers — never treat RPM telemetry as diag
    up_u = up.upper()
    if up_u.startswith(("MAP:ADV", "MAP:INJ", "MAP:END", "CFG:", "IGNDBG:", "FLASH:",
                        "MAPSUM:", "UART:", "UID:", "DEVID:", "PROTO:")):
        tags["kind"] = "diag"
        tags["body"] = up
        if up_u.startswith("UID:") or up_u.startswith("DEVID:"):
            live["ecu_uid"] = up.split(":", 1)[-1].strip()
        return tags

    if ":" not in up:
        return tags

    accepted = 0
    for part in up.split(","):
        part = part.strip()
        if not part or ":" not in part:
            continue
        key, _, val = part.partition(":")
        key = key.strip().upper()
        val = val.strip()
        dest = KEY_MAP.get(key)
        if not dest:
            continue
        try:
            if dest == "mcell":
                if ":" in val:
                    a, b = val.split(":", 1)
                    live["mcell_r"] = int(float(a))
                    live["mcell_c"] = int(float(b))
                accepted += 1
                continue
            if dest == "ecu_uid":
                live["ecu_uid"] = val
                accepted += 1
                continue
            if dest in ("sync", "cam", "fan", "fp", "pwus", "cyl", "dwell", "ase", "dfco", "cut", "lc_decay"):
                live[dest] = int(float(val))
            elif dest == "rpm":
                live[dest] = int(float(val))
            elif dest == "map":
                live[dest] = int(round(float(val)))
            elif dest == "baseinj":
                v = float(val)
                if "." not in val:
                    v = v / 10.0
                live[dest] = v
            else:
                live[dest] = float(val)
            accepted += 1
        except (ValueError, TypeError):
            continue

    if accepted:
        tags["kind"] = "telemetry"
        if live.get("pwus"):
            try:
                live["pw"] = float(live["pwus"]) * 0.001
            except Exception:
                pass
    return tags
