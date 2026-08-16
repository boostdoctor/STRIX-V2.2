"""Persistent unique device ID + last-known COM port for auto-connect."""
from __future__ import annotations

import json
import uuid
from pathlib import Path

from strix_v2.constants import DEVICE_ID_FILE

_DEFAULT = {"device_id": "", "last_port": "", "ecu_uid": ""}


def _path() -> Path:
    return Path.home() / ".strix_v2" / DEVICE_ID_FILE


def _read_raw() -> dict:
    """Read JSON without creating an ID (no recursion)."""
    p = _path()
    if not p.exists():
        return dict(_DEFAULT)
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            return dict(_DEFAULT)
        out = dict(_DEFAULT)
        out.update({k: data.get(k, out[k]) for k in out})
        return out
    except Exception:
        return dict(_DEFAULT)


def _write_raw(data: dict) -> None:
    p = _path()
    p.parent.mkdir(parents=True, exist_ok=True)
    p.write_text(json.dumps(data, indent=2), encoding="utf-8")


def get_or_create_device_id() -> str:
    data = _read_raw()
    did = str(data.get("device_id") or "").strip()
    if did:
        return did
    did = "STRIX-" + uuid.uuid4().hex[:12].upper()
    data["device_id"] = did
    _write_raw(data)
    return did


def load_device_meta() -> dict:
    data = _read_raw()
    if not str(data.get("device_id") or "").strip():
        data["device_id"] = get_or_create_device_id()
    return data


def save_device_meta(
    device_id: str | None = None,
    last_port: str | None = None,
    ecu_uid: str | None = None,
) -> None:
    data = _read_raw()
    if device_id is not None:
        data["device_id"] = device_id
    if last_port is not None:
        data["last_port"] = last_port
    if ecu_uid is not None:
        data["ecu_uid"] = ecu_uid
    # Ensure we always have an ID after save
    if not str(data.get("device_id") or "").strip():
        data["device_id"] = "STRIX-" + uuid.uuid4().hex[:12].upper()
    _write_raw(data)
