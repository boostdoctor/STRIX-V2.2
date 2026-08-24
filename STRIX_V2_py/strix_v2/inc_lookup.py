"""MegaTunix / MSnS .inc AFR lookup parser."""
from __future__ import annotations

import re
from pathlib import Path
from typing import Any


_DB_RE = re.compile(r"(?:DB|dw|db)\s+([0-9]+(?:\.[0-9]+)?)T?", re.I)
_PAIR_RE = re.compile(r"^\s*([0-9]+(?:\.[0-9]+)?)\s*[,;\s]\s*([0-9]+(?:\.[0-9]+)?)")


def parse_inc(path: str | Path) -> dict[str, Any]:
    """Parse a MegaTunix-style AFR .inc file.

    Typical form is 256 ``DB 147T`` entries (AFR × 10) over 0–5 V.
    Also accepts ``volt,afr`` or ``adc,afr`` pairs.
    """
    text = Path(path).read_text(encoding="utf-8", errors="replace")
    name = Path(path).stem
    comments: list[str] = []
    db: list[float] = []
    pairs: list[tuple[float, float]] = []

    for raw in text.splitlines():
        line = raw.strip()
        if not line:
            continue
        if line.startswith(";") or line.startswith("#"):
            comments.append(line.lstrip(";# ").strip())
            continue
        if line.endswith(":") and not line.upper().startswith("DB"):
            name = line[:-1].strip()
            continue
        m = _DB_RE.search(line)
        if m:
            v = float(m.group(1))
            # 73.5T … 220T → AFR 7.35–22.0. Bare 10–22 already AFR.
            if v > 30.0:
                v *= 0.1
            db.append(v)
            continue
        m = _PAIR_RE.match(line)
        if m:
            pairs.append((float(m.group(1)), float(m.group(2))))

    if db:
        values = db
        source = "db"
    elif pairs:
        values = [p[1] for p in pairs]
        source = "pairs"
    else:
        raise ValueError(f"No AFR points in {path}")

    afr_min = min(values)
    afr_max = max(values)
    return {
        "name": name,
        "path": str(path),
        "comments": comments[:8],
        "values": values,
        "pairs": pairs,
        "source": source,
        "n": len(values),
        "afr_min": afr_min,
        "afr_max": afr_max,
        "v_max": 5.0,
    }


def downsample_wb(parsed: dict[str, Any], points: int = 10) -> list[list[float]]:
    """Build [AFR, volts] table for the tuner / SET:WB endpoints."""
    vals = parsed["values"]
    n = len(vals)
    if n < 2:
        return [[parsed["afr_min"], 0.0], [parsed["afr_max"], 5.0]]
    vmax = float(parsed.get("v_max") or 5.0)
    out = []
    for i in range(points):
        idx = int(round(i * (n - 1) / (points - 1)))
        afr = vals[idx]
        volt = vmax * idx / (n - 1)
        out.append([round(afr, 2), round(volt, 3)])
    return out


def linear_wb_span(parsed: dict[str, Any]) -> tuple[float, float, float]:
    """Return (afr_at_0V, afr_at_vmax, vmax) for SET:WB."""
    return (float(parsed["afr_min"]), float(parsed["afr_max"]), float(parsed.get("v_max") or 5.0))
