"""PC-side datalog — CSV of live telemetry."""
from __future__ import annotations

import csv
import time
from pathlib import Path
from typing import Any, Iterable

DEFAULT_FIELDS = (
    "rpm", "map", "tps", "load", "ect", "iat", "bat", "afr", "lam",
    "ign", "pw", "sync", "cam", "fp", "fan", "eth", "stft", "ltft",
)


class DataLogger:
    def __init__(self, fields: Iterable[str] = DEFAULT_FIELDS):
        self.fields = list(fields)
        self.path: Path | None = None
        self._fp = None
        self._writer: csv.writer | None = None
        self.t0 = 0.0
        self.rows = 0

    @property
    def active(self) -> bool:
        return self._writer is not None

    def start(self, path: str | Path) -> None:
        self.stop()
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = self.path.open("w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fp)
        self._writer.writerow(["t_s", *self.fields])
        self.t0 = time.time()
        self.rows = 0

    def append(self, live: dict[str, Any]) -> None:
        if not self._writer:
            return
        t = time.time() - self.t0
        row = [f"{t:.3f}"]
        for k in self.fields:
            v = live.get(k)
            if v is None:
                row.append("")
            elif isinstance(v, float):
                row.append(f"{v:.3f}")
            else:
                row.append(str(v))
        self._writer.writerow(row)
        self.rows += 1
        if self.rows % 25 == 0 and self._fp:
            self._fp.flush()

    def stop(self) -> Path | None:
        p = self.path
        if self._fp:
            try:
                self._fp.flush()
                self._fp.close()
            except Exception:
                pass
        self._fp = None
        self._writer = None
        return p
