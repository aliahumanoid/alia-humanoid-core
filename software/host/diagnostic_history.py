from __future__ import annotations

import json
import re
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict


def _joint_stem(joint_name: str) -> str:
    stem = re.sub(r"[^a-z0-9_-]+", "_", joint_name.strip().lower())
    return stem or "unknown_joint"


class DiagnosticHistoryWriter:
    """Append diagnostic-plane records to one JSONL file per joint."""

    def __init__(self, base_dir: Path, *, source: str) -> None:
        self.base_dir = Path(base_dir)
        self.source = source
        self._lock = threading.Lock()

    def append(self, record: Dict[str, Any]) -> Path:
        self.base_dir.mkdir(parents=True, exist_ok=True)
        payload = dict(record)
        payload["source"] = self.source
        payload["recorded_at_utc"] = (
            datetime.now(timezone.utc)
            .isoformat(timespec="seconds")
            .replace("+00:00", "Z")
        )
        joint_name = str(payload.get("joint_name") or payload.get("joint") or "unknown_joint")
        path = self.base_dir / f"{_joint_stem(joint_name)}.jsonl"
        with self._lock:
            with path.open("a", encoding="utf-8") as handle:
                json.dump(payload, handle, separators=(",", ":"), sort_keys=True)
                handle.write("\n")
        return path
