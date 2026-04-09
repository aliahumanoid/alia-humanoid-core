from __future__ import annotations

import json
import re
import statistics
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


def _joint_stem(joint_name: str) -> str:
    stem = re.sub(r"[^a-z0-9_-]+", "_", joint_name.strip().lower())
    return stem or "unknown_joint"


def load_diagnostic_history(path: Path) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    with Path(path).open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            rows.append(json.loads(line))
    return rows


def load_joint_diagnostic_history(
    history_dir: str | Path,
    joint_name: str,
) -> tuple[Path, list[dict[str, Any]]]:
    path = Path(history_dir) / f"{_joint_stem(joint_name)}.jsonl"
    return path, load_diagnostic_history(path)


def load_exercise_report(path: str | Path) -> dict[str, Any]:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def report_window(report: dict[str, Any]) -> tuple[float | None, float | None]:
    start = report.get("movement_started_at_unix")
    end = report.get("movement_finished_at_unix")
    if start is None:
        start = report.get("started_at_unix")
    if end is None:
        end = report.get("finished_at_unix")
    return start, end


def _within_window(record: dict[str, Any], start_unix: float | None, end_unix: float | None) -> bool:
    timestamp = record.get("timestamp")
    if not isinstance(timestamp, (float, int)):
        return False
    if start_unix is not None and timestamp < start_unix:
        return False
    if end_unix is not None and timestamp > end_unix:
        return False
    return True


def filter_diagnostic_history(
    records: list[dict[str, Any]],
    *,
    joint_name: str | None = None,
    start_unix: float | None = None,
    end_unix: float | None = None,
) -> list[dict[str, Any]]:
    filtered: list[dict[str, Any]] = []
    expected_joint = _joint_stem(joint_name) if joint_name is not None else None
    for record in records:
        if expected_joint is not None:
            record_joint = record.get("joint_name")
            if not isinstance(record_joint, str) or _joint_stem(record_joint) != expected_joint:
                continue
        if not _within_window(record, start_unix, end_unix):
            continue
        filtered.append(record)
    return filtered


def _counter_stats(records: list[dict[str, Any]], key: str, *, modulus: int | None = 256) -> dict[str, Any] | None:
    values = [int(row[key]) for row in records if isinstance(row.get(key), int)]
    if not values:
        return None
    if len(values) == 1:
        return {
            "first": values[0],
            "last": values[0],
            "min": values[0],
            "max": values[0],
            "delta": 0,
            "monotonic": True,
        }

    delta = 0
    monotonic = True
    previous = values[0]
    for current in values[1:]:
        if modulus is None:
            delta += current - previous
        elif current >= previous:
            delta += current - previous
        elif previous >= (modulus - 32) and current <= 32:
            delta += (modulus - previous) + current
        else:
            monotonic = False
        previous = current
    return {
        "first": values[0],
        "last": values[-1],
        "min": min(values),
        "max": max(values),
        "delta": delta if monotonic else None,
        "monotonic": monotonic,
    }


def _loop_stats(records: list[dict[str, Any]]) -> dict[str, Any]:
    loop_rows = [
        row for row in records
        if row.get("type") in {"health_loop_timing", "health_status"}
        and isinstance(row.get("loop_avg_us"), int)
        and isinstance(row.get("loop_max_us"), int)
        and isinstance(row.get("loop_budget_us"), int)
    ]
    if not loop_rows:
        return {
            "sample_count": 0,
            "avg_us_median": None,
            "avg_us_max": None,
            "max_us_median": None,
            "max_us_max": None,
            "budget_us_mode": None,
            "peak_budget_utilization_pct": None,
        }

    avg_values = [int(row["loop_avg_us"]) for row in loop_rows]
    max_values = [int(row["loop_max_us"]) for row in loop_rows]
    budget_values = [int(row["loop_budget_us"]) for row in loop_rows]
    budget_mode = Counter(budget_values).most_common(1)[0][0]
    peak_util = round((max(max_values) / budget_mode) * 100.0, 1) if budget_mode else None
    return {
        "sample_count": len(loop_rows),
        "avg_us_median": round(statistics.median(avg_values), 1),
        "avg_us_max": max(avg_values),
        "max_us_median": round(statistics.median(max_values), 1),
        "max_us_max": max(max_values),
        "budget_us_mode": budget_mode,
        "peak_budget_utilization_pct": peak_util,
    }


def summarize_bench_diagnostics(
    records: list[dict[str, Any]],
    *,
    joint_name: str | None = None,
    start_unix: float | None = None,
    end_unix: float | None = None,
) -> dict[str, Any]:
    filtered = filter_diagnostic_history(
        records,
        joint_name=joint_name,
        start_unix=start_unix,
        end_unix=end_unix,
    )
    if not filtered:
        return {
            "joint_name": joint_name,
            "total_records": 0,
            "type_counts": {},
            "event_counts": {},
            "active_fault_counts": {},
            "counter_stats": {},
            "loop_timing": _loop_stats([]),
            "window": {
                "start_unix": start_unix,
                "end_unix": end_unix,
                "duration_s": (end_unix - start_unix)
                if start_unix is not None and end_unix is not None else None,
            },
        }

    filtered.sort(key=lambda row: float(row["timestamp"]))
    health_rows = [row for row in filtered if row.get("type") == "health_status"]
    fault_rows = [row for row in filtered if row.get("type") == "fault_status"]
    event_rows = [row for row in filtered if row.get("type") == "event_notice"]

    active_fault_counts: Counter[str] = Counter()
    for row in fault_rows:
        for fault in row.get("active_faults", []):
            active_fault_counts[str(fault)] += 1

    event_counts = Counter(str(row.get("event")) for row in event_rows if row.get("event"))
    phase_counts = Counter(str(row.get("phase")) for row in health_rows if row.get("phase"))
    type_counts = Counter(str(row.get("type")) for row in filtered if row.get("type"))

    if health_rows:
        first_health = health_rows[0]
        last_health = health_rows[-1]
        counter_stats = {
            "host_can_tx_error_count": _counter_stats(health_rows, "host_can_tx_error_count"),
            "host_can_rx_error_count": _counter_stats(health_rows, "host_can_rx_error_count"),
            "motor_can_tx_error_count": _counter_stats(health_rows, "motor_can_tx_error_count"),
            "loop_overrun_count": _counter_stats(health_rows, "loop_overrun_count"),
            "watchdog_trip_count": _counter_stats(health_rows, "watchdog_trip_count"),
            "can_recovery_count": _counter_stats(health_rows, "can_recovery_count"),
            "fault_epoch": _counter_stats(health_rows, "fault_epoch"),
        }
    else:
        counter_stats = {}

    window_start = start_unix if start_unix is not None else float(filtered[0]["timestamp"])
    window_end = end_unix if end_unix is not None else float(filtered[-1]["timestamp"])
    summary_joint = joint_name or str(filtered[0].get("joint_name") or "unknown_joint")
    return {
        "joint_name": summary_joint,
        "total_records": len(filtered),
        "type_counts": dict(type_counts),
        "phase_counts": dict(phase_counts),
        "event_counts": dict(event_counts),
        "active_fault_counts": dict(active_fault_counts),
        "counter_stats": counter_stats,
        "loop_timing": _loop_stats(filtered),
        "window": {
            "start_unix": window_start,
            "end_unix": window_end,
            "duration_s": round(window_end - window_start, 3),
        },
    }


def format_bench_summary(summary: dict[str, Any]) -> str:
    if summary["total_records"] == 0:
        return "No diagnostic records matched the selected window."

    window = summary["window"]
    start = datetime.fromtimestamp(window["start_unix"], tz=timezone.utc).isoformat() if window["start_unix"] else "n/a"
    end = datetime.fromtimestamp(window["end_unix"], tz=timezone.utc).isoformat() if window["end_unix"] else "n/a"
    loop = summary["loop_timing"]
    counters = summary["counter_stats"]

    lines = [
        f"Joint: {summary['joint_name']}",
        f"Window: {start} -> {end} ({window['duration_s']} s)",
        f"Records: total={summary['total_records']} types={summary['type_counts']}",
        f"Counter stats: {counters}",
        f"Active faults: {summary['active_fault_counts']}",
        f"Events: {summary['event_counts']}",
        f"Phases: {summary['phase_counts']}",
        (
            "Loop timing: "
            f"samples={loop['sample_count']} "
            f"avg_us median/max={loop['avg_us_median']}/{loop['avg_us_max']} "
            f"max_us median/max={loop['max_us_median']}/{loop['max_us_max']} "
            f"budget={loop['budget_us_mode']} "
            f"peak_util={loop['peak_budget_utilization_pct']}%"
        ),
    ]
    return "\n".join(lines)
