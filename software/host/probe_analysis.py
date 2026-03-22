from __future__ import annotations

import json
import math
from collections import Counter, defaultdict
from datetime import datetime, timedelta, timezone
from pathlib import Path
from statistics import median
from typing import Any

from probe_history import _joint_stem


def _parse_timestamp(payload: dict[str, Any]) -> datetime | None:
    raw = payload.get("recorded_at_utc")
    if isinstance(raw, str) and raw:
        try:
            return datetime.fromisoformat(raw.replace("Z", "+00:00")).astimezone(
                timezone.utc
            )
        except ValueError:
            pass

    raw = payload.get("timestamp")
    if isinstance(raw, (int, float)):
        return datetime.fromtimestamp(float(raw), tz=timezone.utc)

    return None


def _load_jsonl(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    with path.open("r", encoding="utf-8") as handle:
        for line_no, line in enumerate(handle, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                payload = json.loads(text)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSONL at {path}:{line_no}: {exc}") from exc
            if not isinstance(payload, dict):
                raise ValueError(f"Invalid JSONL object at {path}:{line_no}")
            ts = _parse_timestamp(payload)
            if ts is not None:
                payload["_recorded_at"] = ts
            records.append(payload)
    records.sort(key=lambda item: item.get("_recorded_at") or datetime.min.replace(tzinfo=timezone.utc))
    return records


def load_probe_history(path: str | Path) -> list[dict[str, Any]]:
    return _load_jsonl(Path(path))


def load_joint_probe_history(base_dir: str | Path, joint_name: str) -> tuple[Path, list[dict[str, Any]]]:
    path = Path(base_dir) / f"{_joint_stem(joint_name)}.jsonl"
    return path, _load_jsonl(path)


def _bucket_angle(q_deg: float, bin_deg: float) -> float:
    if bin_deg <= 0:
        return round(q_deg, 3)
    return round(q_deg / bin_deg) * bin_deg


def _counter_dict(counter: Counter[str]) -> dict[str, int]:
    return {key: counter[key] for key in sorted(counter)}


def _median(values: list[float]) -> float | None:
    return None if not values else float(median(values))


def _range(values: list[float]) -> dict[str, float] | None:
    return None if not values else {"min": float(min(values)), "max": float(max(values))}


def _aggregate_group(records: list[dict[str, Any]]) -> dict[str, Any]:
    q_values = [float(r["q_deg"]) for r in records if isinstance(r.get("q_deg"), (int, float))]
    pre_values = [float(r["pre_ratio"]) for r in records if isinstance(r.get("pre_ratio"), (int, float))]
    delta_values = [float(r["delta_ratio"]) for r in records if isinstance(r.get("delta_ratio"), (int, float))]
    recruit_values = [float(r["recruit_norm"]) for r in records if isinstance(r.get("recruit_norm"), (int, float))]
    effort_values = [float(r["effort_pre"]) for r in records if isinstance(r.get("effort_pre"), (int, float))]
    classes = Counter(str(r.get("classification") or "UNKNOWN") for r in records)
    sources = Counter(str(r.get("source") or "unknown") for r in records)
    weak = Counter(str(r.get("weak_side") or "?") for r in records)

    return {
        "count": len(records),
        "q_deg_median": _median(q_values),
        "q_deg_range": _range(q_values),
        "pre_ratio_median": _median(pre_values),
        "pre_ratio_range": _range(pre_values),
        "delta_ratio_median": _median(delta_values),
        "delta_ratio_range": _range(delta_values),
        "recruit_norm_median": _median(recruit_values),
        "recruit_norm_range": _range(recruit_values),
        "effort_pre_median": _median(effort_values),
        "classification_counts": _counter_dict(classes),
        "source_counts": _counter_dict(sources),
        "weak_side_counts": _counter_dict(weak),
    }


def summarize_probe_history(
    records: list[dict[str, Any]],
    *,
    angle_bin_deg: float = 5.0,
    last_days: int | None = None,
    source: str | None = None,
) -> dict[str, Any]:
    filtered: list[dict[str, Any]] = []
    cutoff: datetime | None = None
    if last_days is not None:
        cutoff = datetime.now(timezone.utc) - timedelta(days=last_days)

    for record in records:
        if source and record.get("source") != source:
            continue
        ts = record.get("_recorded_at")
        if cutoff is not None:
            if not isinstance(ts, datetime) or ts < cutoff:
                continue
        filtered.append(record)

    source_counts = Counter(str(r.get("source") or "unknown") for r in filtered)
    class_counts = Counter(str(r.get("classification") or "UNKNOWN") for r in filtered)
    joint_names = sorted({str(r.get("joint_name") or "unknown_joint") for r in filtered})
    timestamps = [r["_recorded_at"] for r in filtered if isinstance(r.get("_recorded_at"), datetime)]

    angle_groups: dict[float, list[dict[str, Any]]] = defaultdict(list)
    daily_angle_groups: dict[tuple[str, float], list[dict[str, Any]]] = defaultdict(list)
    for record in filtered:
        q_deg = record.get("q_deg")
        if not isinstance(q_deg, (int, float)) or math.isnan(q_deg):
            continue
        bucket = _bucket_angle(float(q_deg), angle_bin_deg)
        angle_groups[bucket].append(record)
        ts = record.get("_recorded_at")
        day = ts.date().isoformat() if isinstance(ts, datetime) else "unknown"
        daily_angle_groups[(day, bucket)].append(record)

    angle_bins = []
    for bucket in sorted(angle_groups):
        payload = {"angle_bucket_deg": bucket}
        payload.update(_aggregate_group(angle_groups[bucket]))
        angle_bins.append(payload)

    daily_bins = []
    for (day, bucket) in sorted(daily_angle_groups):
        payload = {"day": day, "angle_bucket_deg": bucket}
        payload.update(_aggregate_group(daily_angle_groups[(day, bucket)]))
        daily_bins.append(payload)

    return {
        "total_records": len(filtered),
        "joint_names": joint_names,
        "source_counts": _counter_dict(source_counts),
        "classification_counts": _counter_dict(class_counts),
        "time_range": {
            "start_utc": timestamps[0].isoformat().replace("+00:00", "Z") if timestamps else None,
            "end_utc": timestamps[-1].isoformat().replace("+00:00", "Z") if timestamps else None,
        },
        "angle_bin_deg": angle_bin_deg,
        "angle_bins": angle_bins,
        "daily_angle_bins": daily_bins,
        "recent_records": [
            {
                "recorded_at_utc": (
                    record["_recorded_at"].isoformat().replace("+00:00", "Z")
                    if isinstance(record.get("_recorded_at"), datetime)
                    else record.get("recorded_at_utc")
                ),
                "q_deg": record.get("q_deg"),
                "pre_ratio": record.get("pre_ratio"),
                "delta_ratio": record.get("delta_ratio"),
                "recruit_norm": record.get("recruit_norm"),
                "classification": record.get("classification"),
                "source": record.get("source"),
            }
            for record in filtered[-10:]
        ],
    }


def _fmt_range(payload: dict[str, float] | None, digits: int = 3) -> str:
    if not payload:
        return "--"
    return f"{payload['min']:.{digits}f}..{payload['max']:.{digits}f}"


def format_probe_summary(summary: dict[str, Any]) -> str:
    lines: list[str] = []
    lines.append("Probe History Summary")
    lines.append(f"- Records: {summary['total_records']}")
    lines.append(f"- Joints: {', '.join(summary['joint_names']) or '--'}")
    lines.append(
        f"- Time range: {summary['time_range']['start_utc'] or '--'} -> {summary['time_range']['end_utc'] or '--'}"
    )

    source_counts = summary.get("source_counts") or {}
    if source_counts:
        lines.append(
            "- Sources: "
            + ", ".join(f"{name}={count}" for name, count in source_counts.items())
        )

    class_counts = summary.get("classification_counts") or {}
    if class_counts:
        lines.append(
            "- Classifications: "
            + ", ".join(f"{name}={count}" for name, count in class_counts.items())
        )

    lines.append("")
    lines.append("By Pose Bin")
    if summary.get("angle_bins"):
        for row in summary["angle_bins"]:
            pre_med = "--" if row["pre_ratio_median"] is None else f"{row['pre_ratio_median']:.3f}"
            d_med = "--" if row["delta_ratio_median"] is None else f"{row['delta_ratio_median']:.3f}"
            r_med = "--" if row["recruit_norm_median"] is None else f"{row['recruit_norm_median']:.3f}"
            lines.append(
                "  "
                f"q≈{row['angle_bucket_deg']:.1f}° "
                f"n={row['count']} "
                f"preR_med={pre_med} "
                f"ΔR_med={d_med} "
                f"recruit_med={r_med} "
                f"preR_rng={_fmt_range(row['pre_ratio_range'])} "
                f"cls={row['classification_counts']}"
            )
    else:
        lines.append("  --")

    lines.append("")
    lines.append("By Day And Pose")
    if summary.get("daily_angle_bins"):
        for row in summary["daily_angle_bins"]:
            pre_med = "--" if row["pre_ratio_median"] is None else f"{row['pre_ratio_median']:.3f}"
            d_med = "--" if row["delta_ratio_median"] is None else f"{row['delta_ratio_median']:.3f}"
            lines.append(
                "  "
                f"{row['day']} q≈{row['angle_bucket_deg']:.1f}° "
                f"n={row['count']} preR_med={pre_med} ΔR_med={d_med}"
            )
    else:
        lines.append("  --")

    lines.append("")
    lines.append("Recent Records")
    if summary.get("recent_records"):
        for row in summary["recent_records"]:
            pre = row["pre_ratio"]
            delta = row["delta_ratio"]
            recruit = row["recruit_norm"]
            pre_txt = "--" if not isinstance(pre, (int, float)) else f"{pre:.3f}"
            delta_txt = "--" if not isinstance(delta, (int, float)) else f"{delta:.3f}"
            recruit_txt = "--" if not isinstance(recruit, (int, float)) else f"{recruit:.3f}"
            q_deg = row["q_deg"]
            q_txt = "--" if not isinstance(q_deg, (int, float)) else f"{q_deg:.2f}"
            lines.append(
                "  "
                f"{row['recorded_at_utc'] or '--'} "
                f"q={q_txt}° preR={pre_txt} ΔR={delta_txt} recruit={recruit_txt} "
                f"{row['classification'] or 'UNKNOWN'} [{row['source'] or 'unknown'}]"
            )
    else:
        lines.append("  --")

    return "\n".join(lines)
