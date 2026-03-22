#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def _host_dir() -> Path:
    return Path(__file__).resolve().parents[1]


sys.path.insert(0, str(_host_dir()))

from probe_analysis import (  # noqa: E402
    format_probe_summary,
    load_joint_probe_history,
    load_probe_history,
    summarize_probe_history,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze retension probe JSONL history by joint, pose, and date."
    )
    parser.add_argument(
        "--joint",
        help="Joint name, e.g. KNEE_RIGHT. Uses host/logs/probe_history/<joint>.jsonl",
    )
    parser.add_argument(
        "--path",
        help="Direct path to a probe-history JSONL file. Overrides --joint.",
    )
    parser.add_argument(
        "--history-dir",
        default=str(_host_dir() / "logs" / "probe_history"),
        help="Base directory containing per-joint JSONL files.",
    )
    parser.add_argument(
        "--bin-deg",
        type=float,
        default=5.0,
        help="Angle bin width in degrees for pose aggregation.",
    )
    parser.add_argument(
        "--last-days",
        type=int,
        help="Only include records newer than the last N days.",
    )
    parser.add_argument(
        "--source",
        help="Filter by source, e.g. flask_can_manager or jetson_telemetry.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Emit machine-readable JSON instead of text summary.",
    )
    args = parser.parse_args()

    if not args.path and not args.joint:
        parser.error("Specify --joint or --path")

    if args.path:
        history_path = Path(args.path)
        records = load_probe_history(history_path)
    else:
        history_path, records = load_joint_probe_history(args.history_dir, args.joint)

    summary = summarize_probe_history(
        records,
        angle_bin_deg=args.bin_deg,
        last_days=args.last_days,
        source=args.source,
    )

    if args.json:
        print(json.dumps({"path": str(history_path), **summary}, indent=2, sort_keys=True))
    else:
        print(f"File: {history_path}")
        print(format_probe_summary(summary))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
