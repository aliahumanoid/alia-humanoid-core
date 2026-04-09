#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def _host_dir() -> Path:
    return Path(__file__).resolve().parents[1]


sys.path.insert(0, str(_host_dir()))

from bench_diagnostics import (  # noqa: E402
    format_bench_summary,
    load_diagnostic_history,
    load_exercise_report,
    load_joint_diagnostic_history,
    report_window,
    summarize_bench_diagnostics,
)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze structured bench diagnostics for one exercise window."
    )
    parser.add_argument(
        "--joint",
        help="Joint name, e.g. hip_roll_bench_right. Uses host/logs/diagnostic_history/<joint>.jsonl",
    )
    parser.add_argument(
        "--path",
        help="Direct path to a diagnostic-history JSONL file. Overrides --joint.",
    )
    parser.add_argument(
        "--history-dir",
        default=str(_host_dir() / "logs" / "diagnostic_history"),
        help="Base directory containing per-joint diagnostic JSONL files.",
    )
    parser.add_argument(
        "--report",
        help="Exercise run report JSON from jetson_controller.exercise --report-json",
    )
    parser.add_argument("--start-unix", type=float, help="Window start timestamp (unix seconds).")
    parser.add_argument("--end-unix", type=float, help="Window end timestamp (unix seconds).")
    parser.add_argument("--json", action="store_true", help="Emit machine-readable JSON.")
    args = parser.parse_args()

    report = load_exercise_report(args.report) if args.report else None
    joint_name = args.joint
    if joint_name is None and report:
        selected = report.get("selected_joints") or []
        if len(selected) == 1:
            joint_name = str(selected[0])

    if not args.path and not joint_name:
        parser.error("Specify --joint or --path, or provide a single-joint --report")

    start_unix = args.start_unix
    end_unix = args.end_unix
    if report and (start_unix is None or end_unix is None):
        report_start, report_end = report_window(report)
        if start_unix is None:
            start_unix = report_start
        if end_unix is None:
            end_unix = report_end

    if args.path:
        history_path = Path(args.path)
        records = load_diagnostic_history(history_path)
    else:
        history_path, records = load_joint_diagnostic_history(args.history_dir, joint_name)

    summary = summarize_bench_diagnostics(
        records,
        joint_name=joint_name,
        start_unix=start_unix,
        end_unix=end_unix,
    )

    if args.json:
        print(json.dumps({"path": str(history_path), "report": args.report, **summary}, indent=2, sort_keys=True))
    else:
        print(f"File: {history_path}")
        if args.report:
            print(f"Report: {args.report}")
        print(format_bench_summary(summary))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
