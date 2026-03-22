from __future__ import annotations

import json

from probe_analysis import load_probe_history, summarize_probe_history


def test_probe_analysis_groups_by_pose_and_day(tmp_path):
    path = tmp_path / "knee_right.jsonl"
    records = [
        {
            "joint_name": "KNEE_RIGHT",
            "q_deg": 69.8,
            "pre_ratio": 0.70,
            "delta_ratio": 0.01,
            "recruit_norm": 0.03,
            "classification": "NO_CORRECTION",
            "source": "jetson_telemetry",
            "recorded_at_utc": "2026-03-22T10:00:00Z",
        },
        {
            "joint_name": "KNEE_RIGHT",
            "q_deg": 70.2,
            "pre_ratio": 0.50,
            "delta_ratio": 0.04,
            "recruit_norm": 0.05,
            "classification": "NO_EFFECT",
            "source": "flask_can_manager",
            "recorded_at_utc": "2026-03-22T12:00:00Z",
        },
        {
            "joint_name": "KNEE_RIGHT",
            "q_deg": 79.9,
            "pre_ratio": 0.60,
            "delta_ratio": 0.02,
            "recruit_norm": 0.04,
            "classification": "NO_CORRECTION",
            "source": "jetson_telemetry",
            "recorded_at_utc": "2026-03-23T12:00:00Z",
        },
    ]
    with path.open("w", encoding="utf-8") as handle:
        for row in records:
            json.dump(row, handle)
            handle.write("\n")

    loaded = load_probe_history(path)
    summary = summarize_probe_history(loaded, angle_bin_deg=10.0)

    assert summary["total_records"] == 3
    assert summary["source_counts"] == {
        "flask_can_manager": 1,
        "jetson_telemetry": 2,
    }
    assert summary["classification_counts"] == {
        "NO_CORRECTION": 2,
        "NO_EFFECT": 1,
    }

    angle_bins = {row["angle_bucket_deg"]: row for row in summary["angle_bins"]}
    assert set(angle_bins) == {70.0, 80.0}
    assert angle_bins[70.0]["count"] == 2
    assert angle_bins[70.0]["pre_ratio_median"] == 0.60
    assert angle_bins[70.0]["delta_ratio_median"] == 0.025

    daily_bins = {
        (row["day"], row["angle_bucket_deg"]): row
        for row in summary["daily_angle_bins"]
    }
    assert ("2026-03-22", 70.0) in daily_bins
    assert ("2026-03-23", 80.0) in daily_bins
