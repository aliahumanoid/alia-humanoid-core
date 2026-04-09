from __future__ import annotations

import json

from bench_diagnostics import (
    format_bench_summary,
    report_window,
    summarize_bench_diagnostics,
)


def test_bench_diagnostics_summarize_window():
    records = [
        {
            "type": "health_status",
            "joint_name": "hip_roll_bench_right",
            "timestamp": 100.0,
            "phase": "READY",
            "host_can_tx_error_count": 1,
            "host_can_rx_error_count": 2,
            "motor_can_tx_error_count": 3,
            "loop_overrun_count": 4,
            "watchdog_trip_count": 0,
            "can_recovery_count": 0,
            "fault_epoch": 10,
            "loop_avg_us": 900,
            "loop_max_us": 1200,
            "loop_budget_us": 2000,
        },
        {
            "type": "health_loop_timing",
            "joint_name": "hip_roll_bench_right",
            "timestamp": 101.0,
            "loop_avg_us": 1000,
            "loop_max_us": 1500,
            "loop_budget_us": 2000,
        },
        {
            "type": "fault_status",
            "joint_name": "hip_roll_bench_right",
            "timestamp": 101.5,
            "active_faults": ["MOTOR_CAN_WARN", "LOOP_OVERRUN"],
        },
        {
            "type": "event_notice",
            "joint_name": "hip_roll_bench_right",
            "timestamp": 102.0,
            "event": "LOOP_OVERRUN_BURST",
        },
        {
            "type": "health_status",
            "joint_name": "hip_roll_bench_right",
            "timestamp": 103.0,
            "phase": "READY",
            "host_can_tx_error_count": 1,
            "host_can_rx_error_count": 2,
            "motor_can_tx_error_count": 8,
            "loop_overrun_count": 11,
            "watchdog_trip_count": 0,
            "can_recovery_count": 1,
            "fault_epoch": 12,
            "loop_avg_us": 950,
            "loop_max_us": 1700,
            "loop_budget_us": 2000,
        },
    ]

    summary = summarize_bench_diagnostics(
        records,
        joint_name="hip_roll_bench_right",
        start_unix=100.0,
        end_unix=103.0,
    )

    assert summary["total_records"] == 5
    assert summary["counter_stats"]["motor_can_tx_error_count"]["delta"] == 5
    assert summary["counter_stats"]["loop_overrun_count"]["delta"] == 7
    assert summary["active_fault_counts"] == {
        "LOOP_OVERRUN": 1,
        "MOTOR_CAN_WARN": 1,
    }
    assert summary["event_counts"] == {"LOOP_OVERRUN_BURST": 1}
    assert summary["loop_timing"]["sample_count"] == 3
    assert summary["loop_timing"]["max_us_max"] == 1700
    assert summary["loop_timing"]["budget_us_mode"] == 2000
    assert "Joint: hip_roll_bench_right" in format_bench_summary(summary)


def test_bench_diagnostics_counter_delta_handles_uint8_wraparound():
    records = [
        {
            "type": "health_status",
            "joint_name": "hip_roll_bench_right",
            "timestamp": 100.0,
            "phase": "READY",
            "host_can_tx_error_count": 0,
            "host_can_rx_error_count": 0,
            "motor_can_tx_error_count": 250,
            "loop_overrun_count": 254,
            "watchdog_trip_count": 0,
            "can_recovery_count": 0,
            "fault_epoch": 254,
        },
        {
            "type": "health_status",
            "joint_name": "hip_roll_bench_right",
            "timestamp": 101.0,
            "phase": "READY",
            "host_can_tx_error_count": 0,
            "host_can_rx_error_count": 0,
            "motor_can_tx_error_count": 3,
            "loop_overrun_count": 2,
            "watchdog_trip_count": 0,
            "can_recovery_count": 0,
            "fault_epoch": 1,
        },
    ]

    summary = summarize_bench_diagnostics(records, joint_name="hip_roll_bench_right")
    assert summary["counter_stats"]["motor_can_tx_error_count"]["delta"] == 9
    assert summary["counter_stats"]["loop_overrun_count"]["delta"] == 4
    assert summary["counter_stats"]["fault_epoch"]["delta"] == 3


def test_bench_diagnostics_counter_stats_marks_non_monotonic_series():
    records = [
        {
            "type": "health_status",
            "joint_name": "hip_roll_bench_right",
            "timestamp": 100.0,
            "phase": "READY",
            "host_can_tx_error_count": 2,
            "host_can_rx_error_count": 0,
            "motor_can_tx_error_count": 10,
            "loop_overrun_count": 255,
            "watchdog_trip_count": 0,
            "can_recovery_count": 0,
            "fault_epoch": 6,
        },
        {
            "type": "health_status",
            "joint_name": "hip_roll_bench_right",
            "timestamp": 101.0,
            "phase": "READY",
            "host_can_tx_error_count": 1,
            "host_can_rx_error_count": 0,
            "motor_can_tx_error_count": 5,
            "loop_overrun_count": 255,
            "watchdog_trip_count": 0,
            "can_recovery_count": 0,
            "fault_epoch": 6,
        },
    ]

    summary = summarize_bench_diagnostics(records, joint_name="hip_roll_bench_right")
    assert summary["counter_stats"]["host_can_tx_error_count"]["delta"] is None
    assert summary["counter_stats"]["host_can_tx_error_count"]["monotonic"] is False


def test_bench_diagnostics_report_window_prefers_movement_interval(tmp_path):
    report_path = tmp_path / "report.json"
    report_path.write_text(
        json.dumps(
            {
                "started_at_unix": 10.0,
                "movement_started_at_unix": 20.0,
                "movement_finished_at_unix": 40.0,
                "finished_at_unix": 50.0,
            }
        ),
        encoding="utf-8",
    )

    start, end = report_window(json.loads(report_path.read_text(encoding="utf-8")))
    assert start == 20.0
    assert end == 40.0
