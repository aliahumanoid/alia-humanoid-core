from pathlib import Path
from unittest.mock import MagicMock

from flask import Flask

import routes
from motor_can_bench import DEFAULT_MOTOR_ID, DEFAULT_OUTPUT_RATIO


def build_client(mock_can_manager=None, mock_serial_manager=None):
    app = Flask(
        __name__,
        template_folder=str(Path(__file__).resolve().parent.parent / "templates"),
    )
    app.config["TESTING"] = True
    routes.CAN_AVAILABLE = True
    if mock_can_manager is None:
        mock_can_manager = MagicMock()
    if mock_serial_manager is None:
        mock_serial_manager = MagicMock()
    routes.register_routes(app, mock_serial_manager, mock_can_manager)
    return app.test_client(), mock_can_manager


def test_motor_test_page_renders():
    client, _ = build_client()

    response = client.get("/motor_test")

    assert response.status_code == 200
    assert b"Single Motor CAN Test" in response.data
    assert b"Motor ID 1" in response.data
    assert b"Bench Presets" in response.data


def test_motor_tuning_page_renders():
    client, _ = build_client()

    response = client.get("/motor_tuning")

    assert response.status_code == 200
    assert b"Motor CAN Tuning" in response.data
    assert b"READ ONLY" in response.data
    assert b"Read PID (0x30)" in response.data


def test_motor_test_status_returns_payload():
    client, mock_can_manager = build_client()
    mock_can_manager.get_motor_test_status.return_value = {
        "connected": True,
        "config": {"interface": "serial", "channel": "/dev/tty"},
        "output_ratio": DEFAULT_OUTPUT_RATIO,
        "recent_logs": [],
    }

    response = client.get("/api/motor_test/status")

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["status"] == "success"
    mock_can_manager.get_motor_test_status.assert_called_once_with(
        motor_id=DEFAULT_MOTOR_ID,
        output_ratio=DEFAULT_OUTPUT_RATIO,
    )


def test_motor_tuning_status_returns_payload():
    client, mock_can_manager = build_client()
    mock_can_manager.get_motor_tuning_status.return_value = {
        "connected": True,
        "config": {"interface": "serial", "channel": "/dev/tty"},
        "output_ratio": DEFAULT_OUTPUT_RATIO,
        "recent": {"pid": {"position_pid_kp": 100}},
    }

    response = client.get("/api/motor_tuning/status")

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["status"] == "success"
    mock_can_manager.get_motor_tuning_status.assert_called_once_with(
        motor_id=DEFAULT_MOTOR_ID,
        output_ratio=DEFAULT_OUTPUT_RATIO,
    )


def test_motor_tuning_read_uses_fixed_motor_id():
    client, mock_can_manager = build_client()
    mock_can_manager.motor_tuning_read.return_value = {
        "action": "pid",
        "motor_id": DEFAULT_MOTOR_ID,
        "decoded": {"position_pid_kp": 100},
    }

    response = client.post(
        "/api/motor_tuning/read",
        json={"action": "pid", "timeout_s": 0.25, "output_ratio": 11.0},
    )

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["status"] == "success"
    mock_can_manager.motor_tuning_read.assert_called_once_with(
        "pid",
        motor_id=DEFAULT_MOTOR_ID,
        timeout_s=0.25,
        output_ratio=11.0,
    )


def test_motor_test_action_uses_fixed_motor_id():
    client, mock_can_manager = build_client()
    mock_can_manager.motor_test_command.return_value = {
        "action": "torque",
        "motor_id": DEFAULT_MOTOR_ID,
        "decoded": {"speed_dps": 1234},
    }

    response = client.post(
        "/api/motor_test/action",
        json={"action": "torque", "value": 42, "timeout_s": 0.15, "output_ratio": 11.0},
    )

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["status"] == "success"
    mock_can_manager.motor_test_command.assert_called_once_with(
        "torque",
        motor_id=DEFAULT_MOTOR_ID,
        value=42,
        timeout_s=0.15,
        output_ratio=11.0,
    )


def test_motor_test_sweep_forwards_json_parameters():
    client, mock_can_manager = build_client()
    mock_can_manager.motor_test_run_sweep.return_value = {
        "status": "success",
        "samples": 150,
        "csv_name": "test.csv",
    }

    response = client.post(
        "/api/motor_test/sweep",
        json={
            "mode": "torque",
            "profile": "chirp",
            "duration_s": 12,
            "rate_hz": 100,
            "timeout_s": 0.05,
            "output_ratio": 10,
            "bias": 25,
            "amplitude": 5,
            "preload_s": 1,
            "frequency_hz": 1,
            "f0_hz": 0.2,
            "f1_hz": 8,
            "extra_commands": ["single", "multi"],
            "label": "motor1_chirp",
            "stop_at_end": True,
            "motor_on_before": True,
            "power_off_at_end": False,
        },
    )

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["status"] == "success"
    mock_can_manager.motor_test_run_sweep.assert_called_once_with(
        motor_id=DEFAULT_MOTOR_ID,
        mode="torque",
        profile="chirp",
        duration_s=12.0,
        rate_hz=100.0,
        timeout_s=0.05,
        output_ratio=10.0,
        bias=25.0,
        amplitude=5.0,
        preload_s=1.0,
        frequency_hz=1.0,
        f0_hz=0.2,
        f1_hz=8.0,
        extra_commands=["single", "multi"],
        label="motor1_chirp",
        stop_at_end=True,
        motor_on_before=True,
        power_off_at_end=False,
    )


def test_motor_test_logs_returns_entries():
    client, mock_can_manager = build_client()
    mock_can_manager.list_motor_test_logs.return_value = [
        {"name": "bench.csv", "size_bytes": 1024, "modified_ts": 123.0}
    ]

    response = client.get("/api/motor_test/logs?limit=5")

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["status"] == "success"
    assert payload["logs"][0]["name"] == "bench.csv"
    mock_can_manager.list_motor_test_logs.assert_called_once_with(limit=5)


def test_motor_test_log_analysis_returns_payload(tmp_path):
    client, mock_can_manager = build_client()
    log_dir = tmp_path / "logs"
    log_dir.mkdir()
    log_file = log_dir / "bench.csv"
    log_file.write_text(
        "motor_id,output_ratio,duration_s,rate_hz,timeout_s,bias,amplitude,preload_s,frequency_hz,f0_hz,f1_hz,label,t_s,mode,profile,target,temperature_c,iq_counts,iq_current_a_est,speed_dps,encoder_raw,actuator_abs_deg,motor_cmd_echo,actuator_abs_from_feedback_single_deg,motor_single_from_feedback_deg\n"
        "1,10,3,50,0.1,0,50,1,1,0.2,8,test,0.0,torque,step,0,30,0,0,0,100,0.5,161,0.5,5\n"
        "1,10,3,50,0.1,0,50,1,1,0.2,8,test,2.5,torque,step,50,30,10,0.1,1000,200,1.0,161,1.0,10\n",
        encoding="utf-8",
    )
    mock_can_manager.motor_test_log_dir.return_value = log_dir

    response = client.get("/api/motor_test/logs/bench.csv/analysis")

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["status"] == "success"
    assert payload["filename"] == "bench.csv"
    assert payload["row_count"] == 2
    assert payload["summary"]["speed_max"] == 1000.0


def test_motor_test_log_delete_removes_file(tmp_path):
    client, mock_can_manager = build_client()
    log_dir = tmp_path / "logs"
    log_dir.mkdir()
    log_file = log_dir / "delete_me.csv"
    log_file.write_text("a,b\n1,2\n", encoding="utf-8")
    mock_can_manager.motor_test_log_dir.return_value = log_dir

    response = client.delete("/api/motor_test/logs/delete_me.csv")

    assert response.status_code == 200
    payload = response.get_json()
    assert payload["status"] == "success"
    assert not log_file.exists()
