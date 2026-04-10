# Host Application (Python/Flask)

Web-based control interface for the Alia humanoid joint controller system. This application provides real-time monitoring, PID tuning, auto-mapping, and multi-DOF movement control.

## Features

- **Real-time Joint Control**: Web UI for controlling multiple joints with up to 3 DOF each
- **PID Tuning**: Live adjustment of inner and outer loop PID parameters
- **Auto-Mapping**: Automatic calibration and linear equation generation
- **Multi-DOF Movements**: Coordinated movements with different path types and synchronization strategies
- **Encoder Testing**: Real-time encoder data streaming and visualization
- **Serial Management**: Multi-device serial communication with automatic port detection
- **Structured CAN Diagnostics**: Per-joint health, fault, event, and snapshot history persisted as JSONL

## Quick Start

**Requirements**: Python 3.11+

```bash
cd software/host
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python main.py
```

Open browser at: `http://localhost:5001`

## Joint Configuration Sync

The host application automatically loads joint definitions from `software/joint_config.json`, which is generated from the firmware's `config_presets.h`. This ensures firmware and host stay synchronized.

**To update joint configurations:**
1. Edit `software/firmware/joint_controller/include/config_presets.h`
2. Run: `python software/scripts/extract_joint_config.py`
3. Restart the host application

See `software/docs/JOINT_CONFIG_SYNC.md` for details.

For integrated experiments and Jetson bring-up, see
`software/docs/CAN_DIAGNOSTICS_OPERATIONAL_GUIDE.md`.

## Project Structure

- `main.py` - Application entry point
- `config.py` - Configuration (loaded from joint_config.json)
- `routes.py` - Flask API routes
- `diagnostic_history.py` - JSONL persistence for structured CAN diagnostics
- `bench_diagnostics.py` - Windowed summaries for bench diagnostic JSONL histories
- `serial_handler.py` - Serial protocol implementation
- `serial_manager.py` - Multi-device serial management
- `templates/` - HTML templates (Alia Joint Controller UI)
- `static/` - JavaScript, CSS, images
- `logs/` - Session logs (gitignored)
- `mapping_data/` - Auto-mapping data storage (local runtime data, gitignored)
- `can_config.json` - Saved CAN interface selection (created locally, gitignored)

## Development Workflows

### Using Makefile (Recommended)

From `software/` directory:
```bash
make run MODE=host
```
This automatically creates venv, installs dependencies, and runs the server.

### VS Code

Open the repository root or `software/host/`, select the local `.venv` interpreter, and run `main.py`.

### Firmware Builds (Optional)

Build firmware from the same Makefile:
```bash
make run MODE=firmware-controller ENV=pico2
```

If PlatformIO is not in PATH, specify venv:
```bash
PIO=host/.venv/bin/pio make run MODE=firmware-controller ENV=pico2
```

### Flash & Monitor via VS Code

Use PlatformIO extension in VS Code:
1. Open `software/firmware/joint_controller`
2. Use PIO toolbar to upload and monitor

## Environment Variables

- `FLASK_HOST` - Server host (default: `0.0.0.0`)
- `FLASK_PORT` - Server port (default: `5001`)
- `FLASK_DEBUG` - Debug mode (default: `False`)
- `BAUD_RATE` - Serial baud rate (default: `115200`)

## Notes

- Logs are written under `logs/` (gitignored)
- Structured diagnostic histories are written under `logs/diagnostic_history/` (gitignored)
- `mapping_data/` stores auto-mapping results (gitignored)
- `can_config.json` is created after a successful CAN connection and remains local (gitignored)
- Dev server uses `allow_unsafe_werkzeug=True` for Flask-SocketIO compatibility (local dev only)
- On macOS with a CANable flashed to `candleLight`, the reliable host path is
  currently `interface: candle` at `500 kbps`. The repo default still points to
  `slcan`, so bench runs may require a local config override.
- On the same macOS bench, USB enumeration can be visible at the OS level while
  process-local autodetect still returns no CAN adapters. If that happens,
  verify the device in system USB info first, then force the `candle` backend
  explicitly instead of relying on autodetect.

## Bench Diagnostics

For repeatable bench-motion diagnostics, record one exercise window and then
summarize only that time slice from the structured JSONL history:

```bash
cd software/host
python3 -m jetson_controller.exercise --joint hip_roll_bench_right --report-json /tmp/hip_roll_run.json ...
python3 tools/analyze_bench_diagnostics.py --report /tmp/hip_roll_run.json
```

The report anchors the movement window, and the analyzer summarizes:
- health counter deltas
- active fault counts
- event counts
- loop timing samples from `HEALTH_STATUS`

## License

MIT License - see repository root `LICENSE` file.
