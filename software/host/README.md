# Host Application (Python/Flask)

Web-based control interface for the Alia humanoid joint controller system. This application provides real-time monitoring, PID tuning, auto-mapping, and multi-DOF movement control.

> **Note**: This application was migrated and integrated from a private motion-suite repository.

## Features

- **Real-time Joint Control**: Web UI for controlling multiple joints with up to 3 DOF each
- **PID Tuning**: Live adjustment of inner and outer loop PID parameters
- **Auto-Mapping**: Automatic calibration and linear equation generation
- **Multi-DOF Movements**: Coordinated movements with different path types and synchronization strategies
- **Encoder Testing**: Real-time encoder data streaming and visualization
- **Serial Management**: Multi-device serial communication with automatic port detection

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

## Project Structure

- `main.py` - Application entry point
- `config.py` - Configuration (loaded from joint_config.json)
- `routes.py` - Flask API routes
- `serial_handler.py` - Serial protocol implementation
- `serial_manager.py` - Multi-device serial management
- `templates/` - HTML templates (Alia Joint Controller UI)
- `static/` - JavaScript, CSS, images
- `logs/` - Session logs (gitignored)
- `mapping_data/` - Auto-mapping data storage (gitignored)

## Development Workflows

### Using Makefile (Recommended)

From `software/` directory:
```bash
make run MODE=host
```
This automatically creates venv, installs dependencies, and runs the server.

### VS Code Launch (One-Click)

1. Open the multi-root workspace: `alia-multi-root.code-workspace`
2. Use debug configuration: **Host (core): Debug Flask**
3. PreLaunchTask prepares venv + dependencies automatically

### Firmware Builds (Optional)

Build firmware from the same Makefile:
```bash
make run MODE=firmware-controller ENV=pico
make run MODE=firmware-encoders ENV=rpipico2
```

If PlatformIO is not in PATH, specify venv:
```bash
PIO=host/.venv/bin/pio make run MODE=firmware-controller ENV=pico
```

### Flash & Monitor via VS Code

Use PlatformIO extension in VS Code:
1. Open `software/firmware/joint_controller` or `joint_encoders`
2. Use PIO toolbar to upload and monitor

## Environment Variables

- `FLASK_HOST` - Server host (default: `0.0.0.0`)
- `FLASK_PORT` - Server port (default: `5001`)
- `FLASK_DEBUG` - Debug mode (default: `False`)
- `BAUD_RATE` - Serial baud rate (default: `115200`)

## Notes

- Logs are written under `logs/` (gitignored)
- `mapping_data/` stores auto-mapping results (gitignored)
- Dev server uses `allow_unsafe_werkzeug=True` for Flask-SocketIO compatibility (local dev only)

## License

MIT License - see repository root `LICENSE` file.
