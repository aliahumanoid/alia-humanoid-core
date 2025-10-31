# Software Stack

This directory will host the public software for the Alia project.

Structure
```
software/
├─ host/                 # Python host application (to be migrated)
└─ firmware/             # Embedded firmware projects (PlatformIO)
```

Current Status
- Firmware and host application integrated and operational
- Migration from private repository completed
- Build system and tooling fully configured

Dev Runner

Use the Makefile to switch between running the host app and building firmware.

- `make run MODE=host` — run Python host (creates venv, installs deps)
- `make run MODE=firmware-controller ENV=pico2` — build controller firmware
- `make run MODE=firmware-encoders ENV=rpipico2` — build encoders firmware
- `make fw-monitor-controller PORT=/dev/tty.usbmodemXXXXX` — serial monitor 115200

PlatformIO path (optional)
- If you don't have `pio` globally, PlatformIO is also installed inside the host venv.
- Pass `PIO=host/.venv/bin/pio` to Makefile targets, e.g.:
  - `make run MODE=firmware-controller ENV=pico2 PIO=host/.venv/bin/pio`
  - `make run MODE=firmware-encoders ENV=rpipico2 PIO=host/.venv/bin/pio`

Flash & Monitor via VS Code
- You can use the PlatformIO VS Code extension to upload/monitor without CLI.
- Open `software/firmware/joint_controller` or `joint_encoders` as projects in VS Code and use the PIO toolbar (Upload, Monitor).

Host notes
- The dev server uses Werkzeug with `allow_unsafe_werkzeug=True` for local runs only.
- For production-like tests, prefer a proper SocketIO server (e.g., `eventlet` or `gunicorn` + `gevent`).

Host quickstart (manual)
- `cd software/host && python3 -m venv .venv && source .venv/bin/activate`
- `pip install -r requirements.txt`
- `python main.py` then open `http://localhost:5001`
