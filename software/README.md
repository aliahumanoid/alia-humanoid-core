# Software Stack

This directory contains the public software stack for the Alia project.

Structure
```
software/
├─ host/                 # Python host application (Flask + Socket.IO)
└─ firmware/             # Embedded firmware projects (PlatformIO)
```

Current Status
- Firmware and host application integrated and operational
- Repository structure consolidated for public development
- Build system and tooling fully configured

Operational docs
- `docs/CAN_SYSTEM_ARCHITECTURE.md` - wire-level host↔controller protocol and transport architecture
- `docs/CAN_DIAGNOSTICS_OPERATIONAL_GUIDE.md` - CAN-first observability workflow for integrated experiments
- `docs/SET_IMPEDANCE_OPERATIONAL_GUIDE.md` - operational semantics for impedance commands

Dev Runner

Use the Makefile to switch between running the host app and building firmware.

- `make run MODE=host` — run Python host (creates venv, installs deps)
- `make run MODE=firmware-controller ENV=pico2` — build controller firmware
- `make fw-monitor-controller PORT=/dev/tty.usbmodemXXXXX` — serial monitor 115200

PlatformIO path (optional)
- If you don't have `pio` globally, PlatformIO is also installed inside the host venv.
- Pass `PIO=host/.venv/bin/pio` to Makefile targets, e.g.:
  - `make run MODE=firmware-controller ENV=pico2 PIO=host/.venv/bin/pio`

Flash & Monitor via VS Code
- You can use the PlatformIO VS Code extension to upload/monitor without CLI.
- Open `software/firmware/joint_controller` as a project in VS Code and use the PIO toolbar (Upload, Monitor).

Host notes
- The dev server uses Werkzeug with `allow_unsafe_werkzeug=True` for local runs only.
- For production-like tests, prefer a proper SocketIO server (e.g., `eventlet` or `gunicorn` + `gevent`).

Host quickstart (manual)
- `cd software/host && python3 -m venv .venv && source .venv/bin/activate`
- `pip install -r requirements.txt`
- `python main.py` then open `http://localhost:5001`
