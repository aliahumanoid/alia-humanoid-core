#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOST_DIR="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$HOST_DIR/.venv"
PYTHON_BIN="$VENV_DIR/bin/python"

cd "$HOST_DIR"
if [ ! -d "$VENV_DIR" ]; then
  python3 -m venv "$VENV_DIR"
fi
"$PYTHON_BIN" -m pip install -q -r "$HOST_DIR/requirements.txt" 2>/dev/null

exec "$PYTHON_BIN" -u -m jetson_controller.exercise \
  --preflight-auto \
  --joint knee_right \
  --joint ankle_right \
  --duration-sec 300 \
  "$@"
