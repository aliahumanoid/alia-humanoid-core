#!/usr/bin/env bash
# Quick launcher for Alia Jetson Controller
# Double-click or run: ./run.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOST_DIR="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$HOST_DIR/.venv"
PYTHON_BIN="$VENV_DIR/bin/python"

cd "$HOST_DIR"

# Create venv if missing
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
fi

# Ensure host dependencies are installed into the project venv.
"$PYTHON_BIN" -m pip install -q -r "$HOST_DIR/requirements.txt"

exec "$PYTHON_BIN" -m jetson_controller -v "$@"
