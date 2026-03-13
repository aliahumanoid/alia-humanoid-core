#!/usr/bin/env bash
# ─── Alia Jetson Controller Launcher ────────────────────────────────
# Launches the Jetson controller TUI + serial monitor side by side
# in a tmux session.
#
# Usage:
#   ./run.sh              # both panels (CAN TUI + Serial monitor)
#   ./run.sh --no-serial  # CAN TUI only
#   ./run.sh --serial     # Serial monitor only
# ────────────────────────────────────────────────────────────────────

set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOST_DIR="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$HOST_DIR/.venv"
PYTHON_BIN="$VENV_DIR/bin/python"
SESSION_NAME="alia-jetson"

MODE="both"
for arg in "$@"; do
    case "$arg" in
        --no-serial) MODE="can-only" ;;
        --serial)    MODE="serial-only" ;;
    esac
done

# --- Ensure venv & deps ---
cd "$HOST_DIR"
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
fi
"$PYTHON_BIN" -m pip install -q -r "$HOST_DIR/requirements.txt" 2>/dev/null

# --- Single-mode shortcuts ---
if [ "$MODE" = "can-only" ]; then
    exec "$PYTHON_BIN" -m jetson_controller -v
fi
if [ "$MODE" = "serial-only" ]; then
    exec "$PYTHON_BIN" -m jetson_controller.serial_monitor -v
fi

# --- Dual mode: tmux split ---
if ! command -v tmux &>/dev/null; then
    echo "tmux not found — install with: brew install tmux"
    echo "Falling back to CAN TUI only."
    exec "$PYTHON_BIN" -m jetson_controller -v
fi

# Kill existing session if any
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# Create session with CAN TUI in left pane
tmux new-session -d -s "$SESSION_NAME" -x 200 -y 50 \
    "cd '$HOST_DIR' && '$PYTHON_BIN' -u -m jetson_controller -v; read -p 'Press Enter to close...'"

# Split right pane for serial monitor
tmux split-window -h -t "$SESSION_NAME" \
    "cd '$HOST_DIR' && '$PYTHON_BIN' -u -m jetson_controller.serial_monitor -v; read -p 'Press Enter to close...'"

# Equal widths
tmux select-layout -t "$SESSION_NAME" even-horizontal

# Focus left pane (CAN TUI)
tmux select-pane -t "$SESSION_NAME":0.0

# Attach
exec tmux attach-session -t "$SESSION_NAME"
