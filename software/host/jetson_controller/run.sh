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

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOST_DIR="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$HOST_DIR/.venv"
PYTHON_BIN="$VENV_DIR/bin/python"
SESSION_NAME="alia-jetson"

MODE="both"
APP_ARGS=()

print_usage() {
    cat <<EOF
Usage:
  ./run.sh              Launch controller + serial monitor in tmux
  ./run.sh --no-serial  Launch controller only
  ./run.sh --serial     Launch serial monitor only

Launcher options:
  --no-serial   Run only the CAN controller TUI
  --serial      Run only the serial monitor
  -h, --help    Show this help

Pass-through:
  In single-mode, remaining arguments are forwarded to the selected app.
  Examples:
    ./run.sh --no-serial --help
    ./run.sh --no-serial --config /path/to/controller.yaml
    ./run.sh --serial --port /dev/ttyACM0

  In dual mode, app-specific arguments are rejected to avoid ambiguous routing.
EOF
}

while [ "$#" -gt 0 ]; do
    case "$1" in
        --no-serial)
            MODE="can-only"
            shift
            ;;
        --serial)
            MODE="serial-only"
            shift
            ;;
        -h|--help)
            if [ "$MODE" = "both" ] && [ "${#APP_ARGS[@]}" -eq 0 ]; then
                print_usage
                exit 0
            fi
            APP_ARGS+=("$1")
            shift
            ;;
        --)
            shift
            while [ "$#" -gt 0 ]; do
                APP_ARGS+=("$1")
                shift
            done
            ;;
        *)
            APP_ARGS+=("$1")
            shift
            ;;
    esac
done

if [ "$MODE" = "both" ] && [ "${#APP_ARGS[@]}" -gt 0 ]; then
    echo "Error: app-specific arguments are only supported with --no-serial or --serial." >&2
    echo "Use './run.sh --help' for launcher options." >&2
    exit 2
fi

# --- Ensure venv & deps ---
cd "$HOST_DIR"
if [ ! -d "$VENV_DIR" ]; then
    echo "Creating virtual environment..."
    python3 -m venv "$VENV_DIR"
fi
"$PYTHON_BIN" -m pip install -q -r "$HOST_DIR/requirements.txt" 2>/dev/null

# --- Single-mode shortcuts ---
if [ "$MODE" = "can-only" ]; then
    exec "$PYTHON_BIN" -m jetson_controller -v "${APP_ARGS[@]}"
fi
if [ "$MODE" = "serial-only" ]; then
    exec "$PYTHON_BIN" -m jetson_controller.serial_monitor -v "${APP_ARGS[@]}"
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
    "cd '$HOST_DIR' && '$PYTHON_BIN' -u -m jetson_controller -v; printf 'Press Enter to close...'; read -r _"

# Split right pane for serial monitor
tmux split-window -h -t "$SESSION_NAME" \
    "cd '$HOST_DIR' && '$PYTHON_BIN' -u -m jetson_controller.serial_monitor -v; printf 'Press Enter to close...'; read -r _"

# Equal widths
tmux select-layout -t "$SESSION_NAME" even-horizontal

# Focus left pane (CAN TUI)
tmux select-pane -t "$SESSION_NAME":0.0

# Attach
exec tmux attach-session -t "$SESSION_NAME"
