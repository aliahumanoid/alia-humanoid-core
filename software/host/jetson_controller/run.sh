#!/usr/bin/env bash
# ─── Alia Jetson Controller Launcher ────────────────────────────────
# Launches the Jetson controller TUI + serial monitor side by side
# in a tmux session.
#
# Usage:
#   ./run.sh              # both panels (CAN TUI + Serial monitor)
#   ./run.sh --no-serial  # CAN TUI only
#   ./run.sh --serial     # Serial monitor only
#   ./run.sh --no-serial --preflight-auto --joint knee_right
# ────────────────────────────────────────────────────────────────────

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
HOST_DIR="$(dirname "$SCRIPT_DIR")"
VENV_DIR="$HOST_DIR/.venv"
PYTHON_BIN="$VENV_DIR/bin/python"
SESSION_NAME="alia-jetson"

MODE="both"
APP_ARGS=()

shell_quote() {
    printf "'%s'" "$(printf "%s" "$1" | sed "s/'/'\\\\''/g")"
}

join_quoted_args() {
    local joined=""
    local arg
    for arg in "$@"; do
        if [ -n "$joined" ]; then
            joined="$joined "
        fi
        joined="$joined$(shell_quote "$arg")"
    done
    printf "%s" "$joined"
}

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
  Remaining arguments are forwarded only in explicit single-mode launches.
  Use --no-serial to target the CAN controller.
  Use --serial to target the serial monitor.
  Examples:
    ./run.sh --no-serial --joint ankle_right
    ./run.sh --no-serial --config /path/to/controller.yaml
    ./run.sh --no-serial --preflight-auto --joint knee_right
    ./run.sh --no-serial --preflight-serial /dev/cu.usbmodem21301 --joint knee_right
    ./run.sh --serial --port /dev/ttyACM0
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
    echo "error: pass-through arguments require --no-serial or --serial" >&2
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
    exec "$PYTHON_BIN" -m jetson_controller -v "${APP_ARGS[@]}"
fi

CAN_APP_ARGS="$(join_quoted_args "${APP_ARGS[@]}")"

# Kill existing session if any
tmux kill-session -t "$SESSION_NAME" 2>/dev/null || true

# Create session with CAN TUI in left pane
tmux new-session -d -s "$SESSION_NAME" -x 200 -y 50 \
    "cd '$HOST_DIR' && '$PYTHON_BIN' -u -m jetson_controller -v ${CAN_APP_ARGS}; printf 'Press Enter to close...'; read -r _"

# Split right pane for serial monitor
tmux split-window -h -t "$SESSION_NAME" \
    "cd '$HOST_DIR' && '$PYTHON_BIN' -u -m jetson_controller.serial_monitor -v; printf 'Press Enter to close...'; read -r _"

# Equal widths
tmux select-layout -t "$SESSION_NAME" even-horizontal

# Focus left pane (CAN TUI)
tmux select-pane -t "$SESSION_NAME":0.0

# Attach
exec tmux attach-session -t "$SESSION_NAME"
