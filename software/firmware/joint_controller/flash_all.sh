#!/usr/bin/env bash
# Flash firmware to all connected Pico 2 boards, one at a time.
# Uses each board's CDC serial port to trigger BOOTSEL on only that board,
# waits for it to re-enumerate, then moves to the next board.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PIO="${PIO:-$HOME/.platformio/penv/bin/pio}"
PYTHON="${PYTHON:-python3}"
PICO2_VID_HEX=0x2E8A
PICO2_PID_HEX=0x000F
REENUM_TIMEOUT_S="${REENUM_TIMEOUT_S:-20}"
POLL_INTERVAL_S="${POLL_INTERVAL_S:-0.5}"

if [ ! -x "$PIO" ]; then
    echo "PlatformIO CLI not found at: $PIO" >&2
    exit 1
fi

if ! "$PYTHON" - <<'PY' >/dev/null 2>&1
import serial.tools.list_ports
PY
then
    echo "python3 missing pyserial (serial.tools.list_ports)." >&2
    echo "Install it in the active Python environment before using flash_all.sh." >&2
    exit 1
fi

list_pico_rows() {
    "$PYTHON" - "$PICO2_VID_HEX" "$PICO2_PID_HEX" <<'PY'
import serial.tools.list_ports
import sys

vid = int(sys.argv[1], 16)
pid = int(sys.argv[2], 16)

for port in sorted(serial.tools.list_ports.comports(), key=lambda item: item.device):
    if port.vid != vid or port.pid != pid:
        continue
    ident = port.serial_number or port.location or port.hwid or port.device
    desc = (port.description or "").replace("\t", " ").strip()
    print(f"{ident}\t{port.device}\t{desc}")
PY
}

resolve_port_by_ident() {
    local ident="$1"
    local row
    while IFS=$'\t' read -r row_ident row_port _row_desc; do
        if [ "$row_ident" = "$ident" ]; then
            printf '%s\n' "$row_port"
            return 0
        fi
    done < <(list_pico_rows)
    return 1
}

wait_for_runtime_port() {
    local ident="$1"
    local timeout_s="$2"
    local start_s="$SECONDS"
    local port=""

    while (( SECONDS - start_s < timeout_s )); do
        if port="$(resolve_port_by_ident "$ident" 2>/dev/null)"; then
            printf '%s\n' "$port"
            return 0
        fi
        sleep "$POLL_INTERVAL_S"
    done

    return 1
}

echo "Building firmware..."
cd "$SCRIPT_DIR"
"$PIO" run -e pico2 --silent

mapfile -t BOARD_ROWS < <(list_pico_rows)

if [ "${#BOARD_ROWS[@]}" -eq 0 ]; then
    echo "No Pico 2 boards found (expected runtime USB VID:PID ${PICO2_VID_HEX#0x}:${PICO2_PID_HEX#0x})." >&2
    exit 1
fi

declare -a BOARD_IDS=()
echo "Found ${#BOARD_ROWS[@]} board(s):"
for row in "${BOARD_ROWS[@]}"; do
    IFS=$'\t' read -r ident port desc <<<"$row"
    BOARD_IDS+=("$ident")
    if [ -n "$desc" ]; then
        echo "  - $port [$ident] ($desc)"
    else
        echo "  - $port [$ident]"
    fi
done

for i in "${!BOARD_IDS[@]}"; do
    ident="${BOARD_IDS[$i]}"
    n=$((i + 1))

    current_port="$(resolve_port_by_ident "$ident" || true)"
    if [ -z "$current_port" ]; then
        echo "[$n/${#BOARD_IDS[@]}] Board [$ident] is no longer present on a runtime CDC port." >&2
        exit 1
    fi

    echo
    echo "[$n/${#BOARD_IDS[@]}] Flashing $current_port [$ident]..."
    "$PIO" run -e pico2 -t upload --upload-port "$current_port"

    echo "Waiting for board [$ident] to re-enumerate..."
    if ! new_port="$(wait_for_runtime_port "$ident" "$REENUM_TIMEOUT_S")"; then
        echo "Board [$ident] did not return on a runtime CDC port within ${REENUM_TIMEOUT_S}s." >&2
        echo "Stop here to avoid putting multiple boards in BOOTSEL at once." >&2
        exit 1
    fi

    echo "Board [$ident] is back on $new_port."
done

echo
echo "All ${#BOARD_IDS[@]} board(s) flashed."
