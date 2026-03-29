#!/bin/bash
# LeadMe — Flash Arduino firmware
# Run this once whenever you update the Arduino sketch.
#
# Usage:
#   ./flash.sh                        # flash leadme_firmware (default), auto-detect port
#   ./flash.sh /dev/ttyUSB0           # specify port
#   ./flash.sh /dev/ttyUSB0 miniauto  # flash legacy miniauto_firmware instead

set -e

FIRMWARE="${2:-leadme}"
if [ "$FIRMWARE" = "miniauto" ]; then
    SKETCH="$(dirname "$0")/miniauto_firmware/miniauto_firmware.ino"
else
    SKETCH="$(dirname "$0")/leadme_firmware/leadme_firmware.ino"
fi
BOARD="arduino:avr:uno"
PORT="${1:-}"

# ── Install arduino-cli if not present ───────────────────────────────────────
if ! command -v arduino-cli &>/dev/null; then
    echo "Installing arduino-cli..."
    curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
    export PATH="$PATH:$HOME/bin"
fi

# ── Install Arduino AVR core if not present ───────────────────────────────────
if ! arduino-cli core list | grep -q "arduino:avr"; then
    echo "Installing Arduino AVR core..."
    arduino-cli core update-index
    arduino-cli core install arduino:avr
fi

# ── Auto-detect port if not specified ─────────────────────────────────────────
if [ -z "$PORT" ]; then
    PORT=$(arduino-cli board list | grep -E "ttyUSB|ttyACM" | awk '{print $1}' | head -1)
    if [ -z "$PORT" ]; then
        echo "ERROR: No Arduino found. Connect the miniAuto via USB and retry."
        echo "       Or specify the port: ./flash.sh /dev/ttyUSB0"
        exit 1
    fi
    echo "Auto-detected Arduino on $PORT"
fi

# ── Compile ────────────────────────────────────────────────────────────────────
echo "Compiling $SKETCH ..."
arduino-cli compile --fqbn "$BOARD" "$SKETCH"

# ── Upload ────────────────────────────────────────────────────────────────────
echo "Uploading to $PORT ..."
arduino-cli upload --fqbn "$BOARD" --port "$PORT" "$SKETCH"

echo ""
echo "Done. Arduino is running $(basename $SKETCH)."
echo "Start the laptop process: python3 cane/laptop_main.py"
