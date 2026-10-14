#!/usr/bin/env bash
set -e
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
BIN="$DIR/build/ch592_ble_uart.bin"
[ -f "$BIN" ] || { echo "Building first..."; make -C "$DIR"; }
echo "Put CH592 into ISP mode (bridge Pad 2 [BOOT/PB22] to Pad 1 [GND] while connecting USB/power), then flashing..."
wchisp flash -V "$BIN"
echo "Done! Disconnect BOOT bridge."
