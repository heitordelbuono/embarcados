#!/usr/bin/env bash
set -euo pipefail

PORT="${1:-/dev/ttyUSB0}"
BAUD="${2:-115200}"
LOG_DIR="${3:-logs}"

mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d-%H%M%S)"
LOG_FILE="$LOG_DIR/serial-$STAMP.log"

echo "Salvando monitor serial em: $LOG_FILE"
echo "Porta: $PORT | Baud: $BAUD"
echo "Para sair: Ctrl+C"

pio device monitor --port "$PORT" -b "$BAUD" | tee "$LOG_FILE"
