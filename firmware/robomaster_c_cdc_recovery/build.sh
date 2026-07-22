#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

echo "=== Xelerator C-Board CDC Recovery Firmware Build ==="
echo

if ! command -v arm-none-eabi-gcc &>/dev/null; then
    echo "ERROR: arm-none-eabi-gcc not found."
    exit 1
fi

mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release 2>&1
cmake --build . 2>&1

echo
echo "=== Build artifacts ==="
ls -la xelerator_cdc_recovery xelerator_cdc_recovery.bin xelerator_cdc_recovery.hex xelerator_cdc_recovery.map 2>/dev/null
echo
echo "=== SHA256 ==="
sha256sum xelerator_cdc_recovery xelerator_cdc_recovery.bin xelerator_cdc_recovery.hex 2>/dev/null
