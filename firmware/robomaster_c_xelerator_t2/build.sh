#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")"

echo "=== Xelerator T2 C Board Firmware Build ==="
echo

# Check toolchain
if ! command -v arm-none-eabi-gcc &>/dev/null; then
    echo "ERROR: arm-none-eabi-gcc not found."
    echo "Install with: sudo apt-get install -y gcc-arm-none-eabi binutils-arm-none-eabi libnewlib-arm-none-eabi"
    exit 1
fi

# Create build directory
mkdir -p build
cd build

# Configure
cmake .. -DCMAKE_BUILD_TYPE=Release 2>&1

# Build
cmake --build . 2>&1

echo
echo "=== Build artifacts ==="
ls -la xelerator_t2_c_board.elf xelerator_t2_c_board.bin xelerator_t2_c_board.hex xelerator_t2_c_board.map 2>/dev/null || true
echo

echo "=== SHA256 ==="
sha256sum xelerator_t2_c_board.elf xelerator_t2_c_board.bin xelerator_t2_c_board.hex 2>/dev/null || true
