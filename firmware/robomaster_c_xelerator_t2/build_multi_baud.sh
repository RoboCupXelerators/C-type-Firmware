#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"

# Build firmware with different CAN baud rates
# APB1 = 42MHz, BS1=10TQ, BS2=3TQ, SJW=1TQ
# baud = 42M / (Prescaler * (1+10+3)) = 42M / (Prescaler * 14)

declare -A BAUD_PRESCALER=(
    ["1M"]=3
    ["500K"]=6
    ["250K"]=12
    ["200K"]=15
    ["125K"]=24
)

restore_can_baud() {
    sed -i "s/hcan1.Init.Prescaler = [0-9]*/hcan1.Init.Prescaler = 3/" Core/Src/can.c
    sed -i "s/hcan2.Init.Prescaler = [0-9]*/hcan2.Init.Prescaler = 3/" Core/Src/can.c
}

trap restore_can_baud EXIT
for BAUD in "${!BAUD_PRESCALER[@]}"; do
    PRESCALER=${BAUD_PRESCALER[$BAUD]}
    echo "=== Building CAN ${BAUD} (Prescaler=${PRESCALER}) ==="

    # Patch can.c with new prescaler
    sed -i "s/hcan1.Init.Prescaler = [0-9]*/hcan1.Init.Prescaler = ${PRESCALER}/" Core/Src/can.c
    sed -i "s/hcan2.Init.Prescaler = [0-9]*/hcan2.Init.Prescaler = ${PRESCALER}/" Core/Src/can.c

    # Build
    rm -rf build
    bash build.sh > /dev/null 2>&1

    # Copy artifacts
    mkdir -p build_multi_baud/${BAUD}
    cp build/xelerator_t2_c_board.bin build_multi_baud/${BAUD}/
    cp build/xelerator_t2_c_board.hex build_multi_baud/${BAUD}/
    sha256sum build/xelerator_t2_c_board.bin > build_multi_baud/${BAUD}/sha256.txt

    echo "  BIN: build_multi_baud/${BAUD}/xelerator_t2_c_board.bin"
    echo "  SHA256: $(cat build_multi_baud/${BAUD}/sha256.txt)"
done

echo
echo "=== Multi-baud build complete ==="
ls -la build_multi_baud/*/xelerator_t2_c_board.bin
