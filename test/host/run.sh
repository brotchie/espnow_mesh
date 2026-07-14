#!/bin/sh
# Host unit tests for the pure, self-contained logic in the espnow_mesh
# component. Compiles the relevant modules against lightweight stubs (no
# ESP-IDF required) and runs assertion-based tests. Returns non-zero on any
# compile or test failure.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../../src"
CC="${CC:-cc}"
CFLAGS="-std=gnu17 -Wall -Wextra -Werror -Wno-unused-parameter -I$HERE/stubs -I$SRC"
FAIL=0

run() {
    name=$1
    shift
    bin="$HERE/$name.bin"
    if ! $CC $CFLAGS "$@" "$HERE/stubs.c" -o "$bin"; then
        echo "COMPILE FAILED: $name"
        FAIL=1
        return
    fi
    if ! "$bin"; then
        FAIL=1
    fi
}

run test_seq "$HERE/test_seq.c"

run test_packet "$HERE/test_packet.c" "$SRC/espnow_mesh_packet.c" \
    -DCONFIG_ESPNOW_MESH_PAYLOAD_BYTES=64 \
    -DCONFIG_ESPNOW_MESH_ID=0x45534E4D \
    -DCONFIG_ESPNOW_MESH_AUTH_ENABLE=0 \
    -DCONFIG_ESPNOW_MESH_HIL_TEST_ENABLE=0

run test_pll "$HERE/test_pll.c" "$SRC/espnow_mesh_time_sync.c" \
    -DCONFIG_ESPNOW_MESH_TIME_SYNC_PLL_ENABLE=1 \
    -DCONFIG_ESPNOW_MESH_TIME_SYNC_PLL_LOCK_SAMPLES=6 \
    -DCONFIG_ESPNOW_MESH_TIME_SYNC_PLL_PHASE_GAIN_SHIFT=3 \
    -DCONFIG_ESPNOW_MESH_TIME_SYNC_PLL_FREQ_GAIN_SHIFT=8 \
    -DCONFIG_ESPNOW_MESH_TIME_SYNC_PLL_MAX_FREQ_PPB=100000 \
    -DCONFIG_ESPNOW_MESH_TIME_SYNC_PLL_OUTLIER_US=3000 \
    -DCONFIG_ESPNOW_MESH_TIME_SYNC_PLL_DELAY_MARGIN_US=5000 \
    -DCONFIG_ESPNOW_MESH_TIME_SYNC_PLL_RESET_AFTER_REJECTS=8

if [ "$FAIL" = 0 ]; then
    echo "ALL HOST TESTS PASSED"
else
    echo "HOST TESTS FAILED"
fi
exit $FAIL
