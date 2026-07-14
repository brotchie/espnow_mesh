#!/bin/sh
# Build and run the RX-path fuzz targets under libFuzzer + AddressSanitizer +
# UndefinedBehaviorSanitizer. Requires clang with the sanitizer runtimes.
#
# RUNS controls iterations per target (default: a short CI smoke run). Set e.g.
# RUNS=5000000 for a longer local campaign. Returns non-zero on any finding.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
SRC="$HERE/../../../src"
INC="$HERE/../../../include"
CC="${CC:-clang}"
RUNS="${RUNS:-200000}"
SAN="-fsanitize=fuzzer,address,undefined -fno-sanitize-recover=all -g -O1"
# quiet/ shadows the stub esp_log.h to keep fuzzer output readable.
CFLAGS="$SAN -I$HERE/quiet -I$HERE/../stubs -I$SRC -I$INC"
FAIL=0

echo "== building fuzz_rx =="
if $CC $CFLAGS \
    -DCONFIG_ESPNOW_MESH_PAYLOAD_BYTES=64 -DCONFIG_ESPNOW_MESH_ID=0x45534E4D \
    -DCONFIG_ESPNOW_MESH_AUTH_ENABLE=0 -DCONFIG_ESPNOW_MESH_HIL_TEST_ENABLE=0 \
    "$HERE/fuzz_rx.c" "$SRC/espnow_mesh_packet.c" "$HERE/../stubs.c" \
    -o "$HERE/fuzz_rx"; then
    mkdir -p "$HERE/corpus_rx"
    "$HERE/fuzz_rx" -runs="$RUNS" -max_len=260 "$HERE/corpus_rx" || FAIL=1
else
    echo "fuzz_rx build failed"; FAIL=1
fi

echo "== building fuzz_patch =="
if $CC $CFLAGS -include "$HERE/fuzz_config.h" \
    "$HERE/fuzz_patch.c" "$SRC/espnow_mesh_packet.c" "$SRC/espnow_mesh_time_sync.c" \
    "$SRC/espnow_mesh_sync_output.c" "$HERE/fuzz_defs.c" \
    -o "$HERE/fuzz_patch"; then
    mkdir -p "$HERE/corpus_patch"
    "$HERE/fuzz_patch" -runs="$RUNS" -max_len=140 "$HERE/corpus_patch" || FAIL=1
else
    echo "fuzz_patch build failed"; FAIL=1
fi

[ "$FAIL" = 0 ] && echo "FUZZ SMOKE PASSED ($RUNS runs/target)" || echo "FUZZ FINDINGS OR BUILD FAILURE"
exit $FAIL
