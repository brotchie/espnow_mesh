# RX-path fuzzing

Coverage-guided fuzzing (libFuzzer) with AddressSanitizer + UndefinedBehavior-
Sanitizer for the only untrusted input in the system: received ESP-NOW frames.

```sh
# short smoke run (both targets)
./test/host/fuzz/run.sh
# longer local campaign
RUNS=20000000 ./test/host/fuzz/run.sh
```

Requires `clang` with the sanitizer runtimes (`libclang-rt-<version>-dev`).

## Targets

- **`fuzz_rx`** — drives `mesh_rx_packet_type()`, the entry point every frame
  hits first (header parse + classification). Broad coverage over arbitrary
  bytes and lengths; built with authentication disabled so inputs reach the
  length/type logic instead of bouncing off the HMAC tag.
- **`fuzz_patch`** — drives `handle_satellite_patch_block()`, the
  highest-severity path: it writes a wire-controlled slice into the fixed
  `s_patch_staging[]` buffer. The component TU is `#include`d to reach the
  static handler and patch state; a valid patch offer is primed once, then each
  input perturbs the block index / payload length / payload bytes so the fuzzer
  explores exactly the fields feeding the indexed store. ASan flags any
  out-of-bounds write the validators fail to prevent.

Both run in CI as a bounded smoke test (`.github/workflows/ci.yml`).

## Notes

- Fuzzing checks byte-level memory safety of the C parsers; it is complementary
  to the `tla/` specs (protocol logic) and the `test/host` unit tests.
- `fuzz_patch` exercises the real staging `memcpy` (the fuzzer log shows
  `HIL patch block stored ... block=N/8`), so a regression in the block
  validators would surface as an ASan out-of-bounds report.
