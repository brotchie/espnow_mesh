# Host unit tests

Fast, hardware-free tests for the pure, self-contained logic in the component.
Each test compiles the relevant module(s) against the lightweight stubs in
`stubs/` (no ESP-IDF required) and runs assertion-based checks.

```sh
./test/host/run.sh
```

Coverage:

- `test_seq.c` — the wrap-safe serial-number duplicate test (`seq_is_duplicate`).
- `test_packet.c` — channel clamping and header finalize/validate/classify
  (built with authentication disabled).
- `test_pll.c` — time-sync PLL acquisition, tracking, and outlier rejection.

These run in CI (`.github/workflows/ci.yml`) alongside `idf.py` builds of the
examples across esp32 / esp32s3 / esp32c3 and the HIL configuration.
