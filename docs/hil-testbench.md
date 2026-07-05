# HIL Test Bench Notes

The HIL mode uses the synchronized GPIO as an application marker instead of the
normal 10 Hz square wave. The controller schedules commands in controller
monotonic time. Satellites apply at the requested mesh timestamp and emit one
marker pulse per accepted case.

The current HIL cases cover:

| Case | Name | Behavior |
| --- | --- | --- |
| 1 | `softap_channel_discovery` | Satellites start on random channels, discover the controller SoftAP, then accept ESP-NOW traffic |
| 2 | `patch_select_no_loss` | Patch-select command, no injected loss |
| 3 | `param_set_no_loss` | Parameter update, no injected loss |
| 4 | `ack_loss_recovery` | Patch-select with deterministic ACK loss |
| 5 | `data_loss_recovery` | Parameter update with deterministic command RX loss |
| 6 | `combined_loss_recovery` | Command RX loss, ACK loss, and duplicate broadcast |
| 7 | `time_sync_degraded` | Parameter update while time-sync responses are heavily dropped |
| 8 | `patch_distribution_no_loss` | Fragmented patch transfer with no injected loss |
| 9 | `patch_distribution_block_loss` | Fragmented patch transfer with block RX loss |
| 10 | `patch_distribution_request_ready_loss` | Fragmented patch transfer with request and READY loss |

Analyze a capture with the logic-analyzer scripts from `led_patterns`:

```sh
python3 /Users/brotchie/Documents/led_patterns/tools/hil/analyze_espnow_mesh_hil.py \
  --sample-csv /path/to/samples.csv \
  --channels CH0,CH1,CH2 \
  --reference CH0 \
  --samplerate 500kHz \
  --expected-cases 10 \
  --json-out /path/to/hil-report.json
```

A passing run has exactly one marker pulse per case on each expected channel,
no unmatched duplicates, and satellite marker edges within the accepted skew
window relative to the controller reference.
