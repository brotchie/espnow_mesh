# TLA+ model of the `espnow_mesh` protocol

This directory implements the formal-verification plan in
[`../docs/proofs-plan.md`](../docs/proofs-plan.md): a set of TLA+ specs that
model the emergent, distributed behavior of the `espnow_mesh` protocol — one
controller plus N satellites over a lossy, reordering, duplicating channel with
controller reboots and channel changes — and check the safety and liveness
properties the plan calls out. TLC explores every interleaving of a small model
exhaustively.

**Load-bearing caveat (from the plan):** these specs are hand-written from the C
in [`../src/`](../src). They characterize the *protocol design* and catch
logic/interaction errors; they do **not** certify the compiled binary. Each
module header names its **refinement target** — the C functions it stands in for
— so the correspondence stays reviewable. Bounds are small (2–3 satellites,
small sequence/boot ranges): strong evidence, not an ∀N proof.

## Running

```sh
make tools     # fetch tla2tools.jar into ./.tools/ (GitHub release, npm fallback)
make check     # run every check and assert its documented outcome (CI gate)
make clean     # remove TLC scratch output
```

`make check` returns non-zero if any check deviates from its expectation — a
`PASS` that regressed, *or* an expected counterexample that stopped reproducing.
Run a single model interactively with:

```sh
./run-tlc.sh MeshDelivery MeshDelivery          # <Module> <cfg-basename>
```

Requires a JRE (Java 11+). `tla2tools.jar` is not committed; `make tools`
fetches it. In a sandbox without `github.com`, `get-tools.sh` falls back to the
copy vendored in the `tlaplus-mcp` npm package, or set `TLA_TOOLS_URL`.

## Modules

| Module | Plan properties | Refines (C in `src/espnow_mesh.c` unless noted) |
| --- | --- | --- |
| `MeshChannel` | — (shared network model) | ESP-NOW radio + channel (`esp_now_send`, `espnow_recv_cb`, `espnow_mesh_packet.c`) |
| `MeshReplayRecovery` | #1 dedup soundness, #8 controller-loss recovery (R1) | `handle_satellite_data`, `check_controller_timeout`, `attempt_registration_if_due`; controller `init_boot_id`, reboot/channel change |
| `MeshDelivery` | #1, #2 no false completion, #5 bounded effort, #7 eventual delivery | `controller_run`, `begin_reliable_sequence`, `note_ack`, `send_due_unicast_retries`, `handle_satellite_data`/`send_satellite_ack` |
| `MeshTable` | #4 table conservation (B3 race) | `find_or_add_satellite`, `controller_note_registered_satellite` |
| `MeshPatch` | #6 apply safety + agreement, #11 patch progress | `hil_*_patch_*` (controller) and `handle_satellite_patch_*` (satellite) |
| `MeshDiscoverySync` | #3 lock integrity, #9 discovery progress, #10 time-sync liveness | `handle_satellite_data` lock check, `hil_softap_discovery_ready`, registration scan, `handle_time_response`, `check_controller_timeout` |

The MeshChannel module documents the network abstraction shared conceptually by
all specs: `net` is the set of every frame ever transmitted, so **reorder**
(receive any frame), **duplication** (receive a frame repeatedly), and **loss**
(never receive it) come for free; liveness runs add fairness to receive actions,
encoding the plan's "channel delivers infinitely often." Because every frame is
HMAC-authenticated (`espnow_mesh_packet.c`), an attacker can only *replay*
recorded frames, never forge new ones — so replay is redelivery from `net`.

## Checks and expected outcomes

Every check names its expected result. The negative ("VIOLATE") checks are not
failures — they are the plan's counterexamples and design-tension
demonstrations, run continuously so a future change that silently removes a
safeguard is caught. Each pairs the shipped design against a broken variant
selected by a spec constant, proving the safeguard is load-bearing.

| Check (`.cfg`) | Expected | What it shows |
| --- | --- | --- |
| `MeshReplayRecovery_immediate_replay` | ✗ DedupSoundness | Shipped boot-id switch is replayable (#1 counterexample). |
| `MeshReplayRecovery_immediate_benign` | ✓ | Shipped rule is sound and recovers under a benign channel. |
| `MeshReplayRecovery_kconsec_replay` | ✗ DedupSoundness | "k consecutive frames" hardening is still replayable. |
| `MeshReplayRecovery_sticky_benign` | ✗ DeliveryRecovery | Over-strict "never switch" rule breaks R1 recovery (the tension). |
| `MeshReplayRecovery_perboot_replay` | ✓ | Per-boot high-water mark defeats replay (#1). |
| `MeshReplayRecovery_perboot_benign` | ✓ | …without sacrificing R1 recovery (#8). |
| `MeshDelivery` | ✓ | Dedup, no false completion, bounded effort, eventual delivery. |
| `MeshDelivery_brokenack` | ✗ NoFalseCompletion | Dropping `note_ack`'s sequence filter miscounts a stale ACK (#2). |
| `MeshTable` | ✓ | Serialized `find_or_add` never duplicates a table entry (#4). |
| `MeshTable_b3race` | ✗ TableConservation | Torn check-then-insert (the B3 race) duplicates a MAC. |
| `MeshPatch` | ✓ | Apply-safety, agreement, and progress to READY/APPLY (#6, #11). |
| `MeshPatch_earlyapply` | ✗ ApplySafety | APPLY before all-ready breaks apply-safety (#6a). |
| `MeshPatch_nohashcheck` | ✗ ApplyIntegrity | READY without the full-patch hash applies corruption (#6b). |
| `MeshDiscoverySync` | ✓ | Discovery, lock integrity, time-sync + reboot re-sync (#9, #10). |
| `MeshDiscoverySync_lockintegrity` | ✓ | With two controllers, the satellite locks to exactly one (#3). |
| `MeshDiscoverySync_brokenlock` | ✗ LockIntegrity | Removing the controller check makes the satellite flap (#3). |

## The headline result: replay vs. R1 recovery

The plan's "highest-value use" is to validate a boot-id-switch fix *before*
writing C. `MeshReplayRecovery` models the replay weakness and four candidate
switch rules (constant `SwitchRule`), checking dedup-soundness (#1, under an
active replay adversary) **and** recovery-liveness (#8, under a benign fair
channel) *together*:

- **`immediate`** — the shipped C behavior. TLC produces the exploit:

  | state | event | `satBoot` | `accepted` | `reAccepted` |
  | --- | --- | --- | --- | --- |
  | 5 | accept genuine `(boot 1, seq 1)` | 1 | `{(1,1)}` | F |
  | 6 | controller reboots; accept `(boot 2, seq 1)` | 2 | `{(1,1),(2,1)}` | F |
  | 7 | **adversary replays `(boot 1, seq 1)`** → boot switch resets dedup → re-accept | 1 | `{(1,1),(2,1)}` | **T** |

- **`kconsec`** (require *k* consecutive new-boot frames) — still fails: the
  adversary replays the *same* recorded frame *k* times.
- **`sticky`** (never switch once locked) — closes the replay hole but
  **breaks R1**: after a genuine reboot the satellite is stuck on the dead boot
  id and never resumes delivery. This is exactly the tension the plan predicts.
- **`perboot`** (persisted per-boot-id high-water mark, the "monotonic counter"
  variant) — passes **both** #1 under replay and #8 under fair recovery. A
  replayed old frame restores that boot's stored high-water mark (so it is a
  duplicate), while a genuinely fresh boot id starts clean (so recovery works).

That last row is the machine-checked answer to the plan's open question — a rule
that satisfies dedup-soundness and recovery-liveness simultaneously — found
before touching `src/` (no C was changed for this work).

## Modeling limits (stated in each spec header too)

- **No real time.** "Eventually," not "within `RELIABLE_DEADLINE_MS`." Timeouts,
  scans, retry deadlines, and sync-due events are nondeterministic events, not
  durations.
- **Small models only.** Bounds are 1–3 satellites/controllers, small
  sequence/boot/channel ranges. Unbounded N would need TLAPS.
- **Fidelity gap.** These verify the design, not the binary. Liveness is only as
  meaningful as the fairness assumptions (a jammed radio violates "delivers
  infinitely often"). Time-sync convergence *quality* is out of scope — only
  "eventually (re)synced" is modeled; the PLL details live in
  `../src/espnow_mesh_time_sync.c` and the control-theory notes.
- **One channel abstraction.** `MeshDiscoverySync` shares a single mesh channel
  across controllers; reception is gated to live on-channel traffic (a real
  radio does not redeliver frames from a channel the controller has vacated).
