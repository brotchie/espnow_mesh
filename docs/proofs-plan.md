# Formal Verification Plan (TLA+)

This document plans a TLA+ model of the `espnow_mesh` protocol and the guarantees
worth checking with it. It is a plan, not a spec — no `.tla` files exist yet.

## Scope and intent

TLA+/TLC is the right tool for the *emergent, distributed* behavior of this
system: one controller plus N satellites communicating over a **lossy,
reordering, duplicating** channel, with controller reboots and channel changes.
TLC exhaustively explores every interleaving of a small model for **safety**
(invariants true in all reachable states) and checks **liveness** ("eventually
X") under stated **fairness** assumptions.

It is deliberately *not* used for: fixed-point PLL convergence (control theory —
see `docs`/Lean notes), real-time deadlines, or per-instruction C semantics.

### The load-bearing caveat

The TLA+ spec is hand-written from the C in `src/`. It characterizes the
**protocol design** and catches logic/interaction errors; it does not certify
the compiled binary. Its value is bounded by how faithfully the model mirrors
the code, so each module below names its **refinement target** (the C it stands
in for) to keep that correspondence explicit and reviewable.

## Model structure

Three composed specs plus a small config:

- **Controller** — `boot_id`, `active_sequence`, per-satellite
  `{expected, acked, sends}`, the broadcast→unicast retry loop, the SoftAP.
  Reboot action mints a fresh `boot_id`; channel-change action moves the AP.
  Refines: `controller_run` / `note_ack` / `send_due_unicast_retries` /
  `hil_run_patch_case`.
- **Satellite** — states `Unlocked / Discovering / Locked(boot, last_rx, last_seq)`,
  dedup state, time-sync state, registration/discovery, the controller-timeout
  recovery. Refines: `satellite_run` / `handle_satellite_data` /
  `check_controller_timeout` / `attempt_registration_if_due`.
- **Network** — nondeterministically drops, reorders, and duplicates in-flight
  frames; distinguishes broadcast vs unicast; models "wrong channel" as
  undeliverable until discovery. Refines: ESP-NOW radio + channel behavior.
- **Config (.cfg)** — small bounds (2–3 satellites, sequence numbers mod small
  N, bounded retry counts), the invariants, and the fairness constraints.

## Safety invariants (checked exhaustively by TLC)

1. **Dedup soundness.** No satellite acts on the same `(boot_id, sequence)` twice
   under arbitrary loss/reorder/duplication. Core correctness of `s_last_sequence`
   + the wrap-safe compare + boot-id reset.
2. **No false completion.** The controller sets `last_sequence_complete` only if
   every satellite counted as `expected_current` actually accepted *that*
   sequence — i.e. stale/replayed ACKs from other sequences or boots are never
   miscounted (`note_ack` boot-id / active-sequence filtering).
3. **Lock integrity.** A satellite never treats two different MACs as its
   controller at once, and never acts on data before it is locked (and, in
   SoftAP-required mode, before discovery). Refers to `s_have_controller`,
   `s_controller_mac`, `hil_softap_discovery_ready`.
4. **Table conservation.** The controller table never holds two entries for the
   same MAC, even with registration and ACK-discovery interleaved (the B3 race
   class, fixed by routing registration through the mesh task).
5. **Bounded effort.** Per sequence, per satellite: at most `SEND_RETRIES`
   broadcasts + `MAX_ATTEMPTS` unicasts. No unbounded send.
6. **Patch apply safety + agreement.** `PATCH_APPLY` is broadcast only after all
   expected satellites reported READY, and any satellite that applies applies the
   same `patch_hash` (full-patch hash verified before READY).

## Liveness properties (under fairness)

Fairness assumptions: the channel delivers infinitely often, timers eventually
fire, the mesh task keeps running.

7. **Eventual delivery.** A reachable satellite eventually accepts every critical
   sequence (reliability modulo real-time deadlines).
8. **Controller-loss recovery (R1) — headline.** If the controller goes
   permanently silent on the satellite's channel (reboot onto a new channel), the
   satellite eventually unlocks and resumes discovery; if the SoftAP is
   thereafter reachable, it eventually re-locks and resumes delivery.
   `[](silent_beyond_timeout => <>unlocked)` and
   `[]<>controller_reachable => []<>locked`. Also surfaces starvation paths
   (stuck scan, un-rearmed timer).
9. **Discovery progress.** A satellite starting on the wrong channel eventually
   finds the SoftAP and locks.
10. **Time-sync liveness.** Eventually `time_synced`, and eventually *re-synced*
    after a controller reboot (PLL reacquire path). Convergence quality is out of
    scope; "eventually synced" is not.
11. **Patch progress.** Given fair delivery, all expected satellites eventually
    reach READY (paired with the apply-safety invariant in #6).

## Highest-value use: validate a fix before writing C

Model the documented **replay weakness** (adversary replays old authenticated
frames with alternating `boot_id`s) and let TLC produce the concrete
counterexample to invariant #1 — turning a suspected exploit into a
machine-checked trace. Then model a proposed **hardening** (require *k*
consecutive frames of a new boot_id, or a persisted monotonic counter) and
re-check.

The payoff is the **tension** it exposes: too strict a boot-id-switch rule
closes the replay hole but breaks R1 recovery (a genuine reboot looks like an
attack). Checking dedup-soundness (#1) **and** recovery-liveness (#8) together
finds a rule that satisfies both — exactly the interaction that is easy to get
wrong by hand.

## Limits (state these in the spec header too)

- **No real time.** "Eventually," not "within `RELIABLE_DEADLINE_MS`." Timeouts
  are nondeterministic events, not durations.
- **Small models only.** 2–3 satellites, small sequence modulus, bounded
  retries: strong evidence, not an ∀N proof (unbounded N would need TLAPS).
- **Fidelity gap.** Verifies the design, not the binary; liveness is only as
  meaningful as the fairness assumptions (a jammed radio violates "delivers
  infinitely often").

## Recommended order

1. **Recovery + replay pair** (#8 + #1 + fix-validation) — the property the
   design most needs and where controller-reboot / channel-change / dedup
   interactions are hardest to reason about by hand. Exercises the R1 code.
2. **Reliable-delivery core** (#1, #2, #5, #7) — the everyday path.
3. **Patch distribution** (#6, #11) — most differentiated feature; larger state.
4. **Discovery + time-sync liveness** (#9, #10).

## Tooling

Install the TLA+ tools (`tla2tools.jar` / the VS Code TLA+ extension / the
Apalache symbolic checker for larger state spaces). None are currently present
in this repo's environment, so the first step also covers wiring a
`make check` / CI job that runs TLC headless against each `.cfg`.
