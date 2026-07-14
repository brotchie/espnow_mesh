--------------------------- MODULE MeshReplayRecovery ---------------------------
(***************************************************************************)
(* Priority-1 spec from docs/proofs-plan.md: the recovery + replay pair.   *)
(*                                                                          *)
(*   Property #1  Dedup soundness   - no satellite acts on the same        *)
(*                                    (boot_id, sequence) twice.           *)
(*   Property #8  Controller-loss recovery (R1) - a controller that goes   *)
(*                                    silent on the satellite's channel    *)
(*                                    (reboot onto a new channel) is       *)
(*                                    eventually rediscovered and          *)
(*                                    delivery resumes.                    *)
(*                                                                          *)
(* Refines (satellite): handle_satellite_data, check_controller_timeout,   *)
(* attempt_registration_if_due in src/espnow_mesh.c.                       *)
(* Refines (controller): controller_run's DATA broadcast, init_boot_id,    *)
(* the registration SoftAP (configure_registration_ap), and the HIL        *)
(* random-start-channel reboot behavior.                                   *)
(*                                                                          *)
(* The point of this spec (see "Highest-value use" in the plan) is the     *)
(* boot-id switch rule, selected by the constant SwitchRule:               *)
(*                                                                          *)
(*   "immediate" - the shipped C behavior: any valid frame whose           *)
(*                 controller_boot_id differs from the adopted one         *)
(*                 replaces it and clears s_have_last_sequence             *)
(*                 (handle_satellite_data). Against a replay adversary     *)
(*                 TLC produces the documented exploit: replaying one old  *)
(*                 authenticated frame flips the satellite back to a stale *)
(*                 boot id, resets dedup, and the frame is accepted a      *)
(*                 second time.                                            *)
(*   "kconsec"   - proposed hardening: only switch boot ids after KConsec  *)
(*                 consecutive frames carrying the same new boot id.       *)
(*                 TLC shows this is still unsound: the adversary replays  *)
(*                 the SAME recorded frame KConsec times.                  *)
(*   "sticky"    - a deliberately over-strict rule: never switch boot ids  *)
(*                 once one is adopted. Closes the replay hole but TLC     *)
(*                 shows it breaks R1: after a genuine controller reboot   *)
(*                 the satellite never accepts the new session, so         *)
(*                 delivery is dead forever. This is the tension the plan  *)
(*                 predicts.                                               *)
(*   "perboot"   - proposed fix: keep a per-boot-id high-water mark        *)
(*                 (persisted dedup memory, the "monotonic counter"        *)
(*                 variant). Switching back to a previously seen boot id   *)
(*                 restores its last sequence instead of resetting, so     *)
(*                 replays are duplicates; genuinely fresh boot ids start  *)
(*                 clean, so R1 recovery is preserved. TLC passes both     *)
(*                 properties.                                             *)
(*                                                                          *)
(* Model limits (per the plan's "Limits" section):                         *)
(*  - No real time: the controller timeout is an event enabled only while  *)
(*    the controller is genuinely unreachable (different channel), not a   *)
(*    duration. Spurious timeouts are benign (the satellite re-locks) and  *)
(*    are not modeled.                                                     *)
(*  - Boot ids are modeled as fresh integers 1..MaxBoot. The C code uses   *)
(*    random 32-bit ids (init_boot_id); the model assumes no collision.    *)
(*  - The lock is to a single controller; radio source-address spoofing    *)
(*    is out of scope (but note: the frame HMAC does not bind the radio    *)
(*    source address that handle_satellite_data locks on - see the        *)
(*    README).                                                             *)
(*  - ACKs and the controller's retry bookkeeping are elided here; they    *)
(*    are covered by MeshDelivery. A "sequence" here is one broadcast      *)
(*    DATA frame.                                                          *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets

CONSTANTS
    MaxBoot,          \* controller boot ids 1..MaxBoot (reboot increments)
    MaxSeq,           \* sequences 1..MaxSeq per boot
    Channels,         \* Wi-Fi channels, e.g. {1, 2}
    InitChannel,      \* controller's initial channel
    SwitchRule,       \* "immediate" | "kconsec" | "sticky" | "perboot"
    KConsec,          \* k for SwitchRule = "kconsec"
    ReplayAdversary   \* TRUE: recorded frames can be re-injected on any
                      \* channel at any time; FALSE: only frames of the
                      \* current controller boot, on the channel they were
                      \* sent on, are deliverable (benign dup/reorder)

ASSUME SwitchRule \in {"immediate", "kconsec", "sticky", "perboot"}
ASSUME KConsec \in Nat \ {0}
ASSUME InitChannel \in Channels
ASSUME ReplayAdversary \in BOOLEAN

NoBoot == 0
NoSeq  == 0
Boots  == 1..MaxBoot
Seqs   == 1..MaxSeq

VARIABLES
    net,        \* set of every DATA frame ever transmitted (see MeshChannel)
    ctlBoot,    \* controller s_boot_id (abstracted to 1..MaxBoot)
    ctlSeq,     \* last sequence broadcast in the current boot
    ctlChan,    \* controller channel = registration SoftAP channel
    locked,     \* satellite s_have_controller
    satChan,    \* satellite's current channel
    satBoot,    \* adopted boot id (s_controller_boot_id); NoBoot = none
    lastSeq,    \* s_last_sequence; NoSeq = s_have_last_sequence FALSE
    bootSeen,   \* "perboot" rule: high-water mark per boot id
    candBoot,   \* "kconsec" rule: candidate new boot id
    candCnt,    \* "kconsec" rule: consecutive frames seen for candBoot
    accepted,   \* ghost: set of <<boot, seq>> the satellite ever accepted
    reAccepted  \* ghost: TRUE once some <<boot, seq>> was accepted twice

vars == <<net, ctlBoot, ctlSeq, ctlChan, locked, satChan, satBoot, lastSeq,
          bootSeen, candBoot, candCnt, accepted, reAccepted>>

Frame(b, s, c) == [boot |-> b, seq |-> s, chan |-> c]

Init ==
    /\ net = {}
    /\ ctlBoot = 1 /\ ctlSeq = 0 /\ ctlChan = InitChannel
    /\ locked = FALSE
    /\ satChan \in Channels          \* satellite may start on the wrong channel
    /\ satBoot = NoBoot /\ lastSeq = NoSeq
    /\ bootSeen = [b \in Boots |-> NoSeq]
    /\ candBoot = NoBoot /\ candCnt = 0
    /\ accepted = {} /\ reAccepted = FALSE

-----------------------------------------------------------------------------
(* Controller.                                                              *)

satVars == <<locked, satChan, satBoot, lastSeq, bootSeen, candBoot, candCnt,
             accepted, reAccepted>>

\* Broadcast the next critical DATA frame (controller_run / send_controller_packet).
\* Retransmissions need no separate action: frames persist in net.
CtlSend ==
    /\ ctlSeq < MaxSeq
    /\ ctlSeq' = ctlSeq + 1
    /\ net' = net \cup {Frame(ctlBoot, ctlSeq + 1, ctlChan)}
    /\ UNCHANGED <<ctlBoot, ctlChan>>
    /\ UNCHANGED satVars

\* Reboot: mint a fresh boot id (init_boot_id) and possibly come up on a new
\* channel (CONFIG_ESPNOW_MESH_HIL_RANDOM_START_CHANNEL_ENABLE); the
\* registration SoftAP moves with it.
CtlReboot ==
    /\ ctlBoot < MaxBoot
    /\ ctlBoot' = ctlBoot + 1
    /\ ctlSeq' = 0
    /\ ctlChan' \in Channels
    /\ UNCHANGED net
    /\ UNCHANGED satVars

-----------------------------------------------------------------------------
(* Satellite.                                                               *)

\* Which recorded frames can arrive at the satellite right now.
\* Benign radios only redeliver current-session traffic on its own channel
\* (dup/reorder window << reboot interval); a replay adversary re-injects
\* anything it ever captured, on whatever channel the satellite is on.
Audible(f) ==
    IF ReplayAdversary
        THEN TRUE
        ELSE f.boot = ctlBoot /\ f.chan = satChan

\* Boot-id handling for a frame f, per SwitchRule. "adopt" = the frame's
\* boot id becomes (or already is) the satellite's adopted boot id and the
\* frame proceeds to the dedup check, mirroring handle_satellite_data.
FirstBoot     == satBoot = NoBoot
SameBoot(f)   == satBoot = f.boot
Adopts(f) ==
    CASE SwitchRule \in {"immediate", "perboot"} -> TRUE
      [] SwitchRule = "sticky"  -> FirstBoot \/ SameBoot(f)
      [] SwitchRule = "kconsec" ->
            \/ FirstBoot
            \/ SameBoot(f)
            \/ (candBoot = f.boot /\ candCnt + 1 >= KConsec)

\* The dedup baseline after boot handling: the C code clears
\* s_have_last_sequence on a boot switch ("immediate"); the "perboot" fix
\* restores the stored high-water mark instead.
DedupBase(f) ==
    IF SameBoot(f) THEN lastSeq
    ELSE IF SwitchRule = "perboot" THEN bootSeen[f.boot]
    ELSE NoSeq

\* Would processing f accept it (act on it) rather than flag a duplicate?
\* Mirrors: duplicate == s_have_last_sequence && (int32_t)(seq - last) <= 0.
\* (The wrap-safe signed compare reduces to plain <= for the small bounded
\* sequence range modeled here.)
Accepts(f) == Adopts(f) /\ f.seq > DedupBase(f)

\* Receive one deliverable frame. Locking (s_have_controller) happens on any
\* valid DATA frame, then boot handling, then dedup - the exact order of
\* handle_satellite_data. The final conjunct restricts the action to
\* state-changing receptions; inert receptions (duplicates, sticky-ignored
\* frames) are stuttering steps and need no action.
RxData(f) ==
    /\ f \in net
    /\ Audible(f)
    /\ LET adopt  == Adopts(f)
           acc    == Accepts(f)
           switch == adopt /\ ~SameBoot(f) /\ ~FirstBoot
       IN
        /\ \/ ~locked                                   \* (re-)lock
           \/ adopt /\ (~SameBoot(f) \/ acc)            \* boot switch / accept
           \/ SwitchRule = "kconsec" /\ ~adopt /\ ~FirstBoot \* candidate progress
        /\ locked' = TRUE
        /\ satBoot' = IF adopt THEN f.boot ELSE satBoot
        /\ lastSeq' = IF acc THEN f.seq ELSE IF adopt THEN DedupBase(f) ELSE lastSeq
        /\ bootSeen' = IF acc /\ SwitchRule = "perboot"
                           THEN [bootSeen EXCEPT ![f.boot] = f.seq]
                           ELSE bootSeen
        /\ IF SwitchRule = "kconsec" /\ ~adopt /\ ~FirstBoot
               THEN /\ candBoot' = f.boot
                    /\ candCnt'  = IF candBoot = f.boot THEN candCnt + 1 ELSE 1
               ELSE /\ candBoot' = NoBoot               \* chain broken or switched
                    /\ candCnt'  = 0
        /\ accepted'   = IF acc THEN accepted \cup {<<f.boot, f.seq>>} ELSE accepted
        /\ reAccepted' = (reAccepted \/ (acc /\ <<f.boot, f.seq>> \in accepted))
        /\ UNCHANGED <<net, ctlBoot, ctlSeq, ctlChan, satChan>>

\* check_controller_timeout: drop the lock after sustained silence. Untimed
\* abstraction: enabled only while the controller is genuinely unreachable
\* (moved to a different channel). Boot-id and dedup state are kept, as in
\* the C code.
SatTimeout ==
    /\ locked
    /\ satChan # ctlChan
    /\ locked' = FALSE
    /\ UNCHANGED <<net, ctlBoot, ctlSeq, ctlChan, satChan, satBoot, lastSeq,
                   bootSeen, candBoot, candCnt, accepted, reAccepted>>

\* attempt_registration_if_due / scan_for_registration_ap: an unlocked
\* satellite eventually finds the registration SoftAP and adopts its channel.
SatDiscover ==
    /\ ~locked
    /\ satChan # ctlChan
    /\ satChan' = ctlChan
    /\ UNCHANGED <<net, ctlBoot, ctlSeq, ctlChan, locked, satBoot, lastSeq,
                   bootSeen, candBoot, candCnt, accepted, reAccepted>>

-----------------------------------------------------------------------------
Next ==
    \/ CtlSend
    \/ CtlReboot
    \/ \E f \in net : RxData(f)
    \/ SatTimeout
    \/ SatDiscover

\* Receive of a specific frame that is accepted (used for per-frame fairness:
\* "the channel delivers infinitely often" from the plan).
RxAccept(b, s) == \E f \in net : f.boot = b /\ f.seq = s /\ Accepts(f) /\ RxData(f)
RxAnyChange    == \E f \in net : RxData(f)

Fairness ==
    /\ WF_vars(CtlSend)
    /\ WF_vars(SatTimeout)
    /\ WF_vars(SatDiscover)
    /\ WF_vars(RxAnyChange)
    /\ \A b \in Boots, s \in Seqs : WF_vars(RxAccept(b, s))

Spec     == Init /\ [][Next]_vars
FairSpec == Spec /\ Fairness

-----------------------------------------------------------------------------
(* Properties.                                                              *)

\* #1 Dedup soundness: no (boot_id, sequence) pair is ever acted on twice.
DedupSoundness == ~reAccepted

\* Type/sanity invariant.
TypeOK ==
    /\ ctlBoot \in Boots /\ ctlSeq \in 0..MaxSeq /\ ctlChan \in Channels
    /\ locked \in BOOLEAN /\ satChan \in Channels
    /\ satBoot \in {NoBoot} \cup Boots
    /\ lastSeq \in {NoSeq} \cup Seqs
    /\ candBoot \in {NoBoot} \cup Boots
    /\ candCnt \in 0..KConsec
    /\ accepted \subseteq (Boots \X Seqs)

OnAir(b, s) == \E f \in net : f.boot = b /\ f.seq = s

\* #8a: a satellite locked to a controller that has left its channel
\* eventually unlocks (unless the controller comes back first):
\* the plan's [](silent_beyond_timeout => <>unlocked).
UnlockRecovery ==
    [](locked /\ satChan # ctlChan => <>(~locked \/ satChan = ctlChan))

\* #8b: delivery resumes. The plan's []<>controller_reachable => []<>locked,
\* strengthened to actual acceptance so a satellite pinned to a dead boot id
\* is caught. We witness "resumes delivery" with the current boot's highest
\* sequence: because the controller emits sequences monotonically and the
\* dedup high-water mark only rises, the top sequence is never superseded
\* and is therefore the reordering-robust progress marker (an individual
\* lower sequence legitimately becomes a duplicate if a higher one is
\* delivered first - that is property #7's business, not a recovery bug).
\* Reads: whenever the controller's current boot has put its top sequence on
\* air, that sequence is eventually accepted (unless a further reboot
\* supersedes the whole session first).
DeliveryRecovery ==
    \A b \in Boots :
        [](ctlBoot = b /\ OnAir(b, MaxSeq)
              => <>(<<b, MaxSeq>> \in accepted \/ ctlBoot # b))

=============================================================================
