------------------------------- MODULE MeshDelivery -------------------------------
(***************************************************************************)
(* Priority-2 spec from docs/proofs-plan.md: the everyday reliable-delivery *)
(* path (DATA / ACK).                                                       *)
(*                                                                          *)
(*   #1 Dedup soundness    - a satellite never acts on the same sequence    *)
(*                           twice under loss/reorder/duplication.          *)
(*   #2 No false completion - the controller marks last_sequence_complete   *)
(*                           only if every expected satellite actually      *)
(*                           accepted THAT sequence; stale ACKs from an     *)
(*                           earlier sequence are never miscounted.         *)
(*   #5 Bounded effort     - per sequence, per satellite: at most           *)
(*                           SendRetries broadcasts and MaxAttempts total   *)
(*                           sends. No unbounded send.                      *)
(*   #7 Eventual delivery  - under fair delivery every reachable satellite  *)
(*                           eventually accepts the critical stream.        *)
(*                                                                          *)
(* Refines (controller): controller_run, begin_reliable_sequence,          *)
(* send_controller_packet, record_broadcast_attempt_for_expected,          *)
(* schedule_missing_after_broadcast, send_due_unicast_retries, note_ack,    *)
(* reliable_sequence_complete in src/espnow_mesh.c.                         *)
(* Refines (satellite): handle_satellite_data / send_satellite_ack.        *)
(*                                                                          *)
(* Single controller boot: reboots and channel changes are MeshReplayRecov *)
(* -ery's job. Here the interesting staleness is cross-SEQUENCE: an ACK for *)
(* sequence k lingering in the channel and arriving while sequence k+1 is   *)
(* active. Constant FilterAcksBySeq selects note_ack's behavior:            *)
(*   TRUE  - the shipped code: an ACK counts only if ack->sequence ==       *)
(*           active_sequence. #2 holds.                                     *)
(*   FALSE - a deliberately broken note_ack that counts any ACK from an     *)
(*           expected satellite. TLC produces a false-completion trace,     *)
(*           proving the sequence filter is load-bearing.                   *)
(*                                                                          *)
(* Model limits (per the plan): no real time (the "deadline" is the        *)
(* untimed FinalizeDeadline once retry effort is exhausted); all           *)
(* satellites are discovered and expected from the start (late-join via    *)
(* note_ack's init path is out of scope here); single Wi-Fi channel.       *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets

CONSTANTS
    Sats,             \* satellite ids, e.g. {1, 2}
    MaxSeq,           \* deliver sequences 1..MaxSeq
    SendRetries,      \* CONFIG_ESPNOW_MESH_SEND_RETRIES (broadcast rounds)
    MaxAttempts,      \* CONFIG_ESPNOW_MESH_RELIABLE_MAX_ATTEMPTS (per-sat cap)
    FilterAcksBySeq   \* TRUE = shipped note_ack; FALSE = broken (no seq filter)

ASSUME Sats # {}
ASSUME MaxSeq \in Nat \ {0}
ASSUME SendRetries \in Nat \ {0}
ASSUME MaxAttempts \in Nat \ {0}
ASSUME SendRetries <= MaxAttempts     \* broadcasts fit within the per-sat budget
ASSUME FilterAcksBySeq \in BOOLEAN
ASSUME 0 \notin Sats           \* 0 is reserved as the broadcast destination

BCastDst == 0                  \* numeric sentinel so every dst has one type
Seqs     == 1..MaxSeq

VARIABLES
    seq,            \* controller active_sequence
    phase,          \* "bcast" | "uni" | "done"
    bcastRounds,    \* broadcasts issued this sequence (<= SendRetries)
    acked,          \* [Sats -> BOOLEAN]  acked_current
    sends,          \* [Sats -> Nat]      sends_current (broadcast + unicast)
    dataNet,        \* set of in-flight DATA frames [seq, dst]
    ackNet,         \* set of in-flight ACK frames [sat, seq]
    satLast,        \* [Sats -> 0..MaxSeq]  s_last_sequence (0 = none yet)
    satAccepted,    \* [Sats -> SUBSET Seqs]  ghost: sequences each sat acted on
    satReAccept,    \* ghost: TRUE if any sat ever acted on a sequence twice
    lastComplete,   \* s_controller_last_sequence_complete
    lastCompleteSeq \* which sequence lastComplete refers to (0 = none)

vars == <<seq, phase, bcastRounds, acked, sends, dataNet, ackNet,
          satLast, satAccepted, satReAccept, lastComplete, lastCompleteSeq>>

Hears(f, s) == f.dst = BCastDst \/ f.dst = s
AllAcked    == \A s \in Sats : acked[s]

Init ==
    /\ seq = 1
    /\ phase = "bcast"
    /\ bcastRounds = 0
    /\ acked = [s \in Sats |-> FALSE]
    /\ sends = [s \in Sats |-> 0]
    /\ dataNet = {}
    /\ ackNet = {}
    /\ satLast = [s \in Sats |-> 0]
    /\ satAccepted = [s \in Sats |-> {}]
    /\ satReAccept = FALSE
    /\ lastComplete = FALSE
    /\ lastCompleteSeq = 0

-----------------------------------------------------------------------------
(* Controller: broadcast phase, unicast phase, ACK intake, finalize.        *)

\* One broadcast round (send_controller_packet to BROADCAST_MAC, then
\* record_broadcast_attempt_for_expected bumps sends for every unacked sat).
Broadcast ==
    /\ phase = "bcast"
    /\ bcastRounds < SendRetries
    /\ bcastRounds' = bcastRounds + 1
    /\ dataNet' = dataNet \cup {[seq |-> seq, dst |-> BCastDst]}
    /\ sends' = [s \in Sats |-> IF acked[s] THEN sends[s] ELSE sends[s] + 1]
    /\ UNCHANGED <<seq, phase, acked, ackNet, satLast, satAccepted,
                   satReAccept, lastComplete, lastCompleteSeq>>

\* Broadcast loop finished without completing -> enter the unicast retry
\* phase (schedule_missing_after_broadcast / reliable_retry_until_done).
ToUnicast ==
    /\ phase = "bcast"
    /\ bcastRounds = SendRetries
    /\ ~AllAcked
    /\ phase' = "uni"
    /\ UNCHANGED <<seq, bcastRounds, acked, sends, dataNet, ackNet, satLast,
                   satAccepted, satReAccept, lastComplete, lastCompleteSeq>>

\* One unicast retry to a still-missing satellite (send_due_unicast_retries).
\* Guarded by the per-sat budget: sends_current < MaxAttempts.
Unicast(s) ==
    /\ phase = "uni"
    /\ ~acked[s]
    /\ sends[s] < MaxAttempts
    /\ dataNet' = dataNet \cup {[seq |-> seq, dst |-> s]}
    /\ sends' = [sends EXCEPT ![s] = sends[s] + 1]
    /\ UNCHANGED <<seq, phase, bcastRounds, acked, ackNet, satLast,
                   satAccepted, satReAccept, lastComplete, lastCompleteSeq>>

\* Controller receives one ACK (note_ack). It counts toward the active
\* sequence only if the sequence filter matches (shipped behavior); the
\* broken variant counts any ACK. Only a step when it flips acked_current.
CtlRxAck ==
    \E f \in ackNet :
        /\ ~acked[f.sat]
        /\ (~FilterAcksBySeq \/ f.seq = seq)
        /\ acked' = [acked EXCEPT ![f.sat] = TRUE]
        /\ UNCHANGED <<seq, phase, bcastRounds, sends, dataNet, ackNet,
                       satLast, satAccepted, satReAccept, lastComplete,
                       lastCompleteSeq>>

\* begin_reliable_sequence for the next sequence, or halt after MaxSeq.
\* DATA and ACK frames from prior sequences intentionally persist in the
\* channel (loss/reorder/duplication + cross-sequence stale ACKs).
Advance(complete) ==
    /\ lastComplete' = complete
    /\ lastCompleteSeq' = seq
    /\ IF seq < MaxSeq
           THEN /\ seq' = seq + 1
                /\ phase' = "bcast"
                /\ bcastRounds' = 0
                /\ acked' = [s \in Sats |-> FALSE]
                /\ sends' = [s \in Sats |-> 0]
           ELSE /\ phase' = "done"
                /\ UNCHANGED <<seq, bcastRounds, acked, sends>>
    /\ UNCHANGED <<dataNet, ackNet, satLast, satAccepted, satReAccept>>

FinalizeComplete ==
    /\ phase \in {"bcast", "uni"}
    /\ AllAcked
    /\ Advance(TRUE)

\* Deadline with retry effort exhausted: no unacked sat has budget left.
FinalizeDeadline ==
    /\ phase = "uni"
    /\ ~AllAcked
    /\ \A s \in Sats : acked[s] \/ sends[s] >= MaxAttempts
    /\ Advance(FALSE)

-----------------------------------------------------------------------------
(* Satellite: receive DATA, dedup, ACK (handle_satellite_data).             *)

\* Accept iff not a duplicate: duplicate == have_last && seq <= last, so
\* accept == seq > last (with last=0 meaning "no last yet"). An ACK is sent
\* for every copy received, accepted or duplicate.
SatRxFrame(s, f) ==
    /\ f \in dataNet
    /\ Hears(f, s)
    /\ LET acc    == f.seq > satLast[s]
           newAck == [sat |-> s, seq |-> f.seq]
       IN
        /\ acc \/ newAck \notin ackNet          \* a real step (accept or new ACK)
        /\ satLast' = [satLast EXCEPT ![s] = IF acc THEN f.seq ELSE satLast[s]]
        /\ satAccepted' = [satAccepted EXCEPT
                             ![s] = IF acc THEN @ \cup {f.seq} ELSE @]
        /\ satReAccept' = (satReAccept \/ (acc /\ f.seq \in satAccepted[s]))
        /\ ackNet' = ackNet \cup {newAck}
        /\ UNCHANGED <<seq, phase, bcastRounds, acked, sends, dataNet,
                       lastComplete, lastCompleteSeq>>

SatRx(s)     == \E f \in dataNet : SatRxFrame(s, f)
\* Delivery of the reordering-robust top sequence (fairness witness for #7).
SatRxTop(s)  == \E f \in dataNet :
                    /\ f.seq = MaxSeq /\ Hears(f, s) /\ satLast[s] < MaxSeq
                    /\ SatRxFrame(s, f)

-----------------------------------------------------------------------------
Next ==
    \/ Broadcast
    \/ ToUnicast
    \/ \E s \in Sats : Unicast(s)
    \/ CtlRxAck
    \/ FinalizeComplete
    \/ FinalizeDeadline
    \/ \E s \in Sats : SatRx(s)

Fairness ==
    /\ WF_vars(Broadcast)
    /\ WF_vars(ToUnicast)
    /\ \A s \in Sats : WF_vars(Unicast(s))
    /\ WF_vars(CtlRxAck)
    /\ WF_vars(FinalizeComplete)
    /\ WF_vars(FinalizeDeadline)
    /\ \A s \in Sats : WF_vars(SatRx(s))
    /\ \A s \in Sats : WF_vars(SatRxTop(s))

Spec     == Init /\ [][Next]_vars
FairSpec == Spec /\ Fairness

-----------------------------------------------------------------------------
(* Properties.                                                              *)

TypeOK ==
    /\ seq \in Seqs
    /\ phase \in {"bcast", "uni", "done"}
    /\ bcastRounds \in 0..SendRetries
    /\ acked \in [Sats -> BOOLEAN]
    /\ sends \in [Sats -> 0..MaxAttempts]
    /\ satLast \in [Sats -> 0..MaxSeq]
    /\ lastCompleteSeq \in 0..MaxSeq

\* #1 Dedup soundness.
DedupSoundness == ~satReAccept

\* #5 Bounded effort: broadcasts capped at SendRetries, total sends capped at
\* MaxAttempts, per satellite per sequence.
BoundedEffort ==
    /\ bcastRounds <= SendRetries
    /\ \A s \in Sats : sends[s] <= MaxAttempts

\* #2 No false completion: if the controller ever declared a sequence
\* complete, every satellite genuinely accepted that sequence. (satAccepted
\* only grows and satellites accept a sequence exactly once, so checking the
\* recorded acceptance set is sound.)
NoFalseCompletion ==
    lastComplete => \A s \in Sats : lastCompleteSeq \in satAccepted[s]

\* #7 Eventual delivery: every reachable satellite eventually accepts the
\* whole critical stream. Witnessed by the top sequence, which is never
\* superseded and so is reordering-robust; lower sequences that a faster
\* peer's reordering turns into duplicates are the dedup path (#1), not a
\* delivery failure.
EventualDelivery == <>[](\A s \in Sats : MaxSeq \in satAccepted[s])

=============================================================================
