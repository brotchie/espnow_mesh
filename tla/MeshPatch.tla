-------------------------------- MODULE MeshPatch --------------------------------
(***************************************************************************)
(* Priority-3 spec from docs/proofs-plan.md: patch distribution (the       *)
(* HIL pull model).                                                         *)
(*                                                                          *)
(*   #6 Patch apply safety + agreement:                                     *)
(*        (a) PATCH_APPLY is broadcast only after all expected satellites   *)
(*            reported READY;                                               *)
(*        (b) any satellite that applies applies the correct patch - the    *)
(*            full-patch hash is verified before READY, so a satellite      *)
(*            never applies a mis-assembled/stale image, and all appliers   *)
(*            therefore agree on the same patch_hash.                       *)
(*   #11 Patch progress: given fair delivery, every expected satellite      *)
(*        eventually reaches READY (and applies).                           *)
(*                                                                          *)
(* Refines: hil_send_patch_offer, hil_handle_patch_block_request,          *)
(* hil_note_patch_ready, hil_send_patch_apply, hil_run_patch_case          *)
(* (controller) and handle_satellite_patch_offer,                          *)
(* handle_satellite_patch_block, hil_patch_send_ready,                     *)
(* handle_satellite_patch_apply (satellite) in src/espnow_mesh.c.          *)
(*                                                                          *)
(* Pull flow: OFFER (broadcast) -> BLOCK_REQ (per missing block) -> BLOCK   *)
(* -> per-block hash check -> when all blocks in, full-patch hash check ->  *)
(* READY -> (all ready) APPLY. Every message may be lost, reordered, or     *)
(* duplicated (frames persist in `net`).                                    *)
(*                                                                          *)
(* Two design knobs give the safety properties teeth:                       *)
(*   RequireAllReady - controller's APPLY guard. TRUE = shipped             *)
(*        (hil_patch_complete: ready == expected). FALSE = broken (apply    *)
(*        after any single READY); TLC violates the apply-safety invariant. *)
(*   CheckFullHash - satellite's READY gate. TRUE = shipped (staged hash    *)
(*        must equal the offered patch hash before READY, see the           *)
(*        hil_hash_bytes(s_patch_staging,...) check). FALSE = broken (ready *)
(*        once all block indices are present, skipping the full-patch       *)
(*        hash); with a stale block in play TLC violates apply-integrity.   *)
(*                                                                          *)
(* Block content is abstracted to good/stale. The controller always answers *)
(* with good blocks; a stale block models a replayed/mis-indexed block that *)
(* clears the per-block check but corrupts the assembly - exactly what the  *)
(* full-patch hash exists to catch. AllowStaleBlocks gates that adversary   *)
(* (off for the benign progress run, per the plan's "given fair delivery"). *)
(*                                                                          *)
(* Model limits (per the plan): no real time (offer/apply resend and the    *)
(* patch deadline are untimed events); one patch, one boot; block payloads  *)
(* abstracted to a good/stale content bit.                                  *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets

CONSTANTS
    Sats,             \* expected satellites, e.g. {1, 2}
    NumBlocks,        \* blocks per patch (block_count), e.g. 2
    RequireAllReady,  \* controller APPLY guard: TRUE = shipped
    CheckFullHash,    \* satellite READY gate: TRUE = shipped
    AllowStaleBlocks  \* enable the stale-block adversary

ASSUME Sats # {}
ASSUME NumBlocks \in Nat \ {0}
ASSUME RequireAllReady \in BOOLEAN
ASSUME CheckFullHash \in BOOLEAN
ASSUME AllowStaleBlocks \in BOOLEAN

Blocks == 1..NumBlocks

VARIABLES
    net,         \* set of in-flight frames (see per-kind records below)
    cReady,      \* controller readySet: sats it has seen READY from
    cApplySent,  \* controller has broadcast PATCH_APPLY
    sOffer,      \* [Sats -> BOOLEAN] offer accepted / patch offer active
    sRecv,       \* [Sats -> SUBSET Blocks] block indices stored
    sDirty,      \* [Sats -> BOOLEAN] assembly contains a stale block
    sReady,      \* [Sats -> BOOLEAN] PATCH_READY sent (full patch verified)
    sApplied     \* [Sats -> BOOLEAN] PATCH_APPLY acted on

vars == <<net, cReady, cApplySent, sOffer, sRecv, sDirty, sReady, sApplied>>

OfferF     == [k |-> "offer"]
ApplyF     == [k |-> "apply"]
ReqF(s, b) == [k |-> "req", s |-> s, b |-> b]
ReadyF(s)  == [k |-> "ready", s |-> s]
BlockF(s, b, good) == [k |-> "block", s |-> s, b |-> b, good |-> good]

Init ==
    /\ net = {}
    /\ cReady = {}
    /\ cApplySent = FALSE
    /\ sOffer = [s \in Sats |-> FALSE]
    /\ sRecv = [s \in Sats |-> {}]
    /\ sDirty = [s \in Sats |-> FALSE]
    /\ sReady = [s \in Sats |-> FALSE]
    /\ sApplied = [s \in Sats |-> FALSE]

-----------------------------------------------------------------------------
(* Controller.                                                              *)

CtlOffer ==
    /\ ~cApplySent
    /\ OfferF \notin net
    /\ net' = net \cup {OfferF}
    /\ UNCHANGED <<cReady, cApplySent, sOffer, sRecv, sDirty, sReady, sApplied>>

CtlRxReq(s, b) ==
    /\ ReqF(s, b) \in net
    /\ BlockF(s, b, TRUE) \notin net
    /\ net' = net \cup {BlockF(s, b, TRUE)}
    /\ UNCHANGED <<cReady, cApplySent, sOffer, sRecv, sDirty, sReady, sApplied>>

CtlRxReady(s) ==
    /\ ReadyF(s) \in net
    /\ s \notin cReady
    /\ cReady' = cReady \cup {s}
    /\ UNCHANGED <<net, cApplySent, sOffer, sRecv, sDirty, sReady, sApplied>>

CtlApply ==
    /\ ~cApplySent
    /\ IF RequireAllReady THEN cReady = Sats ELSE cReady # {}
    /\ cApplySent' = TRUE
    /\ net' = net \cup {ApplyF}
    /\ UNCHANGED <<cReady, sOffer, sRecv, sDirty, sReady, sApplied>>

-----------------------------------------------------------------------------
(* Satellite.                                                               *)

SatRxOffer(s) ==
    /\ OfferF \in net
    /\ ~sOffer[s]
    /\ sOffer' = [sOffer EXCEPT ![s] = TRUE]
    /\ UNCHANGED <<net, cReady, cApplySent, sRecv, sDirty, sReady, sApplied>>

SatReq(s, b) ==
    /\ sOffer[s] /\ ~sReady[s] /\ b \notin sRecv[s]
    /\ ReqF(s, b) \notin net
    /\ net' = net \cup {ReqF(s, b)}
    /\ UNCHANGED <<cReady, cApplySent, sOffer, sRecv, sDirty, sReady, sApplied>>

\* Replayed / mis-indexed block that passes the per-block hash check but is
\* not the correct content (models what the full-patch hash must catch).
StaleBlock(s, b) ==
    /\ AllowStaleBlocks
    /\ sOffer[s]
    /\ BlockF(s, b, FALSE) \notin net
    /\ net' = net \cup {BlockF(s, b, FALSE)}
    /\ UNCHANGED <<cReady, cApplySent, sOffer, sRecv, sDirty, sReady, sApplied>>

\* Store one block, then run the READY gate. On a completed-but-dirty
\* assembly the shipped full-hash check fails and re-arms the fetch (reset
\* mask), mirroring handle_satellite_patch_block's staged-hash mismatch path.
SatRxBlock(s, b) ==
    /\ ~sReady[s]
    /\ \E f \in net :
        /\ f.k = "block" /\ f.s = s /\ f.b = b
        /\ b \notin sRecv[s]
        /\ LET recv2    == sRecv[s] \cup {b}
               dirty2   == sDirty[s] \/ ~f.good
               complete == recv2 = Blocks
               gateOK   == IF CheckFullHash THEN (complete /\ ~dirty2) ELSE complete
               reset    == CheckFullHash /\ complete /\ dirty2
           IN
            /\ sRecv'  = [sRecv  EXCEPT ![s] = IF reset THEN {}    ELSE recv2]
            /\ sDirty' = [sDirty EXCEPT ![s] = IF reset THEN FALSE ELSE dirty2]
            /\ sReady' = [sReady EXCEPT ![s] = gateOK]
            /\ net'    = IF gateOK THEN net \cup {ReadyF(s)} ELSE net
            /\ UNCHANGED <<cReady, cApplySent, sOffer, sApplied>>

\* Re-send READY on a later offer if lost (hil_patch_request_next_missing).
SatResendReady(s) ==
    /\ sReady[s]
    /\ ReadyF(s) \notin net
    /\ net' = net \cup {ReadyF(s)}
    /\ UNCHANGED <<cReady, cApplySent, sOffer, sRecv, sDirty, sReady, sApplied>>

\* Apply only when locally READY (handle_satellite_patch_apply requires
\* s_patch_ready and a matching hash).
SatRxApply(s) ==
    /\ ApplyF \in net
    /\ sReady[s]
    /\ ~sApplied[s]
    /\ sApplied' = [sApplied EXCEPT ![s] = TRUE]
    /\ UNCHANGED <<net, cReady, cApplySent, sOffer, sRecv, sDirty, sReady>>

-----------------------------------------------------------------------------
Next ==
    \/ CtlOffer
    \/ \E s \in Sats, b \in Blocks : CtlRxReq(s, b)
    \/ \E s \in Sats : CtlRxReady(s)
    \/ CtlApply
    \/ \E s \in Sats : SatRxOffer(s)
    \/ \E s \in Sats, b \in Blocks : SatReq(s, b)
    \/ \E s \in Sats, b \in Blocks : StaleBlock(s, b)
    \/ \E s \in Sats, b \in Blocks : SatRxBlock(s, b)
    \/ \E s \in Sats : SatResendReady(s)
    \/ \E s \in Sats : SatRxApply(s)

Fairness ==
    /\ WF_vars(CtlOffer)
    /\ \A s \in Sats, b \in Blocks : WF_vars(CtlRxReq(s, b))
    /\ \A s \in Sats : WF_vars(CtlRxReady(s))
    /\ WF_vars(CtlApply)
    /\ \A s \in Sats : WF_vars(SatRxOffer(s))
    /\ \A s \in Sats, b \in Blocks : WF_vars(SatReq(s, b))
    /\ \A s \in Sats, b \in Blocks : WF_vars(SatRxBlock(s, b))
    /\ \A s \in Sats : WF_vars(SatResendReady(s))
    /\ \A s \in Sats : WF_vars(SatRxApply(s))

Spec     == Init /\ [][Next]_vars
FairSpec == Spec /\ Fairness

-----------------------------------------------------------------------------
(* Properties.                                                              *)

TypeOK ==
    /\ cReady \subseteq Sats
    /\ cApplySent \in BOOLEAN
    /\ sOffer \in [Sats -> BOOLEAN]
    /\ sRecv \in [Sats -> SUBSET Blocks]
    /\ sReady \in [Sats -> BOOLEAN]
    /\ sApplied \in [Sats -> BOOLEAN]

\* #6a: APPLY only after all expected satellites reported READY.
ApplySafety == cApplySent => cReady = Sats

\* #6b: any satellite that applied did so from a verified, clean assembly
\* (READY gate passed, no stale content). Corollary: all appliers agree on
\* the same correct patch hash.
ApplyIntegrity == \A s \in Sats : sApplied[s] => (sReady[s] /\ ~sDirty[s])

\* #11: every expected satellite eventually reaches READY, and then applies.
PatchProgress == <>[](\A s \in Sats : sReady[s])
ApplyProgress == <>[](\A s \in Sats : sApplied[s])

=============================================================================
