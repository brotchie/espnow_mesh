-------------------------------- MODULE MeshTable --------------------------------
(***************************************************************************)
(* Safety invariant #4 from docs/proofs-plan.md: table conservation.       *)
(*                                                                          *)
(*   The controller's satellite table never holds two entries for the same *)
(*   MAC, even when registration and ACK-discovery are interleaved (the B3  *)
(*   race class, "fixed by routing registration through the mesh task").    *)
(*                                                                          *)
(* Refines: find_or_add_satellite in src/espnow_mesh.c, reached from two    *)
(* sources - note_ack / handle_time_request (the ESP-NOW RX path, already   *)
(* on the mesh task) and controller_note_registered_satellite (the Wi-Fi    *)
(* registration event). find_or_add_satellite is a check-then-insert:       *)
(*                                                                          *)
(*     index = find_satellite(mac);          // scan for existing entry     *)
(*     if (index >= 0) return index;                                        *)
(*     for (...) if (!used) { insert; return; }   // append to a free slot  *)
(*                                                                          *)
(* The check and the insert are NOT individually atomic. Correctness relies *)
(* on both callers running on the single mesh task, so the whole            *)
(* check-then-insert runs without interleaving. The B3 bug was a            *)
(* registration path that ran the insert off the mesh task; the fix routes  *)
(* the registration event through the same queue.                          *)
(*                                                                          *)
(* Constant Serialized selects the design:                                  *)
(*   TRUE  - shipped fix: each find_or_add runs as one atomic mesh-task     *)
(*           step. TableConservation holds.                                 *)
(*   FALSE - the B3 race: each path does a separate "observe absent" step   *)
(*           and a later "insert" step, so two paths can both observe the   *)
(*           MAC absent and both insert. TLC produces the duplicate-entry   *)
(*           counterexample.                                                *)
(*                                                                          *)
(* The table is modeled as a sequence of MAC entries (not a set) precisely  *)
(* so a double insert is representable and thus checkable.                  *)
(***************************************************************************)
EXTENDS Naturals, Sequences

CONSTANTS
    Macs,        \* satellite MACs that may be discovered, e.g. {1, 2}
    Serialized   \* TRUE = atomic find_or_add (fix); FALSE = B3 race

ASSUME Serialized \in BOOLEAN

\* Discovery sources that can learn a MAC (RX/ACK path and registration path).
Sources == {"ack", "reg"}

VARIABLES
    table,     \* sequence of MAC entries appended by find_or_add
    pending    \* [Macs -> [Sources -> "idle"|"observed"|"done"]]
               \* racy mode: "observed" = saw MAC absent, insert still pending

vars == <<table, pending>>

InTable(m) == \E i \in 1..Len(table) : table[i] = m

Init ==
    /\ table = << >>
    /\ pending = [m \in Macs |-> [src \in Sources |-> "idle"]]

-----------------------------------------------------------------------------
(* Serialized (shipped) design: one atomic check-then-insert per event.     *)

AtomicAdd(m, src) ==
    /\ Serialized
    /\ pending[m][src] = "idle"
    /\ pending' = [pending EXCEPT ![m][src] = "done"]
    /\ table' = IF InTable(m) THEN table ELSE Append(table, m)

-----------------------------------------------------------------------------
(* B3 race: check and insert are separate steps that can interleave.        *)

\* Step 1: this source observes the MAC absent (find_satellite returned -1).
Observe(m, src) ==
    /\ ~Serialized
    /\ pending[m][src] = "idle"
    /\ ~InTable(m)
    /\ pending' = [pending EXCEPT ![m][src] = "observed"]
    /\ UNCHANGED table

\* Step 2: having observed it absent earlier, insert into a free slot -
\* without re-checking, mirroring a check-then-insert torn by preemption.
Insert(m, src) ==
    /\ ~Serialized
    /\ pending[m][src] = "observed"
    /\ pending' = [pending EXCEPT ![m][src] = "done"]
    /\ table' = Append(table, m)

Next ==
    \E m \in Macs, src \in Sources :
        \/ AtomicAdd(m, src)
        \/ Observe(m, src)
        \/ Insert(m, src)

Spec == Init /\ [][Next]_vars

-----------------------------------------------------------------------------
\* #4: no MAC appears in the table twice.
TableConservation ==
    \A i, j \in 1..Len(table) : table[i] = table[j] => i = j

=============================================================================
