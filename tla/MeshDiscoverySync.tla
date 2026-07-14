----------------------------- MODULE MeshDiscoverySync -----------------------------
(***************************************************************************)
(* Priority-4 spec from docs/proofs-plan.md: discovery, lock integrity,    *)
(* and time-sync liveness.                                                  *)
(*                                                                          *)
(*   #3  Lock integrity  - a satellite never treats two different          *)
(*                         controller MACs as its controller at once, and  *)
(*                         never acts on data before it is locked (and, in  *)
(*                         SoftAP-required mode, before discovery).         *)
(*   #9  Discovery progress - a satellite starting on the wrong channel     *)
(*                         eventually finds the SoftAP and locks.           *)
(*   #10 Time-sync liveness - eventually time_synced, and eventually        *)
(*                         re-synced to the current epoch after a           *)
(*                         controller reboot (PLL reacquire path).          *)
(*                                                                          *)
(* Refines: handle_satellite_data (locking on s_have_controller /          *)
(* s_controller_mac, the "ignoring packet from extra controller" branch),  *)
(* hil_softap_discovery_ready, attempt_registration_if_due /               *)
(* scan_for_registration_ap / connect_to_registration_ap,                  *)
(* request_time_sync_if_due / handle_time_response,                        *)
(* check_controller_timeout in src/espnow_mesh.c.                          *)
(*                                                                          *)
(* Time sync is abstracted to one SatSync action that stands for a whole    *)
(* successful TIME_REQ -> TIME_RESP -> accepted-sample round; loss and      *)
(* high-delay rejection are folded into its nondeterministic enabling, with *)
(* fairness supplying the plan's "channel delivers infinitely often". A     *)
(* controller reboot changes its time epoch, so a sync taken against an old *)
(* boot is stale (mesh_have_time stays true, but the PLL must reacquire);   *)
(* "synced to the current epoch" is synced /\ syncedBoot = current boot.    *)
(*                                                                          *)
(* Knob EnforceLock selects handle_satellite_data's controller check:       *)
(*   TRUE  - shipped: only the locked controller's frames are acted on,     *)
(*           and (SoftAP-required) only after discovery. #3 holds.          *)
(*   FALSE - broken: act on any controller's frame regardless of lock or    *)
(*           discovery. TLC violates lock integrity (the satellite flaps    *)
(*           between two controllers / acts before discovery).              *)
(*                                                                          *)
(* Model limits (per the plan): no real time (timeout / scan / sync-due are *)
(* untimed events); boot ids and channels are small integers; a single     *)
(* mesh channel shared by all controllers; convergence quality of the PLL   *)
(* is out of scope - only "eventually (re)synced" is modeled.               *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets

CONSTANTS
    Ctls,           \* controller MACs (1 = normal; 2 = extra-controller test)
    Channels,       \* Wi-Fi channels, e.g. {1, 2}
    MaxBoot,        \* per-controller reboots: boot ids 1..MaxBoot
    SoftApRequired, \* CONFIG_ESPNOW_MESH_HIL_REQUIRE_SOFTAP_DISCOVERY
    EnforceLock     \* TRUE = shipped controller check; FALSE = broken

ASSUME Ctls # {}
ASSUME 0 \notin Ctls            \* 0 is the "no controller locked" sentinel
ASSUME Channels # {}
ASSUME MaxBoot \in Nat \ {0}
ASSUME SoftApRequired \in BOOLEAN
ASSUME EnforceLock \in BOOLEAN

NoMac == 0
Boots == 1..MaxBoot

VARIABLES
    meshChan,       \* current controller/mesh channel (shared, SoftAP channel)
    ctlBoot,        \* [Ctls -> Boots] each controller's current boot id
    net,            \* set of in-flight DATA frames [mac, boot, chan]
    satChan,        \* satellite's current channel
    discovered,     \* SoftAP discovery has succeeded (s_hil_registration_discovered)
    locked,         \* s_have_controller
    lockMac,        \* s_controller_mac (NoMac when unlocked)
    satBoot,        \* adopted controller boot id (s_controller_boot_id)
    synced,         \* mesh_have_time (time sync has locked at least once)
    syncedBoot,     \* boot id the current sync was taken against
    lockViolation   \* ghost: TRUE if #3 is ever broken

vars == <<meshChan, ctlBoot, net, satChan, discovered, locked, lockMac,
          satBoot, synced, syncedBoot, lockViolation>>

DataF(mac, boot, chan) == [mac |-> mac, boot |-> boot, chan |-> chan]

Init ==
    /\ meshChan \in Channels
    /\ ctlBoot = [c \in Ctls |-> 1]
    /\ net = {}
    /\ satChan \in Channels        \* may start on the wrong channel (#9)
    /\ discovered = FALSE
    /\ locked = FALSE
    /\ lockMac = NoMac
    /\ satBoot = 0
    /\ synced = FALSE
    /\ syncedBoot = 0
    /\ lockViolation = FALSE

-----------------------------------------------------------------------------
(* Controllers.                                                             *)

CtlSendData(c) ==
    /\ DataF(c, ctlBoot[c], meshChan) \notin net
    /\ net' = net \cup {DataF(c, ctlBoot[c], meshChan)}
    /\ UNCHANGED <<meshChan, ctlBoot, satChan, discovered, locked, lockMac,
                   satBoot, synced, syncedBoot, lockViolation>>

\* Reboot: fresh boot id and possibly a new channel (the mesh moves).
CtlReboot(c) ==
    /\ ctlBoot[c] < MaxBoot
    /\ ctlBoot' = [ctlBoot EXCEPT ![c] = ctlBoot[c] + 1]
    /\ meshChan' \in Channels
    /\ UNCHANGED <<net, satChan, discovered, locked, lockMac, satBoot,
                   synced, syncedBoot, lockViolation>>

-----------------------------------------------------------------------------
(* Satellite.                                                               *)

SoftApOK == ~SoftApRequired \/ discovered

\* attempt_registration_if_due / scan / connect: an unlocked satellite finds
\* the SoftAP, adopts its channel, and records discovery.
SatDiscover ==
    /\ ~locked
    /\ (satChan # meshChan \/ (SoftApRequired /\ ~discovered))
    /\ satChan' = meshChan
    /\ discovered' = TRUE
    /\ UNCHANGED <<meshChan, ctlBoot, net, locked, lockMac, satBoot, synced,
                   syncedBoot, lockViolation>>

\* Receive one DATA frame (handle_satellite_data). The shipped code locks on
\* the first valid frame, then acts only on its locked controller and (in
\* SoftAP mode) only after discovery.
\* A real radio only delivers live, on-channel traffic: reception requires
\* the satellite to be co-channel with the mesh's CURRENT channel. Frames
\* left on a channel the controller has since vacated are not redelivered
\* (otherwise a stale frame could re-lock the satellite forever and starve
\* the scan - an artifact of net's unbounded persistence, not the protocol).
SatRxData(c) ==
    \E f \in net :
        /\ f.mac = c /\ f.chan = meshChan /\ satChan = meshChan
        /\ LET willLock == ~locked
               \* shipped acceptance gate:
               gate    == SoftApOK /\ (~locked \/ lockMac = c)
               \* an improper act the shipped gate would have blocked:
               improper == (~SoftApOK) \/ (locked /\ lockMac # c)
           IN
            /\ IF EnforceLock THEN gate ELSE TRUE
            /\ willLock \/ f.boot # satBoot \/ lockMac # c    \* a real step
            /\ locked' = TRUE
            /\ lockMac' = c
            /\ satBoot' = f.boot
            /\ lockViolation' = (lockViolation \/ improper)
            /\ UNCHANGED <<meshChan, ctlBoot, net, satChan, discovered,
                           synced, syncedBoot>>

\* One successful time-sync round against the locked controller's current
\* epoch (request_time_sync_if_due + handle_time_response accepting a sample).
SatSync ==
    /\ locked
    /\ satChan = meshChan
    /\ SoftApOK
    /\ (~synced \/ syncedBoot # ctlBoot[lockMac])    \* a real (re)sync step
    /\ synced' = TRUE
    /\ syncedBoot' = ctlBoot[lockMac]
    /\ UNCHANGED <<meshChan, ctlBoot, net, satChan, discovered, locked,
                   lockMac, satBoot, lockViolation>>

\* check_controller_timeout: sustained silence (controller moved to another
\* channel) drops the lock; dedup/sync state is kept, discovery resumes.
SatTimeout ==
    /\ locked
    /\ satChan # meshChan
    /\ locked' = FALSE
    /\ lockMac' = NoMac
    /\ UNCHANGED <<meshChan, ctlBoot, net, satChan, discovered, satBoot,
                   synced, syncedBoot, lockViolation>>

-----------------------------------------------------------------------------
Next ==
    \/ \E c \in Ctls : CtlSendData(c)
    \/ \E c \in Ctls : CtlReboot(c)
    \/ SatDiscover
    \/ \E c \in Ctls : SatRxData(c)
    \/ SatSync
    \/ SatTimeout

\* Fairness: controllers keep transmitting, timers fire, the mesh task runs.
\* Reboots are NOT fair (the satellite must cope whether or not they happen).
Fairness ==
    /\ \A c \in Ctls : WF_vars(CtlSendData(c))
    /\ WF_vars(SatDiscover)
    /\ \A c \in Ctls : WF_vars(SatRxData(c))
    /\ WF_vars(SatSync)
    /\ WF_vars(SatTimeout)

Spec     == Init /\ [][Next]_vars
FairSpec == Spec /\ Fairness

-----------------------------------------------------------------------------
(* Properties.                                                              *)

TypeOK ==
    /\ meshChan \in Channels
    /\ ctlBoot \in [Ctls -> Boots]
    /\ satChan \in Channels
    /\ lockMac \in {NoMac} \cup Ctls
    /\ satBoot \in {0} \cup Boots
    /\ syncedBoot \in {0} \cup Boots

\* #3 Lock integrity: at most one controller MAC is ever the lock, and the
\* satellite never acted on data improperly (wrong controller, or before
\* discovery in SoftAP mode).
LockIntegrity ==
    /\ lockMac \in {NoMac} \cup Ctls
    /\ ~lockViolation

\* #9 Discovery progress: the satellite eventually locks, and once churn
\* settles stays locked on the mesh channel.
DiscoveryProgress == <>[](locked /\ satChan = meshChan)

\* #10a Time-sync liveness: eventually time_synced.
EventuallySynced == <>synced

\* #10b Re-sync after reboot: eventually always synced to the CURRENT epoch,
\* i.e. after the last reboot the satellite reacquires and holds.
SyncedCurrent   == locked /\ synced /\ syncedBoot = ctlBoot[lockMac]
ReSyncLiveness  == <>[]SyncedCurrent

=============================================================================
