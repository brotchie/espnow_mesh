------------------------------ MODULE MeshChannel ------------------------------
(***************************************************************************)
(* Network module: models the ESP-NOW radio as a lossy, reordering,       *)
(* duplicating broadcast channel.                                          *)
(*                                                                          *)
(* Refines: the ESP-NOW radio + Wi-Fi channel behavior (esp_now_send /     *)
(* espnow_recv_cb in src/espnow_mesh.c, src/espnow_mesh_packet.c).         *)
(*                                                                          *)
(* Representation: `net` is the set of every frame ever transmitted.       *)
(* Frames are never removed, which gives the three channel faults for      *)
(* free:                                                                    *)
(*   - reordering:  a receive action may pick any frame in the set,        *)
(*   - duplication: a frame can be received any number of times,           *)
(*   - loss:        a schedule may simply never deliver a frame            *)
(*                  (loss is the absence of a receive, so safety runs      *)
(*                  cover arbitrary loss; liveness runs add fairness to    *)
(*                  receive actions, encoding the plan's "channel          *)
(*                  delivers infinitely often" assumption).                *)
(*                                                                          *)
(* Because every frame carries an HMAC (CONFIG_ESPNOW_MESH_AUTH_ENABLE),   *)
(* an adversary cannot forge frames, only replay recorded ones. Replay is  *)
(* therefore also redelivery from `net`; specs that model an active        *)
(* replay adversary widen deliverability (e.g. ignore the channel a frame  *)
(* was captured on, or allow frames from stale controller boots), while    *)
(* benign configurations restrict delivery to what a real radio would      *)
(* redeliver within a dup/reorder window.                                  *)
(*                                                                          *)
(* Every frame record carries at least:                                    *)
(*   dst  : a node id, or the token BCastDst for broadcast                 *)
(*   chan : the Wi-Fi channel the frame was transmitted on                 *)
(* "Wrong channel" is modeled as undeliverable: a node only hears frames   *)
(* whose chan matches its current channel (unless a replay adversary       *)
(* re-injects them on the victim's channel).                               *)
(***************************************************************************)

BCastDst == "bcast"

\* Transmit one frame / a set of frames.
Send(net, f)     == net \cup {f}
SendAll(net, fs) == net \cup fs

\* Radio-level deliverability of frame f to node `node` listening on `chan`.
Hears(f, node, chan) ==
    /\ f.chan = chan
    /\ \/ f.dst = BCastDst
       \/ f.dst = node

=============================================================================
