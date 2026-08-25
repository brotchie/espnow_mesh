/*
 * Fuzz target for the satellite patch-block handler, the highest-severity RX
 * path: handle_satellite_patch_block() writes a wire-controlled slice into the
 * fixed s_patch_staging[] buffer
 *
 *     memcpy(&s_patch_staging[block.block_offset], block.payload, block.payload_len);
 *
 * The whole component TU is #included so the static handler and patch state are
 * reachable. A valid patch offer is primed once; each input then perturbs the
 * block index / payload length / payload bytes of an otherwise-consistent block
 * frame, so the fuzzer explores the exact fields feeding the indexed store.
 * ASan flags any out-of-bounds write the validators fail to prevent.
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "espnow_mesh.c"

static bool s_primed;

static void prime(void)
{
    memset(s_controller_mac, 0xAB, ESP_NOW_ETH_ALEN);
    s_have_controller = true;
    s_have_controller_boot_id = true;
    s_controller_boot_id = 0x1234u;

    s_patch_offer_active = true;
    s_patch_ready = false;
    s_patch_sequence = 1;
    s_patch_controller_boot_id = 0x1234u;
    s_patch_id = 7u;
    s_patch_total_len = CONFIG_ESPNOW_MESH_HIL_PATCH_BYTES;
    s_patch_block_size = CONFIG_ESPNOW_MESH_HIL_PATCH_BLOCK_BYTES;
    s_patch_block_count = hil_patch_block_count_for_len(s_patch_total_len);
    s_patch_hash = hil_patch_hash(s_patch_id, s_patch_total_len);
    s_patch_fault_seed = 0;
    s_primed = true;
}

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    if (!s_primed) {
        prime();
    }
    s_patch_received_mask = 0; /* fresh staging each input */

    espnow_mesh_patch_block_msg_t blk;
    memset(&blk, 0, sizeof(blk));
    blk.sequence = s_patch_sequence;
    blk.controller_boot_id = s_patch_controller_boot_id;
    blk.patch_id = s_patch_id;
    blk.patch_hash = s_patch_hash;
    blk.block_count = s_patch_block_count;
    blk.total_len = s_patch_total_len;
    blk.block_drop_pct = 0;
    blk.fault_seed = 0;

    uint16_t block_index = 0;
    uint16_t payload_len = 0;
    if (size >= 2) {
        block_index = (uint16_t)(data[0] | (data[1] << 8));
    }
    if (size >= 4) {
        payload_len = (uint16_t)(data[2] | (data[3] << 8));
    }
    blk.block_index = block_index;
    /* handler requires block_offset == block_index * block_size to proceed */
    blk.block_offset = (uint32_t)block_index * s_patch_block_size;
    blk.payload_len = payload_len;

    uint16_t copy = payload_len > sizeof(blk.payload) ? (uint16_t)sizeof(blk.payload) : payload_len;
    size_t avail = size > 4 ? size - 4 : 0;
    if (copy > avail) {
        copy = (uint16_t)avail;
    }
    memcpy(blk.payload, data + 4, copy);
    /* match the per-block hash so validation reaches the staging store */
    blk.block_hash = hil_hash_bytes(blk.payload, copy);

    size_t framelen = offsetof(espnow_mesh_patch_block_msg_t, payload) + copy;
    (void)mesh_packet_finalize(&blk, framelen, ESPNOW_MESH_MSG_PATCH_BLOCK);

    espnow_mesh_event_t ev;
    memset(&ev, 0, sizeof(ev));
    memcpy(ev.mac, s_controller_mac, ESP_NOW_ETH_ALEN);
    size_t evn = framelen > sizeof(ev.data) ? sizeof(ev.data) : framelen;
    ev.len = (uint16_t)evn;
    memcpy(ev.data, &blk, evn);

    handle_satellite_patch_block(&ev);
    return 0;
}
