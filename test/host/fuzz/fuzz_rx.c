/*
 * Fuzz target for the receive-path entry point mesh_rx_packet_type(): every
 * ESP-NOW frame off the air hits this first. Feeds arbitrary bytes as the raw
 * event buffer and checks the header parse + classification never reads out of
 * bounds. Built with authentication disabled so inputs reach the length/type
 * logic rather than bouncing off the HMAC tag.
 */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "espnow_mesh_packet.h"

int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    espnow_mesh_event_t ev;
    memset(&ev, 0, sizeof(ev));

    size_t n = size > sizeof(ev.data) ? sizeof(ev.data) : size;
    ev.len = (uint16_t)n;
    memcpy(ev.data, data, n);

    uint8_t type = 0;
    (void)mesh_rx_packet_type(&ev, &type);
    return 0;
}
