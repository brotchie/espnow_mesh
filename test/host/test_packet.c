/* Unit tests for the packet layer: channel clamping, and finalize/validate over
 * the header (built with authentication disabled). */
#include <stddef.h>
#include <string.h>

#include "espnow_mesh_packet.h"
#include "test.h"

int g_test_failures = 0;

static void test_clamp_channel(void)
{
    CHECK(clamp_wifi_channel(0) == 1);
    CHECK(clamp_wifi_channel(1) == 1);
    CHECK(clamp_wifi_channel(6) == 6);
    CHECK(clamp_wifi_channel(14) == 14);
    CHECK(clamp_wifi_channel(15) == 14);
    CHECK(clamp_wifi_channel(255) == 14);
}

static void event_from(espnow_mesh_event_t *ev, const void *frame, size_t len)
{
    memset(ev, 0, sizeof(*ev));
    ev->len = (uint16_t)len;
    memcpy(ev->data, frame, len);
}

static void test_finalize_and_classify(void)
{
    espnow_mesh_data_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.payload_len = 3;
    size_t len = offsetof(espnow_mesh_data_msg_t, payload) + msg.payload_len;

    CHECK(mesh_packet_finalize(&msg, len, ESPNOW_MESH_MSG_DATA) == ESP_OK);

    espnow_mesh_event_t ev;
    event_from(&ev, &msg, len);
    uint8_t type = 0xFF;
    CHECK(mesh_rx_packet_type(&ev, &type));
    CHECK(type == ESPNOW_MESH_MSG_DATA);

    /* Corrupted magic must be rejected. */
    espnow_mesh_event_t bad_magic = ev;
    bad_magic.data[0] ^= 0xFF;
    CHECK(!mesh_rx_packet_type(&bad_magic, &type));

    /* Bumped version must be rejected. */
    espnow_mesh_event_t bad_ver = ev;
    bad_ver.data[offsetof(espnow_mesh_header_t, version)] += 1;
    CHECK(!mesh_rx_packet_type(&bad_ver, &type));

    /* A frame shorter than the header must be rejected. */
    espnow_mesh_event_t too_short = ev;
    too_short.len = (uint16_t)(sizeof(espnow_mesh_header_t) - 1);
    CHECK(!mesh_rx_packet_type(&too_short, &type));
}

static void test_finalize_rejects_undersized(void)
{
    espnow_mesh_data_msg_t msg;
    memset(&msg, 0, sizeof(msg));
    CHECK(mesh_packet_finalize(&msg, 1, ESPNOW_MESH_MSG_DATA) == ESP_ERR_INVALID_SIZE);
    CHECK(mesh_packet_finalize(NULL, 64, ESPNOW_MESH_MSG_DATA) == ESP_ERR_INVALID_SIZE);
}

int main(void)
{
    test_clamp_channel();
    test_finalize_and_classify();
    test_finalize_rejects_undersized();
    printf(g_test_failures ? "test_packet: FAILED\n" : "test_packet: ok\n");
    return g_test_failures ? 1 : 0;
}
