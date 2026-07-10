#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_now.h"
#include "mbedtls/md.h"

#include "espnow_mesh_packet.h"

#if CONFIG_ESPNOW_MESH_AUTH_ENABLE
static uint64_t load_u64_le(const uint8_t *bytes)
{
    uint64_t value = 0;
    for (size_t i = 0; i < ESPNOW_MESH_AUTH_TAG_BYTES; ++i) {
        value |= ((uint64_t)bytes[i]) << (8u * i);
    }
    return value;
}

static bool auth_tags_equal(uint64_t left, uint64_t right)
{
    uint8_t diff = 0;
    for (size_t i = 0; i < ESPNOW_MESH_AUTH_TAG_BYTES; ++i) {
        diff |= (uint8_t)(((left >> (8u * i)) ^ (right >> (8u * i))) & 0xffu);
    }
    return diff == 0;
}

/*
 * One HMAC context is kept keyed and reused for every packet. TX finalize and RX
 * verification both run on the mesh task, so no locking is needed. Reuse avoids
 * the per-computation heap allocation inside mbedtls_md_setup().
 */
static mbedtls_md_context_t s_auth_ctx;
static bool s_auth_ctx_ready;

static void mesh_auth_ctx_teardown(void)
{
    mbedtls_md_free(&s_auth_ctx);
    s_auth_ctx_ready = false;
}

static bool mesh_auth_ctx_begin(void)
{
    if (s_auth_ctx_ready) {
        if (mbedtls_md_hmac_reset(&s_auth_ctx) == 0) {
            return true;
        }
        mesh_auth_ctx_teardown();
    }

    const char *key = CONFIG_ESPNOW_MESH_AUTH_KEY;
    size_t key_len = strlen(key);
    if (key_len == 0) {
        return false;
    }

    const mbedtls_md_info_t *md_info = mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    if (md_info == NULL) {
        return false;
    }

    mbedtls_md_init(&s_auth_ctx);
    if (mbedtls_md_setup(&s_auth_ctx, md_info, 1) != 0 ||
        mbedtls_md_hmac_starts(&s_auth_ctx, (const uint8_t *)key, key_len) != 0) {
        mesh_auth_ctx_teardown();
        return false;
    }
    s_auth_ctx_ready = true;
    return true;
}

static bool mesh_auth_tag_compute(const void *packet, size_t len, uint64_t *auth_tag)
{
    if (packet == NULL || auth_tag == NULL || len < sizeof(espnow_mesh_header_t)) {
        return false;
    }

    const size_t tag_offset = offsetof(espnow_mesh_header_t, auth_tag);
    const size_t tag_end = tag_offset + ESPNOW_MESH_AUTH_TAG_BYTES;
    if (len < tag_end) {
        return false;
    }

    if (!mesh_auth_ctx_begin()) {
        return false;
    }

    const uint8_t zero_tag[ESPNOW_MESH_AUTH_TAG_BYTES] = { 0 };
    uint8_t digest[32] = { 0 };
    const uint8_t *bytes = (const uint8_t *)packet;

    if (mbedtls_md_hmac_update(&s_auth_ctx, bytes, tag_offset) != 0 ||
        mbedtls_md_hmac_update(&s_auth_ctx, zero_tag, sizeof(zero_tag)) != 0 ||
        (len > tag_end &&
         mbedtls_md_hmac_update(&s_auth_ctx, bytes + tag_end, len - tag_end) != 0) ||
        mbedtls_md_hmac_finish(&s_auth_ctx, digest) != 0) {
        mesh_auth_ctx_teardown();
        return false;
    }

    *auth_tag = load_u64_le(digest);
    return true;
}
#endif

esp_err_t mesh_packet_finalize(void *packet, size_t len, uint8_t type)
{
    if (packet == NULL || len < sizeof(espnow_mesh_header_t)) {
        return ESP_ERR_INVALID_SIZE;
    }

    espnow_mesh_header_t *header = (espnow_mesh_header_t *)packet;
    header->magic = ESPNOW_MESH_MAGIC;
    header->version = ESPNOW_MESH_VERSION;
    header->type = type;
    header->mesh_id = CONFIG_ESPNOW_MESH_ID;
    header->auth_tag = 0;

#if CONFIG_ESPNOW_MESH_AUTH_ENABLE
    uint64_t auth_tag = 0;
    if (!mesh_auth_tag_compute(packet, len, &auth_tag)) {
        return ESP_FAIL;
    }
    header->auth_tag = auth_tag;
#endif

    return ESP_OK;
}

bool mesh_rx_packet_type(const espnow_mesh_event_t *event, uint8_t *type_out)
{
    if (event == NULL || type_out == NULL || event->len < sizeof(espnow_mesh_header_t)) {
        return false;
    }

    espnow_mesh_header_t header = { 0 };
    memcpy(&header, event->data, sizeof(header));
    if (header.magic != ESPNOW_MESH_MAGIC || header.version != ESPNOW_MESH_VERSION ||
        header.mesh_id != CONFIG_ESPNOW_MESH_ID) {
        return false;
    }

#if CONFIG_ESPNOW_MESH_AUTH_ENABLE
    uint64_t expected_tag = 0;
    if (!mesh_auth_tag_compute(event->data, event->len, &expected_tag)) {
        return false;
    }
    if (!auth_tags_equal(header.auth_tag, expected_tag)) {
        return false;
    }
#endif

    *type_out = header.type;
    return true;
}

esp_err_t mesh_send(const uint8_t *dest_mac, void *packet, size_t len, uint8_t type)
{
    esp_err_t err = mesh_packet_finalize(packet, len, type);
    if (err != ESP_OK) {
        return err;
    }
    return esp_now_send(dest_mac, (const uint8_t *)packet, len);
}

uint8_t clamp_wifi_channel(uint8_t channel)
{
    if (channel < 1) {
        return 1;
    }
    if (channel > 14) {
        return 14;
    }
    return channel;
}
