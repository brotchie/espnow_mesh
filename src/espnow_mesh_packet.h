#pragma once

/*
 * Packet layer: finalize outbound frames (stamp header + HMAC), authenticate
 * and classify inbound frames, and send. This layer is purely functional over
 * its arguments and holds no mesh protocol state. Names are unprefixed because
 * this is an internal component header, not part of the public API.
 */

#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "espnow_mesh_wire.h"

/* Clamp a Wi-Fi channel number to the valid 2.4 GHz range [1, 14]. */
uint8_t clamp_wifi_channel(uint8_t channel);

/*
 * Stamp the protocol header (magic, version, type, mesh id) into packet and,
 * when authentication is enabled, compute and store the HMAC tag over the whole
 * frame. Returns ESP_ERR_INVALID_SIZE if len is smaller than the header, or
 * ESP_FAIL if the tag could not be computed.
 */
esp_err_t mesh_packet_finalize(void *packet, size_t len, uint8_t type);

/*
 * Validate an inbound event's header fields and (when enabled) its HMAC exactly
 * once, over the raw event buffer. On success writes the message type and
 * returns true; handlers dispatched on that type must still bound-check
 * event->len before copying into a typed struct.
 */
bool mesh_rx_packet_type(const espnow_mesh_event_t *event, uint8_t *type_out);

/* Finalize packet (see mesh_packet_finalize) then send it to dest_mac. */
esp_err_t mesh_send(const uint8_t *dest_mac, void *packet, size_t len, uint8_t type);
