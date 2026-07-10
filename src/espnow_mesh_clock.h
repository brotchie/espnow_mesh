#pragma once

/*
 * Shared mesh clock, owned by the core translation unit. On the controller this
 * is plain monotonic time; on satellites it is the synchronized estimate of the
 * controller clock (PLL or filtered offset). Exposed so the sync-output
 * actuator module can read it.
 */

#include <stdbool.h>
#include <stdint.h>

/*
 * Write the current shared mesh time (microseconds) to *mesh_time_us. Returns
 * false until a satellite has an estimate (before its first accepted time-sync
 * sample); the output still holds the best available value. Always true on the
 * controller.
 */
bool shared_clock_get_time_us(int64_t *mesh_time_us);

/* Convenience wrapper returning the shared mesh time, ignoring readiness. */
int64_t mesh_time_us(void);
