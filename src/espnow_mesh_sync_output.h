#pragma once

/*
 * Synchronized GPIO output actuator: drives a logic-analyzer sync waveform from
 * the shared mesh clock, and (in HIL mode) an application apply-marker pulse.
 * Both are no-ops when their features are disabled at build time.
 */

#include <stdint.h>

/* Configure the sync-output GPIO/timer and, outside HIL mode, start the
 * waveform. No-op when the sync output is disabled. */
void init_sync_output(void);

/* Create the HIL apply-marker timers. No-op when HIL mode is disabled. */
void init_hil_test(void);

#if CONFIG_ESPNOW_MESH_HIL_TEST_ENABLE
/* Schedule a one-shot apply-marker pulse at the given mesh timestamp. */
void hil_schedule_apply_marker(int64_t apply_at_mesh_us);
#endif
