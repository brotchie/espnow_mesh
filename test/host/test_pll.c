/* Unit tests for the time-sync PLL: acquisition, tracking, and outlier
 * rejection. Compiled as the satellite role with the PLL enabled. */
#include <stdint.h>
#include <stdlib.h>

#include "espnow_mesh_time_sync.h"
#include "test.h"

int g_test_failures = 0;

static const int64_t OFFSET = 500000; /* controller is 0.5 s ahead of local */

/* Feed a clean sample where controller time == local time + OFFSET. */
static pll_update_result_t feed(int64_t local_us, int64_t delay_us)
{
    return pll_update_from_sample(local_us, local_us + OFFSET, OFFSET, delay_us);
}

static void test_acquire_track_reject(void)
{
    /* First sample acquires (reacquire path). */
    pll_update_result_t r = feed(1000000, 100);
    CHECK(r.accepted);
    CHECK(r.reacquired);
    CHECK(r.offset_us == OFFSET);

    /* A run of clean samples stays accepted and holds the offset. */
    int64_t local = 1000000;
    for (int i = 0; i < 8; ++i) {
        local += 1000000;
        r = feed(local, 100);
        CHECK(r.accepted);
    }
    CHECK(llabs(r.offset_us - OFFSET) < 1000);

    /* Once locked, a large phase residual is rejected as an outlier. */
    local += 1000000;
    pll_update_result_t bad = pll_update_from_sample(local, local + OFFSET + 100000, OFFSET, 100);
    CHECK(!bad.accepted);
    CHECK(bad.reject_reason != NULL);
}

int main(void)
{
    test_acquire_track_reject();
    printf(g_test_failures ? "test_pll: FAILED\n" : "test_pll: ok\n");
    return g_test_failures ? 1 : 0;
}
