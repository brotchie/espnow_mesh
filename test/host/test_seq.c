/* Unit tests for the wrap-safe serial-number duplicate test in the shared
 * internal header. */
#include <stdint.h>

#include "espnow_mesh_priv.h"
#include "test.h"

int g_test_failures = 0;

static void test_first_packet_never_duplicate(void)
{
    CHECK(!seq_is_duplicate(false, 0, 0));
    CHECK(!seq_is_duplicate(false, 42, 42));
    CHECK(!seq_is_duplicate(false, 0, 1000));
}

static void test_ordering(void)
{
    CHECK(seq_is_duplicate(true, 5, 5));   /* same sequence -> duplicate */
    CHECK(seq_is_duplicate(true, 4, 5));   /* older -> duplicate */
    CHECK(!seq_is_duplicate(true, 6, 5));  /* newer -> not duplicate */
}

static void test_wraparound(void)
{
    /* last just below wrap, new sequence wraps to 0: 0 is newer than 2^32-1 */
    CHECK(!seq_is_duplicate(true, 0u, 0xFFFFFFFFu));
    /* the reverse: 2^32-1 is older than 0 -> duplicate */
    CHECK(seq_is_duplicate(true, 0xFFFFFFFFu, 0u));
    /* a few steps across the boundary */
    CHECK(!seq_is_duplicate(true, 2u, 0xFFFFFFFEu));
    CHECK(seq_is_duplicate(true, 0xFFFFFFFEu, 2u));
}

int main(void)
{
    test_first_packet_never_duplicate();
    test_ordering();
    test_wraparound();
    printf(g_test_failures ? "test_seq: FAILED\n" : "test_seq: ok\n");
    return g_test_failures ? 1 : 0;
}
