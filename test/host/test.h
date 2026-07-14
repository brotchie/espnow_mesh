#pragma once
#include <stdio.h>

extern int g_test_failures;

#define CHECK(cond)                                                            \
    do {                                                                       \
        if (!(cond)) {                                                         \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);           \
            ++g_test_failures;                                                 \
        }                                                                      \
    } while (0)
