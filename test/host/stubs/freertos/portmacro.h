#pragma once
/* Critical sections are no-ops on the single-threaded host test runner. */
typedef struct { int dummy; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED { 0 }
#define portENTER_CRITICAL(m) ((void)(m))
#define portEXIT_CRITICAL(m) ((void)(m))
