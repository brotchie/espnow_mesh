#pragma once
#include <stdint.h>
typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;
#define pdFALSE ((BaseType_t)0)
#define pdTRUE ((BaseType_t)1)
#define pdPASS pdTRUE
#define portMAX_DELAY ((TickType_t)0xffffffffUL)
#define portTICK_PERIOD_MS 10
#define pdMS_TO_TICKS(ms) ((TickType_t)((ms) / portTICK_PERIOD_MS))
#include "freertos/portmacro.h"
