#pragma once
#include "freertos/FreeRTOS.h"
typedef struct tskTaskControlBlock *TaskHandle_t;
typedef void (*TaskFunction_t)(void *);
BaseType_t xTaskCreate(TaskFunction_t pxTaskCode, const char *pcName, uint32_t usStackDepth,
                       void *pvParameters, UBaseType_t uxPriority, TaskHandle_t *pxCreatedTask);
BaseType_t xTaskCreatePinnedToCore(TaskFunction_t pxTaskCode, const char *pcName,
                                   uint32_t usStackDepth, void *pvParameters,
                                   UBaseType_t uxPriority, TaskHandle_t *pxCreatedTask,
                                   BaseType_t xCoreID);
void vTaskDelay(TickType_t xTicksToDelay);
void vTaskDelete(TaskHandle_t xTaskToDelete);
