#ifndef TASK_H_STUB
#define TASK_H_STUB

#include "FreeRTOS.h"

typedef void *TaskHandle_t;
void vTaskDelay(TickType_t ticks);
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, uint16_t stack, void *arg,
                       uint32_t prio, TaskHandle_t *handle);
void vTaskDelete(TaskHandle_t handle);

#endif
