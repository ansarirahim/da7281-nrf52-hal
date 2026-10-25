/* Stand-in for FreeRTOS: tick rate of the nRF5 SDK port (RTC at 32768 Hz -> 1024 Hz). */
#ifndef FREERTOS_H_STUB
#define FREERTOS_H_STUB

#include <stdint.h>

typedef uint32_t TickType_t;
typedef long BaseType_t;
#define pdTRUE  (1)
#define pdFALSE (0)
#define pdPASS  (1)
#define pdFAIL  (0)
#ifndef configTICK_RATE_HZ
#define configTICK_RATE_HZ (1024U)
#endif
#define pdMS_TO_TICKS(ms) \
    ((TickType_t)(((TickType_t)(ms) * (TickType_t)configTICK_RATE_HZ) / (TickType_t)1000U))
#define portMAX_DELAY (0xFFFFFFFFU)

#endif
