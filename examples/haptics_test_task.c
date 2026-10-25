/**
 * @file haptics_test_task.c
 * @brief FreeRTOS Task Example for DA7281 Haptic Driver
 * @author A. R. Ansari
 * @version 2.0.0
 *
 * @copyright Copyright (c) 2024 A. R. Ansari
 *
 * Licensed under the MIT License.
 *
 * @details
 * This example demonstrates how to use the DA7281 HAL in a FreeRTOS
 * environment on the nRF52833 (DWM3001C module). It shows:
 * - Bus and device initialization
 * - Self-test sequence
 * - Override mode operation
 * - Integration with UWB ranging tasks
 */

/*===========================================================================*/
/* INCLUDES                                                                  */
/*===========================================================================*/

#include <stddef.h>

#include "FreeRTOS.h"
#include "da7281.h"
#include "nrf_drv_twi.h"
#include "nrf_log.h"
#include "task.h"

/*===========================================================================*/
/* PRIVATE MACROS AND DEFINES                                                */
/*===========================================================================*/

#define HAPTICS_TASK_PRIORITY          (2U)
#define HAPTICS_TASK_STACK_SIZE        (256U)
#define HAPTICS_TEST_PULSE_DURATION_MS (100U)
#define HAPTICS_IDLE_DELAY_MS          (5000U)

/* Hardware configuration: ADDR_1 tied high, ADDR_0 tied low -> 0x4A; VDD always on */
#define DA7281_I2C_ADDR       DA7281_I2C_ADDRESS(1, 0)
#define DA7281_POWER_GPIO_PIN DA7281_NO_PIN

/*===========================================================================*/
/* PRIVATE VARIABLES                                                         */
/*===========================================================================*/

/* TWI0, initialised and enabled by the application (nrf_drv_twi_init / nrf_drv_twi_enable) */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
static da7281_bus_t g_twi_bus;
static da7281_device_t g_haptic_device;
static TaskHandle_t g_haptics_task_handle = NULL;

/*===========================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                               */
/*===========================================================================*/

static void haptics_test_task(void *pvParameters);
static void haptics_run_test_sequence(void);
static void haptics_log_register_dump(void);

/*===========================================================================*/
/* PUBLIC FUNCTION IMPLEMENTATIONS                                           */
/*===========================================================================*/

/**
 * @brief Initialize and start the haptics test task (call before vTaskStartScheduler)
 *
 * @return pdPASS on success, pdFAIL otherwise
 */
BaseType_t haptics_task_init(void)
{
    da7281_error_t err;
    BaseType_t task_created;

    NRF_LOG_INFO("Haptics Task: Initializing TWI bus and DA7281 device handle");

    /* The bus mutex is created here, before the scheduler starts */
    err = da7281_bus_init(&g_twi_bus, &m_twi);
    if (err == DA7281_OK) {
        err = da7281_init_handle(&g_haptic_device, &g_twi_bus, DA7281_I2C_ADDR,
                                 DA7281_POWER_GPIO_PIN);
    }
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("Haptics Task: Failed to initialize handle, error=%d", err);
        return pdFAIL;
    }

    task_created = xTaskCreate(haptics_test_task, "HAPTICS", HAPTICS_TASK_STACK_SIZE, NULL,
                               HAPTICS_TASK_PRIORITY, &g_haptics_task_handle);
    if (task_created != pdPASS) {
        NRF_LOG_ERROR("Haptics Task: Failed to create task");
        return pdFAIL;
    }

    NRF_LOG_INFO("Haptics Task: Task created successfully");
    return pdPASS;
}

/*===========================================================================*/
/* PRIVATE FUNCTION IMPLEMENTATIONS                                          */
/*===========================================================================*/

/**
 * @brief Main haptics test task
 */
static void haptics_test_task(void *pvParameters)
{
    da7281_error_t err;

    (void)pvParameters;

    NRF_LOG_INFO("Haptics Task: Starting task execution");

    /* Initialize DA7281 device (waits for tON, so it runs in the task) */
    err = da7281_initialize(&g_haptic_device);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("Haptics Task: Initialization failed, error=%d", err);
        vTaskDelete(NULL);
        return;
    }

    haptics_log_register_dump();
    haptics_run_test_sequence();

    NRF_LOG_INFO("Haptics Task: Test complete, entering idle mode");

    /* Main task loop - idle state, ready for UWB ranging operations */
    while (1) {
        /* In a real application, this task would:
         * - Wait for haptic feedback requests from other tasks (queue or task notification;
         *   interrupt handlers must not call the driver)
         * - Coordinate with UWB ranging operations
         * - Handle haptic patterns based on proximity/events
         */
        uint8_t current_mode = 0U;

        vTaskDelay(pdMS_TO_TICKS(HAPTICS_IDLE_DELAY_MS));

        err = da7281_get_operation_mode(&g_haptic_device, &current_mode);
        if (err == DA7281_OK) {
            NRF_LOG_DEBUG("Haptics Task: Current mode = %d", current_mode);
        }
    }
}

/**
 * @brief Run haptic test sequence
 */
static void haptics_run_test_sequence(void)
{
    static const uint8_t test_amplitudes[] = {0x20U, 0x40U, 0x60U, 0x7FU};
    da7281_error_t err;

    NRF_LOG_INFO("Haptics Task: Running self-test (100ms pulse)");
    err = da7281_run_self_test(&g_haptic_device, HAPTICS_TEST_PULSE_DURATION_MS);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("Haptics Task: Self-test failed, error=%d", err);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    NRF_LOG_INFO("Haptics Task: Testing override mode");
    for (uint8_t i = 0U; i < sizeof(test_amplitudes); i++) {
        NRF_LOG_INFO("Haptics Task: Setting override value = 0x%02X", test_amplitudes[i]);

        err = da7281_set_override_value(&g_haptic_device, test_amplitudes[i]);
        if (err == DA7281_OK) {
            err = da7281_set_operation_mode(&g_haptic_device, DA7281_OPERATION_MODE_DRO);
        }
        if (err != DA7281_OK) {
            NRF_LOG_ERROR("Haptics Task: DRO playback failed, error=%d", err);
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(100));

        err = da7281_set_operation_mode(&g_haptic_device, DA7281_OPERATION_MODE_INACTIVE);
        if (err != DA7281_OK) {
            NRF_LOG_ERROR("Haptics Task: Failed to return to inactive, error=%d", err);
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }

    NRF_LOG_INFO("Haptics Task: Test sequence complete");
}

/**
 * @brief Log register dump for debugging
 */
static void haptics_log_register_dump(void)
{
    static const struct {
        uint8_t addr;
        const char *name;
    } regs[] = {
        {DA7281_REG_CHIP_REV, "CHIP_REV"},           {DA7281_REG_FRQ_LRA_PER_H, "FRQ_LRA_PER_H"},
        {DA7281_REG_FRQ_LRA_PER_L, "FRQ_LRA_PER_L"}, {DA7281_REG_ACTUATOR3, "ACTUATOR3"},
        {DA7281_REG_TOP_CFG1, "TOP_CFG1"},           {DA7281_REG_TOP_CTL1, "TOP_CTL1"},
    };

    NRF_LOG_INFO("Haptics Task: Register Dump");
    for (uint8_t i = 0U; i < (uint8_t)(sizeof(regs) / sizeof(regs[0])); i++) {
        uint8_t value = 0U;
        if (da7281_read_register(&g_haptic_device, regs[i].addr, &value) == DA7281_OK) {
            NRF_LOG_INFO("  %s (0x%02X) = 0x%02X", regs[i].name, regs[i].addr, value);
        }
    }
}

/*===========================================================================*/
/* END OF FILE                                                               */
/*===========================================================================*/
