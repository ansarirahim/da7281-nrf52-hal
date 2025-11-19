/**
 * @file haptics_test_task.c
 * @brief FreeRTOS Task Example for DA7281 Haptic Driver
 * @author A. R. Ansari
 * @date 2024-08-15
 * @version 1.0.0
 * 
 * @copyright Copyright (c) 2024 A. R. Ansari
 * 
 * Licensed under the MIT License.
 * 
 * @details
 * This example demonstrates how to use the DA7281 HAL in a FreeRTOS
 * environment on the nRF52833 (DWM3001C module). It shows:
 * - Device initialization
 * - Self-test sequence
 * - Override mode operation
 * - Integration with UWB ranging tasks
 */

/*===========================================================================*/
/* INCLUDES                                                                  */
/*===========================================================================*/

#include "FreeRTOS.h"
#include "task.h"
#include "da7281.h"
#include "nrf_log.h"
#include "nrf_gpio.h"

/*===========================================================================*/
/* PRIVATE MACROS AND DEFINES                                                */
/*===========================================================================*/

#define HAPTICS_TASK_PRIORITY           (2U)
#define HAPTICS_TASK_STACK_SIZE         (256U)
#define HAPTICS_TEST_PULSE_DURATION_MS  (100U)
#define HAPTICS_IDLE_DELAY_MS           (5000U)

/* Hardware configuration for DWM3001C */
#define DA7281_I2C_INSTANCE             (0U)
#define DA7281_I2C_ADDR                 DA7281_I2C_ADDRESS_DEFAULT
#define DA7281_POWER_GPIO_PIN           (28U)  /* Example GPIO pin */

/*===========================================================================*/
/* PRIVATE VARIABLES                                                         */
/*===========================================================================*/

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
 * @brief Initialize and start the haptics test task
 * 
 * @return pdPASS on success, pdFAIL otherwise
 */
BaseType_t haptics_task_init(void)
{
    da7281_error_t err;
    BaseType_t task_created;

    NRF_LOG_INFO("Haptics Task: Initializing DA7281 device handle");

    /* Initialize device handle */
    err = da7281_init_handle(&g_haptic_device,
                              DA7281_I2C_ADDR,
                              DA7281_I2C_INSTANCE,
                              DA7281_POWER_GPIO_PIN);

    if (err != DA7281_OK) {
        NRF_LOG_ERROR("Haptics Task: Failed to initialize handle, error=%d", err);
        return pdFAIL;
    }

    /* Create FreeRTOS task */
    task_created = xTaskCreate(haptics_test_task,
                                "HAPTICS",
                                HAPTICS_TASK_STACK_SIZE,
                                NULL,
                                HAPTICS_TASK_PRIORITY,
                                &g_haptics_task_handle);

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

    /* Initialize DA7281 device */
    err = da7281_initialize(&g_haptic_device);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("Haptics Task: Initialization failed, error=%d", err);
        vTaskDelete(NULL);
        return;
    }

    /* Dump initial register configuration */
    haptics_log_register_dump();

    /* Run test sequence */
    haptics_run_test_sequence();

    NRF_LOG_INFO("Haptics Task: Test complete, entering idle mode");

    /* Main task loop - idle state, ready for UWB ranging operations */
    while (1) {
        /* In a real application, this task would:
         * - Wait for haptic feedback requests from other tasks
         * - Coordinate with UWB ranging operations
         * - Handle haptic patterns based on proximity/events
         */

        vTaskDelay(pdMS_TO_TICKS(HAPTICS_IDLE_DELAY_MS));

        /* Periodic status check */
        uint8_t current_mode = 0;
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
    da7281_error_t err;

    NRF_LOG_INFO("Haptics Task: Starting test sequence");

    /* Test 1: Self-test with maximum amplitude */
    NRF_LOG_INFO("Haptics Task: Running self-test (100ms pulse)");
    err = da7281_run_self_test(&g_haptic_device, HAPTICS_TEST_PULSE_DURATION_MS);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("Haptics Task: Self-test failed, error=%d", err);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /* Test 2: Override mode with varying amplitudes */
    NRF_LOG_INFO("Haptics Task: Testing override mode");

    uint8_t test_amplitudes[] = {0x20, 0x40, 0x60, 0x7F};
    for (uint8_t i = 0; i < sizeof(test_amplitudes); i++) {
        NRF_LOG_INFO("Haptics Task: Setting override value = 0x%02X", test_amplitudes[i]);

        /* Set override value */
        err = da7281_set_override_value(&g_haptic_device, test_amplitudes[i]);
        if (err != DA7281_OK) {
            NRF_LOG_ERROR("Haptics Task: Failed to set override, error=%d", err);
            continue;
        }

        /* Enter DRO mode */
        err = da7281_set_operation_mode(&g_haptic_device, DA7281_OPERATION_MODE_DRO);
        if (err != DA7281_OK) {
            NRF_LOG_ERROR("Haptics Task: Failed to enter DRO mode, error=%d", err);
            continue;
        }

        /* Run for 100ms */
        vTaskDelay(pdMS_TO_TICKS(100));

        /* Return to inactive */
        err = da7281_set_operation_mode(&g_haptic_device, DA7281_OPERATION_MODE_INACTIVE);
        if (err != DA7281_OK) {
            NRF_LOG_ERROR("Haptics Task: Failed to return to inactive, error=%d", err);
        }

        vTaskDelay(pdMS_TO_TICKS(300));
    }

    /* Return to standby */
    NRF_LOG_INFO("Haptics Task: Returning to standby mode");
    err = da7281_set_standby(&g_haptic_device, true);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("Haptics Task: Failed to enter standby, error=%d", err);
    }

    NRF_LOG_INFO("Haptics Task: Test sequence complete");
}

/**
 * @brief Log register dump for debugging
 */
static void haptics_log_register_dump(void)
{
    uint8_t reg_value;
    da7281_error_t err;

    NRF_LOG_INFO("Haptics Task: Register Dump");
    NRF_LOG_INFO("========================================");

    /* Read and log key registers */
    err = da7281_read_register(&g_haptic_device, DA7281_REG_CHIP_REV, &reg_value);
    if (err == DA7281_OK) {
        NRF_LOG_INFO("  CHIP_REV (0x00)    = 0x%02X", reg_value);
    }

    err = da7281_read_register(&g_haptic_device, DA7281_REG_TOP_CFG1, &reg_value);
    if (err == DA7281_OK) {
        NRF_LOG_INFO("  TOP_CFG1 (0x22)    = 0x%02X", reg_value);
    }

    err = da7281_read_register(&g_haptic_device, DA7281_REG_TOP_CTL1, &reg_value);
    if (err == DA7281_OK) {
        NRF_LOG_INFO("  TOP_CTL1 (0x28)    = 0x%02X", reg_value);
    }

    NRF_LOG_INFO("========================================");
}

/*===========================================================================*/
/* END OF FILE                                                               */
/*===========================================================================*/

