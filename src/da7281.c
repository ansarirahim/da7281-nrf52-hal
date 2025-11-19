/**
 * @file da7281.c
 * @brief DA7281 Haptic Driver HAL Implementation
 * @author A. R. Ansari
 * @date 2024-08-15
 * @version 1.0.0
 * 
 * @copyright Copyright (c) 2024 A. R. Ansari
 * 
 * Licensed under the MIT License.
 */

/*===========================================================================*/
/* INCLUDES                                                                  */
/*===========================================================================*/

#include "da7281.h"
#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

/*===========================================================================*/
/* PRIVATE MACROS AND DEFINES                                                */
/*===========================================================================*/

#define DA7281_I2C_TIMEOUT_MS           (100U)
#define DA7281_VALIDATE_PARAM(cond)     if (!(cond)) return DA7281_ERR_INVALID_PARAM

/*===========================================================================*/
/* PRIVATE VARIABLES                                                         */
/*===========================================================================*/

static SemaphoreHandle_t g_i2c_mutex = NULL;
static bool g_i2c_initialized = false;

/*===========================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                               */
/*===========================================================================*/

static da7281_error_t da7281_i2c_write(const da7281_device_t *dev,
                                        uint8_t reg_addr,
                                        const uint8_t *data,
                                        uint8_t length);

static da7281_error_t da7281_i2c_read(const da7281_device_t *dev,
                                       uint8_t reg_addr,
                                       uint8_t *data,
                                       uint8_t length);

static void da7281_delay_ms(uint32_t ms);

/*===========================================================================*/
/* PUBLIC FUNCTION IMPLEMENTATIONS                                           */
/*===========================================================================*/

/**
 * @brief Initialize DA7281 device handle
 */
da7281_error_t da7281_init_handle(da7281_device_t *dev, uint8_t i2c_addr,
                                   uint8_t twi_inst, uint8_t pwr_pin)
{
    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(i2c_addr >= DA7281_I2C_ADDRESS_DEFAULT &&
                          i2c_addr <= DA7281_I2C_ADDRESS_ALT3);

    /* Initialize mutex on first use */
    if (g_i2c_mutex == NULL) {
        g_i2c_mutex = xSemaphoreCreateMutex();
        if (g_i2c_mutex == NULL) {
            return DA7281_ERR_I2C_COMM;
        }
    }

    /* Clear device structure */
    memset(dev, 0, sizeof(da7281_device_t));

    /* Set device parameters */
    dev->i2c_address = i2c_addr;
    dev->twi_instance = twi_inst;
    dev->power_gpio_pin = pwr_pin;
    dev->irq_gpio_pin = 0xFF; /* Not used by default */
    dev->is_initialized = false;
    dev->is_powered = false;

    /* Configure power GPIO */
    nrf_gpio_cfg_output(dev->power_gpio_pin);
    nrf_gpio_pin_clear(dev->power_gpio_pin);

    return DA7281_OK;
}

/**
 * @brief Power on the DA7281 device
 */
da7281_error_t da7281_power_on(da7281_device_t *dev)
{
    DA7281_VALIDATE_PARAM(dev != NULL);

    if (dev->is_powered) {
        return DA7281_OK; /* Already powered */
    }

    /* Enable power GPIO */
    nrf_gpio_pin_set(dev->power_gpio_pin);

    /* Wait for power-up sequence (datasheet: min 1.5ms) */
    da7281_delay_ms(DA7281_POWER_UP_DELAY_MS);

    dev->is_powered = true;

    return DA7281_OK;
}

/**
 * @brief Power off the DA7281 device
 */
da7281_error_t da7281_power_off(da7281_device_t *dev)
{
    DA7281_VALIDATE_PARAM(dev != NULL);

    if (!dev->is_powered) {
        return DA7281_OK; /* Already powered off */
    }

    /* Disable power GPIO */
    nrf_gpio_pin_clear(dev->power_gpio_pin);

    dev->is_powered = false;
    dev->is_initialized = false;

    return DA7281_OK;
}

/**
 * @brief Verify chip ID
 */
da7281_error_t da7281_verify_chip_id(const da7281_device_t *dev)
{
    uint8_t chip_rev = 0;
    da7281_error_t err;

    DA7281_VALIDATE_PARAM(dev != NULL);

    if (!dev->is_powered) {
        return DA7281_ERR_NOT_INITIALIZED;
    }

    err = da7281_read_register(dev, DA7281_REG_CHIP_REV, &chip_rev);
    if (err != DA7281_OK) {
        return err;
    }

    if (chip_rev != DA7281_CHIP_REV_EXPECTED) {
        return DA7281_ERR_CHIP_ID;
    }

    return DA7281_OK;
}

/**
 * @brief Configure LRA parameters
 */
da7281_error_t da7281_configure_lra(const da7281_device_t *dev,
                                     float nom_max_v, float abs_max_v,
                                     uint16_t imax_ma, float impedance_ohm,
                                     uint16_t resonant_freq_hz)
{
    da7281_error_t err;
    uint8_t reg_val;
    uint16_t lra_period;
    uint16_t v2i_factor;

    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);
    DA7281_VALIDATE_PARAM(nom_max_v > 0.0f && nom_max_v <= 6.0f);
    DA7281_VALIDATE_PARAM(abs_max_v > 0.0f && abs_max_v <= 6.0f);
    DA7281_VALIDATE_PARAM(resonant_freq_hz > 0 && resonant_freq_hz <= 300);

    /* Calculate LRA period: Period = 1 / (frequency * 1.024e-6) */
    lra_period = (uint16_t)(1000000.0f / (resonant_freq_hz * 1.024f));

    /* Write LRA period (high byte first) */
    reg_val = (uint8_t)((lra_period >> 8) & 0xFFU);
    err = da7281_write_register(dev, DA7281_REG_LRA_PER_H, reg_val);
    if (err != DA7281_OK) return err;

    reg_val = (uint8_t)(lra_period & 0xFFU);
    err = da7281_write_register(dev, DA7281_REG_LRA_PER_L, reg_val);
    if (err != DA7281_OK) return err;

    /* Calculate V2I factor based on impedance */
    v2i_factor = (uint16_t)(impedance_ohm * 1.5f);

    /* Write V2I calibration values */
    reg_val = (uint8_t)((v2i_factor >> 8) & 0xFFU);
    err = da7281_write_register(dev, DA7281_REG_CALIB_V2I_H, reg_val);
    if (err != DA7281_OK) return err;

    reg_val = (uint8_t)(v2i_factor & 0xFFU);
    err = da7281_write_register(dev, DA7281_REG_CALIB_V2I_L, reg_val);
    if (err != DA7281_OK) return err;

    /* Configure actuator parameters */
    reg_val = (uint8_t)(nom_max_v * 10.0f); /* Scale to register value */
    err = da7281_write_register(dev, DA7281_REG_ACTUATOR1, reg_val);
    if (err != DA7281_OK) return err;

    reg_val = (uint8_t)(abs_max_v * 10.0f);
    err = da7281_write_register(dev, DA7281_REG_ACTUATOR2, reg_val);
    if (err != DA7281_OK) return err;

    reg_val = (uint8_t)(imax_ma / 10U);
    err = da7281_write_register(dev, DA7281_REG_ACTUATOR3, reg_val);
    if (err != DA7281_OK) return err;

    /* Enable acceleration, rapid stop, and frequency tracking */
    reg_val = DA7281_TOP_CFG1_ACCELERATION_EN |
              DA7281_TOP_CFG1_RAPID_STOP_EN |
              DA7281_TOP_CFG1_FREQ_TRACK_EN;
    err = da7281_write_register(dev, DA7281_REG_TOP_CFG1, reg_val);
    if (err != DA7281_OK) return err;

    return DA7281_OK;
}

/**
 * @brief Set operation mode
 */
da7281_error_t da7281_set_operation_mode(const da7281_device_t *dev, uint8_t mode)
{
    uint8_t reg_val;
    da7281_error_t err;

    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);
    DA7281_VALIDATE_PARAM(mode <= DA7281_OPERATION_MODE_ETWM);

    /* Read current register value */
    err = da7281_read_register(dev, DA7281_REG_TOP_CTL1, &reg_val);
    if (err != DA7281_OK) return err;

    /* Clear mode bits and set new mode */
    reg_val &= ~DA7281_TOP_CTL1_OPERATION_MODE_MASK;
    reg_val |= (mode & DA7281_TOP_CTL1_OPERATION_MODE_MASK);

    /* Write back */
    err = da7281_write_register(dev, DA7281_REG_TOP_CTL1, reg_val);
    return err;
}

/**
 * @brief Get current operation mode
 */
da7281_error_t da7281_get_operation_mode(const da7281_device_t *dev, uint8_t *mode)
{
    uint8_t reg_val;
    da7281_error_t err;

    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(mode != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);

    err = da7281_read_register(dev, DA7281_REG_TOP_CTL1, &reg_val);
    if (err != DA7281_OK) return err;

    *mode = reg_val & DA7281_TOP_CTL1_OPERATION_MODE_MASK;
    return DA7281_OK;
}

/**
 * @brief Set override value for DRO mode
 */
da7281_error_t da7281_set_override_value(const da7281_device_t *dev, uint8_t value)
{
    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);
    DA7281_VALIDATE_PARAM(value <= DA7281_OVERRIDE_VAL_MAX);

    return da7281_write_register(dev, DA7281_REG_OVERRIDE_VAL, value);
}

/**
 * @brief Enable or disable standby mode
 */
da7281_error_t da7281_set_standby(const da7281_device_t *dev, bool enable)
{
    uint8_t reg_val;
    da7281_error_t err;

    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);

    err = da7281_read_register(dev, DA7281_REG_TOP_CFG1, &reg_val);
    if (err != DA7281_OK) return err;

    if (enable) {
        reg_val |= DA7281_TOP_CFG1_STANDBY_EN;
    } else {
        reg_val &= ~DA7281_TOP_CFG1_STANDBY_EN;
    }

    err = da7281_write_register(dev, DA7281_REG_TOP_CFG1, reg_val);
    if (err != DA7281_OK) return err;

    if (enable) {
        da7281_delay_ms(DA7281_STANDBY_DELAY_MS);
    }

    return DA7281_OK;
}

/**
 * @brief Run self-test sequence
 */
da7281_error_t da7281_run_self_test(const da7281_device_t *dev, uint16_t duration_ms)
{
    da7281_error_t err;

    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);
    DA7281_VALIDATE_PARAM(duration_ms > 0 && duration_ms <= 5000);

    /* Ensure device is in inactive mode */
    err = da7281_set_operation_mode(dev, DA7281_OPERATION_MODE_INACTIVE);
    if (err != DA7281_OK) return err;

    /* Set maximum override value */
    err = da7281_set_override_value(dev, DA7281_OVERRIDE_VAL_MAX);
    if (err != DA7281_OK) return err;

    /* Enter DRO mode */
    err = da7281_set_operation_mode(dev, DA7281_OPERATION_MODE_DRO);
    if (err != DA7281_OK) return err;

    /* Run for specified duration */
    da7281_delay_ms(duration_ms);

    /* Return to inactive mode */
    err = da7281_set_operation_mode(dev, DA7281_OPERATION_MODE_INACTIVE);
    if (err != DA7281_OK) return err;

    /* Enable standby */
    err = da7281_set_standby(dev, true);
    return err;
}

/**
 * @brief Read register value
 */
da7281_error_t da7281_read_register(const da7281_device_t *dev,
                                     uint8_t reg_addr, uint8_t *value)
{
    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(value != NULL);

    return da7281_i2c_read(dev, reg_addr, value, 1);
}

/**
 * @brief Write register value
 */
da7281_error_t da7281_write_register(const da7281_device_t *dev,
                                      uint8_t reg_addr, uint8_t value)
{
    DA7281_VALIDATE_PARAM(dev != NULL);

    return da7281_i2c_write(dev, reg_addr, &value, 1);
}

/*===========================================================================*/
/* PRIVATE FUNCTION IMPLEMENTATIONS                                          */
/*===========================================================================*/

/**
 * @brief I2C write operation with mutex protection
 */
static da7281_error_t da7281_i2c_write(const da7281_device_t *dev,
                                        uint8_t reg_addr,
                                        const uint8_t *data,
                                        uint8_t length)
{
    ret_code_t nrf_err;
    uint8_t buffer[32];
    da7281_error_t result = DA7281_OK;

    if (length > 31) {
        return DA7281_ERR_INVALID_PARAM;
    }

    /* Take mutex */
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(DA7281_I2C_TIMEOUT_MS)) != pdTRUE) {
        return DA7281_ERR_TIMEOUT;
    }

    /* Prepare buffer: register address + data */
    buffer[0] = reg_addr;
    memcpy(&buffer[1], data, length);

    /* Perform I2C write */
    nrf_err = nrf_drv_twi_tx(NULL, dev->i2c_address, buffer, length + 1, false);

    if (nrf_err != NRF_SUCCESS) {
        result = DA7281_ERR_I2C_COMM;
    }

    /* Release mutex */
    xSemaphoreGive(g_i2c_mutex);

    return result;
}

/**
 * @brief I2C read operation with mutex protection
 */
static da7281_error_t da7281_i2c_read(const da7281_device_t *dev,
                                       uint8_t reg_addr,
                                       uint8_t *data,
                                       uint8_t length)
{
    ret_code_t nrf_err;
    da7281_error_t result = DA7281_OK;

    /* Take mutex */
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(DA7281_I2C_TIMEOUT_MS)) != pdTRUE) {
        return DA7281_ERR_TIMEOUT;
    }

    /* Write register address */
    nrf_err = nrf_drv_twi_tx(NULL, dev->i2c_address, &reg_addr, 1, true);
    if (nrf_err != NRF_SUCCESS) {
        result = DA7281_ERR_I2C_COMM;
        goto cleanup;
    }

    /* Read data */
    nrf_err = nrf_drv_twi_rx(NULL, dev->i2c_address, data, length);
    if (nrf_err != NRF_SUCCESS) {
        result = DA7281_ERR_I2C_COMM;
    }

cleanup:
    /* Release mutex */
    xSemaphoreGive(g_i2c_mutex);

    return result;
}

/**
 * @brief Delay function using FreeRTOS
 */
static void da7281_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/*===========================================================================*/
/* END OF FILE                                                               */
/*===========================================================================*/

