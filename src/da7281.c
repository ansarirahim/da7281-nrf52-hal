/**
 * @file da7281.c
 * @brief DA7281 Haptic Driver HAL Implementation
 * @author A. R. Ansari
 * @version 2.0.0
 *
 * @copyright Copyright (c) 2024 A. R. Ansari
 *
 * Licensed under the MIT License.
 */

/*===========================================================================*/
/* INCLUDES                                                                  */
/*===========================================================================*/

#include "da7281.h"

#include <stddef.h>
#include <string.h>

#include "nrf_gpio.h"
#include "task.h"

/*===========================================================================*/
/* PRIVATE MACROS AND DEFINES                                                */
/*===========================================================================*/

#define DA7281_I2C_TIMEOUT_MS (100U)
#define DA7281_VALIDATE_PARAM(cond)          \
    do {                                     \
        if (!(cond)) {                       \
            return DA7281_ERR_INVALID_PARAM; \
        }                                    \
    } while (0)

/*===========================================================================*/
/* PRIVATE FUNCTION PROTOTYPES                                               */
/*===========================================================================*/

static da7281_error_t da7281_lock(const da7281_device_t *dev);
static void da7281_unlock(const da7281_device_t *dev);
static da7281_error_t da7281_i2c_write_locked(const da7281_device_t *dev, uint8_t reg_addr,
                                              uint8_t value);
static da7281_error_t da7281_i2c_read_locked(const da7281_device_t *dev, uint8_t reg_addr,
                                             uint8_t *value);
static void da7281_delay_at_least_us(uint32_t us);

/*===========================================================================*/
/* PUBLIC FUNCTION IMPLEMENTATIONS                                           */
/*===========================================================================*/

/**
 * @brief Initialise a TWI bus object
 */
da7281_error_t da7281_bus_init(da7281_bus_t *bus, const nrf_drv_twi_t *twi)
{
    DA7281_VALIDATE_PARAM(bus != NULL);
    DA7281_VALIDATE_PARAM(twi != NULL);

    bus->twi = twi;
    /* Created once, before any task can use the bus. Creating it lazily inside a driver call races
     * when two tasks make their first call at the same time. */
    bus->mutex = xSemaphoreCreateMutex();
    return (bus->mutex != NULL) ? DA7281_OK : DA7281_ERR_I2C_COMM;
}

/**
 * @brief Initialize DA7281 device handle
 */
da7281_error_t da7281_init_handle(da7281_device_t *dev, da7281_bus_t *bus, uint8_t i2c_addr,
                                  uint8_t pwr_pin)
{
    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(bus != NULL && bus->twi != NULL && bus->mutex != NULL);
    /* Any 7-bit address: the base address in CIF_I2C2 can be reprogrammed (Section 5.9). */
    DA7281_VALIDATE_PARAM(i2c_addr <= 0x7FU);

    (void)memset(dev, 0, sizeof(*dev));
    dev->bus = bus;
    dev->i2c_address = i2c_addr;
    dev->power_gpio_pin = pwr_pin;
    dev->irq_gpio_pin = DA7281_NO_PIN;

    /* The DA7281 has no enable pin (datasheet Table 1). If a GPIO switches its supply, start off.
     */
    if (pwr_pin != DA7281_NO_PIN) {
        nrf_gpio_cfg_output(pwr_pin);
        nrf_gpio_pin_clear(pwr_pin);
    }

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

    if (dev->power_gpio_pin != DA7281_NO_PIN) {
        nrf_gpio_pin_set(dev->power_gpio_pin);
    }

    /* Cold boot to IDLE takes up to 1.5 ms (Table 8, tON). No I2C access before that. */
    da7281_delay_at_least_us(DA7281_POWER_UP_TIME_US);

    dev->is_powered = true;
    return DA7281_OK;
}

/**
 * @brief Power off the DA7281 device
 */
da7281_error_t da7281_power_off(da7281_device_t *dev)
{
    DA7281_VALIDATE_PARAM(dev != NULL);

    if (dev->power_gpio_pin == DA7281_NO_PIN) {
        return DA7281_ERR_POWER; /* VDD is not switchable; use IDLE (0.36 uA) instead */
    }
    nrf_gpio_pin_clear(dev->power_gpio_pin);
    dev->is_powered = false;
    dev->is_initialized = false;
    return DA7281_OK;
}

/**
 * @brief Verify chip ID
 *
 * CHIP_REV (0x00) resets to 0xCA: minor revision 0xC in bits [7:4], major 0xA in bits [3:0]
 * (Table 21). Only the major revision is checked so newer minor silicon revisions are accepted.
 */
da7281_error_t da7281_verify_chip_id(const da7281_device_t *dev)
{
    uint8_t chip_rev = 0U;
    da7281_error_t err;

    DA7281_VALIDATE_PARAM(dev != NULL);
    if (!dev->is_powered) {
        return DA7281_ERR_NOT_INITIALIZED;
    }

    err = da7281_read_register(dev, DA7281_REG_CHIP_REV, &chip_rev);
    if (err != DA7281_OK) {
        return err;
    }
    return ((chip_rev & DA7281_CHIP_REV_MAJOR_MASK) == DA7281_CHIP_REV_MAJOR) ? DA7281_OK
                                                                              : DA7281_ERR_CHIP_ID;
}

/**
 * @brief Configure LRA parameters
 *
 * Conversions (integer arithmetic, rounded):
 * - LRA_PER = 1 / (f x 1333.32 ns), 15 bits: H = bits [14:7], L = bits [6:0] (Tables 29, 30)
 * - ACTUATOR_NOMMAX / ABSMAX = V / 23.4 mV, 8 bits (Tables 31, 32)
 * - IMAX = (I - 28.6 mA) / 7.2 mA rounded down, 5 bits, limited to 31 (Table 33)
 * - V2I_FACTOR = Z x (IMAX + 4) / 1.6104, Z in ohms, 16 bits (Table 34)
 */
da7281_error_t da7281_configure_lra(const da7281_device_t *dev, float nom_max_v, float abs_max_v,
                                    uint16_t imax_ma, float impedance_ohm,
                                    uint16_t resonant_freq_hz)
{
    da7281_error_t err;
    uint32_t nom_mv;
    uint32_t abs_mv;
    uint32_t z_mohm;
    uint32_t nommax;
    uint32_t absmax;
    uint32_t imax;
    uint32_t lra_per;
    uint64_t v2i;

    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);
    DA7281_VALIDATE_PARAM(nom_max_v > 0.0f && nom_max_v <= 5.967f);
    DA7281_VALIDATE_PARAM(abs_max_v > 0.0f && abs_max_v <= 5.967f);
    DA7281_VALIDATE_PARAM(impedance_ohm > 0.0f && impedance_ohm < 1000.0f);
    DA7281_VALIDATE_PARAM(imax_ma >= 29U);
    DA7281_VALIDATE_PARAM(resonant_freq_hz >= DA7281_LRA_FREQ_MIN_HZ &&
                          resonant_freq_hz <= DA7281_LRA_FREQ_MAX_HZ);

    nom_mv = (uint32_t)(nom_max_v * 1000.0f + 0.5f);
    abs_mv = (uint32_t)(abs_max_v * 1000.0f + 0.5f);
    z_mohm = (uint32_t)(impedance_ohm * 1000.0f + 0.5f);

    lra_per = (uint32_t)((100000000000ULL + (uint64_t)resonant_freq_hz * 133332ULL / 2ULL) /
                         ((uint64_t)resonant_freq_hz * 133332ULL));
    nommax = (nom_mv * 10U + 117U) / 234U;
    absmax = (abs_mv * 10U + 117U) / 234U;
    imax = ((uint32_t)imax_ma * 10U - 286U) / 72U;
    if (imax > DA7281_IMAX_CODE_MAX) {
        imax = DA7281_IMAX_CODE_MAX; /* never wrap into the 5-bit field */
    }
    v2i = ((uint64_t)z_mohm * (imax + 4U) * 10ULL + 8052ULL) / 16104ULL;
    DA7281_VALIDATE_PARAM(nommax <= 0xFFU && absmax <= 0xFFU && v2i <= 0xFFFFU);

    err = da7281_write_register(dev, DA7281_REG_FRQ_LRA_PER_H, (uint8_t)(lra_per >> 7));
    if (err == DA7281_OK) {
        err = da7281_write_register(dev, DA7281_REG_FRQ_LRA_PER_L, (uint8_t)(lra_per & 0x7FU));
    }
    if (err == DA7281_OK) {
        err = da7281_write_register(dev, DA7281_REG_ACTUATOR1, (uint8_t)nommax);
    }
    if (err == DA7281_OK) {
        err = da7281_write_register(dev, DA7281_REG_ACTUATOR2, (uint8_t)absmax);
    }
    if (err == DA7281_OK) {
        err = da7281_update_bits(dev, DA7281_REG_ACTUATOR3, 0x1FU, (uint8_t)imax);
    }
    if (err == DA7281_OK) {
        err = da7281_write_register(dev, DA7281_REG_CALIB_V2I_H, (uint8_t)(v2i >> 8));
    }
    if (err == DA7281_OK) {
        err = da7281_write_register(dev, DA7281_REG_CALIB_V2I_L, (uint8_t)(v2i & 0xFFU));
    }
    if (err == DA7281_OK) {
        /* LRA with frequency tracking, active acceleration and rapid stop. Read-modify-write keeps
         * BEMF_SENSE_EN (set at reset, needed for standard operation) and the other bits. */
        err =
            da7281_update_bits(dev, DA7281_REG_TOP_CFG1,
                               DA7281_TOP_CFG1_ACTUATOR_TYPE | DA7281_TOP_CFG1_FREQ_TRACK_EN |
                                   DA7281_TOP_CFG1_ACCELERATION_EN | DA7281_TOP_CFG1_RAPID_STOP_EN,
                               DA7281_TOP_CFG1_FREQ_TRACK_EN | DA7281_TOP_CFG1_ACCELERATION_EN |
                                   DA7281_TOP_CFG1_RAPID_STOP_EN);
    }
    return err;
}

/**
 * @brief Set operation mode: TOP_CTL1 (0x22) bits [2:0] (Table 48)
 */
da7281_error_t da7281_set_operation_mode(const da7281_device_t *dev, uint8_t mode)
{
    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);
    DA7281_VALIDATE_PARAM(mode <= DA7281_OPERATION_MODE_ETWM);

    return da7281_update_bits(dev, DA7281_REG_TOP_CTL1, DA7281_TOP_CTL1_OPERATION_MODE_MASK,
                              (uint8_t)(mode << DA7281_TOP_CTL1_OPERATION_MODE_SHIFT));
}

/**
 * @brief Get current operation mode
 */
da7281_error_t da7281_get_operation_mode(const da7281_device_t *dev, uint8_t *mode)
{
    uint8_t reg_val = 0U;
    da7281_error_t err;

    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(mode != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);

    err = da7281_read_register(dev, DA7281_REG_TOP_CTL1, &reg_val);
    if (err == DA7281_OK) {
        *mode = (uint8_t)((reg_val & DA7281_TOP_CTL1_OPERATION_MODE_MASK) >>
                          DA7281_TOP_CTL1_OPERATION_MODE_SHIFT);
    }
    return err;
}

/**
 * @brief Set override value for DRO mode: TOP_CTL2 (0x23) (Table 49)
 */
da7281_error_t da7281_set_override_value(const da7281_device_t *dev, uint8_t value)
{
    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);
    /* With ACCELERATION_EN = 1, values above 0x7F give no drive at all. */
    DA7281_VALIDATE_PARAM(value <= DA7281_OVERRIDE_VAL_MAX);

    return da7281_write_register(dev, DA7281_REG_TOP_CTL2, value);
}

/**
 * @brief Choose the state after playback (TOP_CTL1.STANDBY_EN)
 */
da7281_error_t da7281_set_standby(const da7281_device_t *dev, bool enable)
{
    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);

    return da7281_update_bits(dev, DA7281_REG_TOP_CTL1, DA7281_TOP_CTL1_STANDBY_EN,
                              enable ? DA7281_TOP_CTL1_STANDBY_EN : 0U);
}

/**
 * @brief Run self-test sequence
 */
da7281_error_t da7281_run_self_test(const da7281_device_t *dev, uint16_t duration_ms)
{
    da7281_error_t err;

    DA7281_VALIDATE_PARAM(dev != NULL);
    DA7281_VALIDATE_PARAM(dev->is_powered);
    DA7281_VALIDATE_PARAM(duration_ms > 0U && duration_ms <= 5000U);

    err = da7281_set_operation_mode(dev, DA7281_OPERATION_MODE_INACTIVE);
    if (err == DA7281_OK) {
        err = da7281_set_override_value(dev, DA7281_OVERRIDE_VAL_MAX);
    }
    if (err == DA7281_OK) {
        err = da7281_set_operation_mode(dev, DA7281_OPERATION_MODE_DRO);
    }
    if (err == DA7281_OK) {
        vTaskDelay(pdMS_TO_TICKS(duration_ms));
        err = da7281_set_operation_mode(dev, DA7281_OPERATION_MODE_INACTIVE);
    }
    return err;
}

/**
 * @brief Read register value
 */
da7281_error_t da7281_read_register(const da7281_device_t *dev, uint8_t reg_addr, uint8_t *value)
{
    da7281_error_t err;

    DA7281_VALIDATE_PARAM(dev != NULL && dev->bus != NULL);
    DA7281_VALIDATE_PARAM(value != NULL);

    err = da7281_lock(dev);
    if (err == DA7281_OK) {
        err = da7281_i2c_read_locked(dev, reg_addr, value);
        da7281_unlock(dev);
    }
    return err;
}

/**
 * @brief Write register value
 */
da7281_error_t da7281_write_register(const da7281_device_t *dev, uint8_t reg_addr, uint8_t value)
{
    da7281_error_t err;

    DA7281_VALIDATE_PARAM(dev != NULL && dev->bus != NULL);

    err = da7281_lock(dev);
    if (err == DA7281_OK) {
        err = da7281_i2c_write_locked(dev, reg_addr, value);
        da7281_unlock(dev);
    }
    return err;
}

/**
 * @brief Read-modify-write under one bus lock
 */
da7281_error_t da7281_update_bits(const da7281_device_t *dev, uint8_t reg_addr, uint8_t mask,
                                  uint8_t value)
{
    uint8_t old = 0U;
    da7281_error_t err;

    DA7281_VALIDATE_PARAM(dev != NULL && dev->bus != NULL);

    err = da7281_lock(dev);
    if (err == DA7281_OK) {
        err = da7281_i2c_read_locked(dev, reg_addr, &old);
        if (err == DA7281_OK) {
            err = da7281_i2c_write_locked(dev, reg_addr,
                                          (uint8_t)((old & (uint8_t)~mask) | (value & mask)));
        }
        da7281_unlock(dev);
    }
    return err;
}

/*===========================================================================*/
/* PRIVATE FUNCTION IMPLEMENTATIONS                                          */
/*===========================================================================*/

static da7281_error_t da7281_lock(const da7281_device_t *dev)
{
    if (xSemaphoreTake(dev->bus->mutex, pdMS_TO_TICKS(DA7281_I2C_TIMEOUT_MS)) != pdTRUE) {
        return DA7281_ERR_TIMEOUT;
    }
    return DA7281_OK;
}

static void da7281_unlock(const da7281_device_t *dev) { (void)xSemaphoreGive(dev->bus->mutex); }

static da7281_error_t da7281_i2c_write_locked(const da7281_device_t *dev, uint8_t reg_addr,
                                              uint8_t value)
{
    const uint8_t buffer[2] = {reg_addr, value};

    return (nrf_drv_twi_tx(dev->bus->twi, dev->i2c_address, buffer, sizeof(buffer), false) ==
            NRF_SUCCESS)
               ? DA7281_OK
               : DA7281_ERR_I2C_COMM;
}

static da7281_error_t da7281_i2c_read_locked(const da7281_device_t *dev, uint8_t reg_addr,
                                             uint8_t *value)
{
    /* Register pointer without STOP, then the read starts with a repeated START. */
    if (nrf_drv_twi_tx(dev->bus->twi, dev->i2c_address, &reg_addr, 1U, true) != NRF_SUCCESS) {
        return DA7281_ERR_I2C_COMM;
    }
    if (nrf_drv_twi_rx(dev->bus->twi, dev->i2c_address, value, 1U) != NRF_SUCCESS) {
        return DA7281_ERR_I2C_COMM;
    }
    return DA7281_OK;
}

/**
 * @brief Block for at least the given time
 *
 * vTaskDelay(n) returns between n-1 and n tick periods later because it starts mid-tick, so round
 * up to whole ticks and add one. (pdMS_TO_TICKS(2) at the nRF5 SDK's 1024 Hz tick is 2 ticks, which
 * can last only 0.98 ms: shorter than tON.) Must be called from a task.
 */
static void da7281_delay_at_least_us(uint32_t us)
{
    const TickType_t ticks =
        (TickType_t)((((uint64_t)us * configTICK_RATE_HZ) + 999999ULL) / 1000000ULL) + 1U;
    vTaskDelay(ticks);
}

/*===========================================================================*/
/* END OF FILE                                                               */
/*===========================================================================*/
