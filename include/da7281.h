/**
 * @file da7281.h
 * @brief DA7281 Haptic Driver HAL for nRF52833 with FreeRTOS
 * @author A. R. Ansari
 * @version 2.0.0
 *
 * @copyright Copyright (c) 2024 A. R. Ansari
 *
 * Licensed under the MIT License.
 *
 * @details
 * Driver for the Renesas (Dialog) DA7281 LRA/ERM haptic driver over I2C, using the nRF5 SDK TWI
 * driver and FreeRTOS. Register addresses, fields and conversions follow the DA7281 datasheet,
 * Revision 3.1 (10-Feb-2022): Table 20 (register map), Tables 21-49 (fields), Table 8 (timing),
 * Section 5.9 (I2C addressing). Table references are given next to each definition.
 *
 * Features:
 * - Thread-safe register access: one FreeRTOS mutex per TWI bus, held across the register-pointer
 *   write and the read (repeated START)
 * - Multi-device support (up to four DA7281 per bus via the ADDR_1/ADDR_0 pins)
 * - Power-up timing that respects tON (1.5 ms max) at any FreeRTOS tick rate
 * - LRA configuration from actuator ratings (frequency, voltages, current, impedance)
 * - Direct register override (DRO) playback and self-test
 */

#ifndef DA7281_H
#define DA7281_H

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* INCLUDES                                                                  */
/*===========================================================================*/

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "nrf_drv_twi.h"
#include "semphr.h"

/*===========================================================================*/
/* REGISTER DEFINITIONS (Table 20)                                           */
/*===========================================================================*/

#define DA7281_REG_CHIP_REV      (0x00U) /* RO: [7:4] minor, [3:0] major (Table 21) */
#define DA7281_REG_IRQ_EVENT1    (0x03U) /* write 1 to clear (Table 22) */
#define DA7281_REG_IRQ_STATUS1   (0x06U)
#define DA7281_REG_IRQ_MASK1     (0x07U)
#define DA7281_REG_CIF_I2C2      (0x09U) /* [6:0] I2C base address, reset 0x4A (Table 28) */
#define DA7281_REG_FRQ_LRA_PER_H (0x0AU) /* LRA_PER[14:7] (Table 29) */
#define DA7281_REG_FRQ_LRA_PER_L (0x0BU) /* [6:0] = LRA_PER[6:0] (Table 30) */
#define DA7281_REG_ACTUATOR1     (0x0CU) /* ACTUATOR_NOMMAX, 23.4 mV per step (Table 31) */
#define DA7281_REG_ACTUATOR2     (0x0DU) /* ACTUATOR_ABSMAX, 23.4 mV per step (Table 32) */
#define DA7281_REG_ACTUATOR3     (0x0EU) /* [4:0] IMAX (Table 33) */
#define DA7281_REG_CALIB_V2I_H   (0x0FU) /* V2I_FACTOR[15:8] (Table 34) */
#define DA7281_REG_CALIB_V2I_L   (0x10U) /* V2I_FACTOR[7:0] (Table 35) */
#define DA7281_REG_TOP_CFG1      (0x13U) /* Table 38 */
#define DA7281_REG_TOP_CTL1      (0x22U) /* Table 48 */
#define DA7281_REG_TOP_CTL2      (0x23U) /* OVERRIDE_VAL (Table 49) */

/*===========================================================================*/
/* REGISTER BIT DEFINITIONS                                                  */
/*===========================================================================*/

/* CHIP_REV (Table 21): reset value 0xCA */
#define DA7281_CHIP_REV_MAJOR_MASK (0x0FU)
#define DA7281_CHIP_REV_MAJOR      (0x0AU)

/* TOP_CFG1 (Table 38); reset value 0x1E */
#define DA7281_TOP_CFG1_EMBEDDED_MODE   (0x80U)
#define DA7281_TOP_CFG1_ACTUATOR_TYPE   (0x20U) /* 0 = LRA, 1 = ERM */
#define DA7281_TOP_CFG1_BEMF_SENSE_EN   (0x10U) /* 1 = standard closed-loop operation */
#define DA7281_TOP_CFG1_FREQ_TRACK_EN   (0x08U)
#define DA7281_TOP_CFG1_ACCELERATION_EN (0x04U)
#define DA7281_TOP_CFG1_RAPID_STOP_EN   (0x02U)
#define DA7281_TOP_CFG1_AMP_PID_EN      (0x01U)

/* TOP_CTL1 (Table 48) */
#define DA7281_TOP_CTL1_SEQ_START            (0x10U)
#define DA7281_TOP_CTL1_STANDBY_EN           (0x08U) /* state after playback: 0 IDLE, 1 STANDBY */
#define DA7281_TOP_CTL1_OPERATION_MODE_MASK  (0x07U) /* bits [2:0] */
#define DA7281_TOP_CTL1_OPERATION_MODE_SHIFT (0U)

/* Operation mode values (TOP_CTL1.OPERATION_MODE) */
#define DA7281_OPERATION_MODE_INACTIVE (0x00U)
#define DA7281_OPERATION_MODE_DRO      (0x01U) /* direct register override */
#define DA7281_OPERATION_MODE_PWM      (0x02U)
#define DA7281_OPERATION_MODE_RTWM     (0x03U) /* register-triggered waveform memory */
#define DA7281_OPERATION_MODE_ETWM     (0x04U) /* edge-triggered waveform memory */

/*===========================================================================*/
/* CONSTANTS AND MACROS                                                      */
/*===========================================================================*/

/*
 * I2C address (Section 5.9, Table 19): the two LSBs of the 7-bit base address (0x4A at reset) are
 * replaced by the ADDR_1 and ADDR_0 pins, so with the reset base a DA7281 answers at 0x48-0x4B.
 */
#define DA7281_I2C_ADDRESS(addr1, addr0) \
    ((uint8_t)(0x48U | ((addr1) ? 2U : 0U) | ((addr0) ? 1U : 0U)))
#define DA7281_I2C_ADDRESS_DEFAULT (0x4AU) /* ADDR_1 = 1, ADDR_0 = 0 */
#define DA7281_I2C_ADDRESS_ALT1    (0x4BU) /* ADDR_1 = 1, ADDR_0 = 1 */

/* Expected CHIP_REV register value (Table 21) */
#define DA7281_CHIP_REV_EXPECTED (0xCAU)

/* Cold boot to IDLE, maximum (Table 8: tON) */
#define DA7281_POWER_UP_TIME_US (1500U)

/* LRA resonant frequency range supported by the device */
#define DA7281_LRA_FREQ_MIN_HZ (50U)
#define DA7281_LRA_FREQ_MAX_HZ (300U)

/* IMAX is a 5-bit field: 28.6 mA + 31 x 7.2 mA = 251.8 mA is the highest limit */
#define DA7281_IMAX_CODE_MAX (31U)

/* Override value range in DRO mode with ACCELERATION_EN = 1 (Table 49) */
#define DA7281_OVERRIDE_VAL_MIN (0x00U)
#define DA7281_OVERRIDE_VAL_MAX (0x7FU)

/* Value for power_gpio_pin when VDD is not switched by a GPIO */
#define DA7281_NO_PIN (0xFFU)

/*===========================================================================*/
/* TYPE DEFINITIONS                                                          */
/*===========================================================================*/

/**
 * @brief One TWI bus shared by tasks.
 *
 * Initialise it once with da7281_bus_init() before vTaskStartScheduler(), so that the mutex exists
 * before any task can use the bus.
 */
typedef struct {
    const nrf_drv_twi_t *twi; /**< Enabled nRF5 SDK TWI driver instance */
    SemaphoreHandle_t mutex;  /**< Serialises transactions on this bus */
} da7281_bus_t;

/**
 * @brief DA7281 device handle structure
 */
typedef struct {
    da7281_bus_t *bus;      /**< Bus the device is on */
    uint8_t i2c_address;    /**< Effective 7-bit I2C address */
    uint8_t power_gpio_pin; /**< GPIO switching the device's VDD (load switch), or DA7281_NO_PIN */
    uint8_t irq_gpio_pin;   /**< GPIO connected to nIRQ (optional), or DA7281_NO_PIN */
    bool is_initialized;    /**< Initialization status flag */
    bool is_powered;        /**< Power state flag */
} da7281_device_t;

/**
 * @brief DA7281 error codes
 */
typedef enum {
    DA7281_OK = 0,              /**< Operation successful */
    DA7281_ERR_INVALID_PARAM,   /**< Invalid parameter */
    DA7281_ERR_I2C_COMM,        /**< I2C communication error (including address NACK) */
    DA7281_ERR_TIMEOUT,         /**< Bus mutex not obtained in time */
    DA7281_ERR_NOT_INITIALIZED, /**< Device not initialized / not powered */
    DA7281_ERR_CHIP_ID,         /**< CHIP_REV does not identify a DA7281 */
    DA7281_ERR_POWER            /**< Power control error */
} da7281_error_t;

/*===========================================================================*/
/* FUNCTION PROTOTYPES                                                       */
/*===========================================================================*/

/**
 * @brief Initialise a TWI bus object (call before the scheduler starts)
 * @param[out] bus Bus object
 * @param[in] twi Enabled TWI driver instance (NRF_DRV_TWI_INSTANCE(n), after
 * nrf_drv_twi_init/enable)
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_bus_init(da7281_bus_t *bus, const nrf_drv_twi_t *twi);

/**
 * @brief Initialize DA7281 device handle
 * @param[in,out] dev Pointer to device handle structure
 * @param[in] bus Initialised bus the device is on
 * @param[in] i2c_addr Effective 7-bit I2C address (see DA7281_I2C_ADDRESS)
 * @param[in] pwr_pin GPIO switching the device's supply, or DA7281_NO_PIN
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_init_handle(da7281_device_t *dev, da7281_bus_t *bus, uint8_t i2c_addr,
                                  uint8_t pwr_pin);

/**
 * @brief Power on the DA7281 (switch VDD if a pin is configured) and wait for tON
 * @param[in,out] dev Pointer to device handle
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_power_on(da7281_device_t *dev);

/**
 * @brief Power off the DA7281 (only possible when VDD is switched by a GPIO)
 * @param[in,out] dev Pointer to device handle
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_power_off(da7281_device_t *dev);

/**
 * @brief Verify that the device is a DA7281 (CHIP_REV major revision 0xA)
 * @param[in] dev Pointer to device handle
 * @return DA7281_OK on success, DA7281_ERR_CHIP_ID on mismatch, error code otherwise
 */
da7281_error_t da7281_verify_chip_id(const da7281_device_t *dev);

/**
 * @brief Configure LRA parameters from the actuator's datasheet ratings
 *
 * A current rating above 251.8 mA is limited to IMAX = 31 (251.8 mA), the highest limit the device
 * can apply.
 *
 * @param[in] dev Pointer to device handle
 * @param[in] nom_max_v Nominal (rated) voltage, 0 to 5.967 V
 * @param[in] abs_max_v Absolute maximum voltage, 0 to 5.967 V
 * @param[in] imax_ma Maximum current in mA (at least 29)
 * @param[in] impedance_ohm LRA impedance in ohms
 * @param[in] resonant_freq_hz Resonant frequency in Hz (50 to 300)
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_configure_lra(const da7281_device_t *dev, float nom_max_v, float abs_max_v,
                                    uint16_t imax_ma, float impedance_ohm,
                                    uint16_t resonant_freq_hz);

/**
 * @brief Set operation mode (TOP_CTL1.OPERATION_MODE)
 * @param[in] dev Pointer to device handle
 * @param[in] mode Operation mode (DA7281_OPERATION_MODE_*)
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_set_operation_mode(const da7281_device_t *dev, uint8_t mode);

/**
 * @brief Get current operation mode
 * @param[in] dev Pointer to device handle
 * @param[out] mode Pointer to store current mode
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_get_operation_mode(const da7281_device_t *dev, uint8_t *mode);

/**
 * @brief Set override value for DRO mode (TOP_CTL2)
 * @param[in] dev Pointer to device handle
 * @param[in] value Override value (0x00 to 0x7F; 0x7F = full scale)
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_set_override_value(const da7281_device_t *dev, uint8_t value);

/**
 * @brief Choose the state the device returns to after playback (TOP_CTL1.STANDBY_EN)
 *
 * true: STANDBY (about 0.8 mA, 0.15 ms to output); false: IDLE (about 0.36 uA, 0.75 ms to output).
 *
 * @param[in] dev Pointer to device handle
 * @param[in] enable true to return to STANDBY, false to return to IDLE
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_set_standby(const da7281_device_t *dev, bool enable);

/**
 * @brief Run self-test sequence (DRO pulse at full scale)
 * @param[in] dev Pointer to device handle
 * @param[in] duration_ms Test duration in milliseconds (1 to 5000)
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_run_self_test(const da7281_device_t *dev, uint16_t duration_ms);

/**
 * @brief Read register value
 * @param[in] dev Pointer to device handle
 * @param[in] reg_addr Register address
 * @param[out] value Pointer to store register value
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_read_register(const da7281_device_t *dev, uint8_t reg_addr, uint8_t *value);

/**
 * @brief Write register value
 * @param[in] dev Pointer to device handle
 * @param[in] reg_addr Register address
 * @param[in] value Value to write
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_write_register(const da7281_device_t *dev, uint8_t reg_addr, uint8_t value);

/**
 * @brief Read-modify-write: change only the bits in mask, under one bus lock
 * @param[in] dev Pointer to device handle
 * @param[in] reg_addr Register address
 * @param[in] mask Bits to change
 * @param[in] value New values for those bits
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_update_bits(const da7281_device_t *dev, uint8_t reg_addr, uint8_t mask,
                                  uint8_t value);

/**
 * @brief Complete initialization with the default LRA parameters (see da7281_init.c)
 * @param[in,out] dev Pointer to device handle
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_initialize(da7281_device_t *dev);

/**
 * @brief Complete initialization with custom LRA parameters
 * @param[in,out] dev Pointer to device handle
 * @param[in] nom_max_v Nominal (rated) voltage
 * @param[in] abs_max_v Absolute maximum voltage
 * @param[in] imax_ma Maximum current in mA
 * @param[in] impedance_ohm LRA impedance in ohms
 * @param[in] resonant_freq_hz Resonant frequency in Hz
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_initialize_custom(da7281_device_t *dev, float nom_max_v, float abs_max_v,
                                        uint16_t imax_ma, float impedance_ohm,
                                        uint16_t resonant_freq_hz);

#ifdef __cplusplus
}
#endif

#endif /* DA7281_H */
