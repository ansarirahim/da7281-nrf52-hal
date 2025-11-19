/**
 * @file da7281.h
 * @brief DA7281 Haptic Driver HAL for nRF52833 with FreeRTOS
 * @author A. R. Ansari
 * @date 2024-08-15
 * @version 1.0.0
 * 
 * @copyright Copyright (c) 2024 A. R. Ansari
 * 
 * Licensed under the MIT License.
 * 
 * @details
 * This HAL provides a complete driver interface for the Dialog Semiconductor
 * DA7281 haptic driver IC. Supports LRA (Linear Resonant Actuator) and ERM
 * (Eccentric Rotating Mass) motors with I2C control interface.
 * 
 * Features:
 * - Thread-safe I2C communication (FreeRTOS mutex protected)
 * - Multi-device support
 * - Power sequencing with datasheet-compliant timing
 * - Override mode for direct amplitude control
 * - Frequency tracking and acceleration control
 * - Self-test routines
 */

#ifndef DA7281_H
#define DA7281_H

#ifdef __cplusplus
extern "C" {
#endif

/*===========================================================================*/
/* INCLUDES                                                                  */
/*===========================================================================*/

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* REGISTER DEFINITIONS                                                      */
/*===========================================================================*/

/* Device identification */
#define DA7281_REG_CHIP_REV             (0x00U)
#define DA7281_REG_IRQ_EVENT1           (0x03U)
#define DA7281_REG_IRQ_EVENT_WARNING    (0x04U)
#define DA7281_REG_IRQ_EVENT_SEQ        (0x05U)
#define DA7281_REG_IRQ_STATUS1          (0x06U)
#define DA7281_REG_IRQ_MASK1            (0x07U)

/* Configuration registers */
#define DA7281_REG_TOP_CFG1             (0x22U)
#define DA7281_REG_TOP_CFG2             (0x23U)
#define DA7281_REG_TOP_CFG4             (0x25U)
#define DA7281_REG_TOP_INT_CFG1         (0x26U)
#define DA7281_REG_TOP_CTL1             (0x28U)
#define DA7281_REG_TOP_CTL2             (0x29U)

/* LRA configuration */
#define DA7281_REG_ACTUATOR1            (0x2AU)
#define DA7281_REG_ACTUATOR2            (0x2BU)
#define DA7281_REG_ACTUATOR3            (0x2CU)
#define DA7281_REG_CALIB_V2I_H          (0x2DU)
#define DA7281_REG_CALIB_V2I_L          (0x2EU)
#define DA7281_REG_TOP_CFG5             (0x2FU)

/* LRA period configuration */
#define DA7281_REG_LRA_PER_H            (0x30U)
#define DA7281_REG_LRA_PER_L            (0x31U)

/* Operation mode and control */
#define DA7281_REG_MODE                 (0x32U)
#define DA7281_REG_OVERRIDE_VAL         (0x33U)

/* Status registers */
#define DA7281_REG_GPI_0_CTL            (0x40U)
#define DA7281_REG_GPI_1_CTL            (0x41U)
#define DA7281_REG_GPI_2_CTL            (0x42U)

/* Memory and sequencer */
#define DA7281_REG_MEM_CTL1             (0x47U)
#define DA7281_REG_SEQ_CTL2             (0x49U)

/*===========================================================================*/
/* REGISTER BIT DEFINITIONS                                                  */
/*===========================================================================*/

/* TOP_CFG1 register bits */
#define DA7281_TOP_CFG1_STANDBY_EN      (0x80U)
#define DA7281_TOP_CFG1_AMP_PID_EN      (0x10U)
#define DA7281_TOP_CFG1_RAPID_STOP_EN   (0x08U)
#define DA7281_TOP_CFG1_ACCELERATION_EN (0x04U)
#define DA7281_TOP_CFG1_FREQ_TRACK_EN   (0x01U)

/* TOP_CTL1 register bits */
#define DA7281_TOP_CTL1_OPERATION_MODE_MASK (0x07U)
#define DA7281_OPERATION_MODE_INACTIVE      (0x00U)
#define DA7281_OPERATION_MODE_DRO           (0x01U)
#define DA7281_OPERATION_MODE_PWM           (0x02U)
#define DA7281_OPERATION_MODE_RTWM          (0x03U)
#define DA7281_OPERATION_MODE_ETWM          (0x04U)

/*===========================================================================*/
/* CONSTANTS AND MACROS                                                      */
/*===========================================================================*/

#define DA7281_I2C_ADDRESS_DEFAULT      (0x4AU)
#define DA7281_I2C_ADDRESS_ALT1         (0x4BU)
#define DA7281_I2C_ADDRESS_ALT2         (0x4CU)
#define DA7281_I2C_ADDRESS_ALT3         (0x4DU)

#define DA7281_CHIP_REV_EXPECTED        (0xBAU)

#define DA7281_POWER_UP_DELAY_MS        (2U)
#define DA7281_STANDBY_DELAY_MS         (1U)

/* Override value range */
#define DA7281_OVERRIDE_VAL_MIN         (0x00U)
#define DA7281_OVERRIDE_VAL_MAX         (0x7FU)

/*===========================================================================*/
/* TYPE DEFINITIONS                                                          */
/*===========================================================================*/

/**
 * @brief DA7281 device handle structure
 */
typedef struct {
    uint8_t i2c_address;        /**< I2C device address */
    uint8_t twi_instance;       /**< Nordic TWI/I2C instance number */
    uint8_t power_gpio_pin;     /**< GPIO pin for power control */
    uint8_t irq_gpio_pin;       /**< GPIO pin for interrupt (optional) */
    bool is_initialized;        /**< Initialization status flag */
    bool is_powered;            /**< Power state flag */
} da7281_device_t;

/**
 * @brief DA7281 error codes
 */
typedef enum {
    DA7281_OK = 0,              /**< Operation successful */
    DA7281_ERR_INVALID_PARAM,   /**< Invalid parameter */
    DA7281_ERR_I2C_COMM,        /**< I2C communication error */
    DA7281_ERR_TIMEOUT,         /**< Operation timeout */
    DA7281_ERR_NOT_INITIALIZED, /**< Device not initialized */
    DA7281_ERR_CHIP_ID,         /**< Chip ID mismatch */
    DA7281_ERR_POWER            /**< Power control error */
} da7281_error_t;

/*===========================================================================*/
/* FUNCTION PROTOTYPES                                                       */
/*===========================================================================*/

/**
 * @brief Initialize DA7281 device handle
 * @param[in,out] dev Pointer to device handle structure
 * @param[in] i2c_addr I2C address of the device
 * @param[in] twi_inst TWI/I2C instance number
 * @param[in] pwr_pin GPIO pin for power control
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_init_handle(da7281_device_t *dev, uint8_t i2c_addr, 
                                   uint8_t twi_inst, uint8_t pwr_pin);

/**
 * @brief Power on the DA7281 device
 * @param[in,out] dev Pointer to device handle
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_power_on(da7281_device_t *dev);

/**
 * @brief Power off the DA7281 device
 * @param[in,out] dev Pointer to device handle
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_power_off(da7281_device_t *dev);

/**
 * @brief Verify chip ID
 * @param[in] dev Pointer to device handle
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_verify_chip_id(const da7281_device_t *dev);

/**
 * @brief Configure LRA parameters
 * @param[in] dev Pointer to device handle
 * @param[in] nom_max_v Nominal maximum voltage (RMS)
 * @param[in] abs_max_v Absolute maximum voltage (peak)
 * @param[in] imax_ma Maximum current in mA
 * @param[in] impedance_ohm LRA impedance in ohms
 * @param[in] resonant_freq_hz Resonant frequency in Hz
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_configure_lra(const da7281_device_t *dev,
                                     float nom_max_v, float abs_max_v,
                                     uint16_t imax_ma, float impedance_ohm,
                                     uint16_t resonant_freq_hz);

/**
 * @brief Set operation mode
 * @param[in] dev Pointer to device handle
 * @param[in] mode Operation mode (0=Inactive, 1=DRO, 2=PWM, etc.)
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
 * @brief Set override value for DRO mode
 * @param[in] dev Pointer to device handle
 * @param[in] value Override value (0x00 to 0x7F)
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_set_override_value(const da7281_device_t *dev, uint8_t value);

/**
 * @brief Enable or disable standby mode
 * @param[in] dev Pointer to device handle
 * @param[in] enable true to enable standby, false to disable
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_set_standby(const da7281_device_t *dev, bool enable);

/**
 * @brief Run self-test sequence
 * @param[in] dev Pointer to device handle
 * @param[in] duration_ms Test duration in milliseconds
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
da7281_error_t da7281_read_register(const da7281_device_t *dev,
                                     uint8_t reg_addr, uint8_t *value);

/**
 * @brief Write register value
 * @param[in] dev Pointer to device handle
 * @param[in] reg_addr Register address
 * @param[in] value Value to write
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_write_register(const da7281_device_t *dev,
                                      uint8_t reg_addr, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* DA7281_H */

