/**
 * @file da7281_init.c
 * @brief DA7281 Device Initialization Routines
 * @author Abdul Rahman
 * @date 2024-08-15
 * @version 1.0.0
 * 
 * @copyright Copyright (c) 2024 Abdul Rahman
 * 
 * Licensed under the MIT License.
 * 
 * @details
 * This file contains the complete initialization sequence for the DA7281
 * haptic driver, following the datasheet timing requirements and
 * recommended configuration flow.
 */

/*===========================================================================*/
/* INCLUDES                                                                  */
/*===========================================================================*/

#include "da7281.h"
#include "nrf_log.h"

/*===========================================================================*/
/* PRIVATE MACROS AND DEFINES                                                */
/*===========================================================================*/

/* Default LRA configuration parameters */
#define DA7281_DEFAULT_NOM_MAX_V        (2.5f)      /* 2.5V RMS */
#define DA7281_DEFAULT_ABS_MAX_V        (3.5f)      /* 3.5V peak */
#define DA7281_DEFAULT_IMAX_MA          (350U)      /* 350mA */
#define DA7281_DEFAULT_IMPEDANCE_OHM    (6.75f)     /* 6.75 ohms */
#define DA7281_DEFAULT_RESONANT_FREQ_HZ (170U)      /* 170Hz */

/*===========================================================================*/
/* PUBLIC FUNCTION IMPLEMENTATIONS                                           */
/*===========================================================================*/

/**
 * @brief Complete initialization sequence for DA7281
 * 
 * This function performs the full initialization sequence:
 * 1. Power on with proper timing
 * 2. Verify chip ID
 * 3. Configure LRA parameters
 * 4. Set to standby mode
 * 
 * @param[in,out] dev Pointer to device handle
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_initialize(da7281_device_t *dev)
{
    da7281_error_t err;

    if (dev == NULL) {
        return DA7281_ERR_INVALID_PARAM;
    }

    NRF_LOG_INFO("DA7281: Starting initialization sequence");

    /* Step 1: Power on device */
    NRF_LOG_INFO("DA7281: Powering on device (GPIO pin %d)", dev->power_gpio_pin);
    err = da7281_power_on(dev);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("DA7281: Power on failed, error=%d", err);
        return err;
    }

    /* Step 2: Verify chip ID */
    NRF_LOG_INFO("DA7281: Verifying chip ID");
    err = da7281_verify_chip_id(dev);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("DA7281: Chip ID verification failed, error=%d", err);
        da7281_power_off(dev);
        return err;
    }
    NRF_LOG_INFO("DA7281: Chip ID verified successfully");

    /* Step 3: Ensure device is in inactive mode before configuration */
    NRF_LOG_INFO("DA7281: Setting to inactive mode");
    err = da7281_set_operation_mode(dev, DA7281_OPERATION_MODE_INACTIVE);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("DA7281: Failed to set inactive mode, error=%d", err);
        da7281_power_off(dev);
        return err;
    }

    /* Step 4: Configure LRA parameters */
    NRF_LOG_INFO("DA7281: Configuring LRA parameters");
    NRF_LOG_INFO("  - Nominal Max: %.2f V RMS", DA7281_DEFAULT_NOM_MAX_V);
    NRF_LOG_INFO("  - Absolute Max: %.2f V peak", DA7281_DEFAULT_ABS_MAX_V);
    NRF_LOG_INFO("  - Max Current: %d mA", DA7281_DEFAULT_IMAX_MA);
    NRF_LOG_INFO("  - Impedance: %.2f ohms", DA7281_DEFAULT_IMPEDANCE_OHM);
    NRF_LOG_INFO("  - Resonant Freq: %d Hz", DA7281_DEFAULT_RESONANT_FREQ_HZ);

    err = da7281_configure_lra(dev,
                                DA7281_DEFAULT_NOM_MAX_V,
                                DA7281_DEFAULT_ABS_MAX_V,
                                DA7281_DEFAULT_IMAX_MA,
                                DA7281_DEFAULT_IMPEDANCE_OHM,
                                DA7281_DEFAULT_RESONANT_FREQ_HZ);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("DA7281: LRA configuration failed, error=%d", err);
        da7281_power_off(dev);
        return err;
    }

    /* Step 5: Set to standby mode */
    NRF_LOG_INFO("DA7281: Enabling standby mode");
    err = da7281_set_standby(dev, true);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("DA7281: Failed to enable standby, error=%d", err);
        da7281_power_off(dev);
        return err;
    }

    /* Mark as initialized */
    dev->is_initialized = true;

    NRF_LOG_INFO("DA7281: Initialization complete");
    return DA7281_OK;
}

/**
 * @brief Initialize DA7281 with custom LRA parameters
 * 
 * @param[in,out] dev Pointer to device handle
 * @param[in] nom_max_v Nominal maximum voltage (RMS)
 * @param[in] abs_max_v Absolute maximum voltage (peak)
 * @param[in] imax_ma Maximum current in mA
 * @param[in] impedance_ohm LRA impedance in ohms
 * @param[in] resonant_freq_hz Resonant frequency in Hz
 * @return DA7281_OK on success, error code otherwise
 */
da7281_error_t da7281_initialize_custom(da7281_device_t *dev,
                                         float nom_max_v,
                                         float abs_max_v,
                                         uint16_t imax_ma,
                                         float impedance_ohm,
                                         uint16_t resonant_freq_hz)
{
    da7281_error_t err;

    if (dev == NULL) {
        return DA7281_ERR_INVALID_PARAM;
    }

    NRF_LOG_INFO("DA7281: Starting custom initialization");

    /* Power on */
    err = da7281_power_on(dev);
    if (err != DA7281_OK) {
        return err;
    }

    /* Verify chip */
    err = da7281_verify_chip_id(dev);
    if (err != DA7281_OK) {
        da7281_power_off(dev);
        return err;
    }

    /* Set inactive mode */
    err = da7281_set_operation_mode(dev, DA7281_OPERATION_MODE_INACTIVE);
    if (err != DA7281_OK) {
        da7281_power_off(dev);
        return err;
    }

    /* Configure with custom parameters */
    err = da7281_configure_lra(dev, nom_max_v, abs_max_v, imax_ma,
                                impedance_ohm, resonant_freq_hz);
    if (err != DA7281_OK) {
        da7281_power_off(dev);
        return err;
    }

    /* Enable standby */
    err = da7281_set_standby(dev, true);
    if (err != DA7281_OK) {
        da7281_power_off(dev);
        return err;
    }

    dev->is_initialized = true;

    NRF_LOG_INFO("DA7281: Custom initialization complete");
    return DA7281_OK;
}

/*===========================================================================*/
/* END OF FILE                                                               */
/*===========================================================================*/

