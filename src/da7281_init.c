/**
 * @file da7281_init.c
 * @brief DA7281 Device Initialization Routines
 * @author A. R. Ansari
 * @version 2.0.0
 *
 * @copyright Copyright (c) 2024 A. R. Ansari
 *
 * Licensed under the MIT License.
 *
 * @details
 * Initialization sequence: power-up with tON respected, CHIP_REV check, inactive mode, LRA
 * configuration, and the low-power IDLE state between playbacks.
 */

/*===========================================================================*/
/* INCLUDES                                                                  */
/*===========================================================================*/

#include "da7281.h"

#include <stddef.h>

#include "nrf_log.h"

/*===========================================================================*/
/* PRIVATE MACROS AND DEFINES                                                */
/*===========================================================================*/

/* Default LRA configuration parameters (example actuator) */
#define DA7281_DEFAULT_NOM_MAX_V        (2.5f)  /* 2.5 V rated */
#define DA7281_DEFAULT_ABS_MAX_V        (3.5f)  /* 3.5 V absolute maximum */
#define DA7281_DEFAULT_IMAX_MA          (350U)  /* limited by the device to 251.8 mA (IMAX = 31) */
#define DA7281_DEFAULT_IMPEDANCE_OHM    (6.75f) /* 6.75 ohms */
#define DA7281_DEFAULT_RESONANT_FREQ_HZ (170U)  /* 170 Hz */

/*===========================================================================*/
/* PUBLIC FUNCTION IMPLEMENTATIONS                                           */
/*===========================================================================*/

/**
 * @brief Complete initialization sequence for DA7281 with the default parameters
 */
da7281_error_t da7281_initialize(da7281_device_t *dev)
{
    NRF_LOG_INFO("DA7281: Configuring LRA parameters");
    NRF_LOG_INFO("  - Nominal Max: %u mV", (unsigned)(DA7281_DEFAULT_NOM_MAX_V * 1000.0f));
    NRF_LOG_INFO("  - Absolute Max: %u mV", (unsigned)(DA7281_DEFAULT_ABS_MAX_V * 1000.0f));
    NRF_LOG_INFO("  - Max Current: %u mA", (unsigned)DA7281_DEFAULT_IMAX_MA);
    NRF_LOG_INFO("  - Impedance: %u mOhm", (unsigned)(DA7281_DEFAULT_IMPEDANCE_OHM * 1000.0f));
    NRF_LOG_INFO("  - Resonant Freq: %u Hz", (unsigned)DA7281_DEFAULT_RESONANT_FREQ_HZ);

    return da7281_initialize_custom(dev, DA7281_DEFAULT_NOM_MAX_V, DA7281_DEFAULT_ABS_MAX_V,
                                    DA7281_DEFAULT_IMAX_MA, DA7281_DEFAULT_IMPEDANCE_OHM,
                                    DA7281_DEFAULT_RESONANT_FREQ_HZ);
}

/**
 * @brief Initialize DA7281 with custom LRA parameters
 */
da7281_error_t da7281_initialize_custom(da7281_device_t *dev, float nom_max_v, float abs_max_v,
                                        uint16_t imax_ma, float impedance_ohm,
                                        uint16_t resonant_freq_hz)
{
    da7281_error_t err;

    if (dev == NULL) {
        return DA7281_ERR_INVALID_PARAM;
    }

    NRF_LOG_INFO("DA7281: Starting initialization sequence");

    /* Step 1: power on (if switched) and wait for tON */
    err = da7281_power_on(dev);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("DA7281: Power on failed, error=%d", err);
        return err;
    }

    /* Step 2: CHIP_REV (0x00) must report major revision 0xA */
    err = da7281_verify_chip_id(dev);
    if (err != DA7281_OK) {
        NRF_LOG_ERROR("DA7281: Chip ID verification failed, error=%d", err);
        (void)da7281_power_off(dev);
        return err;
    }
    NRF_LOG_INFO("DA7281: Chip ID verified successfully");

    /* Step 3: inactive mode before configuration */
    err = da7281_set_operation_mode(dev, DA7281_OPERATION_MODE_INACTIVE);

    /* Step 4: actuator configuration */
    if (err == DA7281_OK) {
        err = da7281_configure_lra(dev, nom_max_v, abs_max_v, imax_ma, impedance_ohm,
                                   resonant_freq_hz);
    }
    if (imax_ma > 251U) {
        NRF_LOG_WARNING("DA7281: current rating %u mA limited to 251.8 mA (IMAX = 31)",
                        (unsigned)imax_ma);
    }

    /* Step 5: return to IDLE (lowest power, 0.36 uA) after each playback */
    if (err == DA7281_OK) {
        err = da7281_set_standby(dev, false);
    }

    if (err != DA7281_OK) {
        NRF_LOG_ERROR("DA7281: Configuration failed, error=%d", err);
        (void)da7281_power_off(dev);
        return err;
    }

    dev->is_initialized = true;
    NRF_LOG_INFO("DA7281: Initialization complete");
    return DA7281_OK;
}

/*===========================================================================*/
/* END OF FILE                                                               */
/*===========================================================================*/
