/* Stand-in for the nRF5 SDK legacy TWI driver API (same signatures as SDK 17.1 nrf_drv_twi.h).
 * Used for host tests (test/da7281_model.c implements it) and for compile checks without the SDK.
 */
#ifndef NRF_DRV_TWI_H__
#define NRF_DRV_TWI_H__

#include <stdbool.h>
#include <stdint.h>

typedef uint32_t ret_code_t;
#define NRF_SUCCESS                 (0U)
#define NRF_ERROR_DRV_TWI_ERR_ANACK (0x8201U)
#define NRF_ERROR_DRV_TWI_ERR_DNACK (0x8202U)

typedef struct {
    uint8_t inst_idx;
} nrf_drv_twi_t;

#define NRF_DRV_TWI_INSTANCE(id) \
    {                            \
        .inst_idx = (id)         \
    }

ret_code_t nrf_drv_twi_tx(nrf_drv_twi_t const *p_instance, uint8_t address, uint8_t const *p_data,
                          uint8_t length, bool no_stop);
ret_code_t nrf_drv_twi_rx(nrf_drv_twi_t const *p_instance, uint8_t address, uint8_t *p_data,
                          uint8_t length);

#endif
