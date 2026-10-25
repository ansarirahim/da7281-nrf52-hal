/*
 * Host-side DA7281 register model behind the nRF5 SDK TWI API, built from the DA7281 datasheet
 * (Revision 3.1): Table 20 register map with reset values and read-only registers, the address-pin
 * rule (Section 5.9) and the cold-boot time tON = 1.5 ms (Table 8).
 *
 * Time advances only in vTaskDelay, by the SHORTEST time the call can block ((n - 1) tick periods),
 * so boot-time checks are worst case. Reads of unmapped addresses return 0 (undefined on silicon);
 * writes to unmapped or read-only addresses are ignored and counted.
 */
#include "da7281_model.h"

#include <string.h>

#include "nrf_drv_twi.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "semphr.h"
#include "task.h"

/* Logging is discarded in the tests (set DA7281_TEST_LOG to see it) */
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
void nrf_log_stub(const char *fmt, ...)
{
    if (getenv("DA7281_TEST_LOG") != NULL) {
        va_list ap;
        va_start(ap, fmt);
        (void)vprintf(fmt, ap);
        va_end(ap);
        (void)putchar('\n');
    }
}

struct reg {
    uint8_t addr;
    uint8_t reset;
    uint8_t ro;
};

static const struct reg MAP[] = {
    {0x00, 0xCA, 1}, {0x03, 0x00, 0}, {0x04, 0x00, 0}, {0x05, 0x00, 0}, {0x06, 0x00, 0},
    {0x07, 0x00, 0}, {0x08, 0x40, 0}, {0x09, 0x4A, 0}, {0x0A, 0x21, 0}, {0x0B, 0x4F, 0},
    {0x0C, 0x5A, 0}, {0x0D, 0x78, 0}, {0x0E, 0x17, 0}, {0x0F, 0x01, 0}, {0x10, 0x0D, 0},
    {0x11, 0x00, 0}, {0x12, 0x00, 0}, {0x13, 0x1E, 0}, {0x14, 0x01, 0}, {0x15, 0x03, 0},
    {0x16, 0x40, 0}, {0x17, 0x81, 0}, {0x1C, 0x00, 0}, {0x1D, 0x00, 0}, {0x1E, 0x00, 0},
    {0x1F, 0x00, 0}, {0x20, 0x00, 0}, {0x22, 0x00, 0}, {0x23, 0x00, 0}, {0x24, 0x00, 0},
    {0x25, 0x00, 0}, {0x26, 0x00, 0}, {0x27, 0x00, 0}, {0x28, 0x00, 0}, {0x2B, 0x02, 0},
    {0x2C, 0x84, 1}, {0x2D, 0x80, 0}, {0x2E, 0xFF, 1}, {0x2F, 0x00, 1},
};

static uint8_t regs[256];
static uint8_t mapped[256];
static uint8_t read_only[256];
static uint8_t addr_pins = 0x2; /* ADDR_1 = 1, ADDR_0 = 0 -> 0x4A */
static uint8_t ptr;
static int pointer_pending; /* a register pointer was sent with no_stop and not yet read */
static double now_us;
static double power_us = -1.0;
static int lock_depth;
static model_stats_t stats;

void model_reset(void)
{
    (void)memset(regs, 0, sizeof regs);
    (void)memset(mapped, 0, sizeof mapped);
    (void)memset(read_only, 0, sizeof read_only);
    for (size_t i = 0; i < sizeof MAP / sizeof MAP[0]; i++) {
        regs[MAP[i].addr] = MAP[i].reset;
        mapped[MAP[i].addr] = 1;
        read_only[MAP[i].addr] = MAP[i].ro;
    }
    addr_pins = 0x2;
    pointer_pending = 0;
    lock_depth = 0;
    (void)memset(&stats, 0, sizeof stats);
}

void model_apply_power(void) { power_us = now_us; }
void model_remove_power(void) { power_us = -1.0; }
void model_set_addr_pins(uint8_t addr1, uint8_t addr0)
{
    addr_pins = (uint8_t)((addr1 ? 2U : 0U) | (addr0 ? 1U : 0U));
}
uint8_t model_reg(uint8_t addr) { return regs[addr]; }
double model_now_us(void) { return now_us; }
const model_stats_t *model_stats(void) { return &stats; }

static int acks(uint8_t address)
{
    if (power_us < 0.0 || now_us - power_us < 1500.0) {
        stats.nacks_booting += (power_us >= 0.0);
        return 0;
    }
    return address == (uint8_t)((regs[0x09] & 0x7CU) | addr_pins);
}

ret_code_t nrf_drv_twi_tx(nrf_drv_twi_t const *p_instance, uint8_t address, uint8_t const *p_data,
                          uint8_t length, bool no_stop)
{
    if (p_instance == NULL) {
        stats.null_instance++;
        return NRF_ERROR_DRV_TWI_ERR_ANACK;
    }
    if (lock_depth == 0) stats.unlocked_transfers++;
    if (pointer_pending) stats.split_reads++; /* another transfer between pointer write and read */
    pointer_pending = 0;
    if (!acks(address)) return NRF_ERROR_DRV_TWI_ERR_ANACK;
    ptr = p_data[0];
    if (length == 1U) {
        pointer_pending = no_stop ? 1 : 0;
        if (!no_stop) stats.stop_before_read++;
        return NRF_SUCCESS;
    }
    for (uint8_t i = 1; i < length; i++, ptr++) {
        if (!mapped[ptr] || read_only[ptr]) {
            stats.ignored_writes++;
        } else {
            regs[ptr] = p_data[i];
        }
        stats.writes++;
    }
    return NRF_SUCCESS;
}

ret_code_t nrf_drv_twi_rx(nrf_drv_twi_t const *p_instance, uint8_t address, uint8_t *p_data,
                          uint8_t length)
{
    if (p_instance == NULL) {
        stats.null_instance++;
        return NRF_ERROR_DRV_TWI_ERR_ANACK;
    }
    if (lock_depth == 0) stats.unlocked_transfers++;
    pointer_pending = 0;
    if (!acks(address)) return NRF_ERROR_DRV_TWI_ERR_ANACK;
    for (uint8_t i = 0; i < length; i++, ptr++) {
        if (!mapped[ptr]) stats.unmapped_reads++;
        p_data[i] = regs[ptr];
    }
    return NRF_SUCCESS;
}

void nrf_gpio_cfg_output(uint32_t pin) { (void)pin; }
void nrf_gpio_pin_set(uint32_t pin)
{
    (void)pin;
    model_apply_power(); /* in the tests the GPIO drives a load switch on VDD */
}
void nrf_gpio_pin_clear(uint32_t pin)
{
    (void)pin;
    model_remove_power();
}

static int mutex_obj;
SemaphoreHandle_t xSemaphoreCreateMutex(void)
{
    stats.mutexes_created++;
    return &mutex_obj;
}
BaseType_t xSemaphoreTake(SemaphoreHandle_t mutex, TickType_t ticks)
{
    (void)ticks;
    if (mutex == NULL) return pdFALSE;
    lock_depth++;
    return pdTRUE;
}
BaseType_t xSemaphoreGive(SemaphoreHandle_t mutex)
{
    (void)mutex;
    lock_depth--;
    return pdTRUE;
}
void vTaskDelay(TickType_t ticks)
{
    if (ticks > 0U) now_us += (double)(ticks - 1U) * 1e6 / (double)configTICK_RATE_HZ;
}
BaseType_t xTaskCreate(void (*fn)(void *), const char *name, uint16_t stack, void *arg,
                       uint32_t prio, TaskHandle_t *handle)
{
    (void)fn;
    (void)name;
    (void)stack;
    (void)arg;
    (void)prio;
    (void)handle;
    return pdPASS;
}
void vTaskDelete(TaskHandle_t handle) { (void)handle; }
