/*
 * Host tests for the DA7281 HAL, run against the datasheet register model (da7281_model.c).
 * Expected register values are computed by hand from the DA7281 datasheet, Revision 3.1.
 */
#include <stdio.h>

#include "da7281.h"
#include "da7281_model.h"

static int failures;
static int checks;
#define CHECK(cond)                                                \
    do {                                                           \
        checks++;                                                  \
        if (!(cond)) {                                             \
            failures++;                                            \
            printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
        }                                                          \
    } while (0)
#define CHECK_EQ(a, b)                                                                             \
    do {                                                                                           \
        long long a_ = (long long)(a), b_ = (long long)(b);                                        \
        checks++;                                                                                  \
        if (a_ != b_) {                                                                            \
            failures++;                                                                            \
            printf("FAIL %s:%d: %s == 0x%llX, expected 0x%llX\n", __FILE__, __LINE__, #a, a_, b_); \
        }                                                                                          \
    } while (0)

static const nrf_drv_twi_t twi0 = NRF_DRV_TWI_INSTANCE(0);

/* VDD always on: the board applies power, then the driver starts. */
static void setup(da7281_bus_t *bus, da7281_device_t *dev, uint8_t addr)
{
    model_reset();
    CHECK_EQ(da7281_bus_init(bus, &twi0), DA7281_OK);
    CHECK_EQ(da7281_init_handle(dev, bus, addr, DA7281_NO_PIN), DA7281_OK);
    model_apply_power();
}

static void test_default_initialization(void)
{
    da7281_bus_t bus;
    da7281_device_t dev;
    const model_stats_t *s = model_stats();

    setup(&bus, &dev, DA7281_I2C_ADDRESS_DEFAULT);
    CHECK_EQ(da7281_initialize(&dev), DA7281_OK);
    CHECK(dev.is_initialized);

    /* 170 Hz: LRA_PER = 1 / (170 x 1333.32 ns) = 4412 = 0x22 x 128 + 0x3C (Tables 29, 30) */
    CHECK_EQ(model_reg(DA7281_REG_FRQ_LRA_PER_H), 0x22);
    CHECK_EQ(model_reg(DA7281_REG_FRQ_LRA_PER_L), 0x3C);
    /* 2.5 V / 23.4 mV = 107 = 0x6B; 3.5 V -> 150 = 0x96 (Tables 31, 32) */
    CHECK_EQ(model_reg(DA7281_REG_ACTUATOR1), 0x6B);
    CHECK_EQ(model_reg(DA7281_REG_ACTUATOR2), 0x96);
    /* 350 mA is above the 5-bit range: IMAX limited to 31 = 251.8 mA (Table 33) */
    CHECK_EQ(model_reg(DA7281_REG_ACTUATOR3), 31);
    /* V2I = 6.75 x (31 + 4) / 1.6104 = 146.7 -> 147 (Table 34) */
    CHECK_EQ(model_reg(DA7281_REG_CALIB_V2I_H), 0x00);
    CHECK_EQ(model_reg(DA7281_REG_CALIB_V2I_L), 147);
    /* LRA, BEMF sensing kept, tracking + acceleration + rapid stop: the reset value 0x1E */
    CHECK_EQ(model_reg(DA7281_REG_TOP_CFG1), 0x1E);
    /* Inactive, return to IDLE after playback */
    CHECK_EQ(model_reg(DA7281_REG_TOP_CTL1), 0x00);

    CHECK_EQ(s->ignored_writes, 0);
    CHECK_EQ(s->unmapped_reads, 0);
    CHECK_EQ(s->unlocked_transfers, 0);
    CHECK_EQ(s->split_reads, 0);
    CHECK_EQ(s->stop_before_read, 0);
    CHECK_EQ(s->nacks_booting, 0);
    CHECK_EQ(s->null_instance, 0);
    CHECK_EQ(s->mutexes_created, 1);
}

static void test_custom_ratings(void)
{
    da7281_bus_t bus;
    da7281_device_t dev;

    setup(&bus, &dev, DA7281_I2C_ADDRESS_DEFAULT);
    CHECK_EQ(da7281_initialize_custom(&dev, 2.5f, 3.5f, 250, 6.75f, 170), DA7281_OK);
    /* (250 - 28.6) / 7.2 = 30.75 -> 30 (rounded down: limit stays below the rating) */
    CHECK_EQ(model_reg(DA7281_REG_ACTUATOR3), 30);
    /* 6.75 x 34 / 1.6104 = 142.5 -> 143 */
    CHECK_EQ(model_reg(DA7281_REG_CALIB_V2I_L), 143);

    /* 235 Hz -> 1 / (235 x 1333.32 ns) = 3191.5 -> 3192 = 0x18 x 128 + 0x78 */
    CHECK_EQ(da7281_configure_lra(&dev, 2.0f, 3.0f, 100, 8.0f, 235), DA7281_OK);
    CHECK_EQ(model_reg(DA7281_REG_FRQ_LRA_PER_H), 0x18);
    CHECK_EQ(model_reg(DA7281_REG_FRQ_LRA_PER_L), 0x78);
    CHECK_EQ(model_reg(DA7281_REG_ACTUATOR1), 85);   /* 2000 / 23.4 = 85.47 -> 85 */
    CHECK_EQ(model_reg(DA7281_REG_ACTUATOR3), 9);    /* (100 - 28.6) / 7.2 = 9.9 -> 9 */
    CHECK_EQ(model_reg(DA7281_REG_CALIB_V2I_L), 65); /* 8 x 13 / 1.6104 = 64.6 -> 65 */
}

static void test_playback_controls(void)
{
    da7281_bus_t bus;
    da7281_device_t dev;
    uint8_t mode = 0xFF;

    setup(&bus, &dev, DA7281_I2C_ADDRESS_DEFAULT);
    CHECK_EQ(da7281_initialize(&dev), DA7281_OK);

    CHECK_EQ(da7281_set_override_value(&dev, 0x40), DA7281_OK);
    CHECK_EQ(model_reg(DA7281_REG_TOP_CTL2), 0x40); /* OVERRIDE_VAL is TOP_CTL2 (0x23) */
    CHECK_EQ(da7281_set_operation_mode(&dev, DA7281_OPERATION_MODE_DRO), DA7281_OK);
    CHECK_EQ(model_reg(DA7281_REG_TOP_CTL1) & 0x07, DA7281_OPERATION_MODE_DRO);
    CHECK_EQ(da7281_get_operation_mode(&dev, &mode), DA7281_OK);
    CHECK_EQ(mode, DA7281_OPERATION_MODE_DRO);

    /* STANDBY_EN changes only bit 3; the mode bits are kept */
    CHECK_EQ(da7281_set_standby(&dev, true), DA7281_OK);
    CHECK_EQ(model_reg(DA7281_REG_TOP_CTL1), 0x09);
    CHECK_EQ(da7281_set_standby(&dev, false), DA7281_OK);
    CHECK_EQ(model_reg(DA7281_REG_TOP_CTL1), 0x01);

    CHECK_EQ(da7281_run_self_test(&dev, 100), DA7281_OK);
    CHECK_EQ(model_reg(DA7281_REG_TOP_CTL1) & 0x07, DA7281_OPERATION_MODE_INACTIVE);
    CHECK_EQ(model_reg(DA7281_REG_TOP_CTL2), 0x7F);
    CHECK_EQ(model_stats()->ignored_writes, 0);
}

static void test_boot_time(void)
{
    da7281_bus_t bus;
    da7281_device_t dev;
    double t0;

    setup(&bus, &dev, DA7281_I2C_ADDRESS_DEFAULT);
    t0 = model_now_us();
    CHECK_EQ(da7281_power_on(&dev), DA7281_OK);
    /* Shortest possible wait, whatever the tick rate: at least tON = 1.5 ms */
    CHECK(model_now_us() - t0 >= 1500.0);
    CHECK_EQ(da7281_verify_chip_id(&dev), DA7281_OK);
    CHECK_EQ(model_stats()->nacks_booting, 0);
}

static void test_power_switch(void)
{
    da7281_bus_t bus;
    da7281_device_t dev;

    model_reset();
    CHECK_EQ(da7281_bus_init(&bus, &twi0), DA7281_OK);
    /* A GPIO drives a load switch on VDD; init_handle leaves it off */
    CHECK_EQ(da7281_init_handle(&dev, &bus, DA7281_I2C_ADDRESS_DEFAULT, 5), DA7281_OK);
    CHECK_EQ(da7281_initialize(&dev), DA7281_OK);
    CHECK_EQ(da7281_power_off(&dev), DA7281_OK);
    CHECK(!dev.is_powered);

    /* Without a supply switch, power_off cannot turn the DA7281 off (it has no EN pin) */
    setup(&bus, &dev, DA7281_I2C_ADDRESS_DEFAULT);
    CHECK_EQ(da7281_power_off(&dev), DA7281_ERR_POWER);
}

static void test_addressing(void)
{
    da7281_bus_t bus;
    da7281_device_t dev;

    CHECK_EQ(DA7281_I2C_ADDRESS(0, 0), 0x48);
    CHECK_EQ(DA7281_I2C_ADDRESS(0, 1), 0x49);
    CHECK_EQ(DA7281_I2C_ADDRESS(1, 0), 0x4A);
    CHECK_EQ(DA7281_I2C_ADDRESS(1, 1), 0x4B);

    /* Board with both ADDR pins high: the device answers at 0x4B, not 0x4A */
    setup(&bus, &dev, DA7281_I2C_ADDRESS_DEFAULT);
    model_set_addr_pins(1, 1);
    CHECK_EQ(da7281_initialize(&dev), DA7281_ERR_I2C_COMM);

    setup(&bus, &dev, DA7281_I2C_ADDRESS(1, 1));
    model_set_addr_pins(1, 1);
    CHECK_EQ(da7281_initialize(&dev), DA7281_OK);
}

static void test_parameter_checks(void)
{
    da7281_bus_t bus;
    da7281_bus_t uninitialised = {NULL, NULL};
    da7281_device_t dev;

    model_reset();
    CHECK_EQ(da7281_bus_init(&bus, NULL), DA7281_ERR_INVALID_PARAM);
    CHECK_EQ(da7281_init_handle(&dev, &uninitialised, 0x4A, DA7281_NO_PIN),
             DA7281_ERR_INVALID_PARAM);

    setup(&bus, &dev, DA7281_I2C_ADDRESS_DEFAULT);
    CHECK_EQ(da7281_init_handle(&dev, &bus, 0x80, DA7281_NO_PIN), DA7281_ERR_INVALID_PARAM);
    CHECK_EQ(da7281_init_handle(&dev, &bus, 0x4A, DA7281_NO_PIN), DA7281_OK);
    /* not powered yet */
    CHECK_EQ(da7281_set_operation_mode(&dev, DA7281_OPERATION_MODE_DRO), DA7281_ERR_INVALID_PARAM);
    CHECK_EQ(da7281_power_on(&dev), DA7281_OK);
    CHECK_EQ(da7281_set_operation_mode(&dev, 5), DA7281_ERR_INVALID_PARAM);
    CHECK_EQ(da7281_set_override_value(&dev, 0x80), DA7281_ERR_INVALID_PARAM);
    CHECK_EQ(da7281_configure_lra(&dev, 2.5f, 3.5f, 250, 6.75f, 49), DA7281_ERR_INVALID_PARAM);
    CHECK_EQ(da7281_configure_lra(&dev, 2.5f, 3.5f, 250, 6.75f, 301), DA7281_ERR_INVALID_PARAM);
    CHECK_EQ(da7281_configure_lra(&dev, 6.5f, 3.5f, 250, 6.75f, 170), DA7281_ERR_INVALID_PARAM);
    CHECK_EQ(da7281_configure_lra(&dev, 2.5f, 3.5f, 20, 6.75f, 170), DA7281_ERR_INVALID_PARAM);
    CHECK_EQ(da7281_run_self_test(&dev, 0), DA7281_ERR_INVALID_PARAM);
    CHECK_EQ(da7281_read_register(&dev, DA7281_REG_CHIP_REV, NULL), DA7281_ERR_INVALID_PARAM);
}

int main(void)
{
    test_default_initialization();
    test_custom_ratings();
    test_playback_controls();
    test_boot_time();
    test_power_switch();
    test_addressing();
    test_parameter_checks();
    printf("%d/%d checks passed (configTICK_RATE_HZ = %u)\n", checks - failures, checks,
           (unsigned)configTICK_RATE_HZ);
    return failures ? 1 : 0;
}
