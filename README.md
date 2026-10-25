# DA7281 Haptic Driver HAL for nRF52833 + FreeRTOS

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Hardware Abstraction Layer (HAL) for the Renesas (Dialog Semiconductor) DA7281 haptic driver IC, for the Nordic nRF52833 microcontroller (for example on the Qorvo DWM3001C module) with FreeRTOS.

## Features

- **Register-level driver checked against the datasheet**
  - Register addresses, fields and conversions from the DA7281 datasheet, Revision 3.1 (Table 20 and the register tables), with table references in `da7281.h`
  - No magic numbers in code
  - Power-up timing that respects tON (1.5 ms max) at any FreeRTOS tick rate

- **Thread-Safe I2C Communication**
  - One FreeRTOS mutex per TWI bus, created before the scheduler starts
  - The mutex is held across the register-pointer write and the read (repeated START) and across read-modify-write
  - Timeout handling
  - Multi-device support (up to four DA7281 per bus via the ADDR pins)

- **LRA/ERM Motor Support**
  - Resonant period, voltage, current and V2I factor computed from the actuator ratings (integer arithmetic, rounded)
  - Acceleration, rapid stop and frequency tracking; back-EMF sensing kept enabled

- **Operation Modes** (TOP_CTL1.OPERATION_MODE)
  - DRO (Direct Register Override) with OVERRIDE_VAL
  - PWM, RTWM and ETWM mode selection (waveform memory sequences are not implemented yet)

- **Tested without hardware**
  - Host tests against a register model of the datasheet, at 1024, 1000 and 100 Hz ticks
  - Cortex-M4F cross build, cppcheck, clang-format and Doxygen in CI

## Project Structure

```
da7281-nrf52-hal/
├── include/
│   └── da7281.h              # Main HAL header with register definitions
├── src/
│   ├── da7281.c              # Core HAL implementation
│   └── da7281_init.c         # Initialization routines
├── examples/
│   └── haptics_test_task.c   # FreeRTOS task example
├── test/
│   ├── test_da7281.c         # Host tests
│   ├── da7281_model.c/.h     # DA7281 register model (datasheet Rev 3.1)
│   └── stubs/                # nRF5 SDK / FreeRTOS API stand-ins for host builds
├── docs/
│   └── ARCHITECTURE.md       # Architecture documentation
├── .github/workflows/
│   └── build.yml             # CI pipeline
├── CMakeLists.txt            # Build configuration
├── Doxyfile                  # Doxygen configuration
└── README.md                 # This file
```

## Quick Start

### Prerequisites

- Nordic nRF5 SDK v17.1.0 or later
- FreeRTOS (included in nRF5 SDK)
- ARM GCC toolchain
- DWM3001C module or nRF52833 development board

### Hardware Setup

```
DA7281 Haptic Driver    nRF52833 (DWM3001C)
--------------------    --------------------
VDD, VDDIO  --------    3.3V (optionally through a GPIO-controlled load switch)
GND         --------    GND
SDA         --------    P0.26 (TWI SDA, with pull-up to VDDIO)
SCL         --------    P0.27 (TWI SCL, with pull-up to VDDIO)
nIRQ        --------    optional GPIO input (open drain, pull-up)
ADDR_1      --------    VDDIO  } I2C address 0x4A
ADDR_0      --------    GND    }
```

The DA7281 has **no enable pin** (datasheet Table 1: "OFF state via EN pin: No"). Between playbacks it rests in the IDLE state at about 0.36 µA. To switch it off completely, switch its supply and pass that GPIO as `pwr_pin`.

The ADDR pins replace the two least significant bits of the 7-bit base address 0x4A: `DA7281_I2C_ADDRESS(addr1, addr0)` gives 0x48, 0x49, 0x4A or 0x4B.

### Integration

1. **Add files to your project:**
   ```bash
   cp -r include/ src/ your_project/components/da7281/
   ```

2. **Update your Makefile or CMakeLists.txt:**
   ```cmake
   include_directories(components/da7281/include)
   add_sources(components/da7281/src/da7281.c)
   add_sources(components/da7281/src/da7281_init.c)
   ```

3. **Initialize in your main.c:**
   ```c
   #include "da7281.h"

   int main(void)
   {
       // Initialize and enable TWI0 (nrf_drv_twi_init + nrf_drv_twi_enable)
       twi_init();

       // Create the bus object (its mutex) and the haptics task, before the scheduler
       haptics_task_init();

       // Start scheduler
       vTaskStartScheduler();
   }
   ```

## Usage Examples

### Basic Initialization

```c
#include "da7281.h"

static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(0);
static da7281_bus_t twi_bus;
static da7281_device_t haptic_dev;

// Before vTaskStartScheduler():
da7281_bus_init(&twi_bus, &m_twi);
da7281_init_handle(&haptic_dev, &twi_bus,
                   DA7281_I2C_ADDRESS(1, 0),  // ADDR_1 high, ADDR_0 low -> 0x4A
                   DA7281_NO_PIN);            // VDD always on

// In a task (it waits for the 1.5 ms boot time):
da7281_initialize(&haptic_dev);
```

### Custom LRA Configuration

```c
// Initialize with custom parameters
da7281_initialize_custom(&haptic_dev,
                         2.5f,   // Nominal (rated) voltage
                         3.5f,   // Absolute maximum voltage
                         250,    // Max current (mA); the device limit tops out at 251.8 mA
                         6.75f,  // Impedance (ohms)
                         170);   // Resonant frequency (Hz)
```

### Override Mode (Direct Amplitude Control)

```c
// Set override value (0x00 to 0x7F, full scale = 0x7F)
da7281_set_override_value(&haptic_dev, 0x7F);

// Enter DRO mode
da7281_set_operation_mode(&haptic_dev, DA7281_OPERATION_MODE_DRO);

// Run for 100ms
vTaskDelay(pdMS_TO_TICKS(100));

// Return to inactive
da7281_set_operation_mode(&haptic_dev, DA7281_OPERATION_MODE_INACTIVE);
```

### Self-Test Sequence

```c
// Run 100ms self-test at maximum amplitude
da7281_run_self_test(&haptic_dev, 100);
```

## Architecture

### Power-Up Sequence

```
1. Apply VDD (switch it with pwr_pin, or it is always on)
2. Wait at least tON = 1.5 ms (datasheet Table 8): rounded up to whole FreeRTOS ticks + 1,
   because vTaskDelay(n) can return after only n-1 tick periods
3. Verify CHIP_REV (register 0x00, reset value 0xCA, major revision 0xA)
4. Set OPERATION_MODE = 0 (Inactive) in TOP_CTL1
5. Configure LRA parameters
6. STANDBY_EN = 0: return to IDLE (lowest power) after each playback
```

### Thread Safety

All I2C transactions on a bus are protected by that bus's FreeRTOS mutex, created once in `da7281_bus_init()`:

```c
xSemaphoreTake(bus->mutex, timeout)
// register pointer (no STOP), repeated START, read  -- or a read-modify-write
xSemaphoreGive(bus->mutex)
```

Do not call the driver from interrupt handlers: send the request to a task.

## API Reference

Generate the Doxygen documentation with `doxygen Doxyfile` (output in `docs/html/`).

### Core Functions

| Function | Description |
|----------|-------------|
| `da7281_bus_init()` | Initialise a TWI bus object (mutex) before the scheduler |
| `da7281_init_handle()` | Initialize device handle structure |
| `da7281_initialize()` | Complete initialization with defaults |
| `da7281_power_on()` | Power on device with timing |
| `da7281_configure_lra()` | Configure LRA parameters |
| `da7281_set_operation_mode()` | Set operation mode |
| `da7281_set_override_value()` | Set DRO amplitude |
| `da7281_set_standby()` | Choose STANDBY or IDLE after playback |
| `da7281_run_self_test()` | Run self-test sequence |
| `da7281_update_bits()` | Read-modify-write under one bus lock |

## Testing

### Host tests (no hardware)

```bash
cmake -S . -B build-host -DDA7281_HOST_TESTS=ON
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

The tests run the HAL against `test/da7281_model.c`, a register model built from the datasheet: register map, reset values, read-only registers, address pins and the boot time. They check the bytes written for known ratings (for example 170 Hz → FRQ_LRA_PER 0x22/0x3C, 2.5 V → ACTUATOR1 0x6B), that every transfer holds the bus mutex, that reads use a repeated START, and that nothing is sent during the boot window, at 1024, 1000 and 100 Hz ticks.

### Cross build

```bash
cmake -S . -B build-arm -DNRF5_SDK_PATH=/path/to/nRF5_SDK -DDA7281_SDK_CONFIG_DIR=/path/to/your/config
cmake --build build-arm
```

Without an SDK the library is compiled against the API stand-ins in `test/stubs` (a compile check).

### On hardware

Run the example task on a DWM3001CDK:

```bash
nrfjprog --program haptics_test.hex --chiperase --verify --reset
```

Expected RTT log output:
```
Haptics Task: Starting task execution
DA7281: Configuring LRA parameters
  - Nominal Max: 2500 mV
  - Resonant Freq: 170 Hz
DA7281: Starting initialization sequence
DA7281: Chip ID verified successfully
Haptics Task: Running self-test (100ms pulse)
```

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Author

**A. R. Ansari**
- Embedded Firmware Engineer
- Specialization: Nordic nRF52, FreeRTOS, Low-Level Drivers
- Email: ansarirahim1@gmail.com
- GitHub: [ansarirahim](https://github.com/ansarirahim)
- LinkedIn: [Abdul Raheem Ansari](https://www.linkedin.com/in/abdul-raheem-ansari-a6871320/)
- WhatsApp: +919024304883

## Contributing

Contributions welcome! Please follow:
- The style in `.clang-format`, and keep the host tests passing
- Doxygen comment style
- Clean commit messages

## References

- [DA7281 Datasheet](https://www.renesas.com/en/document/dst/da7281-datasheet) (Revision 3.1 used for this driver)
- [nRF52833 Product Specification](https://infocenter.nordicsemi.com/pdf/nRF52833_PS_v1.5.pdf)
- [DWM3001C Module Datasheet](https://www.qorvo.com/products/p/DWM3001C)
