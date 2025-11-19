# DA7281 Haptic Driver HAL for nRF52833 + FreeRTOS

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

Professional Hardware Abstraction Layer (HAL) for the Dialog Semiconductor DA7281 haptic driver IC, designed for Nordic nRF52833 microcontroller with FreeRTOS integration.

## Features

- **Complete DA7281 Register-Level Driver**
  - All configuration registers defined as macros
  - No magic numbers in code
  - Datasheet-compliant timing sequences

- **Thread-Safe I2C Communication**
  - FreeRTOS mutex-protected TWI operations
  - Timeout handling
  - Multi-device support

- **LRA/ERM Motor Support**
  - Automatic resonant frequency configuration
  - V2I factor calculation
  - Acceleration and rapid stop control
  - Frequency tracking

- **Multiple Operation Modes**
  - DRO (Direct Register Override)
  - PWM mode
  - RTWM (Real-Time Waveform Memory)
  - ETWM (Extended Waveform Memory)

- **Professional Code Quality**
  - MISRA-C compliant style
  - Doxygen documentation
  - Clean commit history
  - CI/CD integration

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
├── docs/
│   └── architecture.md       # Architecture documentation
├── .github/
│   └── workflows/
│       └── build.yml         # CI/CD pipeline
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
VDD         --------    3.3V
GND         --------    GND
SDA         --------    P0.26 (TWI SDA)
SCL         --------    P0.27 (TWI SCL)
EN          --------    P0.28 (GPIO - configurable)
```

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
       // Initialize TWI/I2C
       twi_init();
       
       // Initialize FreeRTOS
       // ...
       
       // Start haptics task
       haptics_task_init();
       
       // Start scheduler
       vTaskStartScheduler();
   }
   ```

## Usage Examples

### Basic Initialization

```c
#include "da7281.h"

da7281_device_t haptic_dev;

// Initialize device handle
da7281_init_handle(&haptic_dev, 
                    DA7281_I2C_ADDRESS_DEFAULT,
                    0,    // TWI instance 0
                    28);  // Power GPIO pin

// Complete initialization with default LRA parameters
da7281_initialize(&haptic_dev);
```

### Custom LRA Configuration

```c
// Initialize with custom parameters
da7281_initialize_custom(&haptic_dev,
                          2.5f,   // Nominal max voltage (RMS)
                          3.5f,   // Absolute max voltage (peak)
                          350,    // Max current (mA)
                          6.75f,  // Impedance (ohms)
                          170);   // Resonant frequency (Hz)
```

### Override Mode (Direct Amplitude Control)

```c
// Set override value (0x00 to 0x7F)
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
1. Assert power GPIO (EN pin)
2. Wait 2ms (datasheet requirement: min 1.5ms)
3. Verify CHIP_REV register (0xBA expected)
4. Set MODE=0 (Inactive)
5. Configure LRA parameters
6. Enable STANDBY mode
```

### Thread Safety

All I2C operations are protected by a FreeRTOS mutex:

```c
xSemaphoreTake(g_i2c_mutex, timeout)
// ... I2C transaction ...
xSemaphoreGive(g_i2c_mutex)
```

## API Reference

See [Doxygen documentation](docs/html/index.html) for complete API reference.

### Core Functions

| Function | Description |
|----------|-------------|
| `da7281_init_handle()` | Initialize device handle structure |
| `da7281_initialize()` | Complete initialization with defaults |
| `da7281_power_on()` | Power on device with timing |
| `da7281_configure_lra()` | Configure LRA parameters |
| `da7281_set_operation_mode()` | Set operation mode |
| `da7281_set_override_value()` | Set DRO amplitude |
| `da7281_run_self_test()` | Run self-test sequence |

## Testing

Run the example task on DWM3001CDK:

```bash
cd examples/
# Flash to device
nrfjprog --program haptics_test.hex --chiperase --verify --reset
```

Expected RTT log output:
```
Haptics Task: Starting initialization sequence
DA7281: Powering on device (GPIO pin 28)
DA7281: Chip ID verified successfully
DA7281: Configuring LRA parameters
  - Nominal Max: 2.50 V RMS
  - Resonant Freq: 170 Hz
Haptics Task: Running self-test (100ms pulse)
```

## License

MIT License - see [LICENSE](LICENSE) file for details.

## Author

**A. R. Ansari**
- Embedded Firmware Engineer
- Specialization: Nordic nRF52, FreeRTOS, Low-Level Drivers
- Email: ansarirahim1@gmail.com
- GitHub: [@ansarirahim](https://github.com/ansarirahim)
- LinkedIn: [Abdul Raheem Ansari](https://www.linkedin.com/in/abdul-raheem-ansari-a6871320/)
- WhatsApp: +919024304883

## Contributing

Contributions welcome! Please follow:
- MISRA-C coding standards
- Doxygen comment style
- Clean commit messages

## References

- [DA7281 Datasheet](https://www.renesas.com/en/document/dst/da7281-datasheet)
- [nRF52833 Product Specification](https://infocenter.nordicsemi.com/pdf/nRF52833_PS_v1.5.pdf)
- [DWM3001C Module Datasheet](https://www.qorvo.com/products/p/DWM3001C)

