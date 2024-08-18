# DA7281 HAL Architecture

## Overview

This document describes the architecture and design decisions for the DA7281 Haptic Driver HAL implementation for nRF52833 with FreeRTOS.

## Design Principles

### 1. Thread Safety
All I2C operations are protected by FreeRTOS mutexes to ensure safe concurrent access from multiple tasks.

```
Task A                  Task B
  |                       |
  |-- Lock Mutex          |
  |-- I2C Write           |
  |-- Unlock Mutex        |
  |                       |-- Lock Mutex (waits)
  |                       |-- I2C Read
  |                       |-- Unlock Mutex
```

### 2. Datasheet Compliance
All timing requirements from the DA7281 datasheet are strictly followed:
- Power-up delay: 2ms (datasheet min: 1.5ms)
- Standby transition: 1ms
- Register access timing via I2C standard mode (100kHz) or fast mode (400kHz)

### 3. Error Handling
Every function returns a `da7281_error_t` status code:
- `DA7281_OK`: Success
- `DA7281_ERR_INVALID_PARAM`: Invalid parameter
- `DA7281_ERR_I2C_COMM`: I2C communication failure
- `DA7281_ERR_TIMEOUT`: Operation timeout
- `DA7281_ERR_NOT_INITIALIZED`: Device not initialized
- `DA7281_ERR_CHIP_ID`: Chip ID mismatch
- `DA7281_ERR_POWER`: Power control error

### 4. Multi-Device Support
The HAL supports multiple DA7281 devices on the same I2C bus through the `da7281_device_t` handle structure.

## Module Structure

### Core Files

#### `da7281.h`
- Register address definitions
- Register bit field definitions
- Public API declarations
- Type definitions

#### `da7281.c`
- Core HAL implementation
- I2C communication layer
- Register read/write functions
- Operation mode control
- Override mode control

#### `da7281_init.c`
- Initialization sequences
- Default configuration
- Custom configuration support

## Initialization Flow

```
┌─────────────────────────────────────┐
│ da7281_init_handle()                │
│ - Clear device structure            │
│ - Set I2C address                   │
│ - Configure power GPIO              │
└──────────────┬──────────────────────┘
               │
               ▼
┌─────────────────────────────────────┐
│ da7281_initialize()                 │
│ ┌─────────────────────────────────┐ │
│ │ 1. Power On (GPIO high)         │ │
│ │    Wait 2ms                     │ │
│ └─────────────────────────────────┘ │
│ ┌─────────────────────────────────┐ │
│ │ 2. Verify Chip ID               │ │
│ │    Read CHIP_REV (expect 0xBA)  │ │
│ └─────────────────────────────────┘ │
│ ┌─────────────────────────────────┐ │
│ │ 3. Set Inactive Mode            │ │
│ │    MODE = 0                     │ │
│ └─────────────────────────────────┘ │
│ ┌─────────────────────────────────┐ │
│ │ 4. Configure LRA                │ │
│ │    - Calculate LRA_PER          │ │
│ │    - Calculate V2I_FACTOR       │ │
│ │    - Set ACTUATOR params        │ │
│ │    - Enable features            │ │
│ └─────────────────────────────────┘ │
│ ┌─────────────────────────────────┐ │
│ │ 5. Enable Standby               │ │
│ │    STANDBY_EN = 1               │ │
│ └─────────────────────────────────┘ │
└─────────────────────────────────────┘
```

## LRA Configuration Calculations

### Resonant Period
```c
LRA_PER = 1 / (f_res * 1.024e-6)
```
For 170Hz LRA:
```
LRA_PER = 1 / (170 * 1.024e-6) = 5747
```

### V2I Factor
```c
V2I_FACTOR = impedance_ohm * 1.5
```
For 6.75Ω LRA:
```
V2I_FACTOR = 6.75 * 1.5 = 10.125 ≈ 10
```

### Actuator Values
- ACTUATOR_NOMMAX: `nom_max_v * 10` (2.5V → 25)
- ACTUATOR_ABSMAX: `abs_max_v * 10` (3.5V → 35)
- ACTUATOR_IMAX: `imax_ma / 10` (350mA → 35)

## Operation Modes

### Mode Transitions

```
┌──────────────┐
│   INACTIVE   │ ◄─── Power On / Reset
│   (MODE=0)   │
└──────┬───────┘
       │
       ├──────────────────┐
       │                  │
       ▼                  ▼
┌──────────────┐   ┌──────────────┐
│     DRO      │   │     PWM      │
│   (MODE=1)   │   │   (MODE=2)   │
└──────────────┘   └──────────────┘
       │                  │
       └──────────┬───────┘
                  │
                  ▼
           ┌──────────────┐
           │   STANDBY    │
           │ (STANDBY_EN) │
           └──────────────┘
```

### DRO Mode (Direct Register Override)
Used for simple amplitude control:
1. Set `OVERRIDE_VAL` (0x00 to 0x7F)
2. Set `MODE = 1`
3. Motor drives at specified amplitude
4. Set `MODE = 0` to stop

## FreeRTOS Integration

### Task Structure
```c
void haptics_task(void *pvParameters)
{
    // Initialize device
    da7281_initialize(&dev);
    
    while (1) {
        // Wait for haptic event
        xQueueReceive(haptic_queue, &event, portMAX_DELAY);
        
        // Execute haptic pattern
        da7281_set_override_value(&dev, event.amplitude);
        da7281_set_operation_mode(&dev, DA7281_OPERATION_MODE_DRO);
        vTaskDelay(pdMS_TO_TICKS(event.duration_ms));
        da7281_set_operation_mode(&dev, DA7281_OPERATION_MODE_INACTIVE);
    }
}
```

### Mutex Protection
```c
static SemaphoreHandle_t g_i2c_mutex;

// In I2C write function:
xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(100));
nrf_drv_twi_tx(...);
xSemaphoreGive(g_i2c_mutex);
```

## Memory Usage

### Static Memory
- Device handle: ~16 bytes per device
- I2C mutex: ~80 bytes (FreeRTOS)

### Stack Usage
- Typical function call: ~64 bytes
- I2C transaction buffer: 32 bytes max

### Code Size
- Core HAL: ~2KB
- Initialization: ~1KB
- Total: ~3KB

## Performance

### I2C Transaction Times
- Single register write: ~100μs @ 400kHz
- Single register read: ~150μs @ 400kHz
- Full LRA configuration: ~1.5ms

### Power Consumption
- Standby mode: <10μA
- Active (DRO mode): Depends on LRA current (typ. 100-350mA)

## Testing Strategy

### Unit Tests
- Register read/write operations
- Parameter validation
- Error handling

### Integration Tests
- Full initialization sequence
- Mode transitions
- Multi-device operation

### Hardware Tests
- Self-test sequence
- Override mode at various amplitudes
- Long-duration operation

## Future Enhancements

1. **Waveform Memory Support**
   - RTWM mode implementation
   - ETWM mode implementation
   - Pattern sequencing

2. **Advanced Features**
   - IRQ handling
   - Auto-calibration
   - Thermal monitoring

3. **Power Management**
   - Dynamic power gating
   - Sleep mode integration

## References

- DA7281 Datasheet Rev 1.0
- nRF52833 Product Specification v1.5
- FreeRTOS Kernel Documentation

