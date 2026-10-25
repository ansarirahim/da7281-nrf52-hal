# DA7281 HAL Architecture

## Overview

This document describes the architecture and design decisions for the DA7281 Haptic Driver HAL for the nRF52833 with FreeRTOS. Datasheet references are to the DA7281 datasheet, Revision 3.1 (10-Feb-2022).

## Design Principles

### 1. Thread Safety

Each TWI bus is a `da7281_bus_t` holding the nRF5 SDK driver instance and one FreeRTOS mutex. `da7281_bus_init()` creates the mutex. Call it from `main()` before `vTaskStartScheduler()`: if the mutex were created lazily inside the first driver call, two tasks making their first call together could each create one, and they would not exclude each other.

The mutex covers a whole transaction:
- a register read: the register pointer is written with no STOP, then the read starts with a repeated START;
- a read-modify-write (`da7281_update_bits`).

The device keeps a register pointer between transfers, so another task's write between the two halves of a read would make the read return a different register.

```
Task A                          Task B
  |                               |
  |-- Lock bus mutex              |
  |-- TX [reg] (no STOP)          |
  |                               |-- Lock bus mutex (waits)
  |-- Sr, RX [value]              |
  |-- Unlock                      |
  |                               |-- TX [reg, value]
  |                               |-- Unlock
```

The mutex is taken with a 100 ms timeout (`DA7281_ERR_TIMEOUT`). Interrupt handlers must not call the driver: post the request to a task.

### 2. Datasheet Compliance

- **Power-up:** cold boot to IDLE takes up to tON = 1.5 ms (Table 8).
  - `vTaskDelay(n)` returns after n−1 to n tick periods, because the call starts part-way through a tick. The driver therefore waits ceil(1.5 ms × tick rate) + 1 ticks: 3 ticks at the nRF5 SDK's 1024 Hz.
  - `pdMS_TO_TICKS(2)` would be 2 ticks, which can be as short as 0.98 ms.
- **No enable pin (Table 1):** power is switched externally, if at all, through an optional `pwr_pin`.
- **I²C:** standard (100 kHz) or fast mode (400 kHz).

### 3. Error Handling

Every function returns a `da7281_error_t` status code:

| Code | Meaning |
|------|---------|
| `DA7281_OK` | Success |
| `DA7281_ERR_INVALID_PARAM` | Invalid parameter (including values that do not fit a register) |
| `DA7281_ERR_I2C_COMM` | I²C failure, including no ACK (wrong address, no power) |
| `DA7281_ERR_TIMEOUT` | Bus mutex not obtained in time |
| `DA7281_ERR_NOT_INITIALIZED` | Device not powered/initialized |
| `DA7281_ERR_CHIP_ID` | CHIP_REV does not identify a DA7281 |
| `DA7281_ERR_POWER` | Power control not possible (no supply switch) |

### 4. Multi-Device Support

The two LSBs of the 7-bit I²C address follow the ADDR_1/ADDR_0 pins, so up to four devices can share a bus at 0x48–0x4B with the reset base address 0x4A (Section 5.9, Table 19). Each has its own `da7281_device_t` on a shared `da7281_bus_t`.

## Register Map Used (Table 20)

| Address | Register | Use |
|---------|----------|-----|
| 0x00 | CHIP_REV | RO; reset 0xCA (minor 0xC, major 0xA) |
| 0x0A/0x0B | FRQ_LRA_PER_H/L | LRA period, 15 bits (8 + 7) |
| 0x0C | ACTUATOR1 | ACTUATOR_NOMMAX |
| 0x0D | ACTUATOR2 | ACTUATOR_ABSMAX |
| 0x0E | ACTUATOR3 | [4:0] IMAX |
| 0x0F/0x10 | CALIB_V2I_H/L | V2I_FACTOR |
| 0x13 | TOP_CFG1 | ACTUATOR_TYPE, BEMF_SENSE_EN, FREQ_TRACK_EN, ACCELERATION_EN, RAPID_STOP_EN, AMP_PID_EN (reset 0x1E) |
| 0x22 | TOP_CTL1 | SEQ_START, STANDBY_EN, OPERATION_MODE[2:0] |
| 0x23 | TOP_CTL2 | OVERRIDE_VAL |

## Initialization Flow

```
da7281_bus_init()        (main, before the scheduler)
  - store the TWI instance, create the bus mutex
da7281_init_handle()
  - bus, I2C address, optional supply GPIO (left off)
da7281_initialize()      (in a task)
  1. Power on: supply GPIO high (if any), wait >= 1.5 ms
  2. Verify CHIP_REV (0x00): major revision 0xA
  3. OPERATION_MODE = 0 (Inactive)
  4. Configure LRA: LRA_PER, NOMMAX, ABSMAX, IMAX, V2I,
     TOP_CFG1 by read-modify-write (BEMF_SENSE_EN kept)
  5. STANDBY_EN = 0: return to IDLE after playback
```

## LRA Configuration Calculations

All conversions use integer arithmetic with rounding. They are checked by the host tests.

### Resonant Period (Tables 29, 30)

```
LRA_PER = 1 / (f_res x 1333.32 ns)          15 bits
FRQ_LRA_PER_H = LRA_PER >> 7,  FRQ_LRA_PER_L = LRA_PER & 0x7F
```
For a 170 Hz LRA: LRA_PER = 4412, giving 0x22 / 0x3C. The reset pair 0x21 / 0x4F (4303) corresponds to 174 Hz.

### Voltages (Tables 31, 32)

```
ACTUATOR_NOMMAX = V_nom / 23.4 mV,  ACTUATOR_ABSMAX = V_abs / 23.4 mV    (8 bits, max 5.967 V)
```
2.5 V gives 107 (0x6B); 3.5 V gives 150 (0x96).

### Current (Table 33)

```
IMAX = (I_max - 28.6 mA) / 7.2 mA, rounded down     (5 bits: at most 31 = 251.8 mA)
```
A higher rating is limited to 31, never masked into 5 bits.

### V2I Factor (Table 34)

```
V2I_FACTOR = Z x (IMAX + 4) / 1.6104       (Z in ohms, 16 bits)
```
For 6.75 Ω with IMAX = 31: 147.

## Operation Modes and States

`OPERATION_MODE` (TOP_CTL1[2:0]) selects the playback source:
- 0: Inactive
- 1: DRO
- 2: PWM
- 3: RTWM
- 4: ETWM

The device states are separate from the mode (Section 5.2.1):

```
 VDD on ──> BOOT (<= 1.5 ms) ──> IDLE (0.36 uA) <──────────────┐
                                   │  ▲                         │ fault
                         trigger   │  │ playback done,          │
                                   ▼  │ STANDBY_EN = 0          │
                                 DRIVE ───────────────────────────┘
                                   │  ▲
                 playback done,    │  │ trigger
                 STANDBY_EN = 1    ▼  │
                                 STANDBY (0.8 mA, 0.15 ms to output)
```

### DRO Mode (Direct Register Override)

Used for simple amplitude control:
1. Set `OVERRIDE_VAL` (0x00 to 0x7F; with ACCELERATION_EN = 1, 0x80–0xFF give no drive).
2. Set `OPERATION_MODE = 1`. The motor drives at the given fraction of full scale.
3. Set `OPERATION_MODE = 0` to stop.

## FreeRTOS Integration

### Task Structure

```c
void haptics_task(void *pvParameters)
{
    da7281_initialize(&dev);

    while (1) {
        xQueueReceive(haptic_queue, &event, portMAX_DELAY);

        da7281_set_override_value(&dev, event.amplitude);
        da7281_set_operation_mode(&dev, DA7281_OPERATION_MODE_DRO);
        vTaskDelay(pdMS_TO_TICKS(event.duration_ms));
        da7281_set_operation_mode(&dev, DA7281_OPERATION_MODE_INACTIVE);
    }
}
```

## Memory Usage

Measured with `arm-none-eabi-size` (GCC, Cortex-M4F, `-O3`):
- **Code:** `da7281.c` is 2268 bytes. `da7281_init.c` is 825 bytes when logging compiles to function calls; less with `NRF_LOG` disabled.
- **Data and bss:** no data or bss. All state lives in the caller's `da7281_bus_t` and `da7281_device_t`.

## Testing Strategy

### Host tests (`test/`, run in CI)

- The HAL runs against `da7281_model.c`, a register model built from the datasheet:
  - register map, reset values and read-only registers;
  - the address-pin rule;
  - the boot window, with delays taking their shortest possible time.
- The tests check:
  - the register values for known ratings;
  - read-modify-write preserving other bits;
  - mode and standby bits, and the self-test;
  - address selection and parameter validation;
  - that every transfer holds the bus mutex and that reads use a repeated START.
- They run at 1024, 1000 and 100 Hz ticks.

### Hardware Tests

- Self-test sequence
- Override mode at various amplitudes
- Measure the drive frequency on OUTP/OUTN against the LRA's resonance

## Future Enhancements

1. **Waveform Memory Support**
   - RTWM/ETWM sequence programming
   - Pattern sequencing
2. **Advanced Features**
   - nIRQ handling (IRQ_EVENT1 is write-1-to-clear)
   - Wideband and custom-waveform operation
3. **Power Management**
   - Use STANDBY_EN per application (latency vs current)

## References

- DA7281 Datasheet, Revision 3.1 (Renesas)
- nRF52833 Product Specification v1.5
- FreeRTOS Kernel Documentation
