# Register Map Used by `feetech_hls_rpi`

This wrapper separates registers into two confidence groups:

1. **HLS SDK core**: addresses explicitly declared by `scservo_sdk/hls.py` and used by its HLS methods.
2. **Extended magnetic-family fields**: common Feetech magnetic-encoder EEPROM/status fields. These are useful on HLS firmware but should be treated as model/firmware dependent until verified against the exact motor's control table.

## HLS SDK core registers

| Name | Address | Bytes | Access | Meaning / conversion |
|---|---:|---:|---|---|
| Model number | 3 | 2 | R | Used by SDK `ping()` model read. |
| ID | 5 | 1 | R/W EEPROM | 0–253; 254 is broadcast. |
| Baud rate | 6 | 1 | R/W EEPROM | Code 0–7. |
| Minimum angle | 9 | 2 | R/W EEPROM | Position counts. |
| Maximum angle | 11 | 2 | R/W EEPROM | Position counts. |
| CW dead zone | 26 | 1 | R/W EEPROM | Counts. |
| CCW dead zone | 27 | 1 | R/W EEPROM | Counts. |
| Position offset | 31 | 2 | R/W EEPROM | Sign-magnitude position correction. |
| Mode | 33 | 1 | R/W EEPROM | 0 position, 1 speed, 2 torque/current; 3 open-loop PWM (firmware dependent). |
| Torque enable | 40 | 1 | R/W SRAM | 0 off, 1 on. |
| Acceleration | 41 | 1 | R/W SRAM | Raw acceleration. |
| Goal position | 42 | 2 | R/W SRAM | Sign-magnitude counts. |
| Goal torque | 44 | 2 | R/W SRAM | HLS raw target torque/current. |
| Goal speed | 46 | 2 | R/W SRAM | Sign-magnitude speed. |
| Lock | 55 | 1 | R/W SRAM | 0 unlock EEPROM persistence, 1 lock. |
| Present position | 56 | 2 | R | HLS sign-magnitude value. |
| Present speed | 58 | 2 | R | HLS sign-magnitude speed. |
| Present load | 60 | 2 | R | Drive/load feedback. |
| Present voltage | 62 | 1 | R | Raw × 0.1 V. |
| Present temperature | 63 | 1 | R | Degrees Celsius. |
| Moving | 66 | 1 | R | 1 moving, 0 stopped. |
| Present current | 69 | 2 | R | Raw × 6.5 mA using the common magnetic-family scale; verify exact firmware when precision matters. |

## Baud codes

| Code | Baud rate |
|---:|---:|
| 0 | 1,000,000 |
| 1 | 500,000 |
| 2 | 250,000 |
| 3 | 128,000 |
| 4 | 115,200 |
| 5 | 76,800 |
| 6 | 57,600 |
| 7 | 38,400 |

## Runtime status block

`HLSMotor.read_status()` reads address 56 with length 15:

| Offset from 56 | Address | Data |
|---:|---:|---|
| 0–1 | 56–57 | Position |
| 2–3 | 58–59 | Speed |
| 4–5 | 60–61 | Load |
| 6 | 62 | Voltage |
| 7 | 63 | Temperature |
| 8 | 64 | Registered-instruction flag |
| 9 | 65 | Status bit field |
| 10 | 66 | Moving flag |
| 11–12 | 67–68 | Current target position |
| 13–14 | 69–70 | Current |

## Status bits

| Bit | Value | Meaning |
|---:|---:|---|
| 0 | 1 | Voltage error |
| 1 | 2 | Angle-sensor error |
| 2 | 4 | Overheat |
| 3 | 8 | Overcurrent |


## Extended magnetic-family registers

The package exposes named entries for these because they are common in Feetech's magnetic-encoder table and useful on HLS-class devices. Verify them before writing:

| Name | Address | Bytes | Unit / notes |
|---|---:|---:|---|
| Secondary ID | 7 | 1 | Secondary write ID; model and firmware dependent. |
| Response level | 8 | 1 | 0 read/ping only, 1 all instructions. |
| Max temperature | 13 | 1 | °C. |
| Max voltage | 14 | 1 | 0.1 V. |
| Min voltage | 15 | 1 | 0.1 V. |
| Max torque | 16 | 2 | Raw × 0.001. |
| Phase | 18 | 1 | Special bit field; do not change casually. |
| Unload condition | 19 | 1 | Protection bit field. |
| LED alarm condition | 20 | 1 | Alarm bit field. |
| Position P/D/I | 21/22/23 | 1 each | Controller coefficients. |
| Minimum startup force | 24 | 1 | Raw × 0.001. |
| Integral limit | 25 | 1 | Controller-dependent. |
| Protection current | 28 | 2 | Raw × 6.5 mA. |
| Angular resolution | 30 | 1 | Multi-turn/resolution multiplier. |
| Current loop P / I | 34 / 35 | 1 each | Controller coefficients. |
| Undefined | 36 | 1 | Do not write. |
| Speed P | 37 | 1 | Speed-loop coefficient. |
| Overcurrent time | 38 | 1 | Raw × 10 ms. |
| Speed I | 39 | 1 | Raw coefficient. |
| Torque limit | 48 | 2 | Runtime limit on common magnetic firmware. |
| Registered instruction | 64 | 1 | REG_WRITE pending flag. |
| Status | 65 | 1 | Protection/status bit field. |

## Raw read example

```python
value = motor.read_register(address=69, length=2)
current_a = motor.read_current()  # signed 6.5 mA units
```

## Sign-magnitude decoding

Position, speed, current, goal position, goal speed, goal current, and position
offset use bit 15 as the sign flag. Load uses bit 10. Named-register reads expose
both values:

```python
value = motor.read_named_register("present_speed")
print(value.raw)      # original 16-bit register
print(value.decoded)  # signed sign-magnitude value
```

## Current-based HL-3915-C001 torque estimate

The HL-3915-C001 product specification gives a torque constant of
`9.3 kg.cm/A`. The library can therefore report:

```text
estimated torque magnitude = absolute measured current × 9.3 kg.cm/A
```

This is not a direct shaft-torque measurement. No-load current, gearbox loss,
temperature, controller dynamics, and mechanical efficiency affect the real
output torque.

## Raw write example

```python
with motor.eeprom_unlocked():
    motor.write_register(address=26, value=1, length=1)
```

Back up register values before any persistent change:

```python
snapshot = motor.dump_registers(skip_errors=True)
for name, value in snapshot.values.items():
    print(name, value.raw)
```

Disable torque before unlocking EEPROM for persistent changes. Confirm the exact model firmware table before writes.
