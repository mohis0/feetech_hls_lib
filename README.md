# Feetech HLS Raspberry Pi Library

A focused Python wrapper for a Feetech HLS-series serial servo—developed around the **HLS3950M**—connected to a Raspberry Pi UART with this half-duplex circuit:

- Raspberry Pi **RX is connected continuously** to the servo DATA line.
- Raspberry Pi **TX is connected or disconnected** through a CD4051 analog multiplexer.
- Because RX always monitors the bus, every transmitted packet is received as a local echo. The library removes that echo before the Feetech SDK parses the servo response.

The package builds on the `scservo_sdk` bundled by Vassar Robotics' Feetech SDK and adds the Raspberry Pi/CD4051 transport, safe cleanup, continuous encoder unwrapping, typed status data, register helpers, configuration methods, filtering, and examples.

## Features

- CD4051-controlled half-duplex TTL transport for Raspberry Pi 5.
- Continuous RX with automatic TX-echo removal.
- Wheel/speed mode, position mode, torque enable/disable, stop-on-close, and constant-current/torque target helpers.
- Non-wrapping software encoder values:
  - signed `continuous_counts` that can grow for the life of the process;
  - `absolute_counts` that always increases with traveled distance.
- Position and speed in one read transaction.
- A coherent status-block read containing position, speed, load, voltage, temperature, moving flag, error flags, and current.
- `read_snapshot()` returns that status block together with the updated non-wrapping encoder from the same UART transaction.
- Current-based HLS3950M torque estimates in kg.cm and N.m, clearly separated from direct measurements.
- Exponential voltage smoothing, including the requested `old * 0.95 + new * 0.05` filter.
- Raw and named register reads/writes.
- Persistent configuration helpers for ID, baud rate, response level, return delay, angle limits, voltage/temperature limits, dead zones, and PID values.
- JSON configuration backups plus ID scanning and baud-rate recovery helpers.
- Context-manager cleanup and repeated zero-speed packets when the program exits or receives `Ctrl+C`.

## Hardware wiring

### Servo power

Do not power the HLS3950M from the Raspberry Pi.

```text
External regulated 9–12.6 V  ---- Servo V+
External supply GND           ---- Servo GND
Raspberry Pi GND              ---- Servo GND / supply GND / CD4051 GND
```

Use a supply with enough transient current. The HLS3950M product specification lists a 2.4 A stall current at 12 V, so a supply with sensible margin is recommended.

### UART and CD4051

This package assumes the working wiring established during testing:

```text
Raspberry Pi GPIO15 / RX ------------------------ Servo DATA

Raspberry Pi GPIO14 / TX ---- CD4051 selected channel ----+
                                                           |
CD4051 COM ------------------------------------------------+---- Servo DATA

CD4051 unused channel ------------------------------------------ not connected
Raspberry Pi GPIO23 ---------------- CD4051 A/select
```

For the current circuit:

```text
GPIO23 HIGH -> TX connected to servo DATA
GPIO23 LOW  -> TX disconnected from servo DATA
```

The levels are configurable in `HLSMotor` if your CD4051 channels are reversed.

A typical CD4051BE DIP assignment is:

| CD4051BE pin | Function | Connection |
|---:|---|---|
| 16 | VDD | 3.3 V |
| 8 | VSS | GND |
| 7 | VEE | GND |
| 6 | INH | GND |
| 11 | A | Raspberry Pi GPIO23 |
| 10 | B | GND |
| 9 | C | GND |
| 3 | COM | Servo DATA |
| one selected channel | TX path | Raspberry Pi GPIO14/TX |
| one alternate channel | TX-off path | Leave unconnected |

RX does **not** pass through the multiplexer. It remains connected to DATA at all times.

## Raspberry Pi UART

The code defaults to:

```text
UART device: /dev/ttyAMA0
Baud rate:   38400
Servo ID:    1
Mux GPIO:    23
```

On the tested Raspberry Pi 5 setup, GPIO14 and GPIO15 were mapped to `TXD0` and `RXD0`, while `/dev/serial0` pointed elsewhere. Use `/dev/ttyAMA0` when that is the device attached to GPIO14/15.

Your user should belong to `dialout`:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in after changing group membership.

## Installation

Inside your virtual environment:

```bash
pip install vassar-feetech-servo-sdk pyserial lgpio
```

Install this wrapper in editable mode:

```bash
cd feetech_hls_rpi_library
pip install -e .
```

Confirm imports:

```bash
python3 -c "from feetech_hls_rpi import HLSMotor; print('library ready')"
```

## Quick start

```python
from feetech_hls_rpi import HLSMotor

with HLSMotor(
    servo_id=1,
    port="/dev/ttyAMA0",
    baudrate=38_400,
    mux_gpio=23,
    tx_connected_level=1,
    tx_disconnected_level=0,
) as motor:
    print("Model:", motor.ping())
    print("Voltage:", motor.read_voltage(), "V")
    print("Current:", motor.read_current(), "A")
    print("Estimated torque:", motor.estimate_torque_kg_cm(), "kg.cm")
```

The context manager calls `close()` on exit. With the default `stop_on_close=True`, `close()` sends repeated zero-speed commands before it releases the UART and GPIO.

## Fixed wheel speed with continuous encoder

```python
import time
from feetech_hls_rpi import HLSMotor

DIRECTION = -1
SPEED = 60

motor = HLSMotor(servo_id=1, baudrate=38_400)

try:
    motor.connect(ping=True)
    motor.reset_incremental_encoder()
    motor.set_wheel_speed(
        DIRECTION * SPEED,
        acceleration=5,
        torque=200,
    )

    last_print = time.monotonic()

    while True:
        sample = motor.read_encoder(
            include_speed=True,
            direction_policy="command",
        )

        if time.monotonic() - last_print >= 0.2:
            print(
                "position:", sample.continuous_counts,
                "distance:", sample.absolute_counts,
                "speed:", sample.speed_raw,
                "rpm:", sample.speed_rpm,
            )
            last_print = time.monotonic()

except KeyboardInterrupt:
    print("Stopping")
finally:
    motor.close()
```

### Encoder fields

`read_encoder()` returns an `EncoderSample`.

| Field | Meaning |
|---|---|
| `sdk_position` | The sign-magnitude value returned by the Feetech SDK. It can visibly wrap. |
| `delta_counts` | Corrected movement since the previous valid sample. |
| `continuous_counts` | Signed software position. It does not wrap during the process lifetime. |
| `absolute_counts` | Total traveled distance; always increases. |
| `continuous_revolutions` | `continuous_counts / counts_per_revolution`. |
| `absolute_revolutions` | `absolute_counts / counts_per_revolution`. |
| `speed_raw` | Signed present-speed register decoded by the SDK. |
| `speed_rpm` | Approximate RPM using the SDK example's `0.732 RPM/raw unit`. |
| `valid` | False if a step was rejected as an implausible jump. |
| `glitch_reason` | Explanation when `valid` is false. |

### Why the SDK position appears to jump

`scservo_sdk.hls.ReadPos()` reads a 16-bit register and uses bit 15 as a sign/direction bit. In wheel mode the observed stream can contain transitions such as:

```text
349 -> -660
-31755 -> -4
32745 -> -15
```

A direct subtraction can interpret those transitions as enormous jumps. The library tracks the 32,768-count sign-magnitude cycle and, in `direction_policy="command"`, uses the sign of the last wheel-speed command to choose a monotonic movement direction. Python integers then accumulate the corrected deltas without overflow.

The default `encoder_max_step_counts=8192` rejects implausibly large single-sample changes instead of adding them to the total. At the measured ~125 Hz read rate and the HLS3950M's physical speed, a valid step should be far below this limit.

Available direction policies:

- `"command"`: use the sign of the last `set_wheel_speed()` command. Best for a powered wheel that should remain monotonic.
- `"measured"`: use the sign of the present-speed feedback.
- `"auto"`: infer direction only from neighboring position values.

Reset only the software counter with:

```python
motor.reset_incremental_encoder()
```

This does not change any servo register.

## Wheel control

### Raw speed

```python
motor.set_wheel_speed(
    -60,
    acceleration=5,
    torque=200,
)
```

- Positive and negative values select opposite directions.
- The official HLS SDK wheel example uses approximately `0.732 RPM` per raw speed unit.
- A raw command near `100` is already around the HLS3950M's rated no-load speed.
- Very large commands such as `1000` saturate the controller and can increase whine, heat, vibration, and gearbox noise. The library emits a warning when the requested RPM is far above `rated_no_load_rpm`.

### RPM helper

```python
raw_command = motor.set_wheel_rpm(
    -40.0,
    acceleration=5,
    torque=200,
)
print("raw command:", raw_command)
```

### Stop and release torque

```python
motor.stop()
```

To stop and make the output shaft free afterward:

```python
motor.stop(disable_torque=True)
```

The safest application structure is always:

```python
motor = HLSMotor(...)
try:
    motor.connect()
    # operate motor
finally:
    motor.close()
```

`Ctrl+C` raises `KeyboardInterrupt`, but `finally` still runs and stops the motor.

## Position control

```python
with HLSMotor(servo_id=1, baudrate=38_400) as motor:
    motor.move_to_degrees(
        180.0,
        speed=40,
        acceleration=5,
        torque=300,
    )
```

Raw position control is also available:

```python
motor.move_to_position(2048, speed=40, acceleration=5, torque=300)
```

## Full motor status

`read_status()` reads 15 bytes beginning at address 56 in one transaction:

```python
status = motor.read_status()

print(status.position_sdk)
print(status.speed_raw, status.speed_rpm)
print(status.voltage_v)
print(status.current_a)
print(status.estimated_torque_kg_cm, status.estimated_torque_nm)
print(status.temperature_c)
print(status.load_raw, status.load_fraction)
print(status.moving)
print(status.status_flags)
```

### Status units

| Value | Conversion used |
|---|---|
| Voltage | raw × 0.1 V |
| Current | raw × 6.5 mA |
| Speed | raw × 0.732 RPM, following the HLS SDK example |
| Load | absolute decoded raw × 0.001 |
| Position | 4096 counts per mechanical revolution for single-turn conversion |

`present_load` is the controller's drive/load feedback, described in the magnetic-family control table as motor-drive duty feedback. It is **not a calibrated torque sensor**. The library therefore exposes raw load, scaled load fraction, current, and goal torque separately instead of claiming a physical N·m torque measurement.

The HLS3950M product specification lists a motor torque constant of `20.8 kg.cm/A`. By default, the library multiplies measured current by that value and exposes the result as `estimated_torque_kg_cm` and `estimated_torque_nm`. This remains an estimate: it includes no-load current and does not compensate for gearbox losses, temperature, controller transients, or mechanical efficiency. Disable it with:

```python
motor = HLSMotor(torque_constant_kg_cm_per_a=None)
```

The `6.5 mA/raw` current conversion follows Feetech's common magnetic-encoder control table. Check the exact firmware table if precision current measurement matters.

### Encoder plus complete status in one call

```python
snapshot = motor.read_snapshot(direction_policy="command")

print(snapshot.encoder.continuous_counts)
print(snapshot.encoder.speed_rpm)
print(snapshot.status.voltage_v)
print(snapshot.status.current_a)
print(snapshot.status.temperature_c)
print(snapshot.status.load_fraction)
```

This is the preferred high-rate call when the application needs encoder, speed, voltage, current, load, temperature, moving state, and status flags together.

### Error/status flags

`StatusFlag` includes:

- `VOLTAGE`
- `ANGLE_SENSOR`
- `OVERHEAT`
- `OVERCURRENT`
- `OVERLOAD`

Both the packet response status and register 65 status are included in `MotorStatus`.

## Individual feedback methods

```python
position = motor.read_position()
speed_raw = motor.read_speed()
speed_rpm = motor.read_speed_rpm()
voltage = motor.read_voltage()
temperature = motor.read_temperature()
current_raw = motor.read_current_raw()
current_a = motor.read_current()
load_raw = motor.read_load_raw()
load_fraction = motor.read_load_fraction()
moving = motor.is_moving()
flags = motor.read_status_flags()
goal_position = motor.read_goal_position()
goal_speed = motor.read_goal_speed()
goal_torque = motor.read_goal_torque()
mode = motor.read_mode()
torque_enabled = motor.read_torque_enabled()
targets = motor.read_control_targets()
```

## Smooth voltage

The default filter implements:

```text
filtered = previous × 0.95 + new × 0.05
```

Use:

```python
filtered_voltage = motor.read_voltage_filtered()
```

Change alpha:

```python
filtered_voltage = motor.read_voltage_filtered(alpha=0.1)
```

Or use the helper directly:

```python
from feetech_hls_rpi import ExponentialMovingAverage

filt = ExponentialMovingAverage(alpha=0.05)
filtered = filt.update(new_voltage)
```

## Register access

### Raw access

```python
raw_voltage = motor.read_register(62, 1)
raw_current = motor.read_register(69, 2)

motor.write_register(40, 1)       # torque enable
motor.write_register(46, 60, length=2)
motor.write_register_bytes(41, bytes([5, 0, 0, 200, 0, 60, 0]))
```

### Named access

```python
value = motor.read_named_register("present_voltage")
print(value.raw, value.decoded, value.scaled, value.unit)
```

### Register dump

```python
snapshot = motor.dump_registers(skip_errors=True)
for name, value in snapshot.values.items():
    print(name, value.raw, value.decoded, value.scaled, value.unit)
```

The `skip_errors=True` default is intentional. Feetech's HLS SDK confirms the core HLS addresses, while several extended EEPROM entries are shared with the broader magnetic-encoder family and can be firmware/model dependent. Review `REGISTER_MAP.md` before writing extended parameters.

Back up readable registers before persistent changes:

```python
path = motor.backup_configuration("hls3950m_backup.json")
print(path)
```

The JSON file includes address, byte length, raw value, decoded value, scaled value, unit, encoding, and the model-dependent warning flag.

## Persistent configuration

EEPROM changes use the servo lock register:

```python
with motor.eeprom_unlocked():
    motor.write_register(address, value)
```

The context manager attempts to relock EEPROM even if the write fails.

### Change ID

Connect only one servo when changing IDs:

```python
motor.set_id(2, verify=True)
```

The ID write is sent without waiting for an acknowledgement because some firmware can apply the new ID before transmitting the response. The wrapper then uses the new ID to relock EEPROM and verify communication.

### Change baud rate

```python
motor.set_baudrate(115_200, verify=True)
```

Supported rates:

```text
1,000,000
500,000
250,000
128,000
115,200
76,800
57,600
38,400
```

The wrapper writes the servo's baud code, changes `/dev/ttyAMA0` to the same baud, relocks EEPROM, and optionally pings the servo.

When the ID is known but the baud is uncertain, use:

```python
detected = motor.detect_baudrate()
print("Detected:", detected)
```

`detect_baudrate()` tries the eight supported HLS rates and keeps the working rate by default. When both ID and baud are unknown, try `scan_ids()` at each likely baud.

### Change ID and baud

```python
motor.set_id_and_baudrate(2, 115_200, verify=True)
```

After persistent communication changes, a full servo power cycle is still a sensible validation step.

### Other configuration helpers

```python
motor.set_response_level(1)
motor.set_return_delay_us(100)
motor.set_angle_limits(0, 4095)
motor.set_voltage_limits(9.0, 12.6)
motor.set_temperature_limit(70)
motor.set_dead_zones(1, 1)
motor.set_position_pid(p=32, i=0, d=32)
motor.set_speed_pid(p=10, i=200)
```

PID, phase, protection, and other EEPROM fields can vary by model/firmware. Back up existing values before changing them. `set_phase_raw()` requires `confirm=True` because the phase byte controls several low-level behaviors.

The wrapper handles response level `0` correctly: READ/PING transactions still wait for replies, while ordinary WRITE commands are sent TX-only because that mode suppresses write acknowledgements. Response level `1` uses normal write/status exchanges.

## Main API

### Lifecycle

- `connect(ping=False)`
- `close()`
- context manager: `with HLSMotor(...) as motor:`

### Identity

- `ping()`
- `read_info()`
- `scan_ids(start=0, end=253)`

### Control

- `enable_torque()`
- `disable_torque()`
- `set_mode(mode, persistent=False)`
- `set_wheel_mode(persistent=False)`
- `set_wheel_speed(speed, acceleration=5, torque=200)`
- `set_wheel_rpm(rpm, ...)`
- `stop(disable_torque=False, retries=3)`
- `move_to_position(...)`
- `move_to_degrees(...)`
- `set_goal_torque_raw(...)`
- `set_goal_torque_fraction(fraction, ...)`

### Encoder

- `read_encoder(include_speed=True, direction_policy="command")`
- `iter_encoder(rate_hz=None)`
- `reset_incremental_encoder()`

### Feedback

- `read_status()`
- `read_snapshot(direction_policy="command")`
- `read_control_targets()`
- `read_position()`
- `read_speed()` / `read_speed_rpm()`
- `read_voltage()` / `read_voltage_filtered()`
- `read_temperature()`
- `read_current_raw()` / `read_current()`
- `estimate_torque_kg_cm()` / `estimate_torque_nm()`
- `read_load_raw()` / `read_load_fraction()`
- `read_goal_position()` / `read_goal_speed()` / `read_goal_torque()` / `read_goal_torque_fraction()`
- `read_mode()` / `read_torque_enabled()`
- `is_moving()`
- `read_status_flags()`

### Registers/configuration

- `read_register_bytes(address, length)`
- `read_register(address, length=1)`
- `write_register_bytes(address, data, expect_reply=None)`
- `write_register(address, value, length=1, expect_reply=None)`
- `read_named_register(name)`
- `dump_registers(...)`
- `backup_configuration(path, ...)`
- `unlock_eeprom()` / `lock_eeprom()`
- `eeprom_unlocked()`
- `set_id()`
- `set_baudrate()`
- `detect_baudrate()`
- `set_id_and_baudrate()`
- `set_response_level()`
- `set_return_delay_us()`
- `set_angle_limits()`
- `set_voltage_limits()`
- `set_temperature_limit()`
- `set_dead_zones()`
- `set_position_pid()`
- `set_speed_pid()`

## Examples

| File | Purpose |
|---|---|
| `examples/01_ping_and_info.py` | Ping and read identity/configuration. |
| `examples/02_fixed_wheel_incremental_encoder.py` | Fixed wheel speed, non-wrapping encoder, safe Ctrl+C stop. |
| `examples/03_monitor_full_status.py` | Position, speed, load, voltage, temperature, current, and flags. |
| `examples/04_smooth_voltage_fast.py` | Maximum-rate voltage reading with 0.05 EMA. |
| `examples/05_position_control.py` | Single-turn position control. |
| `examples/06_change_id_or_baud.py` | Persistent communication configuration. |
| `examples/07_dump_registers.py` | Named register dump. |
| `examples/08_backup_and_recover.py` | JSON backup and baud-recovery helper. |

Run an example after installation:

```bash
python3 examples/02_fixed_wheel_incremental_encoder.py
```

## Performance notes

- The library does not sleep inside `read_encoder()` or `read_status()`.
- Printing is normally the largest application-level bottleneck. Read every sample but print only every 0.2–0.5 seconds.
- At 38,400 baud, the tested encoder loop reached about 125 successful reads/s.
- Higher baud rates can improve throughput, but 38,400 baud provides generous timing margin for Linux GPIO/UART handling.
- `read_status()` collects all runtime feedback in one request and is more efficient than separate reads when every parameter is needed.
- `read_snapshot()` adds continuous encoder tracking to that same status request without another servo transaction.

## Noise reduction

For quieter operation, begin with conservative wheel settings:

```python
motor.set_wheel_speed(
    40,
    acceleration=3,
    torque=150,
)
```

Then increase gradually. Commands around raw `100` are already near the HLS3950M's rated no-load RPM. Very high raw commands can saturate the speed loop.

Other software measures:

- use low acceleration;
- reduce torque when the load permits;
- call `stop(disable_torque=True)` when holding torque is unnecessary;
- avoid changing PID registers until you have recorded the factory values.

Mechanical gearbox noise, mounting resonance, supply ripple, and load alignment cannot be removed entirely in software.

## Safety and limitations

- Lift the wheel or secure the mechanism during first tests.
- The HLS3950M can produce high torque even at low command values.
- Always use a common ground between Pi, CD4051, servo, and external supply.
- Confirm that servo DATA remains within the Raspberry Pi's 3.3 V GPIO range.
- Do not put multiple servos on the bus when changing IDs with uncertain existing IDs.
- Persistent register writes can make communication difficult to recover. Keep a record of the current ID, baud rate, and configuration dump.
- `present_load` and goal torque are controller-domain values, not a calibrated physical torque measurement.
- The extended magnetic-family register map can differ by firmware. Core addresses used by the HLS SDK are the most reliable subset.

## Tests

Pure conversion and encoder tests run without hardware:

```bash
pip install -e ".[dev]"
pytest
```

The tests include the observed boundary transitions:

```text
349 -> -660
-31755 -> -4
32745 -> -15
```

## Sources

- Vassar Feetech Servo SDK: https://github.com/vassar-robotics/feetech-servo-sdk
- HLS handler/register constants: https://raw.githubusercontent.com/vassar-robotics/feetech-servo-sdk/main/scservo_sdk/hls.py
- Protocol packet handler: https://raw.githubusercontent.com/vassar-robotics/feetech-servo-sdk/main/scservo_sdk/protocol_packet_handler.py
- Official magnetic-encoder protocol manual: https://www.feetechrc.com/Data/feetechrc/upload/file/20240702/%E8%88%B5%E6%9C%BA%E5%8D%8F%E8%AE%AE%E6%89%8B%E5%86%8C-%E7%A3%81%E7%BC%96%E7%A0%81%E7%89%88%E6%9C%AC.pdf
- HLS3950M product page/specification: https://www.feetechrc.com/563788.html
