#!/usr/bin/env python3
"""Run at a fixed speed and print a non-wrapping encoder every 0.2 s."""

import time

from feetech_hls_rpi import HLSMotor


DIRECTION = -1          # 1 or -1
SPEED_RAW = 60          # About 43.9 RPM according to the SDK example
ACCELERATION = 5
TORQUE = 200
PRINT_INTERVAL = 0.2


motor = HLSMotor(
    servo_id=1,
    port="/dev/ttyAMA0",
    baudrate=38_400,
    mux_gpio=23,
    tx_connected_level=1,
    tx_disconnected_level=0,
    stop_on_close=True,
)

try:
    motor.connect(ping=True)
    motor.reset_incremental_encoder()
    motor.set_wheel_speed(
        DIRECTION * SPEED_RAW,
        acceleration=ACCELERATION,
        torque=TORQUE,
    )

    last_print = time.monotonic()
    samples = 0
    samples_at_print = 0

    while True:
        sample = motor.read_encoder(include_speed=True, direction_policy="command")
        samples += 1
        now = time.monotonic()

        if now - last_print >= PRINT_INTERVAL:
            rate = (samples - samples_at_print) / (now - last_print)
            print(
                f"sdk={sample.sdk_position:7d}  "
                f"delta={sample.delta_counts:5d}  "
                f"continuous={sample.continuous_counts:10d}  "
                f"absolute={sample.absolute_counts:10d}  "
                f"speed={sample.speed_raw:6d}  "
                f"rpm={sample.speed_rpm:7.2f}  "
                f"rate={rate:6.1f} Hz"
            )
            last_print = now
            samples_at_print = samples

except KeyboardInterrupt:
    print("\nCtrl+C: stopping motor")
finally:
    # close() sends repeated zero-speed commands because stop_on_close=True.
    motor.close()
