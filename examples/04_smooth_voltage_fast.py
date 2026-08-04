#!/usr/bin/env python3
"""Read voltage at maximum practical rate and print the EMA every 0.5 s."""

import time

from feetech_hls_rpi import HLSMotor


PRINT_INTERVAL = 0.5

with HLSMotor(servo_id=1, baudrate=38_400, voltage_filter_alpha=0.05) as motor:
    last_print = time.monotonic()
    samples = 0
    samples_at_print = 0

    try:
        while True:
            raw_voltage = motor.read_voltage()
            filtered_voltage = motor.voltage_filter.update(raw_voltage)
            samples += 1
            now = time.monotonic()

            if now - last_print >= PRINT_INTERVAL:
                rate = (samples - samples_at_print) / (now - last_print)
                print(
                    f"raw={raw_voltage:4.1f} V  "
                    f"filtered={filtered_voltage:6.3f} V  "
                    f"rate={rate:6.1f} Hz"
                )
                last_print = now
                samples_at_print = samples
    except KeyboardInterrupt:
        pass
