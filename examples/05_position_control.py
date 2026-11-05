#!/usr/bin/env python3
"""Move the HL-3915-C001 between three single-turn positions."""

import time

from feetech_hls_rpi import HLSMotor


with HLSMotor(servo_id=1, baudrate=38_400, disable_torque_on_close=True) as motor:
    for degrees in (90.0, 180.0, 270.0):
        position = motor.move_to_degrees(
            degrees,
            speed=40,
            acceleration=5,
            torque=300,
        )
        print(f"Commanded {degrees:.1f} degrees, raw target {position}")
        time.sleep(2.0)
