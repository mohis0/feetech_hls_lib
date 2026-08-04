#!/usr/bin/env python3
"""Open the bus, ping ID 1, and print basic configuration."""

from feetech_hls_rpi import HLSMotor


with HLSMotor(
    servo_id=1,
    port="/dev/ttyAMA0",
    baudrate=38_400,
    mux_gpio=23,
    tx_connected_level=1,
    tx_disconnected_level=0,
) as motor:
    print("Model number:", motor.ping())
    print(motor.read_info())
