#!/usr/bin/env python3
"""Back up registers and demonstrate baud-rate recovery helpers."""

from feetech_hls_rpi import HLSMotor


with HLSMotor(servo_id=1, baudrate=38_400, stop_on_close=False) as motor:
    path = motor.backup_configuration("hls3915m_register_backup.json")
    print("Saved register backup to:", path)

    # Use this only when the ID is known but the baud rate is uncertain:
    # detected = motor.detect_baudrate()
    # print("Detected baud rate:", detected)
