#!/usr/bin/env python3
"""Read continuous encoder and all runtime feedback in one transaction."""

import time

from feetech_hls_rpi import HLSMotor


with HLSMotor(servo_id=1, baudrate=38_400) as motor:
    motor.reset_incremental_encoder()

    try:
        while True:
            snapshot = motor.read_snapshot(direction_policy="command")
            encoder = snapshot.encoder
            status = snapshot.status

            torque_text = (
                "n/a"
                if status.estimated_torque_kg_cm is None
                else f"{status.estimated_torque_kg_cm:5.2f} kg.cm"
            )

            print(
                f"continuous={encoder.continuous_counts:10d}  "
                f"pos={status.position_sdk:7d}  "
                f"speed={status.speed_raw:6d} ({status.speed_rpm:7.2f} rpm)  "
                f"voltage={status.voltage_v:4.1f} V  "
                f"current={status.current_a:5.3f} A  "
                f"torque_est={torque_text}  "
                f"temp={status.temperature_c:3d} C  "
                f"load={status.load_fraction:5.3f}  "
                f"moving={status.moving}  "
                f"flags={status.status_flags!s}"
            )
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
