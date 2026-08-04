#!/usr/bin/env python3
"""Print known register values; unsupported extended entries are skipped."""

from feetech_hls_rpi import HLSMotor


with HLSMotor(servo_id=1, baudrate=38_400) as motor:
    snapshot = motor.dump_registers(skip_errors=True)
    for name, item in snapshot.values.items():
        unit = f" {item.unit}" if item.unit else ""
        caution = " [model-dependent]" if item.model_dependent else ""
        print(
            f"{name:26s} addr={item.address:3d} "
            f"raw={item.raw:8d} decoded={item.decoded:8d} "
            f"scaled={item.scaled:g}{unit}{caution}"
        )
