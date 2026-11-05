#!/usr/bin/env python3
"""Examples of persistent ID and baud changes.

Run only one requested change at a time and keep exactly one servo on the bus.
"""

from feetech_hls_rpi import HLSMotor


with HLSMotor(servo_id=1, baudrate=38_400, stop_on_close=False) as motor:
    # Before any persistent change, explicitly disable torque.
    # WARNING: this releases the motor; ensure the mechanism is supported.
    # motor.disable_torque()

    # Example ID change:
    # motor.set_id(2, verify=True)

    # Example baud change:
    # motor.set_baudrate(115_200, verify=True)

    # Example combined change:
    # motor.set_id_and_baudrate(2, 115_200, verify=True)

    print("No configuration was changed. Uncomment exactly one example line.")
