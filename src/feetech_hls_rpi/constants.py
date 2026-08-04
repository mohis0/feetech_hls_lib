"""Register addresses, units, modes, and conversion constants.

The core HLS addresses in this module match the HLS handler shipped by the
Vassar Feetech SDK.  A few extended EEPROM registers are shared by Feetech's
magnetic-encoder family and can vary with firmware; those entries are marked
``model_dependent=True`` in :data:`REGISTER_SPECS`.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import IntEnum, IntFlag
from typing import Dict, Optional


class BaudRate(IntEnum):
    """Supported HLS UART baud rates."""

    BPS_1_000_000 = 1_000_000
    BPS_500_000 = 500_000
    BPS_250_000 = 250_000
    BPS_128_000 = 128_000
    BPS_115_200 = 115_200
    BPS_76_800 = 76_800
    BPS_57_600 = 57_600
    BPS_38_400 = 38_400


BAUD_CODE_TO_RATE: Dict[int, int] = {
    0: 1_000_000,
    1: 500_000,
    2: 250_000,
    3: 128_000,
    4: 115_200,
    5: 76_800,
    6: 57_600,
    7: 38_400,
}
RATE_TO_BAUD_CODE: Dict[int, int] = {rate: code for code, rate in BAUD_CODE_TO_RATE.items()}


class OperatingMode(IntEnum):
    """HLS operating modes.

    ``STEP`` is present in the common magnetic-encoder control table but may
    be firmware/model dependent.  HLS3950M users normally use POSITION,
    SPEED, and TORQUE.
    """

    POSITION = 0
    SPEED = 1
    TORQUE = 2
    STEP = 3


class ResponseLevel(IntEnum):
    """Servo response policy stored at address 8."""

    READ_AND_PING_ONLY = 0
    ALL_INSTRUCTIONS = 1


class StatusFlag(IntFlag):
    """Bits used by the protocol status byte and status register."""

    NONE = 0
    VOLTAGE = 1 << 0
    ANGLE_SENSOR = 1 << 1
    OVERHEAT = 1 << 2
    OVERCURRENT = 1 << 3
    OVERLOAD = 1 << 5


class Register(IntEnum):
    """Known HLS / magnetic-encoder control-table addresses."""

    FIRMWARE_MAJOR = 0
    FIRMWARE_MINOR = 1
    MODEL_NUMBER = 3
    ID = 5
    BAUD_RATE = 6
    RETURN_DELAY = 7
    RESPONSE_LEVEL = 8
    MIN_ANGLE_LIMIT = 9
    MAX_ANGLE_LIMIT = 11
    MAX_TEMPERATURE_LIMIT = 13
    MAX_VOLTAGE_LIMIT = 14
    MIN_VOLTAGE_LIMIT = 15
    MAX_TORQUE = 16
    PHASE = 18
    UNLOAD_CONDITION = 19
    LED_ALARM_CONDITION = 20
    POSITION_P = 21
    POSITION_D = 22
    POSITION_I = 23
    MIN_STARTUP_FORCE = 24
    INTEGRAL_LIMIT = 25
    CW_DEAD_ZONE = 26
    CCW_DEAD_ZONE = 27
    PROTECTION_CURRENT = 28
    ANGULAR_RESOLUTION = 30
    POSITION_OFFSET = 31
    MODE = 33
    PROTECTION_TORQUE = 34
    PROTECTION_TIME = 35
    OVERLOAD_TORQUE = 36
    SPEED_P = 37
    OVERCURRENT_TIME = 38
    SPEED_I = 39

    TORQUE_ENABLE = 40
    ACCELERATION = 41
    GOAL_POSITION = 42
    GOAL_TORQUE = 44
    GOAL_SPEED = 46
    TORQUE_LIMIT = 48
    LOCK = 55

    PRESENT_POSITION = 56
    PRESENT_SPEED = 58
    PRESENT_LOAD = 60
    PRESENT_VOLTAGE = 62
    PRESENT_TEMPERATURE = 63
    REGISTERED_INSTRUCTION = 64
    STATUS = 65
    MOVING = 66
    PRESENT_CURRENT = 69


@dataclass(frozen=True)
class RegisterSpec:
    """Description of a register exposed by the library."""

    address: int
    length: int
    readable: bool
    writable: bool
    persistent: bool
    unit: Optional[str]
    scale: float
    description: str
    model_dependent: bool = False
    encoding: str = "unsigned"


REGISTER_SPECS: Dict[str, RegisterSpec] = {
    "firmware_major": RegisterSpec(0, 1, True, False, True, None, 1.0, "Firmware major version", True),
    "firmware_minor": RegisterSpec(1, 1, True, False, True, None, 1.0, "Firmware minor version", True),
    "model_number": RegisterSpec(3, 2, True, False, True, None, 1.0, "Servo model number"),
    "id": RegisterSpec(5, 1, True, True, True, None, 1.0, "Bus ID, valid range 0..253"),
    "baud_rate": RegisterSpec(6, 1, True, True, True, "baud code", 1.0, "Baud-rate code 0..7"),
    "return_delay": RegisterSpec(7, 1, True, True, True, "2 us", 2.0, "Response delay", True),
    "response_level": RegisterSpec(8, 1, True, True, True, None, 1.0, "Status response level", True),
    "min_angle_limit": RegisterSpec(9, 2, True, True, True, "counts", 1.0, "Minimum position limit"),
    "max_angle_limit": RegisterSpec(11, 2, True, True, True, "counts", 1.0, "Maximum position limit"),
    "max_temperature_limit": RegisterSpec(13, 1, True, True, True, "degC", 1.0, "Over-temperature threshold", True),
    "max_voltage_limit": RegisterSpec(14, 1, True, True, True, "V", 0.1, "Over-voltage threshold", True),
    "min_voltage_limit": RegisterSpec(15, 1, True, True, True, "V", 0.1, "Under-voltage threshold", True),
    "max_torque": RegisterSpec(16, 2, True, True, True, "fraction", 0.001, "Maximum torque setting", True),
    "phase": RegisterSpec(18, 1, True, True, True, None, 1.0, "Special phase/direction bit field", True),
    "unload_condition": RegisterSpec(19, 1, True, True, True, "bit field", 1.0, "Protection/unload condition bits", True),
    "led_alarm_condition": RegisterSpec(20, 1, True, True, True, "bit field", 1.0, "LED alarm condition bits", True),
    "position_p": RegisterSpec(21, 1, True, True, True, None, 1.0, "Position-loop proportional coefficient", True),
    "position_d": RegisterSpec(22, 1, True, True, True, None, 1.0, "Position-loop derivative coefficient", True),
    "position_i": RegisterSpec(23, 1, True, True, True, None, 1.0, "Position-loop integral coefficient", True),
    "min_startup_force": RegisterSpec(24, 1, True, True, True, "fraction", 0.001, "Minimum startup force", True),
    "integral_limit": RegisterSpec(25, 1, True, True, True, None, 1.0, "Integral limit", True),
    "cw_dead_zone": RegisterSpec(26, 1, True, True, True, "counts", 1.0, "Clockwise dead zone"),
    "ccw_dead_zone": RegisterSpec(27, 1, True, True, True, "counts", 1.0, "Counter-clockwise dead zone"),
    "protection_current": RegisterSpec(28, 2, True, True, True, "A", 0.0065, "Over-current threshold", True),
    "angular_resolution": RegisterSpec(30, 1, True, True, True, None, 1.0, "Angular-resolution multiplier", True),
    "position_offset": RegisterSpec(
        31,
        2,
        True,
        True,
        True,
        "counts",
        1.0,
        "Position correction/offset",
        False,
        "sign_magnitude_15",
    ),
    "mode": RegisterSpec(33, 1, True, True, True, None, 1.0, "Operating mode"),
    "protection_torque": RegisterSpec(34, 1, True, True, True, "fraction", 0.01, "Torque retained during overload protection", True),
    "protection_time": RegisterSpec(35, 1, True, True, True, "s", 0.01, "Overload protection delay", True),
    "overload_torque": RegisterSpec(36, 1, True, True, True, "fraction", 0.01, "Overload threshold", True),
    "speed_p": RegisterSpec(37, 1, True, True, True, None, 1.0, "Speed-loop proportional coefficient", True),
    "overcurrent_time": RegisterSpec(38, 1, True, True, True, "s", 0.01, "Over-current protection delay", True),
    "speed_i": RegisterSpec(39, 1, True, True, True, None, 0.1, "Speed-loop integral coefficient", True),
    "torque_enable": RegisterSpec(40, 1, True, True, False, None, 1.0, "Torque output enable"),
    "acceleration": RegisterSpec(41, 1, True, True, False, "raw", 1.0, "Goal acceleration"),
    "goal_position": RegisterSpec(
        42,
        2,
        True,
        True,
        False,
        "counts",
        1.0,
        "Goal position",
        False,
        "sign_magnitude_15",
    ),
    "goal_torque": RegisterSpec(
        44,
        2,
        True,
        True,
        False,
        "raw",
        1.0,
        "HLS goal torque/current target",
        False,
        "sign_magnitude_15",
    ),
    "goal_speed": RegisterSpec(
        46,
        2,
        True,
        True,
        False,
        "raw",
        1.0,
        "Goal speed",
        False,
        "sign_magnitude_15",
    ),
    "torque_limit": RegisterSpec(48, 2, True, True, False, "fraction", 0.001, "Runtime torque limit", True),
    "lock": RegisterSpec(55, 1, True, True, False, None, 1.0, "EEPROM write lock"),
    "present_position": RegisterSpec(
        56,
        2,
        True,
        False,
        False,
        "counts",
        1.0,
        "Present position",
        False,
        "sign_magnitude_15",
    ),
    "present_speed": RegisterSpec(
        58,
        2,
        True,
        False,
        False,
        "raw",
        1.0,
        "Present speed",
        False,
        "sign_magnitude_15",
    ),
    "present_load": RegisterSpec(
        60,
        2,
        True,
        False,
        False,
        "fraction",
        0.001,
        "Motor drive/load feedback",
        False,
        "sign_magnitude_15",
    ),
    "present_voltage": RegisterSpec(62, 1, True, False, False, "V", 0.1, "Input voltage"),
    "present_temperature": RegisterSpec(63, 1, True, False, False, "degC", 1.0, "Internal temperature"),
    "registered_instruction": RegisterSpec(64, 1, True, False, False, None, 1.0, "Pending REG_WRITE flag", True),
    "status": RegisterSpec(65, 1, True, False, False, "bit field", 1.0, "Current protection/status bits", True),
    "moving": RegisterSpec(66, 1, True, False, False, None, 1.0, "Moving flag"),
    "present_current": RegisterSpec(
        69,
        2,
        True,
        False,
        False,
        "A",
        0.0065,
        "Motor current; 6.5 mA/raw follows the common magnetic-family table",
        True,
    ),
}


# Conversions used by Feetech's HLS wheel example and magnetic table.
SPEED_RAW_TO_RPM = 0.732
ACCELERATION_RAW_TO_DEG_S2 = 8.7890625  # 100 counts/s^2 * 360 / 4096
CURRENT_RAW_TO_AMP = 0.0065
VOLTAGE_RAW_TO_VOLT = 0.1
POSITION_COUNTS_PER_REVOLUTION = 4096
POSITION_COUNT_TO_DEG = 360.0 / POSITION_COUNTS_PER_REVOLUTION

# The HLS3950M product page specifies a torque constant of 20.8 kg.cm/A.
# This permits a useful current-based estimate, but it is not a direct output
# shaft torque measurement and does not compensate for no-load current,
# gearbox losses, temperature, or controller dynamics.
HLS3950M_TORQUE_CONSTANT_KG_CM_PER_A = 20.8
KG_CM_TO_NM = 0.0980665

# ReadPos() returns a sign-magnitude value with bit 15 removed.  In the
# observed wheel-mode stream, each positive/negative magnitude cycle is 2^15.
ENCODER_SDK_MODULUS = 32_768
ENCODER_SDK_HALF_MODULUS = 16_384

BROADCAST_ID = 0xFE
MIN_SERVO_ID = 0
MAX_SERVO_ID = 253
