"""Raspberry Pi library for Feetech HLS servos with resistor-connected TX."""

from .constants import (
    BAUD_CODE_TO_RATE,
    RATE_TO_BAUD_CODE,
    BaudRate,
    OperatingMode,
    Register,
    ResponseLevel,
    StatusFlag,
)
from .encoder import IncrementalEncoder, movement_magnitude, wrapped_sdk_delta
from .exceptions import (
    CommunicationError,
    ConfigurationError,
    EchoFilterError,
    HLSMotorError,
    MissingDependencyError,
    NotConnectedError,
    ServoStatusError,
)
from .filters import ExponentialMovingAverage
from .models import (
    ConfigurationSnapshot,
    ControlTargets,
    EncoderSample,
    MotorInfo,
    MotorSnapshot,
    MotorStatus,
    RegisterValue,
)
from .motor import HLSMotor, decode_sign_magnitude, encode_sign_magnitude
from .transport import MuxPortHandler

__all__ = [
    "BAUD_CODE_TO_RATE",
    "RATE_TO_BAUD_CODE",
    "BaudRate",
    "CommunicationError",
    "ConfigurationError",
    "ConfigurationSnapshot",
    "ControlTargets",
    "EchoFilterError",
    "EncoderSample",
    "ExponentialMovingAverage",
    "HLSMotor",
    "HLSMotorError",
    "IncrementalEncoder",
    "MissingDependencyError",
    "MotorInfo",
    "MotorSnapshot",
    "MotorStatus",
    "MuxPortHandler",
    "NotConnectedError",
    "OperatingMode",
    "Register",
    "RegisterValue",
    "ResponseLevel",
    "ServoStatusError",
    "StatusFlag",
    "decode_sign_magnitude",
    "encode_sign_magnitude",
    "movement_magnitude",
    "wrapped_sdk_delta",
]

__version__ = "0.1.0"
