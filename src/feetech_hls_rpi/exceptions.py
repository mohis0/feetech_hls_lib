"""Exception hierarchy for :mod:`feetech_hls_rpi`."""

from __future__ import annotations


class HLSMotorError(Exception):
    """Base exception for all library errors."""


class MissingDependencyError(HLSMotorError):
    """A required runtime dependency is missing."""


class NotConnectedError(HLSMotorError):
    """An operation was requested before the serial connection was opened."""


class CommunicationError(HLSMotorError):
    """The Feetech SDK reported a transmit/receive failure."""

    def __init__(self, operation: str, result: int, message: str) -> None:
        self.operation = operation
        self.result = result
        self.message = message
        super().__init__(f"{operation}: {message} (result={result})")


class ServoStatusError(HLSMotorError):
    """The servo replied, but its status/error byte was non-zero."""

    def __init__(self, operation: str, status: int, message: str) -> None:
        self.operation = operation
        self.status = status
        self.message = message
        super().__init__(f"{operation}: {message} (status=0x{status:02X})")


class EchoFilterError(HLSMotorError):
    """The continuous RX path did not echo the transmitted packet as expected."""


class ConfigurationError(HLSMotorError):
    """A requested register/configuration value is invalid or unsafe."""
