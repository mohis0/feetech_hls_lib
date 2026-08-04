"""Public data models returned by the HLS motor wrapper."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional, Union

from .constants import StatusFlag


@dataclass(frozen=True)
class MotorInfo:
    """Identity and basic configuration information."""

    servo_id: int
    model_number: int
    baudrate: int
    baud_code: int
    operating_mode: int
    torque_enabled: bool
    firmware_major: Optional[int] = None
    firmware_minor: Optional[int] = None


@dataclass(frozen=True)
class EncoderSample:
    """One encoder sample with continuous software unwrapping."""

    timestamp: float
    sdk_position: int
    delta_counts: int
    continuous_counts: int
    absolute_counts: int
    continuous_revolutions: float
    absolute_revolutions: float
    speed_raw: Optional[int]
    speed_rpm: Optional[float]
    direction_hint: int
    valid: bool = True
    glitch_reason: Optional[str] = None
    servo_status: StatusFlag = StatusFlag.NONE


@dataclass(frozen=True)
class MotorStatus:
    """Coherent status block read from addresses 56 through 70."""

    timestamp: float
    position_raw_register: int
    position_sdk: int
    position_single_turn_deg: float
    speed_raw: int
    speed_rpm: float
    load_raw: int
    load_signed: int
    load_fraction: float
    voltage_v: float
    temperature_c: int
    registered_instruction: bool
    status_byte: int
    status_flags: StatusFlag
    moving: bool
    current_raw: int
    current_a: float
    estimated_torque_kg_cm: Optional[float]
    estimated_torque_nm: Optional[float]
    response_status_byte: int
    response_status_flags: StatusFlag


@dataclass(frozen=True)
class ControlTargets:
    """Current control-mode targets from addresses 40 through 49."""

    torque_enabled: bool
    acceleration_raw: int
    goal_position_register: int
    goal_position: int
    goal_torque_register: int
    goal_torque: int
    goal_speed_register: int
    goal_speed: int
    goal_speed_rpm: float
    torque_limit_raw: int


@dataclass(frozen=True)
class MotorSnapshot:
    """One coherent runtime status read plus updated software encoder."""

    timestamp: float
    encoder: EncoderSample
    status: MotorStatus


@dataclass(frozen=True)
class RegisterValue:
    """One decoded register value."""

    name: str
    address: int
    length: int
    raw: int
    decoded: int
    scaled: float
    unit: Optional[str]
    description: str
    model_dependent: bool
    encoding: str


@dataclass(frozen=True)
class ConfigurationSnapshot:
    """Named register values captured from the servo."""

    values: Mapping[str, RegisterValue]

    def raw(self, name: str) -> int:
        """Return the raw integer for a named register."""

        return self.values[name].raw

    def scaled(self, name: str) -> float:
        """Return the scaled engineering value for a named register."""

        return self.values[name].scaled

    def decoded(self, name: str) -> int:
        """Return a register value after sign-magnitude decoding, if applicable."""

        return self.values[name].decoded

    def as_dict(self) -> Mapping[str, Mapping[str, Any]]:
        """Return a JSON-serializable register dictionary."""

        return {
            name: {
                "address": item.address,
                "length": item.length,
                "raw": item.raw,
                "decoded": item.decoded,
                "scaled": item.scaled,
                "unit": item.unit,
                "description": item.description,
                "model_dependent": item.model_dependent,
                "encoding": item.encoding,
            }
            for name, item in self.values.items()
        }

    def to_json(self, *, indent: int = 2) -> str:
        """Serialize this snapshot as JSON text."""

        return json.dumps(self.as_dict(), indent=indent, sort_keys=True)

    def save_json(self, path: Union[str, Path], *, indent: int = 2) -> Path:
        """Write the snapshot to ``path`` and return the resulting path."""

        output = Path(path)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(self.to_json(indent=indent) + "\n", encoding="utf-8")
        return output
