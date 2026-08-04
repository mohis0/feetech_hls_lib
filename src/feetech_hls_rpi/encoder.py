"""Continuous encoder unwrapping for HLS wheel mode."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .constants import (
    ENCODER_SDK_HALF_MODULUS,
    ENCODER_SDK_MODULUS,
    POSITION_COUNTS_PER_REVOLUTION,
)


@dataclass(frozen=True)
class EncoderUpdate:
    """Internal result produced by :class:`IncrementalEncoder`."""

    delta: int
    continuous_counts: int
    absolute_counts: int
    valid: bool
    glitch_reason: Optional[str]


def wrapped_sdk_delta(current: int, previous: int) -> int:
    """Return the smallest signed delta on the 32,768-count SDK cycle.

    This directly handles the observed reverse-wheel transition such as
    ``-31755 -> -4`` as approximately ``-1017`` counts rather than a large
    positive jump.
    """

    delta = current - previous
    if delta > ENCODER_SDK_HALF_MODULUS:
        delta -= ENCODER_SDK_MODULUS
    elif delta < -ENCODER_SDK_HALF_MODULUS:
        delta += ENCODER_SDK_MODULUS
    return delta


def movement_magnitude(current: int, previous: int) -> int:
    """Estimate traveled counts independent of direction.

    Feetech's sign-magnitude output can cross zero or a magnitude rollover in
    different forms.  At a sufficiently high sampling rate, the physically
    valid movement is the smallest path among the possible transitions.
    """

    current_abs = abs(current)
    previous_abs = abs(previous)

    candidates = {
        abs(current - previous),
        current_abs + previous_abs,
        (ENCODER_SDK_MODULUS - previous_abs) + current_abs,
        (ENCODER_SDK_MODULUS - current_abs) + previous_abs,
    }
    return min(value for value in candidates if value >= 0)


class IncrementalEncoder:
    """Maintain signed and absolute encoder counts without rollover.

    Python integers do not overflow, so ``continuous_counts`` can grow for the
    whole process lifetime.  For wheel mode, passing a direction hint of ``1``
    or ``-1`` makes the tracker monotonic in the commanded direction and avoids
    ambiguities caused by Feetech's sign-magnitude representation.
    """

    def __init__(
        self,
        *,
        counts_per_revolution: int = POSITION_COUNTS_PER_REVOLUTION,
        max_step_counts: Optional[int] = 8192,
    ) -> None:
        if counts_per_revolution <= 0:
            raise ValueError("counts_per_revolution must be positive")
        if max_step_counts is not None and max_step_counts <= 0:
            raise ValueError("max_step_counts must be positive or None")

        self.counts_per_revolution = counts_per_revolution
        self.max_step_counts = max_step_counts
        self.previous_position: Optional[int] = None
        self.continuous_counts = 0
        self.absolute_counts = 0

    def reset(self, *, continuous_counts: int = 0, absolute_counts: int = 0) -> None:
        """Reset software accumulation; the servo register is not modified."""

        self.previous_position = None
        self.continuous_counts = int(continuous_counts)
        self.absolute_counts = int(absolute_counts)

    def update(self, position: int, *, direction_hint: int = 0) -> EncoderUpdate:
        """Process one SDK position value.

        Args:
            position: Value returned by ``packet_handler.ReadPos`` or the
                position element of ``ReadPosSpeed``.
            direction_hint: ``1`` for expected increasing travel, ``-1`` for
                decreasing travel, or ``0`` to infer direction from adjacent
                samples.
        """

        if direction_hint not in (-1, 0, 1):
            raise ValueError("direction_hint must be -1, 0, or 1")

        if self.previous_position is None:
            self.previous_position = int(position)
            return EncoderUpdate(0, self.continuous_counts, self.absolute_counts, True, None)

        if direction_hint:
            magnitude = movement_magnitude(int(position), self.previous_position)
            delta = direction_hint * magnitude
        else:
            delta = wrapped_sdk_delta(int(position), self.previous_position)

        if self.max_step_counts is not None and abs(delta) > self.max_step_counts:
            reason = (
                f"rejected encoder step {delta}; limit is "
                f"{self.max_step_counts} counts"
            )
            return EncoderUpdate(0, self.continuous_counts, self.absolute_counts, False, reason)

        self.previous_position = int(position)
        self.continuous_counts += delta
        self.absolute_counts += abs(delta)
        return EncoderUpdate(delta, self.continuous_counts, self.absolute_counts, True, None)

    @property
    def continuous_revolutions(self) -> float:
        return self.continuous_counts / self.counts_per_revolution

    @property
    def absolute_revolutions(self) -> float:
        return self.absolute_counts / self.counts_per_revolution
