"""Small signal-processing helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


@dataclass
class ExponentialMovingAverage:
    """Exponential moving average.

    ``alpha=0.05`` implements the user's requested filter::

        filtered = previous * 0.95 + sample * 0.05
    """

    alpha: float = 0.05
    value: Optional[float] = None

    def __post_init__(self) -> None:
        if not 0.0 < self.alpha <= 1.0:
            raise ValueError("alpha must be in the range (0, 1]")

    def reset(self, value: Optional[float] = None) -> None:
        """Clear the filter or initialize it to ``value``."""

        self.value = value

    def update(self, sample: float) -> float:
        """Add one sample and return the filtered value."""

        if self.value is None:
            self.value = float(sample)
        else:
            self.value = self.value * (1.0 - self.alpha) + float(sample) * self.alpha
        return self.value
