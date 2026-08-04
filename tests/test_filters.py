import pytest

from feetech_hls_rpi.filters import ExponentialMovingAverage


def test_requested_voltage_filter():
    filt = ExponentialMovingAverage(alpha=0.05)
    assert filt.update(12.0) == pytest.approx(12.0)
    assert filt.update(10.0) == pytest.approx(12.0 * 0.95 + 10.0 * 0.05)
