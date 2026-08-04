from feetech_hls_rpi.constants import BAUD_CODE_TO_RATE, RATE_TO_BAUD_CODE, StatusFlag
from feetech_hls_rpi.motor import decode_sign_magnitude, encode_sign_magnitude


def test_baud_mapping_round_trip():
    for code, rate in BAUD_CODE_TO_RATE.items():
        assert RATE_TO_BAUD_CODE[rate] == code


def test_sign_magnitude_round_trip():
    for value in (-32767, -100, -1, 0, 1, 100, 32767):
        assert decode_sign_magnitude(encode_sign_magnitude(value)) == value


def test_status_flags():
    flags = StatusFlag.VOLTAGE | StatusFlag.OVERHEAT
    assert StatusFlag.VOLTAGE in flags
    assert StatusFlag.OVERHEAT in flags
    assert StatusFlag.OVERLOAD not in flags
