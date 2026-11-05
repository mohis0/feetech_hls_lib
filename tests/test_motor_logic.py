from __future__ import annotations

from pathlib import Path

import pytest

import feetech_hls_rpi.motor as motor_module
from feetech_hls_rpi.constants import Register, StatusFlag


class FakePort:
    def __init__(self, port, **kwargs):
        self.port = port
        self.baudrate = 1_000_000
        self.is_open = False
        self.shutdown_called = False

    def openPort(self):
        self.is_open = True
        return True

    def setBaudRate(self, baud):
        self.baudrate = baud
        self.is_open = True
        return True

    def shutdown(self):
        self.shutdown_called = True
        self.is_open = False


class FakePacket:
    def __init__(self, port):
        self.port = port
        self.memory = bytearray(256)
        self.memory[3:5] = (3950).to_bytes(2, "little")
        self.memory[5] = 1
        self.memory[6] = 7
        self.memory[8] = 1
        self.memory[33] = 1
        self.memory[40] = 1
        self.memory[41] = 5
        self.memory[42:44] = (2048).to_bytes(2, "little")
        self.memory[44:46] = (200).to_bytes(2, "little")
        self.memory[46:48] = (60).to_bytes(2, "little")
        self.memory[48:50] = (800).to_bytes(2, "little")
        self.memory[55] = 1
        self.position_stream = [32745, -15, -55]
        self.position_index = 0
        self.speed = 60
        self.last_write_spec = None
        self.write_tx_only_calls = 0

        # Runtime block 56..70.
        self._write_sign_magnitude(56, 100)
        self._write_sign_magnitude(58, 60)
        self.memory[60:62] = ((1 << 10) | 250).to_bytes(2, "little")
        self.memory[62] = 120
        self.memory[63] = 32
        self.memory[64] = 0
        self.memory[65] = int(StatusFlag.OVERCURRENT)
        self.memory[66] = 1
        self.memory[69:71] = (100).to_bytes(2, "little")

    def _write_sign_magnitude(self, address, value):
        encoded = motor_module.encode_sign_magnitude(value)
        self.memory[address : address + 2] = encoded.to_bytes(2, "little")

    def getTxRxResult(self, result):
        return f"result={result}"

    def getRxPacketError(self, error):
        return f"status={error}"

    def ping(self, servo_id):
        return 3950, 0, 0

    def readTxRx(self, servo_id, address, length):
        return list(self.memory[address : address + length]), 0, 0

    def writeTxRx(self, servo_id, address, length, data):
        self.memory[address : address + length] = bytes(data[:length])
        return 0, 0

    def writeTxOnly(self, servo_id, address, length, data):
        self.write_tx_only_calls += 1
        self.memory[address : address + length] = bytes(data[:length])
        return 0

    def read1ByteTxRx(self, servo_id, address):
        return self.memory[address], 0, 0

    def write1ByteTxRx(self, servo_id, address, value):
        self.memory[address] = value
        return 0, 0

    def ReadPos(self, servo_id):
        value = self.position_stream[min(self.position_index, len(self.position_stream) - 1)]
        self.position_index += 1
        return value, 0, 0

    def ReadSpeed(self, servo_id):
        return self.speed, 0, 0

    def ReadPosSpeed(self, servo_id):
        position, result, error = self.ReadPos(servo_id)
        return position, self.speed, result, error

    def WheelMode(self, servo_id):
        self.memory[33] = 1
        return 0, 0

    def WriteSpec(self, servo_id, speed, acceleration, torque):
        self.last_write_spec = (speed, acceleration, torque)
        self.speed = speed
        return 0, 0

    def WritePosEx(self, servo_id, position, speed, acceleration, torque):
        return 0, 0


@pytest.fixture
def fake_motor(monkeypatch):
    packets = []

    monkeypatch.setattr(motor_module, "require_hardware_dependencies", lambda: None)
    monkeypatch.setattr(motor_module, "MuxPortHandler", FakePort)

    def make_packet(port):
        packet = FakePacket(port)
        packets.append(packet)
        return packet

    monkeypatch.setattr(motor_module, "hls", make_packet)
    motor = motor_module.HLSMotor(stop_on_close=False)
    motor.connect()
    yield motor, packets[0]
    motor.close()


def test_named_register_decodes_sign_magnitude(fake_motor):
    motor, packet = fake_motor
    packet._write_sign_magnitude(Register.PRESENT_SPEED, -60)
    value = motor.read_named_register("present_speed")
    assert value.raw == 0x803C
    assert value.decoded == -60
    assert value.scaled == -60


def test_status_and_torque_estimate(fake_motor):
    motor, _ = fake_motor
    status = motor.read_status()
    assert status.speed_raw == 60
    assert status.load_signed == -250
    assert status.voltage_v == pytest.approx(12.0)
    assert status.current_a == pytest.approx(0.65)
    assert status.estimated_torque_kg_cm == pytest.approx(6.045)
    assert status.estimated_torque_nm == pytest.approx(6.045 * 0.0980665)
    assert StatusFlag.OVERCURRENT in status.status_flags


def test_control_targets(fake_motor):
    motor, _ = fake_motor
    targets = motor.read_control_targets()
    assert targets.torque_enabled
    assert targets.goal_position == 2048
    assert targets.goal_torque == 200
    assert targets.goal_speed == 60
    assert targets.goal_speed_rpm == pytest.approx(43.92)


def test_snapshot_encoder_does_not_jump(fake_motor):
    motor, packet = fake_motor
    # read_snapshot uses the 15-byte status block, so update that block between
    # calls to reproduce the user's observed boundary.
    motor._last_commanded_speed = 60

    packet._write_sign_magnitude(56, 32745)
    first = motor.read_snapshot()
    assert first.encoder.continuous_counts == 0

    packet._write_sign_magnitude(56, -15)
    second = motor.read_snapshot()
    assert second.encoder.delta_counts == 38
    assert second.encoder.continuous_counts == 38


def test_wheel_command_and_stop(fake_motor):
    motor, packet = fake_motor
    motor.set_wheel_speed(-60, acceleration=5, torque=200)
    assert packet.memory[41] == 5
    assert int.from_bytes(packet.memory[44:46], "little") == 200
    assert int.from_bytes(packet.memory[46:48], "little") == 0x803C
    assert motor.last_commanded_speed == -60
    motor.stop(retries=1)
    assert int.from_bytes(packet.memory[46:48], "little") == 0


def test_response_level_zero_uses_tx_only_for_writes(fake_motor):
    motor, packet = fake_motor
    packet.memory[8] = 0
    motor._response_level_cache = None
    motor.set_wheel_speed(60, acceleration=5, torque=200)
    assert packet.write_tx_only_calls >= 3  # mode, torque enable, command block


def test_configuration_snapshot_json(fake_motor, tmp_path: Path):
    motor, _ = fake_motor
    path = motor.backup_configuration(
        tmp_path / "backup.json",
        names=["id", "baud_rate", "present_voltage"],
    )
    text = path.read_text(encoding="utf-8")
    assert '"present_voltage"' in text
    assert '"decoded"' in text


def test_signed_current_and_load_registers(fake_motor):
    motor, packet = fake_motor
    packet.memory[69:71] = (0x8000 | 100).to_bytes(2, "little")
    assert motor.read_current() == pytest.approx(-0.65)
    assert motor.read_status().current_a == pytest.approx(-0.65)
    assert motor.read_named_register("present_current").decoded == -100
    assert motor.read_named_register("present_load").decoded == -250


def test_return_delay_cannot_write_secondary_id(fake_motor):
    motor, packet = fake_motor
    with pytest.raises(motor_module.ConfigurationError):
        motor.set_return_delay_us(100)
    assert packet.memory[7] == 0


def test_eeprom_unlock_requires_torque_off(fake_motor):
    motor, packet = fake_motor
    with pytest.raises(motor_module.ConfigurationError):
        motor.unlock_eeprom()
    assert packet.memory[55] == 1
    packet.memory[40] = 0
    motor.unlock_eeprom()
    assert packet.memory[55] == 0
