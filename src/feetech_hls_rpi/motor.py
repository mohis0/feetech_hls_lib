"""High-level HLS servo controller for Raspberry Pi + CD4051."""

from __future__ import annotations

import contextlib
import math
import threading
import time
import warnings
from pathlib import Path
from typing import Dict, Iterable, Iterator, Mapping, Optional, Union

from ._compat import COMM_SUCCESS, hls, require_hardware_dependencies
from .constants import (
    BAUD_CODE_TO_RATE,
    CURRENT_RAW_TO_AMP,
    HLS3950M_TORQUE_CONSTANT_KG_CM_PER_A,
    KG_CM_TO_NM,
    MAX_SERVO_ID,
    MIN_SERVO_ID,
    OperatingMode,
    POSITION_COUNT_TO_DEG,
    POSITION_COUNTS_PER_REVOLUTION,
    RATE_TO_BAUD_CODE,
    REGISTER_SPECS,
    Register,
    ResponseLevel,
    SPEED_RAW_TO_RPM,
    StatusFlag,
    VOLTAGE_RAW_TO_VOLT,
)
from .encoder import IncrementalEncoder
from .exceptions import (
    CommunicationError,
    ConfigurationError,
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
from .transport import MuxPortHandler


def decode_sign_magnitude(value: int, sign_bit: int = 15) -> int:
    """Decode Feetech sign-magnitude data in the same way as the SDK."""

    sign_mask = 1 << sign_bit
    if value & sign_mask:
        return -(value & ~sign_mask)
    return value


def encode_sign_magnitude(value: int, sign_bit: int = 15) -> int:
    """Encode a signed integer using Feetech's direction/sign bit."""

    limit = (1 << sign_bit) - 1
    if not -limit <= value <= limit:
        raise ValueError(f"value must be in range {-limit}..{limit}")
    if value < 0:
        return (-value) | (1 << sign_bit)
    return value


def _word_le(low: int, high: int) -> int:
    return (low & 0xFF) | ((high & 0xFF) << 8)


def _sign(value: int) -> int:
    return 1 if value > 0 else -1 if value < 0 else 0


class HLSMotor:
    """Control and monitor one Feetech HLS servo.

    The class owns the UART, CD4051 GPIO, Feetech packet handler, incremental
    encoder state, and voltage filter.  It is safe to use as a context manager;
    on exit it sends stop packets before closing the interface.
    """

    def __init__(
        self,
        *,
        servo_id: int = 1,
        port: str = "/dev/ttyAMA0",
        baudrate: int = 38_400,
        mux_gpio: int = 23,
        gpio_chip: int = 0,
        tx_connected_level: int = 1,
        tx_disconnected_level: int = 0,
        packet_timeout_ms: float = 20.0,
        echo_timeout_ms: float = 20.0,
        strict_echo: bool = True,
        counts_per_revolution: int = POSITION_COUNTS_PER_REVOLUTION,
        encoder_max_step_counts: Optional[int] = 8192,
        voltage_filter_alpha: float = 0.05,
        stop_on_close: bool = True,
        disable_torque_on_close: bool = False,
        strict_servo_status: bool = False,
        rated_no_load_rpm: Optional[float] = 75.0,
        torque_constant_kg_cm_per_a: Optional[float] = (
            HLS3950M_TORQUE_CONSTANT_KG_CM_PER_A
        ),
    ) -> None:
        require_hardware_dependencies()
        self._validate_servo_id(servo_id)
        if baudrate not in RATE_TO_BAUD_CODE:
            raise ConfigurationError(
                f"Unsupported baud rate {baudrate}; choose one of "
                f"{sorted(RATE_TO_BAUD_CODE)}"
            )

        self.servo_id = int(servo_id)
        self.port = port
        self.baudrate = int(baudrate)
        self.stop_on_close = bool(stop_on_close)
        self.disable_torque_on_close = bool(disable_torque_on_close)
        self.strict_servo_status = bool(strict_servo_status)
        self.rated_no_load_rpm = rated_no_load_rpm
        if torque_constant_kg_cm_per_a is not None and torque_constant_kg_cm_per_a <= 0:
            raise ConfigurationError("torque_constant_kg_cm_per_a must be positive or None")
        self.torque_constant_kg_cm_per_a = torque_constant_kg_cm_per_a

        self.port_handler = MuxPortHandler(
            port,
            mux_gpio=mux_gpio,
            gpio_chip=gpio_chip,
            tx_connected_level=tx_connected_level,
            tx_disconnected_level=tx_disconnected_level,
            packet_timeout_ms=packet_timeout_ms,
            echo_timeout_ms=echo_timeout_ms,
            strict_echo=strict_echo,
        )
        self.packet_handler = hls(self.port_handler)
        self.encoder = IncrementalEncoder(
            counts_per_revolution=counts_per_revolution,
            max_step_counts=encoder_max_step_counts,
        )
        self.voltage_filter = ExponentialMovingAverage(alpha=voltage_filter_alpha)

        self._connected = False
        self._closed = False
        self._last_commanded_speed = 0
        self._last_servo_error = 0
        self._response_level_cache: Optional[ResponseLevel] = None
        self._lock = threading.RLock()

    # ------------------------------------------------------------------
    # Lifecycle and error handling
    # ------------------------------------------------------------------

    @staticmethod
    def _validate_servo_id(servo_id: int) -> None:
        if not MIN_SERVO_ID <= int(servo_id) <= MAX_SERVO_ID:
            raise ConfigurationError("servo_id must be in the range 0..253")

    def connect(self, *, ping: bool = False) -> "HLSMotor":
        """Open UART and optionally ping the configured servo."""

        with self._lock:
            if self._connected:
                return self
            self.port_handler.baudrate = self.baudrate
            if not self.port_handler.openPort():
                raise CommunicationError("open UART", -1, f"could not open {self.port}")
            self._connected = True
            if ping:
                self.ping()
            return self

    def close(self) -> None:
        """Stop the motor, optionally disable torque, and release resources."""

        with self._lock:
            if self._closed:
                return
            if self._connected and self.stop_on_close:
                try:
                    self.stop(
                        disable_torque=self.disable_torque_on_close,
                        retries=5,
                        ignore_errors=True,
                    )
                except Exception:
                    pass
            self.port_handler.shutdown()
            self._connected = False
            self._closed = True

    def __enter__(self) -> "HLSMotor":
        return self.connect()

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.close()

    def _require_connected(self) -> None:
        if not self._connected:
            raise NotConnectedError("Call connect() before using the motor")

    def _check_result(
        self,
        operation: str,
        result: int,
        servo_error: int,
        *,
        allow_status: bool = False,
    ) -> StatusFlag:
        if result != COMM_SUCCESS:
            message = self.packet_handler.getTxRxResult(result)
            raise CommunicationError(operation, result, message)
        self._last_servo_error = int(servo_error)
        flags = StatusFlag(int(servo_error))
        if servo_error and self.strict_servo_status and not allow_status:
            message = self.packet_handler.getRxPacketError(servo_error)
            raise ServoStatusError(operation, servo_error, message)
        return flags

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def last_commanded_speed(self) -> int:
        return self._last_commanded_speed

    @property
    def last_servo_status(self) -> StatusFlag:
        return StatusFlag(self._last_servo_error)

    # ------------------------------------------------------------------
    # Raw register access
    # ------------------------------------------------------------------

    def read_register_bytes(self, address: int, length: int) -> bytes:
        """Read an arbitrary register span and return raw bytes."""

        self._require_connected()
        if not 0 <= int(address) <= 253:
            raise ValueError("address must be in range 0..253")
        if not 1 <= int(length) <= 240:
            raise ValueError("length must be in range 1..240")

        with self._lock:
            data, result, error = self.packet_handler.readTxRx(
                self.servo_id, int(address), int(length)
            )
            self._check_result(f"read register {address}", result, error)
            if len(data) != length:
                raise CommunicationError(
                    f"read register {address}",
                    -7,
                    f"expected {length} bytes, received {len(data)}",
                )
            return bytes(data)

    def read_register(self, address: int, length: int = 1) -> int:
        """Read a little-endian register value as an unsigned integer."""

        return int.from_bytes(self.read_register_bytes(address, length), "little")

    def write_register_bytes(
        self,
        address: int,
        data: Union[bytes, bytearray, Iterable[int]],
        *,
        expect_reply: Optional[bool] = None,
    ) -> None:
        """Write a consecutive byte sequence beginning at ``address``.

        When ``expect_reply`` is ``None``, the library uses the cached/read
        response-level register: level 1 waits for a WRITE status packet,
        while level 0 sends TX-only because that mode replies only to READ and
        PING instructions.
        """

        self._require_connected()
        payload = bytes(data)
        if not payload:
            raise ValueError("data must contain at least one byte")
        if len(payload) > 240:
            raise ValueError("data must contain at most 240 bytes")
        if expect_reply is None:
            expect_reply = self._write_acknowledgements_enabled()

        with self._lock:
            if expect_reply:
                result, error = self.packet_handler.writeTxRx(
                    self.servo_id, int(address), len(payload), list(payload)
                )
                self._check_result(f"write register {address}", result, error)
            else:
                result = self.packet_handler.writeTxOnly(
                    self.servo_id, int(address), len(payload), list(payload)
                )
                self._check_result(f"write register {address}", result, 0)

    def write_register(
        self,
        address: int,
        value: int,
        *,
        length: int = 1,
        expect_reply: Optional[bool] = None,
    ) -> None:
        """Write a raw little-endian register value."""

        if not 0 <= int(value) < 1 << (8 * length):
            raise ValueError(f"value does not fit in {length} byte(s)")
        self.write_register_bytes(
            address,
            int(value).to_bytes(length, "little"),
            expect_reply=expect_reply,
        )

    def read_named_register(self, name: str) -> RegisterValue:
        """Read and decode one entry from :data:`REGISTER_SPECS`."""

        try:
            spec = REGISTER_SPECS[name]
        except KeyError as exc:
            raise KeyError(f"Unknown register name: {name}") from exc
        raw = self.read_register(spec.address, spec.length)
        if spec.encoding == "sign_magnitude_15":
            decoded = decode_sign_magnitude(raw, 15)
        elif spec.encoding == "unsigned":
            decoded = raw
        else:
            raise ConfigurationError(
                f"Unsupported register encoding {spec.encoding!r} for {name}"
            )
        return RegisterValue(
            name=name,
            address=spec.address,
            length=spec.length,
            raw=raw,
            decoded=decoded,
            scaled=decoded * spec.scale,
            unit=spec.unit,
            description=spec.description,
            model_dependent=spec.model_dependent,
            encoding=spec.encoding,
        )

    def dump_registers(
        self,
        names: Optional[Iterable[str]] = None,
        *,
        skip_errors: bool = True,
    ) -> ConfigurationSnapshot:
        """Read named registers into a snapshot.

        By default all known registers are attempted.  Extended magnetic-family
        registers can be firmware dependent, so ``skip_errors=True`` is useful.
        """

        selected = list(names) if names is not None else list(REGISTER_SPECS)
        values: Dict[str, RegisterValue] = {}
        for name in selected:
            try:
                values[name] = self.read_named_register(name)
            except (CommunicationError, KeyError):
                if not skip_errors:
                    raise
        return ConfigurationSnapshot(values)

    def backup_configuration(
        self,
        path: Union[str, Path],
        names: Optional[Iterable[str]] = None,
        *,
        skip_errors: bool = True,
    ) -> Path:
        """Save a named-register snapshot as JSON.

        This is intentionally a backup-only helper.  Restoring an arbitrary
        configuration automatically is unsafe because some extended register
        meanings vary by HLS firmware.
        """

        snapshot = self.dump_registers(names=names, skip_errors=skip_errors)
        return snapshot.save_json(path)

    # ------------------------------------------------------------------
    # Identity and general information
    # ------------------------------------------------------------------

    def ping(self) -> int:
        """Ping the servo and return its model number."""

        self._require_connected()
        with self._lock:
            model, result, error = self.packet_handler.ping(self.servo_id)
            self._check_result("ping", result, error)
            return int(model)

    def read_info(self) -> MotorInfo:
        """Read identity, baud code, mode, and torque state."""

        model = self.read_register(Register.MODEL_NUMBER, 2)
        servo_id = self.read_register(Register.ID, 1)
        baud_code = self.read_register(Register.BAUD_RATE, 1)
        mode = self.read_register(Register.MODE, 1)
        torque_enabled = bool(self.read_register(Register.TORQUE_ENABLE, 1))
        try:
            firmware_major = self.read_register(Register.FIRMWARE_MAJOR, 1)
            firmware_minor = self.read_register(Register.FIRMWARE_MINOR, 1)
        except CommunicationError:
            firmware_major = None
            firmware_minor = None

        return MotorInfo(
            servo_id=servo_id,
            model_number=model,
            baudrate=BAUD_CODE_TO_RATE.get(baud_code, self.baudrate),
            baud_code=baud_code,
            operating_mode=mode,
            torque_enabled=torque_enabled,
            firmware_major=firmware_major,
            firmware_minor=firmware_minor,
        )

    def scan_ids(self, start: int = 0, end: int = 253) -> Mapping[int, int]:
        """Ping an ID range and return ``{id: model_number}`` for replies."""

        self._require_connected()
        if not 0 <= start <= end <= 253:
            raise ValueError("scan range must satisfy 0 <= start <= end <= 253")

        found: Dict[int, int] = {}
        original_id = self.servo_id
        try:
            for candidate in range(start, end + 1):
                self.servo_id = candidate
                try:
                    found[candidate] = self.ping()
                except CommunicationError:
                    continue
        finally:
            self.servo_id = original_id
        return found

    def detect_baudrate(
        self,
        candidates: Optional[Iterable[int]] = None,
        *,
        keep_detected: bool = True,
    ) -> int:
        """Find the baud rate used by the configured servo ID.

        The UART must already be open.  Each supported baud is tried with a
        normal ping.  On success, the detected rate is returned.  By default
        the host UART and this object remain configured at that rate.

        This is a recovery helper for uncertain EEPROM changes; it cannot find
        a servo whose ID is also unknown.  Use :meth:`scan_ids` at each likely
        baud when both values are uncertain.
        """

        self._require_connected()
        rates = list(candidates) if candidates is not None else list(RATE_TO_BAUD_CODE)
        if not rates:
            raise ValueError("candidates must contain at least one baud rate")
        unsupported = [rate for rate in rates if int(rate) not in RATE_TO_BAUD_CODE]
        if unsupported:
            raise ConfigurationError(f"Unsupported baud-rate candidates: {unsupported}")

        original_baud = self.baudrate
        last_error: Optional[Exception] = None
        for rate in rates:
            rate = int(rate)
            if not self.port_handler.setBaudRate(rate):
                continue
            self.baudrate = rate
            try:
                self.ping()
            except CommunicationError as exc:
                last_error = exc
                continue

            if not keep_detected and rate != original_baud:
                if not self.port_handler.setBaudRate(original_baud):
                    raise CommunicationError(
                        "restore host baud rate",
                        -1,
                        f"could not restore {original_baud}",
                    )
                self.baudrate = original_baud
            return rate

        if self.baudrate != original_baud:
            self.port_handler.setBaudRate(original_baud)
            self.baudrate = original_baud
        detail = f"; last error: {last_error}" if last_error is not None else ""
        raise CommunicationError(
            "detect baud rate",
            -1,
            f"servo ID {self.servo_id} did not reply at {rates}{detail}",
        )

    # ------------------------------------------------------------------
    # Status and feedback
    # ------------------------------------------------------------------

    def read_status(self) -> MotorStatus:
        """Read position, speed, load, voltage, temperature, flags, and current.

        A single 15-byte READ starting at address 56 gives a coherent snapshot
        and is faster than issuing separate requests for every parameter.
        """

        data = self.read_register_bytes(Register.PRESENT_POSITION, 15)
        position_raw = _word_le(data[0], data[1])
        speed_register = _word_le(data[2], data[3])
        load_register = _word_le(data[4], data[5])
        position_sdk = decode_sign_magnitude(position_raw, 15)
        speed_raw = decode_sign_magnitude(speed_register, 15)
        load_signed = decode_sign_magnitude(load_register, 15)
        status_byte = data[9]
        response_status = self._last_servo_error
        current_raw = _word_le(data[13], data[14])
        current_a = current_raw * CURRENT_RAW_TO_AMP
        estimated_torque_kg_cm = self.estimate_torque_kg_cm(current_a=current_a)
        estimated_torque_nm = self.estimate_torque_nm(current_a=current_a)

        return MotorStatus(
            timestamp=time.monotonic(),
            position_raw_register=position_raw,
            position_sdk=position_sdk,
            position_single_turn_deg=(position_sdk % POSITION_COUNTS_PER_REVOLUTION)
            * POSITION_COUNT_TO_DEG,
            speed_raw=speed_raw,
            speed_rpm=speed_raw * SPEED_RAW_TO_RPM,
            load_raw=load_register,
            load_signed=load_signed,
            load_fraction=abs(load_signed) * 0.001,
            voltage_v=data[6] * VOLTAGE_RAW_TO_VOLT,
            temperature_c=data[7],
            registered_instruction=bool(data[8]),
            status_byte=status_byte,
            status_flags=StatusFlag(status_byte),
            moving=bool(data[10]),
            current_raw=current_raw,
            current_a=current_a,
            estimated_torque_kg_cm=estimated_torque_kg_cm,
            estimated_torque_nm=estimated_torque_nm,
            response_status_byte=response_status,
            response_status_flags=StatusFlag(response_status),
        )

    def read_control_targets(self) -> ControlTargets:
        """Read torque-enable, acceleration, and goal targets in one request.

        Addresses 40 through 49 are contiguous.  The final two-byte torque
        limit field is present in the common magnetic-family table and can be
        firmware dependent; the other fields are part of the HLS SDK core.
        """

        data = self.read_register_bytes(Register.TORQUE_ENABLE, 10)
        goal_position_register = _word_le(data[2], data[3])
        goal_torque_register = _word_le(data[4], data[5])
        goal_speed_register = _word_le(data[6], data[7])
        goal_speed = decode_sign_magnitude(goal_speed_register, 15)

        return ControlTargets(
            torque_enabled=bool(data[0]),
            acceleration_raw=data[1],
            goal_position_register=goal_position_register,
            goal_position=decode_sign_magnitude(goal_position_register, 15),
            goal_torque_register=goal_torque_register,
            goal_torque=decode_sign_magnitude(goal_torque_register, 15),
            goal_speed_register=goal_speed_register,
            goal_speed=goal_speed,
            goal_speed_rpm=goal_speed * SPEED_RAW_TO_RPM,
            torque_limit_raw=_word_le(data[8], data[9]),
        )

    def read_position(self) -> int:
        self._require_connected()
        with self._lock:
            value, result, error = self.packet_handler.ReadPos(self.servo_id)
            self._check_result("read position", result, error)
            return int(value)

    def read_speed(self) -> int:
        self._require_connected()
        with self._lock:
            value, result, error = self.packet_handler.ReadSpeed(self.servo_id)
            self._check_result("read speed", result, error)
            return int(value)

    def read_speed_rpm(self) -> float:
        return self.read_speed() * SPEED_RAW_TO_RPM

    def read_load_raw(self) -> int:
        raw = self.read_register(Register.PRESENT_LOAD, 2)
        return decode_sign_magnitude(raw, 15)

    def read_load_fraction(self) -> float:
        """Return absolute drive/load feedback scaled by 0.001.

        This is controller output/load feedback, not a calibrated torque sensor.
        """

        return abs(self.read_load_raw()) * 0.001

    def read_voltage(self) -> float:
        return self.read_register(Register.PRESENT_VOLTAGE, 1) * VOLTAGE_RAW_TO_VOLT

    def read_voltage_filtered(self, *, alpha: Optional[float] = None) -> float:
        """Read voltage and apply an exponential moving average."""

        if alpha is not None and not math.isclose(alpha, self.voltage_filter.alpha):
            self.voltage_filter = ExponentialMovingAverage(alpha=alpha)
        return self.voltage_filter.update(self.read_voltage())

    def reset_voltage_filter(self, value: Optional[float] = None) -> None:
        self.voltage_filter.reset(value)

    def read_temperature(self) -> int:
        return self.read_register(Register.PRESENT_TEMPERATURE, 1)

    def read_current_raw(self) -> int:
        return self.read_register(Register.PRESENT_CURRENT, 2)

    def read_current(self) -> float:
        return self.read_current_raw() * CURRENT_RAW_TO_AMP

    def estimate_torque_kg_cm(self, *, current_a: Optional[float] = None) -> Optional[float]:
        """Estimate output torque magnitude from current and the configured Kt.

        The HLS3950M product specification gives ``Kt = 20.8 kg.cm/A``.  This
        method is an engineering estimate, not a direct torque-sensor reading;
        it does not remove no-load current or compensate gearbox losses,
        temperature, or controller dynamics.  Pass
        ``torque_constant_kg_cm_per_a=None`` to disable this estimate.
        """

        if self.torque_constant_kg_cm_per_a is None:
            return None
        measured_current = self.read_current() if current_a is None else float(current_a)
        return abs(measured_current) * self.torque_constant_kg_cm_per_a

    def estimate_torque_nm(self, *, current_a: Optional[float] = None) -> Optional[float]:
        """Return the current-based torque estimate in N.m."""

        estimate = self.estimate_torque_kg_cm(current_a=current_a)
        return None if estimate is None else estimate * KG_CM_TO_NM

    def read_goal_position(self) -> int:
        """Read and decode the current goal-position register."""

        raw = self.read_register(Register.GOAL_POSITION, 2)
        return decode_sign_magnitude(raw, 15)

    def read_goal_torque(self) -> int:
        """Read and decode the raw HLS goal torque/current target."""

        raw = self.read_register(Register.GOAL_TORQUE, 2)
        return decode_sign_magnitude(raw, 15)

    def read_goal_torque_fraction(self) -> float:
        """Return the raw goal-torque target scaled to approximately -1..1."""

        return self.read_goal_torque() / 1000.0

    def read_goal_speed(self) -> int:
        """Read and decode the current goal-speed register."""

        raw = self.read_register(Register.GOAL_SPEED, 2)
        return decode_sign_magnitude(raw, 15)

    def read_mode(self) -> int:
        """Read the operating-mode register."""

        return self.read_register(Register.MODE, 1)

    def read_torque_enabled(self) -> bool:
        """Return whether motor torque output is enabled."""

        return bool(self.read_register(Register.TORQUE_ENABLE, 1))

    def is_moving(self) -> bool:
        return bool(self.read_register(Register.MOVING, 1))

    def read_status_flags(self) -> StatusFlag:
        return StatusFlag(self.read_register(Register.STATUS, 1))

    def _build_encoder_sample(
        self,
        *,
        position: int,
        speed: Optional[int],
        flags: StatusFlag,
        direction_policy: str,
        timestamp: Optional[float] = None,
    ) -> EncoderSample:
        """Update the software encoder from already-read feedback."""

        if direction_policy == "command":
            direction_hint = _sign(self._last_commanded_speed)
        elif direction_policy == "measured":
            direction_hint = _sign(speed or 0)
        elif direction_policy == "auto":
            direction_hint = 0
        else:
            raise ValueError("direction_policy must be 'command', 'measured', or 'auto'")

        update = self.encoder.update(int(position), direction_hint=direction_hint)
        sample_time = time.monotonic() if timestamp is None else float(timestamp)
        return EncoderSample(
            timestamp=sample_time,
            sdk_position=int(position),
            delta_counts=update.delta,
            continuous_counts=update.continuous_counts,
            absolute_counts=update.absolute_counts,
            continuous_revolutions=update.continuous_counts
            / self.encoder.counts_per_revolution,
            absolute_revolutions=update.absolute_counts
            / self.encoder.counts_per_revolution,
            speed_raw=int(speed) if speed is not None else None,
            speed_rpm=(int(speed) * SPEED_RAW_TO_RPM) if speed is not None else None,
            direction_hint=direction_hint,
            valid=update.valid,
            glitch_reason=update.glitch_reason,
            servo_status=flags,
        )

    def read_encoder(
        self,
        *,
        include_speed: bool = True,
        direction_policy: str = "command",
    ) -> EncoderSample:
        """Read encoder feedback and return a non-wrapping software position.

        ``direction_policy`` can be:

        * ``"command"``: use the sign of the last wheel-speed command.  This is
          the most robust monotonic option for a driven wheel.
        * ``"measured"``: use the sign of measured speed when non-zero.
        * ``"auto"``: infer signed movement from adjacent positions only.
        """

        self._require_connected()
        with self._lock:
            if include_speed:
                position, speed, result, error = self.packet_handler.ReadPosSpeed(self.servo_id)
            else:
                position, result, error = self.packet_handler.ReadPos(self.servo_id)
                speed = None
            flags = self._check_result("read encoder", result, error)

        return self._build_encoder_sample(
            position=int(position),
            speed=int(speed) if speed is not None else None,
            flags=flags,
            direction_policy=direction_policy,
        )

    def read_snapshot(self, *, direction_policy: str = "command") -> MotorSnapshot:
        """Read all runtime feedback and update the incremental encoder.

        This uses the same single 15-byte status transaction as
        :meth:`read_status`, then feeds its position and speed into the
        non-wrapping software encoder.  It is the most efficient method when
        an application needs encoder, speed, voltage, current, temperature,
        load, movement, and status flags together.
        """

        status = self.read_status()
        encoder = self._build_encoder_sample(
            position=status.position_sdk,
            speed=status.speed_raw,
            flags=status.response_status_flags,
            direction_policy=direction_policy,
            timestamp=status.timestamp,
        )
        return MotorSnapshot(timestamp=status.timestamp, encoder=encoder, status=status)

    def reset_incremental_encoder(
        self,
        *,
        continuous_counts: int = 0,
        absolute_counts: int = 0,
    ) -> None:
        """Reset only the software encoder accumulator."""

        self.encoder.reset(
            continuous_counts=continuous_counts,
            absolute_counts=absolute_counts,
        )

    def iter_encoder(
        self,
        *,
        include_speed: bool = True,
        direction_policy: str = "command",
        rate_hz: Optional[float] = None,
    ) -> Iterator[EncoderSample]:
        """Yield encoder samples continuously.

        ``rate_hz=None`` performs no intentional sleeping and polls at the
        maximum practical rate.
        """

        period = None if rate_hz is None else 1.0 / rate_hz
        while True:
            started = time.perf_counter()
            yield self.read_encoder(
                include_speed=include_speed,
                direction_policy=direction_policy,
            )
            if period is not None:
                remaining = period - (time.perf_counter() - started)
                if remaining > 0:
                    time.sleep(remaining)

    # ------------------------------------------------------------------
    # Motor control
    # ------------------------------------------------------------------

    def enable_torque(self) -> None:
        self.write_register(Register.TORQUE_ENABLE, 1)

    def disable_torque(self) -> None:
        self.write_register(Register.TORQUE_ENABLE, 0)

    def set_mode(self, mode: Union[int, OperatingMode], *, persistent: bool = False) -> None:
        mode_value = int(OperatingMode(int(mode)))
        if persistent:
            with self.eeprom_unlocked():
                self.write_register(Register.MODE, mode_value)
        else:
            self.write_register(Register.MODE, mode_value)

    def set_wheel_mode(self, *, persistent: bool = False) -> None:
        self.set_mode(OperatingMode.SPEED, persistent=persistent)

    def _write_spec(self, speed: int, acceleration: int, torque: int) -> None:
        """Write the seven-byte HLS wheel/torque command block."""

        encoded_speed = encode_sign_magnitude(int(speed), 15)
        payload = bytes(
            [
                int(acceleration) & 0xFF,
                0,
                0,
                int(torque) & 0xFF,
                (int(torque) >> 8) & 0xFF,
                encoded_speed & 0xFF,
                (encoded_speed >> 8) & 0xFF,
            ]
        )
        self.write_register_bytes(Register.ACCELERATION, payload)

    def _write_position_ex(
        self,
        position: int,
        speed: int,
        acceleration: int,
        torque: int,
    ) -> None:
        """Write the seven-byte HLS extended position command block."""

        encoded_position = encode_sign_magnitude(int(position), 15)
        payload = bytes(
            [
                int(acceleration) & 0xFF,
                encoded_position & 0xFF,
                (encoded_position >> 8) & 0xFF,
                int(torque) & 0xFF,
                (int(torque) >> 8) & 0xFF,
                int(speed) & 0xFF,
                (int(speed) >> 8) & 0xFF,
            ]
        )
        self.write_register_bytes(Register.ACCELERATION, payload)

    def set_wheel_speed(
        self,
        speed: int,
        *,
        acceleration: int = 5,
        torque: int = 200,
        ensure_mode: bool = True,
        enable_torque: bool = True,
    ) -> None:
        """Command constant-speed wheel mode.

        Positive and negative speed values select opposite directions.  The SDK
        wheel example uses approximately 0.732 RPM per raw speed unit.
        """

        if not -32767 <= int(speed) <= 32767:
            raise ValueError("speed must be in range -32767..32767")
        if not 0 <= int(acceleration) <= 254:
            raise ValueError("acceleration must be in range 0..254")
        if not 0 <= int(torque) <= 1000:
            raise ValueError("torque must be in range 0..1000")

        requested_rpm = abs(speed) * SPEED_RAW_TO_RPM
        if (
            self.rated_no_load_rpm is not None
            and requested_rpm > self.rated_no_load_rpm * 1.25
        ):
            warnings.warn(
                f"Raw speed {speed} requests about {requested_rpm:.1f} RPM, "
                f"well above the configured rated speed of "
                f"{self.rated_no_load_rpm:.1f} RPM. The servo will saturate "
                "and may become noisy or hot.",
                RuntimeWarning,
                stacklevel=2,
            )

        self._require_connected()
        with self._lock:
            if ensure_mode:
                self.set_wheel_mode()
            if enable_torque:
                self.enable_torque()
            self._write_spec(int(speed), int(acceleration), int(torque))
            self._last_commanded_speed = int(speed)

    def set_wheel_rpm(self, rpm: float, **kwargs) -> int:
        """Convert RPM to a raw speed command, send it, and return the raw value."""

        raw = int(round(float(rpm) / SPEED_RAW_TO_RPM))
        self.set_wheel_speed(raw, **kwargs)
        return raw

    def stop(
        self,
        *,
        disable_torque: bool = False,
        retries: int = 3,
        acceleration: int = 5,
        torque: int = 200,
        ignore_errors: bool = False,
    ) -> None:
        """Command zero speed; optionally release torque afterward."""

        self._require_connected()
        last_exception: Optional[Exception] = None
        for _ in range(max(1, int(retries))):
            try:
                with self._lock:
                    self._write_spec(0, int(acceleration), int(torque))
                    self._last_commanded_speed = 0
                    last_exception = None
            except Exception as exc:
                last_exception = exc
            time.sleep(0.02)

        if disable_torque:
            try:
                self.disable_torque()
            except Exception as exc:
                last_exception = exc

        if last_exception is not None and not ignore_errors:
            raise last_exception

    def move_to_position(
        self,
        position: int,
        *,
        speed: int = 60,
        acceleration: int = 10,
        torque: int = 500,
        ensure_mode: bool = True,
    ) -> None:
        """Command HLS position mode using ``WritePosEx``."""

        if not -32767 <= int(position) <= 32767:
            raise ValueError("position must be in range -32767..32767")
        if not 0 <= int(speed) <= 32767:
            raise ValueError("speed must be in range 0..32767")
        if not 0 <= int(acceleration) <= 254:
            raise ValueError("acceleration must be in range 0..254")
        if not 0 <= int(torque) <= 1000:
            raise ValueError("torque must be in range 0..1000")
        self._require_connected()
        with self._lock:
            if ensure_mode:
                self.set_mode(OperatingMode.POSITION)
            self.enable_torque()
            self._write_position_ex(
                int(position), int(speed), int(acceleration), int(torque)
            )
            self._last_commanded_speed = 0

    def move_to_degrees(self, degrees: float, **kwargs) -> int:
        """Move to a single-turn angle from 0 to 360 degrees."""

        if not 0.0 <= float(degrees) <= 360.0:
            raise ValueError("degrees must be in range 0..360")
        position = int(round(float(degrees) * 4095.0 / 360.0))
        self.move_to_position(position, **kwargs)
        return position

    def set_goal_torque_raw(
        self,
        torque: int,
        *,
        acceleration: int = 5,
        ensure_mode: bool = True,
    ) -> None:
        """Set HLS constant-current/torque mode target at address 44.

        This is a raw controller target, not a calibrated N.m measurement.
        """

        encoded = encode_sign_magnitude(int(torque), 15)
        if not 0 <= int(acceleration) <= 254:
            raise ValueError("acceleration must be in range 0..254")
        if ensure_mode:
            self.set_mode(OperatingMode.TORQUE)
        self.enable_torque()
        self.write_register(Register.ACCELERATION, int(acceleration), length=1)
        self.write_register(Register.GOAL_TORQUE, encoded, length=2)
        self._last_commanded_speed = 0

    def set_goal_torque_fraction(
        self,
        fraction: float,
        *,
        acceleration: int = 5,
        ensure_mode: bool = True,
    ) -> int:
        """Set constant-current/torque mode using a normalized -1..1 target.

        Returns the raw target written to address 44.  The normalized value is
        a controller-domain command, not a calibrated physical torque.
        """

        if not -1.0 <= float(fraction) <= 1.0:
            raise ValueError("fraction must be in range -1.0..1.0")
        raw = int(round(float(fraction) * 1000.0))
        self.set_goal_torque_raw(
            raw,
            acceleration=acceleration,
            ensure_mode=ensure_mode,
        )
        return raw

    # ------------------------------------------------------------------
    # Configuration helpers
    # ------------------------------------------------------------------

    def _write_acknowledgements_enabled(self) -> bool:
        """Best-effort detection of whether ordinary WRITE replies are enabled."""

        if self._response_level_cache is not None:
            return self._response_level_cache == ResponseLevel.ALL_INSTRUCTIONS
        try:
            raw = self.read_register(Register.RESPONSE_LEVEL, 1)
            self._response_level_cache = ResponseLevel(raw)
            return self._response_level_cache == ResponseLevel.ALL_INSTRUCTIONS
        except (CommunicationError, ValueError):
            # Core HLS firmware normally defaults to write replies. Falling
            # back to True preserves error reporting when register 8 is absent.
            return True

    def unlock_eeprom(self, *, expect_reply: Optional[bool] = None) -> None:
        if expect_reply is None:
            expect_reply = self._write_acknowledgements_enabled()
        self.write_register(Register.LOCK, 0, expect_reply=expect_reply)

    def lock_eeprom(self, *, expect_reply: Optional[bool] = None) -> None:
        if expect_reply is None:
            expect_reply = self._write_acknowledgements_enabled()
        self.write_register(Register.LOCK, 1, expect_reply=expect_reply)

    @contextlib.contextmanager
    def eeprom_unlocked(self, *, expect_reply: Optional[bool] = None):
        """Unlock EEPROM writes and always attempt to lock them afterward."""

        if expect_reply is None:
            expect_reply = self._write_acknowledgements_enabled()
        self.unlock_eeprom(expect_reply=expect_reply)
        try:
            yield self
        finally:
            self.lock_eeprom(expect_reply=expect_reply)

    def set_id(self, new_id: int, *, verify: bool = True) -> None:
        """Change servo ID and update this object to use the new ID."""

        self._validate_servo_id(new_id)
        if int(new_id) == self.servo_id:
            return
        self._require_connected()

        old_id = self.servo_id
        id_write_sent = False
        self.unlock_eeprom()
        try:
            # The reply may use either the old or new ID, so use TX-only.
            self.write_register(Register.ID, int(new_id), length=1, expect_reply=False)
            id_write_sent = True
            time.sleep(0.05)
            self.servo_id = int(new_id)
            self.lock_eeprom()
        except Exception as exc:
            # Once the TX-only write has left the UART, the device may already
            # be using the new ID even if relocking or verification fails.
            self.servo_id = int(new_id) if id_write_sent else old_id
            raise ConfigurationError(
                "ID change did not complete cleanly. The write packet was "
                f"{'sent' if id_write_sent else 'not sent'}; this object is now "
                f"set to ID {self.servo_id}. Try ping(), or scan IDs at the "
                f"current baud. Original error: {exc}"
            ) from exc

        if verify:
            self.ping()

    def set_baudrate(self, new_baudrate: int, *, verify: bool = True) -> None:
        """Change servo and Raspberry Pi UART baud rate together."""

        if int(new_baudrate) not in RATE_TO_BAUD_CODE:
            raise ConfigurationError(
                f"Unsupported baud rate {new_baudrate}; choose one of "
                f"{sorted(RATE_TO_BAUD_CODE)}"
            )
        if int(new_baudrate) == self.baudrate:
            return
        self._require_connected()

        old_baudrate = self.baudrate
        code = RATE_TO_BAUD_CODE[int(new_baudrate)]
        baud_write_sent = False
        host_switched = False
        self.unlock_eeprom()
        try:
            # Once applied, the servo may immediately stop speaking old baud.
            self.write_register(Register.BAUD_RATE, code, length=1, expect_reply=False)
            baud_write_sent = True
            time.sleep(0.05)
            if not self.port_handler.setBaudRate(int(new_baudrate)):
                raise CommunicationError(
                    "change host baud rate", -1, f"UART rejected {new_baudrate}"
                )
            host_switched = True
            self.baudrate = int(new_baudrate)
            self.lock_eeprom()
        except Exception as exc:
            # If the servo baud write was sent, rolling the object blindly back
            # to the old rate is misleading. Keep whichever host rate is active
            # and direct the caller to detect_baudrate() for recovery.
            self.baudrate = int(new_baudrate) if host_switched else old_baudrate
            raise ConfigurationError(
                "Baud-rate change did not complete cleanly. Servo write sent="
                f"{baud_write_sent}, host switched={host_switched}. Current "
                f"object baud is {self.baudrate}. Use detect_baudrate() if the "
                f"servo no longer replies. Original error: {exc}"
            ) from exc

        if verify:
            self.ping()

    def set_id_and_baudrate(
        self,
        new_id: int,
        new_baudrate: int,
        *,
        verify: bool = True,
    ) -> None:
        """Change ID first, then baud rate, using the working current link."""

        self.set_id(new_id, verify=False)
        self.set_baudrate(new_baudrate, verify=verify)

    def set_response_level(self, level: Union[int, ResponseLevel]) -> None:
        """Set the servo's status-response policy safely.

        Response level 0 still allows READ and PING responses but suppresses
        normal WRITE acknowledgements. Therefore this method sends the level
        write and final EEPROM-lock write as TX-only packets, then verifies the
        new value with a READ transaction.
        """

        value = int(ResponseLevel(int(level)))
        self.unlock_eeprom()
        try:
            self.write_register(
                Register.RESPONSE_LEVEL,
                value,
                expect_reply=False,
            )
            time.sleep(0.02)
            self.lock_eeprom(expect_reply=False)
            time.sleep(0.02)
        except Exception:
            with contextlib.suppress(Exception):
                self.lock_eeprom(expect_reply=False)
            raise

        read_back = self.read_register(Register.RESPONSE_LEVEL, 1)
        if read_back != value:
            raise ConfigurationError(
                f"response-level verification failed: wrote {value}, read {read_back}"
            )
        self._response_level_cache = ResponseLevel(value)

    def set_return_delay_us(self, delay_us: int) -> int:
        """Set response delay in 2-us units and return the applied delay."""

        if not 0 <= int(delay_us) <= 508:
            raise ValueError("delay_us must be in range 0..508")
        raw = int(round(delay_us / 2.0))
        with self.eeprom_unlocked():
            self.write_register(Register.RETURN_DELAY, raw)
        return raw * 2

    def set_angle_limits(self, minimum: int, maximum: int) -> None:
        if not 0 <= int(minimum) < int(maximum) <= 4095:
            raise ValueError("angle limits must satisfy 0 <= minimum < maximum <= 4095")
        with self.eeprom_unlocked():
            self.write_register(Register.MIN_ANGLE_LIMIT, int(minimum), length=2)
            self.write_register(Register.MAX_ANGLE_LIMIT, int(maximum), length=2)

    def set_voltage_limits(self, minimum_v: float, maximum_v: float) -> None:
        if not 0.0 <= minimum_v < maximum_v <= 25.4:
            raise ValueError("voltage limits must satisfy 0 <= min < max <= 25.4 V")
        with self.eeprom_unlocked():
            self.write_register(Register.MIN_VOLTAGE_LIMIT, round(minimum_v * 10))
            self.write_register(Register.MAX_VOLTAGE_LIMIT, round(maximum_v * 10))

    def set_temperature_limit(self, temperature_c: int) -> None:
        if not 0 <= int(temperature_c) <= 100:
            raise ValueError("temperature_c must be in range 0..100")
        with self.eeprom_unlocked():
            self.write_register(Register.MAX_TEMPERATURE_LIMIT, int(temperature_c))

    def set_dead_zones(self, cw: int, ccw: int) -> None:
        if not 0 <= int(cw) <= 32 or not 0 <= int(ccw) <= 32:
            raise ValueError("dead-zone values must be in range 0..32")
        with self.eeprom_unlocked():
            self.write_register(Register.CW_DEAD_ZONE, int(cw))
            self.write_register(Register.CCW_DEAD_ZONE, int(ccw))

    def set_position_pid(self, p: int, i: int, d: int) -> None:
        for name, value in {"p": p, "i": i, "d": d}.items():
            if not 0 <= int(value) <= 254:
                raise ValueError(f"{name} must be in range 0..254")
        with self.eeprom_unlocked():
            self.write_register(Register.POSITION_P, int(p))
            self.write_register(Register.POSITION_I, int(i))
            self.write_register(Register.POSITION_D, int(d))

    def set_speed_pid(self, p: int, i: int) -> None:
        for name, value in {"p": p, "i": i}.items():
            if not 0 <= int(value) <= 254:
                raise ValueError(f"{name} must be in range 0..254")
        with self.eeprom_unlocked():
            self.write_register(Register.SPEED_P, int(p))
            self.write_register(Register.SPEED_I, int(i))

    def set_phase_raw(self, phase: int, *, confirm: bool = False) -> None:
        """Write the special phase byte; requires explicit confirmation."""

        if not confirm:
            raise ConfigurationError(
                "The phase register changes several low-level behaviors. "
                "Pass confirm=True only after checking your exact firmware table."
            )
        if not 0 <= int(phase) <= 254:
            raise ValueError("phase must be in range 0..254")
        with self.eeprom_unlocked():
            self.write_register(Register.PHASE, int(phase))

    def factory_reset(self, *, confirm: bool = False) -> None:
        """Send the protocol recovery instruction.

        The external SDK does not expose RECOVERY directly, so this helper is
        intentionally omitted from automatic operation.  Use raw protocol tools
        only after making a complete register backup.
        """

        if not confirm:
            raise ConfigurationError("factory_reset requires confirm=True")
        raise NotImplementedError(
            "Factory recovery is intentionally not automated by this wrapper. "
            "Back up registers and use Feetech's official configuration tool."
        )
