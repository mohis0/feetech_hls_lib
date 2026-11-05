"""Raspberry Pi UART transport for direct RX and resistor-connected TX."""

from __future__ import annotations

import threading
import time

try:
    import termios
except ImportError as exc:  # pragma: no cover - Raspberry Pi/Linux only
    raise RuntimeError("feetech_hls_rpi requires Linux termios support") from exc

from ._compat import PortHandler, require_hardware_dependencies
from .exceptions import EchoFilterError


class MuxPortHandler(PortHandler):
    """Feetech SDK port handler for the user's half-duplex wiring.

    Wiring assumption:

    * Raspberry Pi RX is continuously connected to servo DATA.
    * Raspberry Pi TX reaches servo DATA through a 1 kohm resistor.

    Continuous RX means the Pi receives its own transmitted packet.  This
    handler removes exactly that echo before the SDK parses the servo reply.
    """

    def __init__(
        self,
        port_name: str,
        *,
        mux_gpio: int = 23,
        gpio_chip: int = 0,
        tx_connected_level: int = 1,
        tx_disconnected_level: int = 0,
        packet_timeout_ms: float = 20.0,
        echo_timeout_ms: float = 20.0,
        strict_echo: bool = True,
    ) -> None:
        require_hardware_dependencies()
        super().__init__(port_name)

        self.packet_timeout_ms = float(packet_timeout_ms)
        self.echo_timeout_ms = float(echo_timeout_ms)
        self.strict_echo = bool(strict_echo)

        self.last_echo = b""
        self.echo_mismatch_count = 0
        self._transport_lock = threading.RLock()
        self._shutdown = False

    def clearPort(self) -> None:  # noqa: N802 - SDK API name
        """Clear stale RX bytes before the SDK starts a new transaction."""

        if self.ser is not None:
            self.ser.reset_input_buffer()

    def _discard_tx_echo(self, expected: bytes) -> None:
        echo = bytearray()
        deadline = time.perf_counter() + self.echo_timeout_ms / 1000.0

        while len(echo) < len(expected):
            waiting = self.ser.in_waiting
            if waiting:
                need = len(expected) - len(echo)
                chunk = self.ser.read(min(waiting, need))
                if chunk:
                    echo.extend(chunk)
                    continue
            if time.perf_counter() >= deadline:
                break
            time.sleep(0)

        self.last_echo = bytes(echo)
        if self.last_echo != expected:
            self.echo_mismatch_count += 1
            if self.strict_echo:
                raise EchoFilterError(
                    "TX echo mismatch. Expected "
                    f"{expected.hex(' ')}, received {self.last_echo.hex(' ')}. "
                    "Check that RX is directly connected to servo DATA and "
                    "that TX is connected through 1 kohm."
                )

    def writePort(self, packet):  # noqa: N802 - SDK API name
        """Transmit and remove the locally received echo."""

        packet_bytes = bytes(packet)
        with self._transport_lock:
            if self.ser is None:
                return 0

            try:
                written = self.ser.write(packet_bytes)
                self.ser.flush()
                termios.tcdrain(self.ser.fileno())
                self._discard_tx_echo(packet_bytes)
                return written
            except Exception:
                # The SDK sets is_using before it calls writePort(). If a
                # Python/echo-filter exception escapes, clear that state so a
                # later recovery attempt is not reported as a permanently busy
                # port.
                self.is_using = False
                raise
    def setPacketTimeout(self, packet_length):  # noqa: N802 - SDK API name
        """Use a fixed timeout, avoiding the SDK's large default latency."""

        self.setPacketTimeoutMillis(self.packet_timeout_ms)

    def shutdown(self) -> None:
        """Close the UART."""

        if self._shutdown:
            return
        self._shutdown = True

        if getattr(self, "is_open", False):
            try:
                super().closePort()
            except Exception:
                pass

    def __enter__(self) -> "MuxPortHandler":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.shutdown()
