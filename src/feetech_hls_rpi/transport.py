"""Raspberry Pi UART transport for direct RX plus CD4051-switched TX."""

from __future__ import annotations

import threading
import time

try:
    import termios
except ImportError as exc:  # pragma: no cover - Raspberry Pi/Linux only
    raise RuntimeError("feetech_hls_rpi requires Linux termios support") from exc

from ._compat import PortHandler, lgpio_module, require_hardware_dependencies
from .exceptions import EchoFilterError


class MuxPortHandler(PortHandler):
    """Feetech SDK port handler for the user's half-duplex wiring.

    Wiring assumption:

    * Raspberry Pi RX is continuously connected to servo DATA.
    * Raspberry Pi TX reaches servo DATA through a CD4051 channel.
    * A GPIO selects the connected TX channel or an unused/disconnected channel.

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

        if tx_connected_level not in (0, 1) or tx_disconnected_level not in (0, 1):
            raise ValueError("TX mux levels must be 0 or 1")
        if tx_connected_level == tx_disconnected_level:
            raise ValueError("TX connected and disconnected levels must differ")

        self.mux_gpio = int(mux_gpio)
        self.gpio_chip_number = int(gpio_chip)
        self.tx_connected_level = int(tx_connected_level)
        self.tx_disconnected_level = int(tx_disconnected_level)
        self.packet_timeout_ms = float(packet_timeout_ms)
        self.echo_timeout_ms = float(echo_timeout_ms)
        self.strict_echo = bool(strict_echo)

        self.last_echo = b""
        self.echo_mismatch_count = 0
        self._gpio = lgpio_module
        self._gpio_handle = self._gpio.gpiochip_open(self.gpio_chip_number)
        self._gpio.gpio_claim_output(
            self._gpio_handle,
            self.mux_gpio,
            self.tx_disconnected_level,
        )
        self._transport_lock = threading.RLock()
        self._shutdown = False

    def connect_tx(self) -> None:
        self._gpio.gpio_write(
            self._gpio_handle,
            self.mux_gpio,
            self.tx_connected_level,
        )

    def disconnect_tx(self) -> None:
        self._gpio.gpio_write(
            self._gpio_handle,
            self.mux_gpio,
            self.tx_disconnected_level,
        )

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
                    "that the CD4051 connects only TX."
                )

    def writePort(self, packet):  # noqa: N802 - SDK API name
        """Transmit, release TX, and remove the locally received echo."""

        packet_bytes = bytes(packet)
        with self._transport_lock:
            if self.ser is None:
                return 0

            try:
                self.connect_tx()
                written = self.ser.write(packet_bytes)
                self.ser.flush()
                termios.tcdrain(self.ser.fileno())
                # Release the one-wire bus even if serial.write/flush fails.
                self.disconnect_tx()
                self._discard_tx_echo(packet_bytes)
                return written
            except Exception:
                # The SDK sets is_using before it calls writePort(). If a
                # Python/echo-filter exception escapes, clear that state so a
                # later recovery attempt is not reported as a permanently busy
                # port.
                self.is_using = False
                raise
            finally:
                try:
                    self.disconnect_tx()
                except Exception:
                    pass

    def setPacketTimeout(self, packet_length):  # noqa: N802 - SDK API name
        """Use a fixed timeout, avoiding the SDK's large default latency."""

        self.setPacketTimeoutMillis(self.packet_timeout_ms)

    def shutdown(self) -> None:
        """Release TX, close UART, and release the GPIO line."""

        if self._shutdown:
            return
        self._shutdown = True

        try:
            self.disconnect_tx()
        except Exception:
            pass

        if getattr(self, "is_open", False):
            try:
                super().closePort()
            except Exception:
                pass

        try:
            self._gpio.gpio_free(self._gpio_handle, self.mux_gpio)
        except Exception:
            pass
        try:
            self._gpio.gpiochip_close(self._gpio_handle)
        except Exception:
            pass

    def __enter__(self) -> "MuxPortHandler":
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.shutdown()
