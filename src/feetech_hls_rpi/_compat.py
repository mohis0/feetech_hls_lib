"""Optional imports kept in one place so pure helpers remain testable."""

from __future__ import annotations

from .exceptions import MissingDependencyError

try:  # pragma: no cover - hardware dependency
    import lgpio as lgpio_module
except ImportError:  # pragma: no cover - exercised on non-Pi CI
    lgpio_module = None

try:  # pragma: no cover - external SDK dependency
    from scservo_sdk import COMM_SUCCESS, PortHandler, hls
except ImportError:  # pragma: no cover - exercised on non-Pi CI
    COMM_SUCCESS = 0
    hls = None

    class PortHandler:  # type: ignore[no-redef]
        """Placeholder used only so the package can be imported without SDK."""

        pass


def require_hardware_dependencies() -> None:
    missing = []
    if lgpio_module is None:
        missing.append("lgpio")
    if hls is None:
        missing.append("vassar-feetech-servo-sdk (scservo_sdk)")
    if missing:
        raise MissingDependencyError(
            "Missing runtime dependencies: " + ", ".join(missing) + ". "
            "Install with: pip install vassar-feetech-servo-sdk pyserial lgpio"
        )
