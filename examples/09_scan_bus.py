#!/usr/bin/env python3
"""Find HLS servos by pinging IDs at each supported baud rate.

Examples:
    python examples/09_scan_bus.py
    python examples/09_scan_bus.py --port /dev/ttyAMA0 --start-id 1 --end-id 20
    python examples/09_scan_bus.py --baud 38400 1000000

The full scan takes time: each nonresponding ID must time out. This sends
unicast PING packets only; it does not change servo configuration or stop motors.
Devices sharing an ID and baud may collide and cannot be reliably identified.
"""

import argparse

from feetech_hls_rpi import BAUD_CODE_TO_RATE, CommunicationError, HLSMotor


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Pi UART device (default: %(default)s)")
    parser.add_argument("--start-id", type=int, default=0, help="First ID, inclusive (default: 0)")
    parser.add_argument("--end-id", type=int, default=253, help="Last ID, inclusive (default: 253)")
    parser.add_argument("--baud", type=int, nargs="+", choices=sorted(BAUD_CODE_TO_RATE.values()),
                        default=list(BAUD_CODE_TO_RATE.values()), help="Baud rates to scan (default: all supported)")
    args = parser.parse_args(argv)
    if not 0 <= args.start_id <= args.end_id <= 253:
        parser.error("IDs must satisfy 0 <= start-id <= end-id <= 253")

    found = []
    skipped_rates = []
    for baud in dict.fromkeys(args.baud):
        print(f"Scanning {baud} baud, IDs {args.start_id} through {args.end_id}...", flush=True)
        motor = HLSMotor(port=args.port, baudrate=baud, servo_id=args.start_id, stop_on_close=False)
        try:
            try:
                motor.connect()
            except (CommunicationError, OSError, ValueError) as exc:
                print(f"  Could not open UART at {baud} baud: {exc}", flush=True)
                skipped_rates.append(baud)
                continue
            for servo_id in range(args.start_id, args.end_id + 1):
                motor.servo_id = servo_id
                try:
                    model = motor.ping()
                except CommunicationError:
                    continue
                found.append((baud, servo_id, model))
                print(f"  FOUND: baud={baud}, ID={servo_id}, model={model}", flush=True)
        finally:
            motor.close()

    print(f"Scan finished: {len(found)} response(s).")
    if skipped_rates:
        print("Baud rates not scanned:", ", ".join(map(str, skipped_rates)))
    if not found:
        print("No responses found; check power, wiring, UART, and selected rates/IDs.")
    return 1 if skipped_rates else 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nScan interrupted.")
        raise SystemExit(130)
