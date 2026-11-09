#!/usr/bin/env python3
"""Print every packet the boat's RF kill board sends, to find out how it reports kills.

Stop piss_tower first (it holds the port), run this, then press and release the
kill switches and flip the remote between manual and autonomous. Each packet is
printed with its class, subclass and payload, so a kill shows up as whichever
packet appears when you press the switch.

    python3 scripts/rf_kill_listen.py [--port /dev/serial/by-id/...] [--baud 115200]
"""

import argparse
import struct
import time

SYNC = b"\x37\x01"
DEFAULT_PORT = "/dev/serial/by-id/usb-Raspberry_Pi_Pico_E6635C08CB725C2C-if00"


def fletcher16(data):
    sum1 = sum2 = 0
    for byte in data:
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
    return (sum2 << 8) | sum1


def packets(buffer):
    """Complete packets at the front of buffer, and what is left over.

    A packet is sync, class, subclass, little-endian payload length, payload,
    and a Fletcher-16 checksum over everything after the sync bytes.
    """
    found = []
    while True:
        start = buffer.find(SYNC)
        if start < 0:
            return found, buffer[-1:]
        buffer = buffer[start:]
        if len(buffer) < 6:
            return found, buffer
        class_id, subclass_id, length = struct.unpack_from("<BBH", buffer, 2)
        end = 6 + length + 2
        if len(buffer) < end:
            return found, buffer
        body = buffer[2 : 6 + length]
        (checksum,) = struct.unpack_from("<H", buffer, 6 + length)
        found.append(
            (
                class_id,
                subclass_id,
                buffer[6 : 6 + length],
                checksum == fletcher16(body),
            ),
        )
        buffer = buffer[end:]


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    import serial

    port = serial.Serial(args.port, args.baud, timeout=0.1)
    print(f"listening on {args.port}; press the kill switches now (Ctrl-C to stop)")
    start = time.monotonic()
    buffer = b""
    try:
        while True:
            buffer += port.read(256)
            found, buffer = packets(buffer)
            for class_id, subclass_id, payload, ok in found:
                print(
                    f"{time.monotonic() - start:8.2f} s  class 0x{class_id:02x}"
                    f"  subclass 0x{subclass_id:02x}  payload [{payload.hex(' ')}]"
                    + ("" if ok else "  BAD CHECKSUM"),
                    flush=True,
                )
    except KeyboardInterrupt:
        pass
    finally:
        port.close()


if __name__ == "__main__":
    main()
