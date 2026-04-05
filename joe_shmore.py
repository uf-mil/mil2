import subprocess
import time

import serial

port = "/dev/ttyV0"
target = "vectornav"
baud = 921600


def sub_online():
    try:
        result = subprocess.run(
            ["lsof", port],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        return result.stdout
    except Exception as e:
        print(f"Error checking port: {e}")


# Run every second
while True:
    if not sub_online():
        with serial.Serial("/dev/ttyV0", baud, timeout=1) as ser:
            ser.reset_input_buffer()
            ser.reset_output_buffer()

    time.sleep(0.5)
