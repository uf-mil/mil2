#!/usr/bin/env python3

"""Code meant to connect to servo tube and do test with"""

import struct
import serial

#define MAX_FWR_VAL 1900 // us
#define MAX_REV_VAL 1100 // us
#define STOP_VAL 1500 // us
MAX_FWR_VAL = 1900
MAX_REV_VAL = 1100
STOPVAL = 1500
# ---------------------------------------------------------------------------- #
#                                   FUNCTIONS                                  #
# ---------------------------------------------------------------------------- #


def fletcher16(data: bytes):
    """Compute the Fletcher-16 checksum of a byte sequence."""
    sum1 = 0
    sum2 = 0
    for index, byte in enumerate(data):
        sum1 = (sum1 + byte) % 255
        sum2 = (sum2 + sum1) % 255
        # print(f"Data {index}: {byte:#02x}")
        # print(f"sum1: {sum1}")
        # print(f"sum2: {sum2}")

    print(sum2, sum1)
    return (sum2 << 8) | sum1


def make_msg(class_id: int, subclass_id: int, payload: bytes = b'') -> bytes:
    """
    Create a message packet to send via usb.

    Inputs:
    - class_id (int): the device that it is being sent to (check uf-mil.github.io page or USB-TO-CAN.H)
    - subclass_id (int): what operation is the device doing (check the github.io or .h file)
    - playload (byte): message being sent (again check the github.io or .h file)

    Returns
    - bytes: full message byte array
    """
    sync = b'\x37\x01'
    cid_byte = struct.pack('<B', class_id)
    scid_byte = struct.pack('<B', subclass_id)
    length = struct.pack('<H', len(payload))

    # get the current msg to find checksum
    data1 = cid_byte + scid_byte + length + payload

    checksum = fletcher16(data1)
    checksum_bytes = struct.pack('<H', checksum)

    data = sync + data1 + checksum_bytes

    return data


def msg_to_string(msg):
    """Convert a message to a string to print."""
    return ' '.join(f'{byte:02x}' for byte in msg)

def map(x: int, in_min:int, in_max: int, out_min: int, out_max:int ) -> int:
    return int((x-in_min)*(out_max-out_min)/ (in_max-in_min) + out_min)

def get_servo_duty(speed: float) -> int:
    speed = max(-1, min(1, speed))
    
    if speed == 0:
        return STOPVAL
    
    speed_int = int(speed*100)

    return map(speed_int, -100, 100, MAX_REV_VAL, MAX_FWR_VAL)

# ---------------------------------------------------------------------------- #
#                                     MAIN                                     #
# ---------------------------------------------------------------------------- #

# Connect to device
# ON WINDOWS, CHANGE THIS TO COM# , /dev/ is chad linux
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# pre-declared variables cause I am lazy
class_id = 0x02
subclass_id = 0x00

# -------------------------------- subclasses -------------------------------- #
acceptable_subclasses = [0, 1, 2, 69]
acceptable_id = range(0,8)
help_id = 69


help_msg =\
    f"""
Sub 9 Thruster Controller
Class ID: {class_id}
Subclasses:
    - 0x00 : Heart Beat Set (DO NOT USE, NOT IMPLEMENTED YET)
    - 0x01 : Heart Beat Received (DO NOT USE) [NOT IMPLEMENTED YET, BUT IS MEANT TO BE SENT BY PICO, NOT COMPUTER]
    - 0x02 : Thruster Set
    - 0x03 : Thruster Kill Set
    - 0x04 : Thruster Kill Recieved (DO NOT USE) [NOT IMPLEMENTED YET, BUT IS MEANT TO BE SENT BY PICO, NOT COMPUTER]

To repeat this message use subclass id 69
    """

print("Starting Temp RF Kill Board test file...\n", help_msg)

# main loop
while True:
    try:
        subclass_id = int(input("Enter id (type 69 for help): "))

        while subclass_id not in acceptable_subclasses:
            subclass_id = int(input("Invalid option. Enter id (69 for help): "))

        if subclass_id == 0:

            print("Not impemented yet...")
            continue
            # toKill = int(input("Do you want to kill (1) or unkill (0): "))

            # while toKill not in acceptable_kill:
            #     toKill = int(input("Invalid option, Kill (1) or Unkill (0): "))

            # if toKill:
            #     payload = struct.pack("<?B", True, 0)
            # else:
            #     payload = struct.pack("<?B", False, 0)

            # msg = make_msg(class_id, subclass_id, payload)

            # print(f"Message send: {msg_to_string(msg)}")
            # ser.write(msg)

        elif subclass_id == 2:
            thruster_id = int(input("Enter thruster ID [0-7], type 420 for more info: "))

            while thruster_id not in acceptable_id:
                if thruster_id == 420:
                    print("The ids correspond to the following...\nFLH = 0,\nFRH = 1,\nFLV = 2,\nFRV = 3,\nBLH = 4,\nBRH = 5,\nBLV = 6,\nBRV = 7")
                    thruster_id = int(input("Enter thruster ID [0-7], type 420 for more info: "))
                else:
                    thruster_id = int(input("Invalid option, enter thruster ID [0-7], type 420 for more info: "))

            speed = float(input("Enter a speed [-1,1]: "))

            while abs(speed) > 1:
                speed = float(input("Invalid option, enter a speed [-1,1]: "))

            print("The speed is: ", speed)

            payload = struct.pack("<Bf", thruster_id, speed)

            msg = make_msg(class_id, subclass_id, payload)

            print(f"Message send: {msg_to_string(msg)}. The PWM would be: {get_servo_duty(speed)}")
            ser.write(msg)

        elif subclass_id == help_id:
            print(help_msg)
            continue

        # check if ack or nak
        response = ser.read(1000)
        print(f"Receieved the message {msg_to_string(response)}")

    except KeyboardInterrupt:
        print("\nKeyboard Interrupt has occured! Ending program now...")
        break
    except Exception as e:
        print(e)

ser.read(1000)  # read any messages that were recieved
ser.close()
