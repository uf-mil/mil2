"""
This document contains the acoustic_modem class which provides functions for controlling an acoustic modem
"""

# Dependencies
import time

import serial


# Acoustic modem class
class acoustic_modem:

    ########## <Variables> ##########
    # Identifier
    name = ""

    # Verbose output (debugging)
    verbose_output = True

    # Properties for serial communication (private)
    __serial_port = "/dev/ttyACM0"
    __baud_rate = 19200
    __serial_obj = None

    # Control variables (private)
    __modem_initialized = (
        False  # keeps track of whether or not the modem has been initialized
    )
    __guard_time = 1  # time to wait before/after sending +++ to enter command mode
    __serial_timeout = (
        2.0  # timeout for readline, if left blank, serial read would block forever
    )
    __AT_delay_time = (
        0.25  # brief delay to wait between sending instruction and reading res
    )

    # Acting settings (private)
    __acting_settings = {
        "Source Level": "",
        "Source Level Control": "",
        "Gain": "",
        "Carrier Waveform ID": "",
        "Local Address": "",
        "Highest Address": "",
        "Cluster Size": "",
        "Packet Time": "",
        "Retry Count": "",
        "Retry Timeout": "",
        "Wake Up Active Time": "",
        "Wake Up Period": "",
        "Promiscuous Mode": "",
        "Sound Speed": "",
        "IM Retry Count": "",
        "Pool Size": "",
        "Hold Timeout": "",
        "Idle Timeout": "",
        "Remote Address": "",  # remote address is not read by AT&V, must be read separately
    }

    # Acoustic modem settings (private)
    __mode = "DATA"  # "DATA" = data mode, "COMMAND" = command mode
    ########## </Variables> ##########

    ########## <Helper functions (private)> ##########
    # Sends data to the modem
    def __send_data(self, data):

        # encode data to bytes if not yet encoded
        if isinstance(data, str):
            data = data.encode("ascii")

        self.__serial_obj.write(data)
        self.__serial_obj.flush()

    # Reads data from the modem
    def __read_data(self):
        return self.__serial_obj.readline().decode().strip()

    # Reads data from modem that is wrapped in newlines
    def __read_data_multiline(self):
        while True:
            line = self.__read_data()
            if line:
                return line

    # Reads unmodified serial output (debugging)
    def read_raw(self):
        return self.__serial_obj.read(self.__serial_obj.in_waiting)

    ########## </Helper functions (private)> ##########

    ########## <Functions for Init / Deinit> ##########
    # Constructor
    def __init__(self, name, serial_port):
        self.name = name
        self.__serial_port = serial_port

    # Initialize modem
    def init_modem(self):

        # Initialize modem on serial port
        self.__serial_obj = serial.Serial(
            port=self.__serial_port,
            baudrate=self.__baud_rate,
            timeout=self.__serial_timeout,
        )

        print("**********Initializing Modem**********")
        print(
            f"[{self.name}] Serial connection established on {self.__serial_obj.name}",
        )

        # Enter command mode
        if not self.enter_command_mode():
            raise RuntimeError("Unable to enter command mode on modem init")

        # Get settings
        if not self.get_settings():
            raise RuntimeError("Unable to read modem settings on modem init")

        # Enter data mode
        if not self.enter_data_mode():
            raise RuntimeError("Unable to switch modem back to data mode on modem init")

        # Print success message
        print(f"{self.name} initialized!")
        print("**************************************")

        # Update init boolean
        self.__modem_initialized = True

        # Return
        return True

    # Deinitialize modem
    def deinit_serial(self):
        self.__serial_obj.close()

    # Checks if this modem has been initialized
    def check_if_init(self):
        if not self.__modem_initialized:
            print(f"[{self.name}] not yet initialized!")
            return False

        return True

    ########## </Functions for Init / Deinit> ##########

    ########## <Changing device mode> ##########
    def enter_command_mode(self):

        # Print debug message
        print(f"[{self.name}] Attempting to enter command mode...")

        # Send instruction to enter command mode
        time.sleep(self.__guard_time)  # wait guard time
        self.__send_data("+++")  # send instruction to enter command mode
        time.sleep(self.__guard_time)  # wait guard time

        # Read response from modem
        response = self.__read_data()

        # Update modem mode based on result
        if response == "OK":
            print("\t> Successfully entered command mode")
            self.__mode = "COMMAND"
            return True  # return true if init was successful
        else:
            print("\t> [ERROR]: failed to enter command mode")
            return False  # return false if init was unsuccessful

    def enter_data_mode(self):

        # Print debug message
        print(f"[{self.name}] Entering data mode...")

        # Send enter data mode instruction
        self.__send_data("ATO\r")  # send instruction to enter data mode
        time.sleep(self.__AT_delay_time)

        # Check if switch to data mode was successful
        data = self.read_raw()

        if self.verbose_output:
            print(f"\t> Result = {data}")

        if data != b"":
            print(f"[ERROR]: expected b'', got {data}")
            return False

        print("\t> Successfully entered data mode")
        self.__mode = "DATA"
        return True

    ########## </Changing device mode> ##########

    ########## <Functions for accessing / changing settings> ##########

    # Getting acting settings from modem
    def get_settings(self):  # AT&V

        # Print debug message
        print(f"[{self.name}] Getting settings...")

        # Ensure modem is in command mode
        if self.__mode != "COMMAND":
            print("Modem must be in COMMAND mode for this operation!")
            return False

        # Read settings printed out by AT&V instruction
        self.__send_data("AT&V\r")
        time.sleep(self.__AT_delay_time)

        num_settings = 18  # there are 18 settings that AT&V returns

        for i in range(num_settings):
            # read line from serial connection
            data = self.__read_data()

            # separate setting name and setting value
            key, value = data.split(":", 1)
            key = key.strip()
            value = value.strip()

            # check if this is a valid setting
            if key not in self.__acting_settings:
                print("\t> ERROR: {key} is an unrecognized setting")
                return False

            # update setting value in __acting_settngs
            self.__acting_settings[key] = value

        time.sleep(self.__AT_delay_time)

        # Get Remote Address setting
        time.sleep(self.__AT_delay_time)
        self.__send_data("AT?AR\r")
        time.sleep(self.__AT_delay_time)
        data = self.__read_data_multiline()
        self.__acting_settings["Remote Address"] = data

        # If verbose mode is on, print out every read setting
        if self.verbose_output:
            for key, value in self.__acting_settings.items():
                print(f"\t> {key} = {value}")

        # Print out success message
        print("\t> Successfully read modem settings")

        return True

    def get_local_addr(self):  # AT?AL

        # Ensure modem has initialized before continuing
        if not self.check_if_init():
            return None

        # Return local address
        return self.__acting_settings["Local Address"]

    def set_local_addr(self, new_addr):  # AT!AL<#>
        # Print debug message
        print(f"[{self.name}] Changing local address...")

        # Ensure modem has initialized before continuing
        if not self.check_if_init():
            return False

        # Ensure modem is in command mode
        if self.__mode != "COMMAND":
            print("Modem must be in COMMAND mode for this operation!")
            return False

        # [Validate] ensure address is an integer value
        try:
            addr_int = int(new_addr)
        except ValueError:
            print("\t> Invalid input, local addr must be an integer value")
            return False

        # [Validate] ensure new address is [1, max address]
        max_addr = self.__acting_settings["Highest Address"]
        max_addr = int(max_addr)

        if addr_int < 1 or addr_int > max_addr:
            print("\t> [ERROR]: new local address must be less than max address!")
            return False

        # [Validate] ensure new address is different than remote address
        remote_addr = self.__acting_settings["Remote Address"]

        if new_addr == remote_addr:
            print(
                "\t> [ERROR]: new local address must be different than remote address!",
            )
            return False

        # Set new local address
        self.__send_data(f"AT!AL{addr_int}\r")  # send new local addr
        time.sleep(self.__AT_delay_time)  # brief delay

        # Check for 'OK'
        response = self.__read_data()

        if response == "OK":
            print(f"\t> Successfully changed local address to {addr_int}")
            self.__acting_settings["Local Address"] = addr_int
            return True
        else:
            print(f"\t> [ERROR]: failed to change local address to {addr_int}")
            return False

    def get_remote_addr(self):
        # Ensure modem has initialized before continuing
        if not self.check_if_init():
            return None

        # Return remote address
        return self.__acting_settings["Remote Address"]

    def set_remote_addr(self, new_addr):  # AT!AR<#>
        # Print debug message
        print(f"[{self.name}] Changing remote address...")

        # Ensure modem has initialized before continuing
        if not self.check_if_init():
            print("\t> [ERROR]: modem not yet initialized!")
            return False

        # Ensure modem is in command mode
        if self.__mode != "COMMAND":
            print("Modem must be in COMMAND mode for this operation!")
            return False

        # [Validate] ensure address is an integer value
        try:
            addr_int = int(new_addr)
        except ValueError:
            print("\t> Invalid input, remote addr must be an integer value")
            return False

        # [Validate] ensure new address is [1, max address]
        max_addr = self.__acting_settings["Highest Address"]
        max_addr = int(max_addr)

        if addr_int < 1 or addr_int > max_addr:
            print("\t> [ERROR]: new remote address must be less than max address!")
            return False

        # [Validate] ensure new address is different than local address
        local_addr = self.__acting_settings["Local Address"]

        if new_addr == local_addr:
            print(
                "\t> [ERROR]: new remote address must be different than local address!",
            )
            return False

        # Set new remote address
        self.__send_data(f"AT!AR{addr_int}\r")  # send new remote addr
        time.sleep(self.__AT_delay_time)  # brief delay

        # Check for 'OK'
        response = self.__read_data()

        if response == "OK":
            print(f"\t> Successfully changed remote address to {addr_int}")
            self.__acting_settings["Remote Address"] = addr_int
            return True
        else:
            print(f"\t> [ERROR]: failed to change remote address to {addr_int}")
            return False

    def get_max_addr(self):  # AT?AM
        # Ensure modem has initialized before continuing
        if not self.check_if_init():
            return 0

        # Return max address
        return self.__acting_settings["Highest Address"]

    def set_max_addr(self, new_max_addr):  # AT!AM<#>
        # Print debug message
        print(f"[{self.name}] Changing max address...")

        # Ensure modem has initialized before continuing
        if not self.check_if_init():
            return False

        # Ensure modem is in command mode
        if self.__mode != "COMMAND":
            print("]\t> Modem must be in COMMAND mode for this operation!")
            return False

        # Validate max addr
        try:
            addr_int = int(new_max_addr)
        except ValueError:
            print("\t> Invalid input, max addr must be an integer value")
            return False

        allowed_values = {2, 6, 14, 30, 62, 126, 254}
        if addr_int not in allowed_values:
            print("\t> Invalid max address! Must be 2, 6, 14, 30, 62, 126, or 254.")
            return False

        # Send new max address
        self.__send_data(f"AT!AM{new_max_addr}\r")  # send new max addr
        time.sleep(self.__AT_delay_time)  # brief delay

        # Check for 'OK'
        response = self.__read_data()

        if response == "OK":
            print(f"\t> Successfully changed new max address to {new_max_addr}")
            self.__acting_settings["Highest Address"] = new_max_addr
            return True
        else:
            print(f"\t> [ERROR]: failed to change new max address to {new_max_addr}")
            return False

    ########## </Functions for accessing / changing settings> ##########

    ########## <Receiving and transmitting data> ##########
    def transmit(self, data):

        print(f"[{self.name}] Transmitting data...")

        # Ensure modem has initialized before continuing
        if not self.check_if_init():
            return False

        # Ensure modem is in data mode
        if self.__mode != "DATA":
            print("]\t> Modem must be in DATA mode for this operation!")
            return False

        # Transmit data
        self.__send_data(data)

        # Print debug message
        print(f"\t> Successfully transmitted <{data}>")

    ########## </Receiving and transmitting data> ##########

    # IMs

    def read_im(self):
        im_message = None

        while im_message is None:
            data = self.__read_data()
            if data.startswith("RECVIM"):
                im_message = data.split(",")[-1]

        return im_message

    ########## <Nonvolatile operations> ##########
    # Saves currently-saved settings to non-volatile memory
    def save_settings(self):
        self.__send_data("AT&W\r")  # send instruction to store settings

    # Resets modem settings to factory defaults
    def reset_modem(self):
        self.__send_data("AT&F\r")  # send instruction to reset to factory defaults


########## </Nonvolatile operations> ##########
