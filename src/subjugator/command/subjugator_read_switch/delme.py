import time

import Jetson.GPIO as GPIO

# Try with the correct GPIO number for pin 32
gpio_num = 18  # This should be PG.07 if pin 32 maps to it


def main():
    GPIO.setmode(GPIO.BCM)  # Use BCM numbering
    GPIO.setup(gpio_num, GPIO.IN)

    print(f"Starting GPIO monitor on BCM pin {gpio_num}. Press CTRL+C to exit")
    # prev_value = None

    try:
        while True:
            value = GPIO.input(gpio_num)
            print(f"Current value: {value}")  # Always print the value
            time.sleep(0.5)
    finally:
        GPIO.cleanup()


if __name__ == "__main__":
    main()
