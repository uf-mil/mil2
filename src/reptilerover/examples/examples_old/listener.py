import lgpio
import time

# gpio example not used for anything

# Define the GPIO pin number
GPIO_PIN = 4  # Change this to the desired GPIO pin number
              # gpio number not pin number

# Open the GPIO chip (usually chip 0 is used on the Raspberry Pi)
chip = lgpio.gpiochip_open(0)

# Set the GPIO pin as an output
lgpio.gpio_claim_output(chip, GPIO_PIN)

while True:

    # Output a HIGH signal (1) to the pin
    lgpio.gpio_write(chip, GPIO_PIN, 1)
    print(f"Output 1 on GPIO pin {GPIO_PIN}")

    # Wait for 2 seconds
    time.sleep(2)

    # Output a LOW signal (0) to the pin
    lgpio.gpio_write(chip, GPIO_PIN, 0)
    print(f"Output 0 on GPIO pin {GPIO_PIN}")

    if input("a:") == 'q':
        break

# Close the GPIO chip
lgpio.gpiochip_close(chip)
