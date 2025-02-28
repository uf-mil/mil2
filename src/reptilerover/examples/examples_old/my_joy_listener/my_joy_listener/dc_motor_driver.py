from my_joy_listener.PWM_chip_driver import PCA9685
import lgpio

chip = lgpio.gpiochip_open(0)
PWM_FREQ = 50
pwm = PCA9685(0x40, debug=False) # lowk this 0x40 number is non-negoicalble (I2C addy of PWM board)
pwm.setPWMFreq(PWM_FREQ) # 50 MHz ??

class DCMotor:
    '''
    +------------+------------+------------------+
    | gpio_one   | gpio_two   | motor            |
    +------------+------------+------------------+
    | 0          | 0          | motor braking    |
    | 1          | 1          | floating         |
    | 1          | 0          | forward          |
    | 0          | 1          | reverse          |
    +------------+------------+------------------+

    the PWM will determine the speed of rotation
    '''

    def __init__(self, channel, gpio_one, gpio_two, debug=False):
        '''
        channel is an int from 0-15
        gpio_one and gpio_two and GPIO pins on the PI4 (use GPIO number not pin number)
        '''
        assert channel >= 0, "bad channel"
        assert channel <= 15, "bad channel"
        self.channel = channel
        self.gpio_one = gpio_one
        self.gpio_two = gpio_two
        self.debug = debug

        self.debug_msg(f"claiming {self.gpio_one}")
        lgpio.gpio_claim_output(chip, self.gpio_one)

        self.debug_msg(f"claiming {self.gpio_two}")
        lgpio.gpio_claim_output(chip, self.gpio_two)

    def debug_msg(self, msg):
        if self.debug:
            # TODO change to ROS2 version of printing
            console.log(msg)

    def go_forward(self):
        lgpio.gpio_write(chip, self.gpio_one, 1)
        lgpio.gpio_write(chip, self.gpio_two, 0)

    def go_backwards(self):
        lgpio.gpio_write(chip, self.gpio_one, 0)
        lgpio.gpio_write(chip, self.gpio_two, 1)

    def force_break(self):
        lgpio.gpio_write(chip, self.gpio_one, 0)
        lgpio.gpio_write(chip, self.gpio_two, 0)

    def set_speed(self, pwm_percent):
        pwm_percent = abs(pwm_percent)

        assert pwm_percent <= 1.0, "pwm greater than 1"

        pwm.set_pwm_percent(self.channel, pwm_percent)

    
if __name__ == '__main__':
    a_dc_motor = DCMotor(0, 20, 21)

    a_dc_motor.go_forward()
    grain_size = 1000
    for i in range(grain_size):
        a_dc_motor.set_speed((i+1)/grain_size)
