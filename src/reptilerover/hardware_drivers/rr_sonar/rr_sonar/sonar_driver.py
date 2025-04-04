import time
import datetime
# import threading
import lgpio

# in seconds
TIME_BETWEEN_SCANS = 1.0*10**-3

claimed = []

chip = lgpio.gpiochip_open(0)

# TODO having each node make their own of this is lowk a terrible idea
# Open the GPIO chip (usually chip 0 is used on the Raspberry Pi)
# this sonar class will only send pulses once every milisecond
class Sonar:
    def __init__(self, trigger_pin, echo_pin):
        self.trigger_pin = trigger_pin
        self.echo_pin = echo_pin
        self.time_of_last_reading = datetime.datetime.now()
        #self.mutex =threading.Lock()

        # Set the GPIO pins as an output
        #self.mutex.acquire()
        lgpio.gpio_claim_output(chip, self.trigger_pin)
        lgpio.gpio_claim_input(chip, self.echo_pin)
        #self.mutex.release()

    def gpio_write(self, pin, level):
        # is this bad TODO
        #self.mutex.acquire()
        if not (pin in claimed):
            lgpio.gpio_claim_output(chip, pin)
            claimed.append(pin)
        lgpio.gpio_write(chip, pin, level)
        #self.mutex.release()


    # will always return a float, float will be -1.0 if no measurement could be made
    def single_measure(self):
        # first we are gonna check to see if 
        # enough time has passed between measurements
        delta_t_seconds = (datetime.datetime.now() - self.time_of_last_reading).total_seconds()
        if(delta_t_seconds < TIME_BETWEEN_SCANS):
            return -1.0

        self.send_pulse()
        resp_in_delta = self.read_response()
        if resp_in_delta == -1.0:
            return -1.0
        resp_in_seconds = resp_in_delta.total_seconds() * 10**6
        distance_cm = (resp_in_seconds*0.0343) /2

        self.time_of_last_reading = datetime.datetime.now()

        return distance_cm

    # returns a datetime.datetime deltatime or something similar
    def read_response(self):
        #self.mutex.acquire()
        start_of_func = datetime.datetime.now()
        while lgpio.gpio_read(chip, self.echo_pin) == 0:
            time_spend_waiting = datetime.datetime.now() - start_of_func
            if time_spend_waiting.total_seconds() > 3*10**-3:
                print('one')
                return -1.0

        before = datetime.datetime.now()

        while lgpio.gpio_read(chip, self.echo_pin) == 1:
            time_spend_waiting = datetime.datetime.now() - start_of_func
            if time_spend_waiting.total_seconds() > 10*10**-3:
                print("two")
                return -1.0

        after = datetime.datetime.now()

        #self.mutex.release()
        return after-before

    def send_pulse(self):
        #self.mutex.acquire()
        lgpio.gpio_write(chip, self.trigger_pin, 1)
        time.sleep(10 * 10**-6)
        lgpio.gpio_write(chip, self.trigger_pin, 0)
        #self.mutex.release()

if __name__ == '__main__':
    sonar =Sonar(20, 21)
    while True:
        print(sonar.single_measure())
    lgpio.gpiochip_close(chip)

