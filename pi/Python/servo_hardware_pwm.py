from rpi_hardware_pwm import HardwarePWM
from time import sleep


class Servo:
    """
    A class for servo signal control

    Lets the user write to a servo using -100 to 100 numbers instead of duty cycle.
    The class uses hardware pwm instead of standard software pwm.
    Only works on raspberry pi 5.
    The raspberry pi must be configured to enable hardware pwm.
    https://pypi.org/project/rpi-hardware-pwm/

    Args:
        gpio_pin (int): GPIO pin number. Only GPIO pin 12, 13, 18 and 19 is allowed.
    """

    _FREQUENCY = 50
    _PIN_TO_CHANNEL = {12: 0, 13: 1, 18: 2, 19: 3}  # map the pin number into pwm hardware channels

    def __init__(self, gpio_pin):
        if gpio_pin not in self._PIN_TO_CHANNEL:
            raise ValueError("Servo Error: Wrong GPIO pin is selected. Only GPIO 12, 13, 18, and 19 can be used!")

        channel = self._PIN_TO_CHANNEL[gpio_pin]

        self._pwm = HardwarePWM(pwm_channel=channel, hz=self._FREQUENCY, chip=2)
        self._pwm.start(7.5)  # starting duty cycle, in middle position (1500us)

    def write(self, percentage):
        """
        Converts the percentage (from -100% to 100%) into pwm duty cycle.
        Writs the value

        Args:
            percentage (float):
        """
        percentage = min(max(percentage, -100), 100)

        # convert value from -100, 100 to 1100, 1900. (pulse width in us)
        max_pulse_width = 1900  # us
        min_pulse_width = 1100
        pulse_width = min_pulse_width + (percentage+100)/(100+100) * (max_pulse_width-min_pulse_width)

        T = 1 / self._FREQUENCY  # period
        duty_cycle = pulse_width / (T * 10000)

        self._pwm.change_duty_cycle(duty_cycle)

    def __del__(self):
        """
        Stop the hardware pwm
        """
        self._pwm.stop()


if __name__ == '__main__':
    motor = Servo(18)
    try:
        while True:
            motor.write(20)
            print("on")
            sleep(1)
            motor.write(0)
            print("off")
            sleep(1)
    except KeyboardInterrupt:
        print("\nprogram stopped")
