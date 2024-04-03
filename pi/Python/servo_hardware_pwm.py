from rpi_hardware_pwm import HardwarePWM
from time import sleep


class Servo:
    """
    A class for servo signal control

    Lets the user write to a servo using custom input numbers instead of duty cycle.
    The class uses hardware pwm instead of standard software pwm.
    Only works on raspberry pi 5.
    The raspberry pi must be configured to enable hardware pwm.
    https://pypi.org/project/rpi-hardware-pwm/

    Args:
        gpio_pin (int): GPIO pin number. Only GPIO pin 12, 13, 18 and 19 is allowed.
    """

    _FREQUENCY = 50
    _PIN_TO_CHANNEL = {12: 0, 13: 1, 18: 2, 19: 3}  # map the pin number into pwm hardware channels

    _prev_duty_cycle = 7.5

    # choose what input range the write command will use
    _min_in = 0
    _max_in = 100

    # calibrate these values to get the full range of the servo
    _min_out = 5.5  # corresponds to 1100 us pulse width
    _max_out = 9.5  # corresponds to 19000 us pulse width

    # Calibrate to get correct center position
    _offset = 0

    def __init__(self, gpio_pin):
        if gpio_pin not in self._PIN_TO_CHANNEL:
            raise ValueError("Servo Error: Wrong GPIO pin is selected. Only GPIO 12, 13, 18, and 19 can be used!")

        channel = self._PIN_TO_CHANNEL[gpio_pin]

        self._pwm = HardwarePWM(pwm_channel=channel, hz=self._FREQUENCY, chip=2)
        self._pwm.start(7.5)  # starting duty cycle, in middle position (1500us)

    @staticmethod
    def map(min_in, max_in, min_out, max_out, value):
        """
        Convert a value to another scale

        Args:
            min_in: lowest input value
            max_in: highest input value
            min_out: lowest input value
            max_out: highest input value
            value: the number to be changed

        Returns: converted value

        """
        value = min(max(value, min_in), max_in)
        shifted_value = min_out + (value - min_in) / (max_in - min_in) * (max_out - min_out)
        return shifted_value

    def write(self, inn):
        """
        write to the servo
        Args:
            inn: input number, (0 to 100 with standard settings)

        Returns: None

        """
        value = inn + self._offset  # add offset calibration to get correct center
        duty_cycle = Servo.map(self._min_in, self._max_in, self._min_out, self._max_out, value)
        self._pwm.change_duty_cycle(duty_cycle)

    def write_smooth(self, inn, tolerance=0.02, delay=0.005):

        value = inn + self._offset  # add offset calibration to get correct center
        duty_cycle = Servo.map(self._min_in, self._max_in, self._min_out, self._max_out, value)

        while True:
            error = duty_cycle - self._prev_duty_cycle
            if error >= tolerance:
                self._prev_duty_cycle += tolerance
            elif error <= -tolerance:
                self._prev_duty_cycle -= tolerance
            else:
                break
            self._pwm.change_duty_cycle(self._prev_duty_cycle)
            print(self._prev_duty_cycle)
            sleep(delay)


    def __del__(self):
        """
        Stop and close the hardware pwm
        """
        self._pwm.stop()


if __name__ == '__main__':
    # motor setup
    motor = Servo(18)
    motor._min_in = -100  # control motor with -100 to 100 (percentage thrust)
    motor._offset = 1
    motor.write(0)

    # camera tilt setup
    camera = Servo(19)
    # control servo with -90 to 90 (degrees)
    camera._min_in = -90
    camera._max_in = 90
    camera._min_out = 2.5
    camera._max_out = 11.7
    camera.write(100)

    #motor.write(50)

    while True:
        value = float(input("value:"))
        duty_cycle = Servo.map(0, 100, 2.5, 11.7, value)
        motor.write(value)
        sleep(0.5)

'''
    motor.write(0)
    light.write(100)

    sleep(5)
    try:
        #while a<1:
        for i in range(0, 100):
            print(i)
            motor.write(i)
            print("onn")
            sleep(0.5/100)
        sleep(5)
        motor.write(0)
        print("off")
        light.write(-100)
        sleep(1)
    except KeyboardInterrupt:
        print("\nprogram stopped")
    '''