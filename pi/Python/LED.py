from time import sleep
from gpiozero import PWMLED


def map(min_in, max_in, min_out, max_out, value):
    shifted_value = min_out + (value - min_in) / (max_in - min_in) * (max_out - min_out)
    return shifted_value


camera = PWMLED(pin=21, frequency=150)

print("Welcome to light control center. Press Ctrl+C to exit.")

while True:
    light_level = float(input("Write light percentage (0-100): "))
    duty_cycle = map(0, 100, 0.17, 0.27, light_level)
    camera.value = duty_cycle
    sleep(0.5)

