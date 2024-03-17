from gpiozero import Button
from time import sleep

LEAK_GPIO_PIN = 10

LEAK = Button(LEAK_GPIO_PIN, pull_up=False)


def print_water_detected():
    print('\033[31m' + "CRITICAL WARNING: WATER DETECTED!" + '\033[39m')  # red color and back to normal


LEAK.when_pressed = print_water_detected

while True:
    sleep(1)
