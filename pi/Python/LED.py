from time import sleep
from gpiozero import PWMLED
from pymavlink import mavutil


def map(min_in, max_in, min_out, max_out, value):
    shifted_value = min_out + (value - min_in) / (max_in - min_in) * (max_out - min_out)
    return shifted_value


camera = PWMLED(pin=21, frequency=50)

print("Welcome to light control center. Press Ctrl+C to exit.")

link_in = mavutil.mavlink_connection("udp:0.0.0.0:14556")

link_in.wait_heartbeat()

while True:
    message = link_in.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)  # TODO: add timeout
    if message:
        light_level = message.servo13_raw
        duty_cycle = map(1100, 1900, 0.058, 0.091, light_level)
        camera.value = duty_cycle
        print(duty_cycle)
    else:
        print("mavlink timeout")
    sleep(0.5)

