"""
This code is detecting leaks and will send emergency messages to Qgroundcontrol.
NOTE: This script is required to run as a root user (SUDO)
"""

from gpiozero import Button
from pymavlink import mavutil
from time import sleep

LEAK_GPIO_PIN = 10
LEAK = Button(LEAK_GPIO_PIN, pull_up=False)

link_out = mavutil.mavlink_connection('udpout:192.168.2.1:14550', source_system=1)

link_out.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
                           "Water detection enabled".encode())


def water_detected():
    print('\033[31m' + "CRITICAL WARNING: WATER DETECTED!" + '\033[39m')  # red color and back to normal

    global link_out
    link_out.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_CRITICAL,
                               "Warning: Leak detected!".encode())


LEAK.when_pressed = water_detected

print("Leak detection is enabled.")
# keep program alive
while True:
    sleep(1)
    link_out.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
