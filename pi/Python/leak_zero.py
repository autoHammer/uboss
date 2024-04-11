from gpiozero import Button
from pymavlink import mavutil
import time

LEAK_GPIO_PIN = 10

LEAK = Button(LEAK_GPIO_PIN, pull_up=False)

boot_time = time.time()

# Create the connection to the top-side computer as companion computer/autopilot
master = mavutil.mavlink_connection('udpout:192.168.2.1:14550', source_system=1)

# Send a message for QGC to read out loud
#  Severity from https://mavlink.io/en/messages/common.html#MAV_SEVERITY
master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                           "Water detection enabled".encode())


def water_detected():
    print('\033[31m' + "CRITICAL WARNING: WATER DETECTED!" + '\033[39m')  # red color and back to normal

    global boot_time
    global master
    # Send a message for QGC to read out loud
    #  Severity from https://mavlink.io/en/messages/common.html#MAV_SEVERITY
    master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                               "Warning: Leak detected!".encode())


LEAK.when_pressed = water_detected

print("Leak detection is enabled.")
# keep program alive
while True:
    time.sleep(1)
    # TODO: heartbeat

