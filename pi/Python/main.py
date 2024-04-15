from time import sleep
from pymavlink import mavutil
import threading

from Hardware_Interface.servo_hardware_pwm import Servo
from Other.thread_safe_value import ThreadSafeValue


def mavlink_to_hardware_output(stop_event, data):
    """
    Thread for fetching mavlink commands to control motor, servo and LED
    """

    ''' mavlink setup'''
    link = mavutil.mavlink_connection("udp:0.0.0.0:14550") # TODO: updin - test

    link.wait_heartbeat()

    # send initial motor value
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # Confirmation
        9,  # Which servo to control
        2500,  # PWM value
        0, 0, 0, 0, 0  # Unused parameters
    )

    # send initial camera tilt value
    link.mav.command_long_send(
        link.target_system,
        link.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
        0,  # Confirmation
        10,  # Which servo to control
        1500,  # PWM value
        0, 0, 0, 0, 0  # Unused parameters
    )

    while not stop_event.is_set():
        message = link.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)
        if message:
            data.set({"camera_tilt": message.servo9_raw,
                      "motor": message.servo10_raw})


def GPIO_interface(stop_event, data):
    """
    Code for interfacing hardware connected to GPIO
    Args:
        stop_event:
        data: Dictionary with keys:
            - "motor":value
            - "camera_tilt":value
    """
    ''' Motor setup '''
    motor = Servo(18)
    motor._min_in = -100  # control motor with -100 to 100 (percentage thrust)
    motor._offset = 1
    motor.write(0)

    ''' Camera tilt setup '''
    camera = Servo(19)
    # control servo with -90 to 90 (degrees)
    camera._min_in = 0
    camera._max_in = 5000
    camera._min_out = 2.5
    camera._max_out = 11.7
    camera.write(2500)

    while not stop_event.is_set():
        if data.has_new_value():

            new_data = data.take()
            if 'motor' in new_data:
                #motor.write(data["motor"].servo10_raw)
                print("motor:", new_data["motor"])

            if "camera_tilt" in new_data:
                #camera.write(data["camera_tilt"].servo9_raw)
                print("camera:", new_data["camera_tilt"])

        sleep(0.1)


def main():
    #leak_setup()

    stop_event = threading.Event()
    data = ThreadSafeValue()

    mavlink_thread = threading.Thread(target=mavlink_to_hardware_output, args=(stop_event, data))
    mavlink_thread.start()

    GPIO_thread = threading.Thread(target=GPIO_interface, args=(stop_event, data))
    GPIO_thread.start()

    leak_thread = threading.Thread(target=leak_setup)

    try:
        while True:
            sleep(1)

    except KeyboardInterrupt:
        print("Exiting program")
        stop_event.set()
        mavlink_thread.join()
        GPIO_thread.join()


if __name__ == '__main__':
    main()

