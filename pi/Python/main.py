from time import sleep
from pymavlink import mavutil
import threading

from Hardware_Interface.servo_hardware_pwm import Servo
from Hardware_Interface.IMU import mavlink_IMU_input
from Hardware_Interface.ping_logger import ping_distance
from Other.thread_safe_value import ThreadSafeValue
from camera_streamer import *
from Other.kalman import *
gi.require_version('Gst', '1.0')


def mavlink_thruster_control(stop_event, thruster_data):
    link_out = mavutil.mavlink_connection('udpin:192.168.2.3:14550')
    link_out.wait_heartbeat()

    while not stop_event.is_set():
        if thruster_data.has_new_value():
            forward = thruster_data.take()["y"]
            lateral = thruster_data.take()["x"]
            print("Sending RC override")
            link_out.mav.rc_channels_override_send(
                link_out.target_system,
                link_out.target_component,
                65535, 65535, 65535, 65535,
                forward,
                lateral,
                65535, 65535
            )
        sleep(0.05)  # 20hz


def mavlink_servo_input(stop_event, servo_data):
    """
    Thread for fetching mavlink commands to control motor, servo and LED
    """

    ''' mavlink setup'''
    link = mavutil.mavlink_connection("udp:0.0.0.0:14550")

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
        message = link.recv_match(type='SERVO_OUTPUT_RAW', blocking=True)  # TODO: add timeout
        if message:
            servo_data.set({"camera_tilt": message.servo9_raw,
                      "motor": message.servo10_raw})


def GPIO_interface(stop_event, servo_data):
    """
    Code for interfacing hardware connected to GPIO
    Args:
        stop_event:
        servo_data: Dictionary with keys:
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
        if servo_data.has_new_value():

            new_data = servo_data.take()
            if 'motor' in new_data:
                #motor.write(data["motor"].servo10_raw)
                print("motor:", new_data["motor"])

            if "camera_tilt" in new_data:
                #camera.write(data["camera_tilt"].servo9_raw)
                print("camera:", new_data["camera_tilt"])

        sleep(0.1)


def calculate_depth(pressure_abs):
    """
    Calculate depth based on absoulute pressure
    Args:
        pressure_abs: pressure in hPa (hecto Pascal)

    Returns: depth in meter
    """
    SALTWATER_DENSITY = 1025  # kg/m^3
    GRAVITY = 9.81  # m/s^2
    ATMOSPHERE_PRESSURE = 1013  # hPa

    pressure = pressure_abs - ATMOSPHERE_PRESSURE

    depth = (pressure * 100) / (SALTWATER_DENSITY * GRAVITY)  # m

    return depth


def mavlink_depth(stop_event, distance_data):
    link_in = mavutil.mavlink_connection('udp:192.168.2.3:14553')
    link_in.wait_heartbeat()
    print("Depth online")

    # initial data
    distance_data.set({"max_depth": 0})

    while not stop_event.is_set():
        message = link_in.recv_match(type='SCALED_PRESSURE2', blocking=True, timeout=2)
        if message:
            prev_data = distance_data.take()

            pressure = message.press_abs

            depth = calculate_depth(pressure)

            if depth > prev_data["max_depth"]:
                max_depth = depth
            else:
                max_depth = prev_data["max_depth"]

            distance = max_depth - depth

            distance_data.set({"pressure": pressure,
                               "depth": depth,
                               "max_depth": max_depth,
                               "distance": distance})

        else:
            print("mavlink depth timeout. Waiting for connection . . .")
            link_in.wait_heartbeat()
            print("mavlink depth reconnected")


def user_input(stop_event, PID_data, distance_data):
    while not stop_event.is_set():
        print("1. Restart max_depth\n"
              "2. Change PID parameters")
        command = input()

        if command == "1":
            distance_data.set({"max_depth": 0})
            print("max_depth set to 0")

        elif command == "2":
            alternatives = ["x_kp", "x_ki", "x_kd", "y_kp", "y_ki", "y_kd", "z_kp", "z_ki", "z_kd"]

            PID = PID_data.take()

            try:
                line = "\nChoose which parameter to change\n"
                for i in range(len(alternatives)):
                    line = line + f"{i}. {alternatives[i]}={PID[alternatives[i]]}\n"
            except TypeError:
                line = "\nChoose which parameter to change\n"
                for i in range(len(alternatives)):
                    line = line + f"{i}. {alternatives[i]}\n"
            try:
                chosen_parameter = alternatives[int(input(line))]
                chosen_value = input("value: ")
                PID[chosen_parameter] = chosen_value
                PID_data.set(PID)

            except (ValueError, IndexError):
                print(f"Only give number input (INT) in the range 0 to {len(alternatives)}")
            except TypeError:
                print("PID not initialized yet!")
            print("data: ", PID_data.take())

        else:
            print("invalid input")


def autopilot_handler(stop_event):
    link_in = mavutil.mavlink_connection("udp:0.0.0.0:14554")
    link_in.wait_heartbeat()

    while not stop_event.is_set():
        message = link_in.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
        if message:
            auto_btn = message.servo11_raw
            print("message: ", auto_btn)

            if auto_btn > 1500:
                autopilot = True
            else:
                autopilot = False
        else:
            print("mavlink autopilot timeout")

        sleep(0.1)
    return 0


def main():

    stop_event = threading.Event()
    servo_data = ThreadSafeValue()

    mavlink_thread = threading.Thread(target=mavlink_servo_input, args=(stop_event, servo_data))
    #mavlink_thread.start()

    GPIO_thread = threading.Thread(target=GPIO_interface, args=(stop_event, servo_data))
    #GPIO_thread.start()

    IMU_data = ThreadSafeValue()
    IMU_thread = threading.Thread(target=mavlink_IMU_input, args=(stop_event, IMU_data,))
    IMU_thread.start()

    thruster_data = ThreadSafeValue()
    thruster_thread = threading.Thread(target=mavlink_thruster_control, args=(stop_event, thruster_data,))
    #thruster_thread.start()

    ping_thread = threading.Thread(target=ping_distance, args=(stop_event,))
    #ping_thread.start()

    distance_data = ThreadSafeValue()
    depth_thread = threading.Thread(target=mavlink_depth, args=(stop_event,distance_data,))
    depth_thread.start()

    pid_parameters = ThreadSafeValue()
    user_thread = threading.Thread(target=user_input, args=(stop_event, pid_parameters, distance_data,))
    user_thread.start()

    prediction_data = ThreadSafeValue()
    video_streamer = VideoStreamer(stop_event, "camera_capture", predictions=prediction_data)
    camera_streamer_thread = threading.Thread(target=video_streamer.video_streaming_thread, args=())
    #camera_streamer_thread.start()  # TODO: add

    # Distance measurement should be inserted with desired units here (same units as IMU_data)
    #distance_data = 102  # 102cm distance sensor measurement
    controller_output = ThreadSafeValue()
    controller = threading.Thread(target=controller_thread, args=(stop_event, IMU_data,
                                                                  prediction_data, distance_data,
                                                                  pid_parameters, controller_output,))
    controller.start()

    auto_thread = threading.Thread(target=autopilot_handler, args=(stop_event,))
    auto_thread.start()

    sleep(10)  #TODO: remove
    #print("after 10s", flush=True)
    camera_streamer_thread.start()  # TODO: remove
    try:
        while True:
            if IMU_data.has_new_value():
                pass#print("IMU_data:", IMU_data.take())
            if distance_data.has_new_value():
                pass#print("distance_data:", distance_data.take())

            sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping program. . .")
        video_streamer.stop()
        stop_event.set()
        mavlink_thread.join()
        GPIO_thread.join()
        IMU_thread.join()
        ping_thread.join()
        depth_thread.join()
        user_thread.join()
        camera_streamer_thread.join()
        controller.join()
        auto_thread.join()
        print("Program closed successfully.")


if __name__ == '__main__':
    main()
