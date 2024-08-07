from time import sleep
from pymavlink import mavutil
import threading

from Hardware_Interface.servo_hardware_pwm import Servo
from Hardware_Interface.IMU import mavlink_IMU_input
from Hardware_Interface.ping_logger import ping_distance
from Other.thread_safe_value import ThreadSafeValue
from camera_streamer import *
from Other.kalman import *


def mavlink_thruster_control(stop_event, thruster_data, autopilot_enable_event):
    link_out = mavutil.mavlink_connection('udpin:192.168.2.3:14550')
    link_out.wait_heartbeat()

    while not stop_event.is_set():
        if thruster_data.has_new_value() and autopilot_enable_event.is_set():
            forward = int(thruster_data.take()["y"])
            lateral = int(thruster_data.take()["x"])
            vertical = int(thruster_data.take()["z"])
            print(f"{threading.current_thread().name}: Sending RC override: \n"
                  f"Forward: {forward}\n"
                  f"Lateral: {lateral}")
            link_out.mav.rc_channels_override_send(
                link_out.target_system,
                link_out.target_component,
                65535, 65535,
                vertical,
                65535,
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
    link = mavutil.mavlink_connection("udp:0.0.0.0:14552")

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
        message = link.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
        if message:
            servo_data.set({"camera_tilt": message.servo9_raw,
                            "motor": message.servo10_raw})
        else:
            print("Mavlink servo timout")


def GPIO_interface(stop_event, servo_data):
    """
    Code for interfacing hardware connected to GPIO
    Args:
        stop_event:
        servo_data: Dictionary with keys:
            - "motor":value
            - "camera_tilt":value
    """
    # TODO: Only move motor when Ardusub is armed.

    ''' Motor setup '''
    motor = Servo(18)
    motor._min_in = 1100  # control motor with -100 to 100 (percentage thrust)
    motor._max_in = 1900
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
            #print("new_data: ", new_data)
            if 'motor' in new_data:
                motor.write_smooth(new_data["motor"])

            if "camera_tilt" in new_data:
                camera.write(new_data["camera_tilt"])

        sleep(0.1)


def calculate_depth(pressure_abs):
    """
    Calculate depth based on absolute pressure
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


def user_input(stop_event, PID_data, distance_data, enable_object_detection_event):
    while not stop_event.is_set():
        print("1. Restart max_depth\n"
              "2. Change PID parameters\n"
              "3. Toggle object detection")
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
            print("data: ", PID)

        elif command == "3":
            if enable_object_detection_event.is_set():
                enable_object_detection_event.clear()
                print("Object detection: Offline")
            else:
                enable_object_detection_event.set()
                print("Object detection: Online")

        else:
            print("invalid input")


def autopilot_handler(stop_event, autopilot_enable_event):
    link_in = mavutil.mavlink_connection("udp:0.0.0.0:14554")
    link_in.wait_heartbeat()

    link_out = mavutil.mavlink_connection('udpout:192.168.2.1:14550', source_system=1)

    autopilot = False

    while not stop_event.is_set():
        message = link_in.recv_match(type='SERVO_OUTPUT_RAW', blocking=True, timeout=2)
        if message:
            auto_btn = message.servo11_raw

            if auto_btn > 1500 and autopilot is False:
                autopilot = True
                autopilot_enable_event.set()
                link_out.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING,
                                             "Autopilot enabled".encode())
            elif auto_btn < 1501 and autopilot is True:
                autopilot = False
                autopilot_enable_event.clear()
                link_out.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_WARNING,
                                             "Autopilot disabled".encode())
        else:
            print("mavlink autopilot timeout")

        sleep(0.1)
    return 0


def main():

    stop_event = threading.Event()
    servo_data = ThreadSafeValue()

    mavlink_thread = threading.Thread(target=mavlink_servo_input,
                                      args=(stop_event, servo_data),
                                      name="mavlink_thread")
    mavlink_thread.start()

    autopilot_enable_event = threading.Event()
    GPIO_thread = threading.Thread(target=GPIO_interface,
                                   args=(stop_event, servo_data,),
                                   name="GPIO_thread")
    GPIO_thread.start()

    IMU_data = ThreadSafeValue()
    IMU_thread = threading.Thread(target=mavlink_IMU_input,
                                  args=(stop_event, IMU_data,),
                                  name="IMU_thread")
    IMU_thread.start()

    thruster_data = ThreadSafeValue()
    thruster_thread = threading.Thread(target=mavlink_thruster_control,
                                       args=(stop_event, thruster_data, autopilot_enable_event,),
                                       name="thruster_thread")
    thruster_thread.start()

    ping_thread = threading.Thread(target=ping_distance,
                                   args=(stop_event,),
                                   name="ping_thread")
    #ping_thread.start()

    distance_data = ThreadSafeValue()
    depth_thread = threading.Thread(target=mavlink_depth,
                                    args=(stop_event, distance_data,),
                                    name="depth_thread")
    depth_thread.start()

    pid_parameters = ThreadSafeValue()
    object_detection_enabled_event = threading.Event()
    user_thread = threading.Thread(target=user_input,
                                   args=(stop_event, pid_parameters, distance_data, object_detection_enabled_event,),
                                   name="user_thread")
    user_thread.start()

    prediction_data = ThreadSafeValue()
    object_detection_enabled_event.set()
    video_streamer = VideoStreamer(stop_event, prediction_enable_event=object_detection_enabled_event,
                                   capture_pipeline="camera_capture", predictions=prediction_data)
    camera_streamer_thread = threading.Thread(target=video_streamer.video_streaming_thread, args=(),
                                              name="camera_streaming_thread")
    camera_streamer_thread.start()

    # Distance measurement should be inserted with desired units here (same units as IMU_data)
    autopilot_thread = threading.Thread(target=controller_thread,
                                  args=(stop_event, autopilot_enable_event, IMU_data,
                                                                  prediction_data, distance_data,
                                                                  pid_parameters, thruster_data,),
                                  name="autopilot_thread")
    autopilot_thread.start()

    auto_thread = threading.Thread(target=autopilot_handler, args=(stop_event, autopilot_enable_event), name="auto_thread")
    auto_thread.start()

    try:
        while True:
            sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping program. . .")
        stop_event.set()
        mavlink_thread.join()
        GPIO_thread.join()
        IMU_thread.join()
        #ping_thread.join() TODO: is never started
        depth_thread.join()
        user_thread.join()
        camera_streamer_thread.join()
        autopilot_thread.join()
        auto_thread.join()
        print("Program closed successfully.")


if __name__ == '__main__':
    main()
