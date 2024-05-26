import threading

import numpy as np
import time
from Other.pid import PID

def pixel_to_distance_calc(distance, coordinates):
    # Camera parameters:
    theta = 36  # width angle
    alpha = 36  # height angle
    pixel_width = 640
    pixel_height = 480

    frame_width = 2 * distance * np.tan(theta / 2)
    frame_height = 2 * distance * np.tan(alpha/2)

    distance_per_pixel_x = frame_width / pixel_width
    distance_per_pixel_y = frame_height / pixel_height

    x_pos = - distance_per_pixel_x * coordinates["x"]
    y_pos = distance_per_pixel_y * coordinates["y"]

    return x_pos, y_pos


def get_prediction_coordinates(detection_results, desired_object):
    camera_width = 640
    camera_height = 480
    coordinates = {"x": None, "y": None}
    try:
        for detected_object in detection_results:
            if detected_object.boxes.cls.numel() > 0:
                cls = int(detected_object.boxes.cls.item())
                name = detected_object.names[cls]
                #print("Detected object: ", name)
                if detected_object.names[cls] == desired_object:
                    x = int(np.floor(detected_object.boxes.xywh[0][0].item()))
                    y = int(np.floor(detected_object.boxes.xywh[0][1].item()))

                    coordinates["x"] = x - camera_width//2
                    coordinates["y"] = y - camera_height//2
                    break
    except RuntimeError:
        print("Prediction error")

    return coordinates


def controller_thread(stop_event, autopilot_enable_event, IMU_data, prediction_results, distance_data, pid_parameters, output):
    # Kalman parameters:
    initial_x = np.matrix([[0], [0], [0]])
    initial_p = np.matrix([[0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0]])
    H = np.matrix([[1.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0]])
    R_x = np.matrix([[0.001, 0],
                   [0, 0.3266]])
    R_y = np.matrix([[0.001, 0],
                     [0, 0.2999]])
    process_variance = 0.1
    kalman_filter_x = Kalman(x=initial_x, p=initial_p, H=H, R=R_x, process_variance=process_variance)
    kalman_filter_y = Kalman(x=initial_x, p=initial_p, H=H, R=R_y, process_variance=process_variance)

    # constants:
    IMU_X_BIAS = 2.49
    IMU_Y_BIAS = -12.559
    IMU = 2
    DETECTION_ALGORITHM = 1
    RECTANGULAR_CRAB_TRAP = "rectangular fish trap"
    ACCEPTABLE_ERROR_X = 10  # 10 cm
    ACCEPTABLE_ERROR_Y = 10  # 10 cm
    DESIRED_DISTANCE_FROM_TRAP = 20  # 20 cm
    ACCEPTABLE_ERROR_Z = 10  # 10cm
    EXTENTION_TIME = 4  # 4 seconds

    # Controller parameters:
    pid_parameters_dict = {
        "x_kp": 1, "x_ki": 0, "x_kd": 0,
        "y_kp": 1, "y_ki": 0, "y_kd": 0,
        "z_kp": 1, "z_ki": 0, "z_kd": 0,
    }
    pid_parameters.set(pid_parameters_dict)
    PWM_ZERO_OFFSET = 1500
    PID_MIN = 1400
    PID_MAX = 1600

    controller_x = PID(kp=pid_parameters_dict["x_kp"], ki=pid_parameters_dict["x_ki"], kd=pid_parameters_dict["x_kd"],
                       output_min=PID_MIN, output_max=PID_MAX, zero_offset=PWM_ZERO_OFFSET)
    controller_x.sv_ = 0

    controller_y = PID(kp=pid_parameters_dict["y_kp"], ki=pid_parameters_dict["y_ki"], kd=pid_parameters_dict["y_kd"],
                       output_min=PID_MIN, output_max=PID_MAX, zero_offset=PWM_ZERO_OFFSET)
    controller_y.sv_ = 0

    controller_z = PID(kp=pid_parameters_dict["z_kp"], ki=pid_parameters_dict["z_ki"], kd=pid_parameters_dict["z_kd"],
                       output_min=PID_MIN, output_max=PID_MAX, zero_offset=PWM_ZERO_OFFSET)
    z_and_actuator_controller = HeightAndActuatorController(set_point=DESIRED_DISTANCE_FROM_TRAP, pid=controller_z,
                                                            ACCEPTABLE_Z_ERROR=ACCEPTABLE_ERROR_Z,
                                                            actuator_extension_duration=EXTENTION_TIME)

    # access parameters:
    distance_measured = 100

    while not stop_event.is_set():
        # Check for new pid parameters:
        if pid_parameters.has_new_value():
            params = pid_parameters.take()
            controller_x.update_all_params(kp=float(params["x_kp"]), ki=float(params["x_ki"]), kd=float(params["x_kd"]))
            controller_y.update_all_params(kp=float(params["y_kp"]), ki=float(params["y_ki"]), kd=float(params["y_kd"]))
            controller_z.update_all_params(kp=float(params["z_kp"]), ki=float(params["z_ki"]), kd=float(params["z_kd"]))
            print("New parameters gotten: x_kp: ", controller_x.kp_, " x_ki: ", controller_x.ki_,  " x_kd: ",
                  controller_x.kd_)

        # Update kalman
        kalman_filter_x.a_priori()
        kalman_filter_y.a_priori()
        if IMU_data.has_new_value():
            current_imu_data = IMU_data.take()
            # Remove sensor bias:
            current_imu_data["x"] -= IMU_X_BIAS
            current_imu_data["y"] -= IMU_Y_BIAS

            # Update x coordinates
            kalman_filter_x.a_posteriori_asynchronous(measurement=[0, current_imu_data["x"]], sensor=IMU)

            # Update y coordinates
            kalman_filter_y.a_posteriori_asynchronous(measurement=[0, current_imu_data["y"]], sensor=IMU)

        if distance_data.has_new_value():
            distance_dict = distance_data.take()
            if "distance" in distance_dict:
                distance_measured = distance_dict["distance"]

        if prediction_results.has_new_value():
            recognition_results = prediction_results.take()
            coordinates = get_prediction_coordinates(recognition_results, desired_object=RECTANGULAR_CRAB_TRAP)
            # Check for valid coordinates
            if coordinates["x"] is not None:
                x_pos, y_pos = pixel_to_distance_calc(distance_measured, coordinates)
                #print(threading.current_thread().name, "x coordinate: ", coordinates["x"], " y coordinate: ", coordinates["y"])
                #print(threading.current_thread().name, "x pos: ", x_pos, " y pos: ", y_pos)
                kalman_filter_x.a_posteriori_asynchronous(measurement=[x_pos, 0], sensor=DETECTION_ALGORITHM)
                kalman_filter_y.a_posteriori_asynchronous(measurement=[y_pos, 0], sensor=DETECTION_ALGORITHM)

        # Controller output calculation with input from kalman filter
        x_pos_estimate = - kalman_filter_x.x[0]  # Invert to get correct direction
        y_pos_estimate = - kalman_filter_y.x[0]  # Invert to get correct direction

        x_output = controller_x.output(x_pos_estimate)
        y_output = controller_y.output(y_pos_estimate)

        # Start z PID if desired position is reached
        if abs(x_pos_estimate) < ACCEPTABLE_ERROR_X and abs(y_pos_estimate) < ACCEPTABLE_ERROR_Y:
            acceptable_xy_error = True
        else:
            acceptable_xy_error = False

        # Get z output based on measured distance and xy_error:
        z_output, actuator_mode = z_and_actuator_controller.output(distance_measured, acceptable_xy_error,
                                                                   autopilot_enabled=autopilot_enable_event)

        output_dict = {"x": x_output, "y": y_output, "z": z_output, "actuator": actuator_mode}
        #print(threading.current_thread().name, ": Current thrust x:", output_dict["x"], " y: ", output_dict["y"], " z: ", output_dict["z"])
        #print(threading.current_thread().name, ": Estimations: x:", kalman_filter_x.x[0], " y: ", kalman_filter_y.x[0])

        output.set(output_dict)
        # Sleep to give kalman calculation time
        time.sleep(0.01)


class Kalman:
    def __init__(self, x, p, H, R, process_variance):
        self.x = x.copy()
        self.p = p.copy()
        self.t_old = time.time()
        self.H = H.copy()
        self.Ht = np.transpose(H)
        self.z = np.zeros((H.shape[0], 1))
        self.R = R.copy()
        self.process_variance = process_variance
        self.dt = self.A = self.Q = self.G = self.Gt = self.K = None

    def a_priori(self):
        self.build_a()
        self.build_q()
        self.x = self.A * self.x
        self.p = self.A * self.p * np.transpose(self.A) + self.Q

    def a_posteriori(self, measurement):
        self.K = self.p * self.Ht * np.linalg.inv(self.H * self.p * self.Ht + self.R)
        self.build_z(measurement)
        self.x = self.x + self.K * (self.z - self.H * self.x)
        size = np.shape(self.p)
        self.p = (np.identity(size[0]) - self.K * self.H) * self.p

    def a_posteriori_asynchronous(self, measurement, sensor):
        self.K = self.p * self.Ht[:, sensor-1] * np.linalg.inv(self.H[sensor-1, :] * self.p * self.Ht[:, sensor-1] + self.R[sensor-1, sensor-1])
        self.build_z(measurement)
        self.x = self.x + self.K * (self.z[sensor-1] - self.H[sensor-1, :] * self.x)
        size = np.shape(self.p)
        self.p = (np.identity(size[0]) - self.K * self.H[sensor-1, :]) * self.p

    def build_a(self):
        self.dt = time.time() - self.t_old
        self.t_old = time.time()
        self.A = np.matrix([[1.0, self.dt, 0.5 * self.dt ** 2],
                            [0.0, 1.0, self.dt],
                            [0.0, 0.0, 1.0]])

    def build_q(self):
        self.G = np.matrix([[self.dt**3 / 6],
                            [self.dt**2 / 2],
                            [self.dt]])
        self.Gt = np.transpose(self.G)
        self.Q = np.matrix(self.G * self.Gt * self.process_variance)

    def build_z(self, measurements):
        row = 0
        for measurement in measurements:
            self.z[row] = measurement
            row += 1


class HeightAndActuatorController:
    def __init__(self, set_point, pid, ACCEPTABLE_Z_ERROR, actuator_extension_duration):
        self.set_point_ = set_point
        self.state = "idle"
        self.pid = pid
        self.NEUTRAL_PWM = 1500
        self.ACCEPTABLE_Z_ERROR = ACCEPTABLE_Z_ERROR
        self.actuator_extend_duration = actuator_extension_duration
        self.actuator_extension_timer = time.time()
        self.RISE_VALUE = 100  # the amount of cm that the ROV will rise after grabbing the trap

    def output(self, distance, acceptable_xy_error, autopilot_enabled):
        if autopilot_enabled.is_set():
            self.state = "idle"

        match self.state:
            case "idle":  # Wait for acceptable xy error and change to de_escalate mode
                if acceptable_xy_error:
                    self.state = "de_escalate"

                return self.NEUTRAL_PWM, "idle"

            case "de_escalate":  # De-escalate until trap is reached (if xy error is good)
                self.pid.sv_ = self.set_point_
                if acceptable_xy_error:
                    output = self.pid.output(distance)
                else:
                    output = self.NEUTRAL_PWM
                # if setpoint reached switch to actuator grab
                if abs(self.set_point_ - distance) < self.ACCEPTABLE_Z_ERROR:
                    self.state = "grab"
                    self.actuator_extension_timer = time.time()

                return output, "idle"

            case "grab":  # If grabber extended, switch to escalate
                if (time.time() - self.actuator_extension_timer) > self.actuator_extend_duration:
                    self.state = "escalate"
                return self.NEUTRAL_PWM, "extend"

            case "escalate":  # Escalate to new set point and switch to new mode after set point reached
                self.pid.sv_ = self.set_point_ + self.RISE_VALUE  # Rise a certain amount after grabbing
                output = self.pid.output(distance)
                if self.pid.sv_ > distance:
                    self.state = "idle"
                return output, "idle"


