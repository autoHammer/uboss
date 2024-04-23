import numpy as np
import time
from Other.pid import PID


def pixel_to_distance_calc(distance, coordinates):
    # Camera parameters:
    tetha = 36  # width angle
    alpha = 36  # height angle
    pixel_width = 640
    pixel_height = 480

    frame_width = 2 * distance / np.sin(tetha/2)
    frame_height = 2 * distance / np.sin(alpha/2)

    distance_per_pixel_x = frame_width / pixel_width
    distance_per_pixel_y = frame_height / pixel_height

    x_pos = - distance_per_pixel_x * coordinates["x"]
    y_pos = distance_per_pixel_y * coordinates["y"]

    return x_pos, y_pos


def get_prediction_coordinates(detection_results, desired_object):
    camera_width = 640
    camera_height = 480
    coordinates = {"x": None, "y": None}
    for detected_object in detection_results:
        cls = int(detected_object.boxes.cls.item())
        name = detected_object.names[cls]
        print("Detected object: ", name)
        if detected_object.names[cls] == desired_object:
            x = int(np.floor(detected_object.boxes.xywh[0][0].item()))
            y = int(np.floor(detected_object.boxes.xywh[0][1].item()))

            coordinates["x"] = x - camera_width//2
            coordinates["y"] = y - camera_height//2
            break
    return coordinates


def controller_thread(stop_event, IMU_data, prediction_results, distance_data, output):
    # Kalman parameters:
    initial_x = np.matrix([[0], [0], [0]])
    initial_p = np.matrix([[0, 0, 0],
                           [0, 0, 0],
                           [0, 0, 0]])
    H = np.matrix([[1.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0]])
    R_x = np.matrix([[0.001, 0],
                   [0, 0.0.3266]])
    R_y = np.matrix([[0.01, 0],
                     [0, 0.2999]])
    process_variance = 0.1

    # constants:
    IMU = 1
    DETECTION_ALGORITHM = 2
    RECTANGULAR_CRAB_TRAP = "rectangular fish trap"

    # PID parameters:
    controller_x = PID(kp=1, ki=0, kd=0, output_min=1400, output_max=1600)
    controller_x.sv_ = 0
    controller_y = PID(kp=1, ki=0, kd=0, output_min=1400, output_max=1600)
    controller_y.sv_ = 0

    kalman_filter_x = Kalman(x=initial_x, p=initial_p, H=H, R=R_x, process_variance=process_variance)
    kalman_filter_y = Kalman(x=initial_x, p=initial_p, H=H, R=R_y, process_variance=process_variance)

    while not stop_event.is_set():
        kalman_filter_x.a_priori()
        if IMU_data.has_new_value():
            current_imu_data = IMU_data.take()
            # Update x coordinates
            #kalman_filter_x.a_priori()  # TODO: Maybe move this out of the if condition?
            kalman_filter_x.a_posteriori_asynchronous(measurement=[0, current_imu_data["x"]], sensor=IMU)

            # Update y coordinates
            kalman_filter_y.a_priori()
            kalman_filter_y.a_posteriori_asynchronous(measurement=[0, current_imu_data["y"]], sensor=IMU)

        if prediction_results.has_new_value():
            print("Got new results!")
            recognition_results = prediction_results.take()
            coordinates = get_prediction_coordinates(recognition_results, desired_object=RECTANGULAR_CRAB_TRAP)
            #print("x coordinates: ", coordinates["x"], "\n y coordinates: ", coordinates["y"], "\n")
            if coordinates["x"] is not None:
                x_pos, y_pos = pixel_to_distance_calc(distance_data, coordinates)
                #print("x position: ", x_pos, "\nypos: ", y_pos)
                kalman_filter_x.a_posteriori_asynchronous(measurement=[x_pos, 0], sensor=DETECTION_ALGORITHM)
                kalman_filter_y.a_posteriori_asynchronous(measurement=[y_pos, 0], sensor=DETECTION_ALGORITHM)
                print("Kalman x predictions:\n", kalman_filter_x.x)
                print("Kalman y predictions:\n", kalman_filter_y.x)

        # Controller output calculation with input from kalman filter
        x_output = controller_x.output((kalman_filter_x.x[0]))
        y_output = controller_y.output((kalman_filter_y.x[0]))
        output_dict = {"x": x_output, "y": y_output}

        output.set(output_dict)

        #print("Kalman x: ", kalman_filter_x.x[0], "\n Kalman y: ", kalman_filter_y.x[0], "\n")

        time.sleep(0.01)


class Kalman:
    def __init__(self, x, p, H, R, process_variance):
        self.x = x
        self.p = p
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
        print("z matrix: ", self.z)
