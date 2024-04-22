from numpy import sqrt
import threading
from time import sleep

from Hardware_Interface.IMU import IMU_handler
from Other.thread_safe_value import ThreadSafeValue


def get_avg(data):

    len_ = 0
    sum_ = 0

    for one_measurement in data:
        sum_ = sum_ + float(one_measurement)
        len_ = len_ + 1

    return sum_ / len_


def get_variance(data, avg):
    len_ = 0
    variance = 0

    for one_measurement in data:
        variance = variance + (float(one_measurement) - avg) ** 2
        len_ = len_ + 1

    return variance / (len_ - 1)


if __name__ == '__main__':
    stop_event = threading.Event()
    new_IMU_data = ThreadSafeValue()
    IMU_thread = threading.Thread(target=IMU_handler, args=(stop_event, new_IMU_data,))
    IMU_thread.start()

    data_x = []
    data_y = []

    file_path = "/home/pi/Desktop/Data/IMU_data/IMU_data_raw.csv"
    with open(file_path, "w") as file:
        header = "x,y,z\n"
        file.write(header)
        # get IMU data
        print("Measuring IMU. Do not touch the ROV!")
        for i in range(6000):
            # wait for new IMU data
            while not new_IMU_data.has_new_value():
                sleep(0.001)
            data_x.append(new_IMU_data.take()["x"])
            data_y.append(new_IMU_data.take()["y"])
            line = str(new_IMU_data.take()["x"]) + "," + str(new_IMU_data.take()["y"]) + "," + str(new_IMU_data.take()["z"]) + "\n"
            file.write(line)

    stop_event.set()
    IMU_thread.join()

    IMU_x_avg = get_avg(data_x)
    IMU_x_Variance = get_variance(data_x, IMU_x_avg)

    print("IMU 'x' average: " + str(IMU_x_avg))
    print("IMU 'x' varians: " + str(IMU_x_Variance))
    print("IMU 'x' standardavvik: " + str(sqrt(IMU_x_Variance)) + "\n")

    IMU_y_avg = get_avg(data_y)
    IMU_y_Variance = get_variance(data_y, IMU_y_avg)

    print("IMU 'y' average: " + str(IMU_y_avg))
    print("IMU 'y' varians: " + str(IMU_y_Variance))
    print("IMU 'y' standardavvik: " + str(sqrt(IMU_y_Variance)) + "\n")

    file_path = "/home/pi/Desktop/Data/IMU_data/IMU_calibration_result.csv"
    with open(file_path, "w") as result_file:
        result_header = "x_average,x_varians,x_standardvvik,y_average,y_varians,y_standardvvik\n"
        result_file.write(header)
        line = str(IMU_x_avg) + "," + str(IMU_x_Variance) + "," + str(sqrt(IMU_x_Variance)) + "," + str(IMU_y_avg) + "," + str(IMU_y_Variance) + "," + str(sqrt(IMU_y_Variance))
        result_file.write(line)
