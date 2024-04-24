import datetime
import threading
from time import sleep, time
from brping import Ping1D
from pymavlink import mavutil


def now():
    """
    Gives current time in the format YYY-MM-DD_HH-MM-SS
    Returns: (string) time
    """
    current_datetime = datetime.datetime.now()
    formatted_datetime = current_datetime.strftime('%Y-%m-%d_%H-%M-%S')
    return formatted_datetime


def ping_distance(stop_event):
    """
    Function to handle ping sonar.
    It will measure ping distance, store the data in Desktop/Data/Ping_data/ and send the data to qgroundcontrol.
    Args:
        stop_event: threading.Event for stopping the while loop

    Returns: 0 if initialisation fails
    """

    try:
        myPing = Ping1D()
        myPing.connect_serial("/dev/ttyUSB0", 115200)
    except FileNotFoundError:
        print("Failed to initialize Ping!\n"
              "Are the ping connected?")
        return 0
    except Exception:
        print("Exiting ping")
        return 0

    if myPing.initialize() is False:
        print("Failed to initialize Ping!")
        return 0
    print("Ping successfully initialized!")

    boot_time = time()

    link_out = mavutil.mavlink_connection('udpout:192.168.2.1:14550', source_system=1)

    link_out.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_INFO,
                              "Distance sensor online".encode())

    file_path = f"/home/pi/Desktop/Data/Ping_data/ping_data_{now()}.csv"
    with open(file_path, "a") as file:
        # create header for csv file
        header = "time_ms,length_mm,confidence\n"
        file.write(header)
        print(f"Writing ping data to file: {file_path}")

        start_time = time()

        while not stop_event.is_set():
            ping_data = myPing.get_distance()
            data = f"{round(time()-start_time, 3)},{ping_data['distance']},{ping_data['confidence']}\n"
            file.write(data)
            print(f"Writing data: {data}")

            min_distance = 50  # Minimum distance the sensor can measure in centimeters
            max_distance = 1000  # Maximum distance the sensor can measure in centimeters
            current_distance = int(ping_data['distance']/10)  # Current distance measured in centimeters

            # Calculate the time since system boot in milliseconds
            time_boot_ms = int((time() - boot_time) * 1e3)

            # Send the DISTANCE_SENSOR message
            link_out.mav.distance_sensor_send(
                time_boot_ms,
                min_distance,
                max_distance,
                current_distance,
                0, 0, 0, 0
            )

            sleep(0.2)


if __name__ == '__main__':
    stop_event = threading.Event()
    try:
        ping_distance(stop_event)

    except KeyboardInterrupt:
        print("Program closed.")

