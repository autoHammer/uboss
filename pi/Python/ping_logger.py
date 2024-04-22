import datetime
from time import sleep, time
from brping import Ping1D
from pymavlink import mavutil

myPing = Ping1D()
myPing.connect_serial("/dev/ttyUSB0", 115200)

if myPing.initialize() is False:
    print("Failed to initialize Ping!")
    exit(1)
print("Ping successfully initialized!")

boot_time = time()

# Create the connection to the top-side computer as companion computer/autopilot
master = mavutil.mavlink_connection('udpout:192.168.2.1:14550', source_system=1)
# Send a message for QGC to read out loud
#  Severity from https://mavlink.io/en/messages/common.html#MAV_SEVERITY
master.mav.statustext_send(mavutil.mavlink.MAV_SEVERITY_NOTICE,
                          "Distance sensor online".encode())


def now():
    """
    Gives current time in the format YYY-MM-DD_HH-MM-SS
    Returns: (string) time
    """
    current_datetime = datetime.datetime.now()
    formatted_datetime = current_datetime.strftime('%Y-%m-%d_%H-%M-%S')
    return formatted_datetime


file_path = f"/home/pi/Desktop/ping_data/ping_data_{now()}.csv"

with open(file_path, "a") as file:
    # create header for csv file
    header = "time_ms,length_mm,confidence\n"
    file.write(header)
    print(f"Writing ping data to file: {file_path}")

    start_time = time()

    while True:
        ping_data = myPing.get_distance()
        data = f"{round(time()-start_time, 3)},{ping_data['distance']},{ping_data['confidence']}\n"
        file.write(data)
        print(f"Writing data: {data}")

        # Example values for the DISTANCE_SENSOR message fields
        min_distance = 50  # Minimum distance the sensor can measure in centimeters
        max_distance = 1000  # Maximum distance the sensor can measure in centimeters
        current_distance = int(ping_data['distance']/10)  # Current distance measured in centimeters
        type_sensor = 0  # Type of the distance sensor. 0 for Laser rangefinder, etc. See MAVLink documentation for other types.
        id_sensor = 0  # Onboard ID of the sensor
        orientation = 0  # Direction the sensor is facing. See MAV_SENSOR_ORIENTATION enum.
        covariance = 0  # Measurement covariance in centimeters, use 0 for unknown

        # Calculate the time since system boot in milliseconds
        time_boot_ms = int((time() - boot_time) * 1e3)

        # Send the DISTANCE_SENSOR message
        master.mav.distance_sensor_send(
            time_boot_ms,
            min_distance,
            max_distance,
            current_distance,
            type_sensor,
            id_sensor,
            orientation,
            covariance
        )

        sleep(0.2)
