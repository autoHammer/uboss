import datetime
from time import sleep, time
from brping import Ping1D

myPing = Ping1D()
myPing.connect_serial("/dev/ttyUSB0", 115200)

if myPing.initialize() is False:
    print("Failed to initialize Ping!")
    exit(1)
print("Ping successfully initialized!")


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

        sleep(0.2)
