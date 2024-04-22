from pymavlink import mavutil
from Other.thread_safe_value import ThreadSafeValue
# for testing:
import threading


def IMU_handler(stop_event, IMU_data):
    link_in = mavutil.mavlink_connection('udp:0.0.0.0:14550')
    link_in.wait_heartbeat()

    while not stop_event.is_set():
        message = link_in.recv_match(type='RAW_IMU', blocking=True, timeout=2)
        if message:
            IMU_data.set({"x": message.xacc,
                          "y": message.yacc,
                          "z": message.zacc})
        else:
            print("IMU timeout. Waiting for connection . . .")
            link_in.wait_heartbeat()
            print("IMU reconnected")


if __name__ == '__main__':
    IMU_data = ThreadSafeValue()
    stop_event = threading.Event()
    IMU_thread = threading.Thread(target=IMU_handler, args=(stop_event, IMU_data,))
    IMU_thread.start()

    try:
        while True:
            if IMU_data.has_new_value():
                data = IMU_data.take()
                print("data:", data)

    except KeyboardInterrupt:
        print("\nStopping program. . .")
        stop_event.set()
        IMU_thread.join()
        print("Program closed successfully.")
