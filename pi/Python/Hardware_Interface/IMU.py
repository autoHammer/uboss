from pymavlink import mavutil
from Other.thread_safe_value import ThreadSafeValue
# for testing:
import threading


def mavlink_IMU_input(stop_event, IMU_data):
    link_in = mavutil.mavlink_connection('udp:192.168.2.3:14551')
    link_in.wait_heartbeat()
    print("IMU online")
    while not stop_event.is_set():
        message = link_in.recv_match(type='RAW_IMU', blocking=True, timeout=2)
        if message:
            IMU_data.set({"x": message.xacc,
                          "y": message.yacc,
                          "z": message.zacc})
        else:
            print("mavlink IMU timeout. Waiting for connection . . .")
            link_in.wait_heartbeat()
            print("mavlink IMU reconnected")


if __name__ == '__main__':
    IMU_data = ThreadSafeValue()
    stop_event = threading.Event()
    IMU_thread = threading.Thread(target=mavlink_IMU_input, args=(stop_event, IMU_data,))
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
