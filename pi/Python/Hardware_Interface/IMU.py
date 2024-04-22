from pymavlink import mavutil
import threading


class IMU:
    """

    """

    def __init__(self, stop_event):
        self.link_in = mavutil.mavlink_connection('udp:0.0.0.0:14550')
        self.link_in.wait_heartbeat()
        self.lock = threading.Lock()
        self._x = 0
        self._y = 0
        self._z = 0
        self.stop_event = stop_event
        self.IMU_thread = threading.Thread(target=self.update_IMU_data, args=(self.stop_event,))
        self.IMU_thread.start()
        print("IMU connected")

    @property
    def x(self):
        with self.lock:
            new_x = self._x
        return new_x

    @property
    def y(self):
        with self.lock:
            new_y = self._y
        return new_y

    @property
    def z(self):
        with self.lock:
            new_z = self._z
        return new_z

    def update_IMU_data(self, stop_event):
        """

        Returns:

        """
        while not stop_event.is_set():
            message = self.link_in.recv_match(type='RAW_IMU', blocking=True, timeout=2000)
            if message:
                with self.lock:
                    self._x = message.xacc
                    self._y = message.yacc
                    self._z = message.zacc
            else:
                print("IMU timeout. Waiting for connection . . .")
                self.link_in.wait_heartbeat()
                print("IMU reconnected")

    def __del__(self):
        #self.stop_event.set()
        #self.IMU_thread.join()
        print("IMU offline")



if __name__ == '__main__':
    stop_event = threading.Event()
    data = IMU(stop_event)
    try:
        while True:
            print("xacc:", data.x)
            print("yacc:", data.y)
            print("yacc:", data.z)
            print("")
            from time import sleep
            sleep(0.02)
    except KeyboardInterrupt:
        print("Caught KeyboardInterrupt, performing cleanup.")
        stop_event.set()




