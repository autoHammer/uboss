# This file is copied and modified from a project in AIS2202 (2023 group 3) , by William Hammer, Sigbjørn Kleppe and Jonatan Kvernland
import threading


# A class for sending and fetching data between threads.
class ThreadSafeValue:
    """
    A class for storing a single value. Used to safely send data between threads.
    """
    def __init__(self):
        self.lock = threading.Lock()
        self.value = None
        self._has_new_value = False

    def set(self, value):
        """
        Set a value
        Args:
            value: any types
        """
        with self.lock:
            self.value = value
            self._has_new_value = True

    def take(self):
        """
        Take the value

        Returns: previous value given with set()
        """
        with self.lock:
            output = self.value
            self._has_new_value = False
        return output

    def has_new_value(self):
        """
        Checks for new value

        Returns: Bool
        """
        return self._has_new_value
