import threading
import numpy as np
from ultralytics import YOLO
import cv2
import time
from datetime import datetime
from Other.thread_safe_value import ThreadSafeValue
import os

# Recognition font settings
# Recognition font settings:
font = cv2.FONT_HERSHEY_SIMPLEX  # Font type
font_scale = 1  # Font size
font_color = (255, 255, 255)  # Font color (white in this case)
font_thickness = 1  # Thickness of the text


class ObjectDetector:
    def __init__(self, model, store_process_duration=False, thread_safe_results=None):
        print("Current dir: ", model)
        self._model = YOLO(model)
        print("Model classes: \n", self._model.names)
        self._results = ThreadSafeValue()
        self._thread_safe_results = thread_safe_results
        self._current_frame = None
        self._detector_thread = None
        self._STORE = store_process_duration
        self._FILENAME = None
        # Start writing to file if storing is desired

        if self._STORE:
            SCRIPT_PATH = os.path.abspath(__file__)
            SCRIPT_DIR = os.path.dirname(SCRIPT_PATH)
            LOGGING_DIR = SCRIPT_DIR + "/logs/"
            now = datetime.now()
            timestamp = now.strftime("%Y-%m-%d_%H-%M-%S")
            model_name = model.split("/")
            model_name = model_name[-1].split(".")
            self._FILENAME = f"{LOGGING_DIR}{model_name[0]}_{timestamp}.txt"

    def write_to_file(self, data):
        with open(self._FILENAME, "a") as file:
            file.write(str(data) + "\n")

    def detect(self, frame):
        # Update frame
        self._current_frame = frame

        # Initial run:
        if self._detector_thread is None:
            self._detector_thread = threading.Thread(target=self.detector_thread, args=(self._current_frame,))
            self._detector_thread.start()

        elif not self._detector_thread.is_alive():
            self._detector_thread = threading.Thread(target=self.detector_thread, args=(self._current_frame,))
            self._detector_thread.start()

    def detector_thread(self, frame):
        start_t = time.time()
        results = self._model.predict(frame, verbose=False)
        processing_duration = time.time() - start_t
        self._results.set(results)
        # If results are desired, update thread safe results
        if self._thread_safe_results is not None:
            self._thread_safe_results.set(results)
        # If storing is on. Store time durations of process to file
        if self._STORE:
            self.write_to_file(processing_duration)


    def draw_boxes(self, frame):
        if (self._current_frame is None) or (self._results.take() is None):
            return frame

        else:
            results = self._results.take()
            for box in results[0].boxes:
                # Drawing boxes
                x = int(np.floor(box[0].xywh[0][0].item()))
                y = int(np.floor(box[0].xywh[0][1].item()))
                w = int(np.floor(box[0].xywh[0][2].item()))
                h = int(np.floor(box[0].xywh[0][3].item()))
                cv2.rectangle(self._current_frame, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2), (0, 140, 255))

                # Writing object names:
                c = box.cls
                detected_object = self._model.names[int(c)]

                # Calculate text size to position it correctly
                text_size = cv2.getTextSize(detected_object, font, font_scale, font_thickness)[0]
                text_x = x - w // 2
                text_y = y - h // 2 + 20  # Position the inside the rectangle frame

                # Ensure the text background rectangle does not go out of the image bounds
                background_top_left = (text_x, text_y - text_size[1] - 2)
                background_bottom_right = (text_x + text_size[0], text_y + 2)
                cv2.rectangle(self._current_frame, background_top_left, background_bottom_right, (0, 140, 255),
                              cv2.FILLED)

                # Put text
                cv2.putText(self._current_frame, detected_object, (text_x, text_y), font, font_scale, font_color,
                            font_thickness)

            return self._current_frame

