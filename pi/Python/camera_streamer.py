import threading

import gi
import cv2
import numpy as np
from ultralytics import YOLO
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import os
from object_recognizer import ObjectDetector
from Other.thread_safe_value import ThreadSafeValue
import time

# Initialize GStreamer
Gst.init(None)

# Parameters:
SCRIPT_PATH = os.path.abspath(__file__)
SCRIPT_DIR = os.path.dirname(SCRIPT_PATH)
IMAGE_RECOGNITION_DIR = SCRIPT_DIR + "/../Image_Recognition"

YOLOV8_FIRST_BEST_PT = IMAGE_RECOGNITION_DIR + "/yolov8/Weights/v8_small_first_best.pt"
YOLOV8_TRAPS_ONLY_SMALL_PT = IMAGE_RECOGNITION_DIR + "/yolov8/Weights/traps_only_small.pt"
YOLOV8_CUSTOM_TRAPS_SMALL_PT = IMAGE_RECOGNITION_DIR + "/yolov8/Weights/custom_traps_small.pt"
YOLOV8_CUSTOM_TRAPS_NANO_PT = IMAGE_RECOGNITION_DIR + "/yolov8/Weights/custom_traps_nano.pt"

CURRENT_ALGORITHM = YOLOV8_CUSTOM_TRAPS_NANO_PT  # Here the chosen algorithm is specified

VIDEO_FILE_PATH = SCRIPT_DIR + "/../temp/third_dive.mkv"

# Detector object
#object_detector = ObjectDetector(YOLOV8_CUSTOM_TRAPS_NANO_PT)


class VideoStreamer:
    def __init__(self, stop_event, prediction_enable_event, capture_pipeline, video_file="",
                 store_process_duration=False, predictions=ThreadSafeValue()):
        self.stop_event = stop_event
        # Capture pipeline
        if capture_pipeline == "camera_capture":
            self.capture_pipeline = Gst.parse_launch(
                "v4l2src ! videoconvert ! videoscale ! "
                "video/x-raw,format=BGR,width=640,height=480 ! appsink name=mysink emit-signals=True"
            )
        elif capture_pipeline == "videofile_capture":
            self.capture_pipeline = Gst.parse_launch(
                "filesrc location={} ! decodebin ! videoconvert ! videoscale ! "
                "video/x-raw,format=BGR,width=640,height=480 ! appsink name=mysink emit-signals=True".format(video_file)
            )

        appsink = self.capture_pipeline.get_by_name('mysink')
        appsink.connect('new-sample', self.on_new_sample)

        # Streaming pipeline
        self.stream_pipeline = Gst.parse_launch(
            "appsrc name=mysrc is-live=True format=GST_FORMAT_TIME ! "
            "video/x-raw,format=BGR,width=640,height=480,framerate=30/1 ! videoconvert ! "
            "x264enc tune=zerolatency bitrate=500 speed-preset=superfast ! rtph264pay ! "
            "udpsink host=192.168.2.1 port=5601"
        )
        self.appsrc = self.stream_pipeline.get_by_name('mysrc')
        self.recognition_results = predictions
        self.object_detector = ObjectDetector(CURRENT_ALGORITHM,
                                              store_process_duration=store_process_duration,
                                              thread_safe_results=predictions)
        self.prediction_enabled_event = prediction_enable_event

    def on_new_sample(self, appsink):
        sample = appsink.emit('pull-sample')
        buffer = sample.get_buffer()
        caps = sample.get_caps()
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if success:
            # Convert Gst.Buffer to np.ndarray
            frame = np.ndarray(
                shape=(caps.get_structure(0).get_value('height'),
                       caps.get_structure(0).get_value('width'), 3),
                buffer=map_info.data,
                dtype=np.uint8)

            # Image processing:
            if self.prediction_enabled_event.is_set():
                self.object_detector.detect(frame)
                frame = self.object_detector.draw_boxes(frame)

            self.push_frame_to_appsrc(frame)

            buffer.unmap(map_info)

        return Gst.FlowReturn.OK

    def push_frame_to_appsrc(self, frame):
        buffer = Gst.Buffer.new_allocate(None, frame.nbytes, None)
        buffer.fill(0, frame.tobytes())
        self.appsrc.emit('push-buffer', buffer)

    def start(self):
        self.capture_pipeline.set_state(Gst.State.PLAYING)
        self.stream_pipeline.set_state(Gst.State.PLAYING)

    def stop(self):
        self.capture_pipeline.set_state(Gst.State.NULL)
        self.stream_pipeline.set_state(Gst.State.NULL)

    def video_streaming_thread(self):
        self.start()  # Start the GStreamer pipelines

        bus = self.capture_pipeline.get_bus()
        timeout = 100  # Milliseconds

        while not self.stop_event.is_set():
            message = bus.timed_pop_filtered(timeout * Gst.MSECOND,
                                             Gst.MessageType.ERROR | Gst.MessageType.EOS)

            # Check for GStreamer specific messages
            if message:
                if message.type == Gst.MessageType.ERROR:
                    err, debug = message.parse_error()
                    print("Error:", err, debug)
                    self.stop_event.set()
                    break
                elif message.type == Gst.MessageType.EOS:
                    print("End of Stream")
                    self.stop_event.set()
                    break
            time.sleep(0.01)  # Sleep to reduce CPU usage

        # Call object detector destructor and stop streaming pipelines
        del self.object_detector
        self.stop()  # Stop the GStreamer pipelines


if __name__ == "__main__":
    print("Script starting")
    camera_streaming = "camera_capture"
    video_file_streaming = "videofile_capture"
    stop_event = threading.Event()
    prediction_enable_event = threading.Event()
    prediction_enable_event.set()
    store_process_duration = False  # Store the duration times of each image prediction
    try:
        video_streamer = VideoStreamer(stop_event=stop_event, prediction_enable_event=prediction_enable_event,
                                       capture_pipeline=video_file_streaming,
                                       store_process_duration=store_process_duration, video_file=VIDEO_FILE_PATH)
        video_streamer.video_streaming_thread()

    except KeyboardInterrupt:
        stop_event.set()
        print("Video stream stopped")
        time.sleep(5)  # Wait for prediction to stop properly

    finally:
        print("Program stopped")
