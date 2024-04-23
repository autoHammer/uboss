import gi
import cv2
import numpy as np
from ultralytics import YOLO
from gi.repository import Gst, GLib
import os
from object_recognizer import ObjectDetector
from Other.thread_safe_value import ThreadSafeValue

gi.require_version('Gst', '1.0')

# Initialize GStreamer
Gst.init(None)

# Parameters:
IMAGE_RECOGNITION_DIR = os.getcwd() + "/../Image_Recognition"

YOLOV8_FIRST_BEST_PT = IMAGE_RECOGNITION_DIR + "/yolov8/Weights/v8_small_first_best.pt"
YOLOV8_TRAPS_ONLY_SMALL_PT = IMAGE_RECOGNITION_DIR + "/yolov8/Weights/traps_only_small.pt"
YOLOV8_CUSTOM_TRAPS_SMALL_PT = IMAGE_RECOGNITION_DIR + "/yolov8/Weights/custom_traps_small.pt"
YOLOV8_CUSTOM_TRAPS_NANO_PT = IMAGE_RECOGNITION_DIR + "/yolov8/Weights/custom_traps_nano.pt"

# Detector object
#object_detector = ObjectDetector(YOLOV8_CUSTOM_TRAPS_NANO_PT)


class VideoStreamer:
    def __init__(self, capture_pipeline, video_file="", predictions=ThreadSafeValue()):
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
        self.object_detector = ObjectDetector(YOLOV8_CUSTOM_TRAPS_NANO_PT)

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


#def start_stream():
def start_stream(predictions):
    # Video dir:
    #VIDEO_DIR = os.getcwd() + "/../temp/second_dive_trap_detection.mkv"
    processor = VideoStreamer("camera_capture", predictions)
    #processor = VideoProcessor("videofile_capture", video_file=VIDEO_DIR)
    processor.start()

    # Run the gstream loop
    gstream = GLib.MainLoop()
    try:
        gstream.run()
    except KeyboardInterrupt:
        processor.stop()
        gstream.quit()


start_stream()
"""
if __name__ == "__main__":
    main()
"""
