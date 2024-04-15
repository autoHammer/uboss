import gi
import cv2
import numpy as np
from ultralytics import YOLO
import os

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

# Parameters:
IMAGE_RECOGNITION_DIR = os.getcwd() + "/ImageRecognitionPTs"

YOLOV8_PT_S = IMAGE_RECOGNITION_DIR + "/yolov8/v8_small_first_best.pt"

model = YOLO(YOLOV8_PT_S)

# Recognition font settings:
# Text settings
font = cv2.FONT_HERSHEY_SIMPLEX  # Font type
font_scale = 1  # Font size
font_color = (255, 255, 255)  # Font color (white in this case)
font_thickness = 1  # Thickness of the text


class VideoProcessor:
    def __init__(self):
        # Capture pipeline
        self.capture_pipeline = Gst.parse_launch(
            "v4l2src ! videoconvert ! videoscale ! "
            "video/x-raw,format=BGR,width=640,height=480 ! appsink name=mysink emit-signals=True"
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
        self.counter = 0
        self._RECOGNITION_INTERVAL = 30
        self.recognition_results = None

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
            if self.counter == 0:
                self.recognition_results = model.predict(frame)

            self.counter += 1
            if self.counter == self._RECOGNITION_INTERVAL:
                self.counter = 0

            # Draw boxes and text
            frame = self.draw_boxes(frame)

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

    def draw_boxes(self, frame):
        for box in self.recognition_results[0].boxes:
            # Drawing boxes
            x = int(np.floor(box[0].xywh[0][0].item()))
            y = int(np.floor(box[0].xywh[0][1].item()))
            w = int(np.floor(box[0].xywh[0][2].item()))
            h = int(np.floor(box[0].xywh[0][3].item()))
            cv2.rectangle(frame, (x - w // 2, y - h // 2), (x + w // 2, y + h // 2), (0, 140, 255))

            # Writing object names:
            c = box.cls
            detected_object = model.names[int(c)]

            # Calculate text size to position it correctly
            text_size = cv2.getTextSize(detected_object, font, font_scale, font_thickness)[0]
            text_x = x - w // 2
            text_y = y - h // 2 - 10  # Position the text 10px above the rectangle

            # Ensure the text background rectangle does not go out of the image bounds
            background_top_left = (text_x, text_y - text_size[1] - 2)
            background_bottom_right = (text_x + text_size[0], text_y + 2)
            cv2.rectangle(frame, background_top_left, background_bottom_right, (0, 140, 255), cv2.FILLED)

            # Put text
            cv2.putText(frame, detected_object, (text_x, text_y), font, font_scale, font_color, font_thickness)

            return frame


def main():
    processor = VideoProcessor()
    processor.start()

    # Run the main loop
    mainloop = GLib.MainLoop()
    try:
        mainloop.run()
    except KeyboardInterrupt:
        processor.stop()
        mainloop.quit()


if __name__ == '__main__':
    main()
