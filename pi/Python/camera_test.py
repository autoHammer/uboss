from picamera2 import Picamera2
from ultralytics import YOLO
import socket
import cv2
import pickle
import struct
import os
import numpy as np

camera = Picamera2()

# Configure the camera
preview_config = camera.create_preview_configuration()
camera.configure(preview_config)
camera.start()
encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]


# Model configuration:
os.chdir("..")
IMAGE_RECOGNITION_DIR = os.getcwd() + "/Image_Recognition"

YOLOV8_PT_S = IMAGE_RECOGNITION_DIR + "/yolov8/Weights/v8_small_first_best.pt"

model = YOLO(YOLOV8_PT_S)


# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('10.24.90.17', 8485))


try:
    while True:
        frame = camera.capture_array()

        cvframe = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        results = model.predict(cvframe)
        
        for box in results[0].boxes:
            x = int(np.floor(box[0].xywh[0][0].item()))
            y = int(np.floor(box[0].xywh[0][1].item()))
            w = int(np.floor(box[0].xywh[0][2].item()))
            h = int(np.floor(box[0].xywh[0][3].item()))

            #cv2.rectangle(cvframe, (x, y), (x + w, y + h), (0, 255, 0))
            #cv2.rectangle(cvframe, (x-w/2, y-h/2), (x+w/2, y+h/2), (0, 255, 0))
            cv2.rectangle(cvframe, (x-w//2, y-h//2), (x + w // 2, y + h // 2), (0, 255, 0))

        # Fungerende gammel tcp kode
        #  """
        result, image = cv2.imencode('.jpg', cvframe)
        data = pickle.dumps(image, 0)
        size = len(data)

        client_socket.sendall(struct.pack(">L", size) + data)
        #  """

except KeyboardInterrupt:
    print("\nInterrupted")
    print("Connection lost")
    client_socket.close()
    camera.stop()


except socket.error:
    print("Connection lost...")
    client_socket.close()
    camera.stop()


finally:
    client_socket.close()
    camera.stop()

