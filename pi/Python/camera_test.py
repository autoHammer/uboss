from picamera2 import Picamera2
import socket
import cv2
import pickle
import struct

camera = Picamera2()

# Configure the camera
preview_config = camera.create_preview_configuration()
camera.configure(preview_config)
camera.start()
print("Test")

# Create a socket object
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('10.24.90.17', 8485))

img_counter = 0


encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]

try:
    while True:
        frame = camera.capture_array()

        result, image = cv2.imencode('.jpg', frame)
        data = pickle.dumps(image, 0)
        size = len(data)

        client_socket.sendall(struct.pack(">L", size) + data)

        img_counter += 1


except KeyboardInterrupt:
    print("\nInterrupted")
    print("Connection lost")
    client_socket.close()
    camera.stop()


except:
    print("\nCrash")
    print("Connection lost...")
    client_socket.close()
    camera.stop()
