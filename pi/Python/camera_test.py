import cv2

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()

    # Add code to display the frame and detections...
    cv2.imshow('YOLOv7 Detections', frame)
    if cv2.waitKey(1) == ord('q'):  # Exit loop if 'q' is pressed
        break

