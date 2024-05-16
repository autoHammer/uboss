# Uboss bachelor project 2024
Code by: William Hammer & Sigbjørn N. Kleppe

## Introduction
This github project serves as the software development and documentation for the interdisciplinary bachelor thesis 
"Uboss" at NTNU Ålesund the Spring of 2024. The motivation for the thesis is the development of technology relating to the problem of
ghost fishing. This project attempts to explore the possibility of utilizing ROVs to mark crab traps for further pickup,
and the possibility of performing this marking solution autonomously. 

The project utilizes a BlueROV2 heavy which is made for modular customization. The group has made a custom extension
module to the BlueROV2 heavy consisting of a custom electronics tube, fitted with the following equipment:

* **Raspberry Pi 5:** Responsible for the on-board logic. Communicates between the surface computer and the ROV, as well as
    it controls the custom on board equipment. The Raspberry is supposed to run the main.py script when the
    vehicle is in operation. 

* **h264 USB camera:** Used for the camera stream to the ground control station on the surface computer.

* **Servo:** Moves the camara mount such that a greater field of view is possible.

* **BlueRobotics basic ESC:** Controls the BlueRobotics T200 thruster attached to the linear actuator with the use of PWM.

* **BlueRobotics T200 Thruster:** Attaches to the linear actuator to attract and re-tract the crab trap attachment hook.

* **Serial to RS485 adapter:** Converts RS485 signal from the ping sonar to serial communication to the Raspberry Pi.

* **Ping sonar:** Sonar distance sensor to measure the distance from the seabed.

* **Subsea light:** Lighting to help see below the surface. 


All of the code is written exclusively in Python. Thus all required Python libraries are contained neatly in the included
requirements.txt file. 


The project utilizes PyMavlink to communicate to both the ROV and surface computer over ethernet/IP. Thus it is important
to note that any attempt of building this project, requires the Raspberry Pi 5's ethernet Ip to be set to static with the
ip: "192.168.2.3" using the network manager and NOT dhcpcd which is the old standard. It is also important to manually 
add custom endpoints to the BlueROV2 heavy BlueOS which can be done following the [guide by BlueRobotics](https://blueos.cloud/docs/blueos/1.0/advanced-usage/).
Then the following endpoints need to be created:

# ADD ENDPOINTS HERE


## File structure
The project mainly consists of two directories; the "Python" directory containing all python scripts utilized by the
project and the "Image_Recognition" directory which contain all the YOLO object detection algorithms which were trained
during the development of this project. The Python directory also include three subdirectories "Hardware_Interface",
"logs" and "Other". Both the Hardware_Interface and Other directories contain further Python scripts used in the project.
The logs subdirectory however, contain all logs which are stored using the different scripts.

<!-- 
### Python files:
* **camera_streamer.py:** Introduces the "VideoStreamer" class 

This is a markdown comment
-->


## Threads
Most threads utilized in the main program are initialized and started in the main.py file. The main thread simply has the
objective of starting all other threads and close them once a keyboard interrupt is given by the user. The following
threads and their functions are initiated in the main thread:

**mavlink_thread** is running mavlink_servo_input. This thread is responsible for fetching commands for controlling the 
motor speed and camera tilt servo. SERVO_OUTPUT_RAW is the mavlink message that contains the values of the different 
servo outputs. The values are adjusted by the controller connected to the onshore laptop. The controller can either 
increase, decrease or reset to middle position. It is done using three separate buttons. Originally, a quick press on 
the controller resulted in a too big change in the value. For the camera, it meant the position went from center to max 
in only three button presses. To change the resolution, a different min, max and trim value was set in the autopilot 
parameter settings. Instead of the standard 1100 to 1900 range, 0 to 5000 was used. This gave a lot more fine control 
of the camera  position. When the autopilot boots, all servo values will be 1500 even if the trim value is changed. 
This means the code must send a message setting the value to 2500 right after getting the first heartbeat. 
The motor speed control did not require fine control and standard settings are used. 
The thread loop is waiting for new messages and writing new data to the servo_$data ThreadSafeValue.


**GPIO_thread** is running GPIO_interface. This thread is interfacing with the GPIO pins used by motor and camera tilt 
control. The thread loop is waiting for new data on the servo_data ThreadSafeValue. New data is written to the corresponding 
servo output. A custom servo class is created to handle each servo output. The same class can be used by both as they 
are controlled by the same servo signal type.  It is utilizing the rpi_hardware_pwm library for GPIO 
control. Hardware pwm must be enabled for this class to work. A standard servo frequency of 50Hz is used. A method for 
mapping the input into a more suitable output is created. The output is the duty cycle in percentage. The mapping function 
makes it possible for the user to decide what the input range should be. A typical input range can be 0 to 100% or 1100 
to 1900μs. A write function is changing the duty cycle on the pin, based on output from the map function. A write_smooth 
function is created to give a cleaner output to a components like a motor. Motors can have a very high start current. 
The start current can be significantly reduced by slowly ramping up the speed. The motor is configure to use 0.5 seconds 
to ramp up to full speed.  


**ping_thread** is in normal operation running ping_distance. The thread is handling everything related to the ping sonar 
distance senor. The sonar is interfaced using BlueRobitecs's library, brping. With the library, a ping object is 
initialized with a USB port. It can gather distance and confidence data from the sonar. This data is sent over mavlink 
using a predefined function, distance_sensor_send. The data sent with this function will be automatically displayed in 
the QGroundControl user interface. Additionally to sending the data, it is also being stored in a csv file. This makes 
it possible to design a filter at a later stage if this proves to be necessary. 


**depth_thread** is running mavlink_depth. This thread was supposed to read distance data from the ping sonar and send 
the data to QGroundControl. However, as will be discussed more in Results  the sonars were defective. This serves as a 
workaround for estimating the height, as it is needed for the autopilot. The height is estimated using the depth pressure 
sensor on the BlueROV. This sensor is accurate enough for testing purposes. It is self-explanatory, but the pressure 
sensor gives the distance to the surface and not the distance to the seabed. To get the distance to the seabed a 
calibration must first be done. It can be calibrated by measuring the pressure when the ROV is sitting on the seabed 
next to the crab trap. From there it is possible to calculate distance by comparing measured pressure with the highest 
measured pressure or a known pressure point.


**IMU_thread** is running mavlink_IMU_input. This thread is only fetching the RAW_IMU mavlink message, and stores it in 
the IMU_data ThreadSafeValue. At default the IMU data sent at 1HZ from the Navigator. A mavlink parameter can be changed 
to enable up to 50Hz sending rate. 


The leak detection sensor is sending boolean values to a GPIO pin. The library used, gpiozero, is not working inside a 
virtual environment because it is using system resources. The program is therefore made to run independently from the 
main code. This also gives the advantage that the code can be started automatically when the raspberry pi boots. This 
makes it always active as long as the Raspberry pi is powered on, regardless of the state of the main code. The leak 
detection code is establishing a connection with mavlink using udpout. A function, water_detected, is created to send a 
status text over mavlink. A status text is a type of message that can be assigned a severity scale. If using a high 
severity type like MAV_SEVERITY_CRITICAL, then the message will be both pop up on the QGroundControl user interface 
and additionally read the message aloud. This function is given to gpiozero object, and will be ran whenever the pin is 
activated. The code also regularly sends a heartbeat to QGroundControl. Another warning messege will then be displayed 
on the user interface if connection to the leak detection program is lost. 


The led light control is also using the gpiozero library and is ran individually. This is because the Raspberry pi only 
has 2 hardware pwm outputs. Of the three devices, a stable light is the least critical. gpiozero supports software pwm. 
It was decided to have the light controlled by the the same servo signal that the existing lights on the BlueROV is using. 
This means no extra buttons are needed. The existing led lights are controlled by a servo signal in the SERVO_OUTPUT_RAW 
message. Getting this signal is the same as previously explained by making a mavlink udpin connection, and receiving 
messages with the corresponding message name. The input is converted to duty cycle with a map function, and written to 
the led pin. 


**camera_streamer_thread** handles all camera related activities. It runs the function video_streaming_thread contained 
within the VideoStreamer class. The class contains multiple arguments, like whether or not object detection should be 
enabled (handled by a threading event object), if the video camera or a video file should be used as the streaming source, 
a path to the desired video file if video file streaming is chosen as well as the ability to store the processing duration 
of each frame using the specified object detection algorithm. The class also includes an instance of the class 
ObjectDetector defined in object$\_$recognizer.py. This object handles all object detecting. The video_streaming_thread 
function starts by starting the GStreamer pipelines and then goes into the while loop simply checking for errors or a 
stop event. Gstreamer then starts running in the background, and starts running the on_new_sample function every time a 
new frame is acquired. If object detection is enabled, the frame is then pushed to the ObjectDetector object where a new 
thread is started which starts processing the frame. This thread is started as to not obstruct the video streaming. 
This means that whilst the object detection is happening in the background, the video stream is not affected. 
Once prediction is performed, the results are stored and boxes and class names are drawn on the frame. 
The results are also set to the prediction_data ThreadSafeValue to be further used in the autopilot thread. To avoid 
opening a thread for every frame, new object detection threads are only created once the previous is closed. Lastly, the 
frame is pushed to the QGC stream for display.


**autopilot_thread** is responsible for the handling of the autopilot logic. It runs the controller_thread function 
defined in the kalman.py script. First it declares two kalman filters as well as three PID controllers. The two kalman 
filters and two of the PID controllers are responsible for the sway- and surge movement, while the last PID controller 
is responsible for the heave movement. After this, the thread enters the while loop which checks for new data from the 
IMU, sonar sensor and object detection results. If new data is acquired, the kalman filters are updated asynchronously 
using sensor fusion. Based on the kalman state estimations, the PID controllers' outputs are updated. The outputs are 
then set to the thruster_data ThreadSafeValue as a dictionary where the key "x" defining the sway output, "y" defining 
the surge and "z" defining the heave. In the while loop, the pid_parameters ThreadSafeValue is also checked for new 
values. If new parameters are gotten, the specified PID controller's parameters are updated. Currently the user_thread 
is the only thread that can update these parameters through the use of the terminal. 


**user_thread** is a thread dedicated for debugging. It is running a function, user_input, that is fetching input from 
a terminal. It currently has three features; restarting max depth used in distance estimation, change PID parameters for 
autopilot and toggle object detection. The interface is a primitive UI with a number input for selecting setting to 
change. After selecting, the number is used to run the corresponding code section. For the max depth restarting, the 
distance_data:max_depth is set to 0. The PID values is selected by a second input prompt, asking which parameter to change. 
All PID values in all 3 axis can be changed. The toggle object detection setting, is there in case the object detection 
hogs too much resources on the Raspberry pi. It can potentially slow down the rest of the system and a method to easily 
turn it off is nice to have.

* ****

