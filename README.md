## Face Detection Overview


This project uses OpenCV to detect faces in real-time and communicates the results to both an Arduino and a ROS (Robot Operating System) node.

1. **Face Detection and Communication:**
   - The Python script captures video from your webcam and detects faces.
   - When a face is found, the script sends a signal (`1`) to the Arduino and logs a message ("Face detected!") to a ROS node.

2. **Arduino and LED Control:**
   - The Arduino receives the signal from the Python script.
   - It controls an LED, turning it on when a face is detected and off when no face is detected.

3. **Scripts Overview:**
   - There are two main scripts: one for the Python code that handles face detection and communication with the Arduino, and another for the Arduino code that controls the LED.
   - The Python script will also act as a ROS node, sending `1` when a face is detected and `0` when no face is detected.
   - The Arduino script will be updated to receive this data from ROS, so it can control the LED based on whether a face is detected or not.
