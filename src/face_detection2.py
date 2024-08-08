import cv2
import rospy
from std_msgs.msg import Int32

if __name__ == "__main__":

    # Load the pre-trained face detection model from the specified XML file
    face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')

    # Open a connection to the camera (default camera index 0)
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Camera not accessible.")
        exit()

    # Initialize the ROS node named 'face_detector_node'
    rospy.init_node('face_detector_node')

    # Create a ROS publisher to send face detection status messages
    pub = rospy.Publisher('face_detector_status', Int32, queue_size=10)

    while not rospy.is_shutdown() and cap.isOpened():
        # Capture a single frame from the video feed
        ret, frame = cap.read()

        # Initialize the face detection flag for the current frame
        face_detected_in_this_frame = False

        if ret:
            # Convert the captured frame to grayscale
            image_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # Parameters for the face detection algorithm
            scale_factor = 1.3
            min_neighbors = 5

            # Perform face detection on the grayscale image
            faces = face_cascade.detectMultiScale(image_gray, scale_factor, min_neighbors)

            # Iterate through the detected faces
            for (x, y, w, h) in faces:
                # Draw a rectangle around the detected face
                cv2.rectangle(image_gray, (x, y), (x+w, y+h), (255, 255, 255), 2)
                # Set the face detection flag to True as a face is detected in this frame
                face_detected_in_this_frame = True

            # Publish the face detection status to the ROS topic
            status_msg = Int32(1) if face_detected_in_this_frame else Int32(0)
            pub.publish(status_msg)

            # Print the face detection status to the terminal
            if face_detected_in_this_frame:
                print("Face detected in this frame.")
            else:
                print("No face detected in this frame.")
            
            # Print the published message to the terminal
            print(f"Published message: {status_msg.data}")

            # Display the frame with detected faces highlighted
            cv2.imshow('frame', image_gray)

            # Wait for a key event for 1 millisecond
            # If the ESC key (key code 27) is pressed, exit the loop
            c = cv2.waitKey(1)
            if c == 27:
                # Close all OpenCV windows and exit the loop
                cv2.destroyAllWindows()
                break
        else:
            print("Error: Unable to capture video.")
            break

    cap.release()
    cv2.destroyAllWindows()
