import cv2
import numpy as np
import rclpy

from std_msgs.msg import Float32MultiArray
from rclpy.node import Node
import csv
import matplotlib.pyplot as plt
import math
import numpy as np
import heapq
import serial
import time
from geometry_msgs.msg import Pose  # Assuming PoseStamped is used
from scipy.ndimage import zoom
from scipy.ndimage import binary_dilation
color = [87, 139, 46]  # Green in BGR

c = np.uint8([[color]])  # BGR values
hsv_c = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
hue = hsv_c[0][0][0]  # Get the hue value

        # Handle red hue wrap-around
if hue >= 165:  # Upper limit for red hue
    lower_limit = np.array([hue - 5, 100, 100], dtype=np.uint8)
    upper_limit = np.array([180, 255, 255], dtype=np.uint8)
elif hue <= 15:  # Lower limit for red hue
    lower_limit = np.array([0, 100, 100], dtype=np.uint8)
    upper_limit = np.array([hue + 10, 255, 255], dtype=np.uint8)
else:
    lower_limit = np.array([45, 100, 100], dtype=np.uint8)
    upper_limit = np.array([75, 255, 255], dtype=np.uint8)

class ObjectTrackingNode(Node):
    def __init__(self):
        super().__init__('object_tracking_node')
        self.timer = self.create_timer(0.05, self.bottle_found)
        self.detected=False
        # Capture video from webcam
        self.webcam_video = cv2.VideoCapture(0,cv2.CAP_V4L2)
        self.vr=-40
        self.vl=40
        arduino.write(f"1,{self.vr},{self.vl}\n".encode('utf-8'))

        if not self.webcam_video.isOpened():
            self.get_logger().error("Error: Could not open webcam.")
            exit()

        # Initialize the frame center x coordinate
        self.frame_center_x = 0
    
    def send_command(self):
        """
        Send a command to Arduino and check for the response 'X'.
        :param command: The command string to send (e.g., '1,100,50', '3,120,80', or '2').
        :return: True if Arduino responds with 'X', False otherwise.
        """
        global current_state
        try:
            # Send the command
            arduino.write(f"{current_state},{self.vr},{self.vl}\n".encode('utf-8'))
            
            time.sleep(0.1)  # Allow some time for Arduino to process
            # Listen for Arduino response
            response = arduino.readline().decode('utf-8').strip()
            if response == "X":
                self.vr=0
                self.vl=0
                return True
            return False
        except Exception as e:
            print(f"Error: {e}")
            return False
    def process_video(self):
        # Read frame from webcam
        success, video = self.webcam_video.read()
        if not success:
            print("PROCESSING BUT CAMERA READ FAILED")
            return

        # Get the frame dimensions
        frame_height, frame_width, _ = video.shape
        self.frame_center_x = frame_width // 2  # Horizontal center of the frame

        # Convert BGR image to HSV
        img = cv2.cvtColor(video, cv2.COLOR_BGR2HSV)

        # Create a mask for green objects
        mask = cv2.inRange(img, lower_limit, upper_limit)

        # Clean up the mask with morphological operations
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Closing operation to merge nearby areas

        # Find contours in the mask
        mask_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # If an object is detected
        if len(mask_contours) != 0:
            mask_contours = sorted(mask_contours, key=cv2.contourArea, reverse=True)
            largest_contour = mask_contours[0]
            
            if cv2.contourArea(largest_contour) > 500:  # Ignore small objects
                x, y, w, h = cv2.boundingRect(largest_contour)
                self.detected=True
                if isDebug:
                # Draw the bounding box around the detected object
                    cv2.rectangle(video, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green color, thickness 2

                # Calculate the center of the object
                cx = x + w // 2
                cy = y + h // 2
                if isDebug:
                # Draw a circle at the center of the object
                    cv2.circle(video, (cx, cy), 5, (255, 0, 0), -1)  # Blue dot for object center

                # Calculate the error (distance from the center of the frame)
                error = self.frame_center_x - cx

                # # Velocity adjustment based on error
                # if error > 100:  # Object is to the right of the center
                #     self.vl = 0.03
                #     self.vr = 0.08
                # elif error < -100:  # Object is to the left of the center
                #     self.vl = 0.08
                #     self.vr = 0.03
                # else:  # Object is close to the center
                #     self.vl = 0.05
                #     self.vr = 0.05
                # arduino.write(f"1,{0},{0}\n".encode('utf-8'))
                # time.sleep(0.5)
                Kp = 0.1  # Proportional constant
                Ki = 0.001 # Integral constant
                Kd = 0.001  # Derivative constant

                # Initialize PID variables
                previous_error = 0
                integral = 0
            
                proportional = error
                integral += error
                derivative = error - previous_error
                
                # PID output
                pid_output = Kp * proportional + Ki * integral + Kd * derivative
                # if (pid_output>0.05):
                #     pid_output=0.05
                # elif (pid_output<-0.05:
                #       pid_output=0.05)
                # print(pid_output)
                if abs(error)<100:
                    v_base=55
                    Kp=0.05

                else:
                    
                    v_base= 80
                # v_base = 100
                self.vl = v_base - pid_output
                self.vr = v_base + pid_output
                # print(self.vl, self.vr)

                # Update previous error
                previous_error = error
            
                # Display the error on the video feed
                arduino.write(f"1,{self.vr},{self.vl}\n".encode('utf-8'))
                # self.send_command()
                print(self.vr, self.vl)
                if isDebug:
                    cv2.putText(mask, f"Error: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

                # Show object center and error for debugging
                self.get_logger().info(f"Object Center: x={cx}, y={cy}, Error: {error}")
        else:
            if (self.detected==False):
                # # No object detected, rotate the robot
                self.vl = 40
                self.vr = -40
                self.get_logger().info(f"No object found. Rotating...{self.vr},{self.vl}")
                arduino.write(f"1,{self.vr},{self.vl}\n".encode('utf-8'))
                # self.send_command

        if isDebug:
            # Display the frames
            cv2.imshow("Mask Image", mask)
            cv2.imshow("Webcam Video", video)
    
    def bottle_found(self):
        """Check for messages from Arduino."""
        if arduino.in_waiting > 0:
            message = arduino.readline().decode('utf-8').strip()
            if message == "X":
                self.get_logger().info("Bottle found!")

                # print("Bottle found!")
                global current_state
                current_state= 2
                print("Stopping motors and generating path...")
                arduino.write(f"2,{0},{0}\n".encode('utf-8'))
                self.destroy_node()
                print('found')
                return True
        return False
    def spin(self):
        while rclpy.ok():
            self.process_video()
            # Check if bottle is found
                # time.sleep(1)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        # Release resources when done
        self.webcam_video.release()
        cv2.destroyAllWindows()
                