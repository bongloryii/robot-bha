import cv2
import numpy as np
import serial
import time

# PID Controller Class
class PIDController:
    def __init__(self, kp, ki, kd, dt):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt

        self.prev_error = 0
        self.integral = 0

    def compute(self, error):
        # Proportional term
        proportional = self.kp * error

        # Integral term
        self.integral += error * self.dt
        integral = self.ki * self.integral

        # Derivative term
        derivative = self.kd * (error - self.prev_error) / self.dt
        self.prev_error = error

        # PID Output
        return proportional + integral + derivative

# Initialize serial communication with Arduino
# arduino = serial.Serial('/dev/ttyUSB1', 9600)  # Replace with your Arduino port
# time.sleep(2)  # Wait for Arduino to initialize

# Color range for green in HSV
lower = np.array([36, 50, 50])
upper = np.array([70, 255, 255])

# Capture video from webcam
webcam_video = cv2.VideoCapture(1, cv2.CAP_DSHOW)

if not webcam_video.isOpened():
    print("Error: Could not open webcam.")
    exit()

# PID Controller Parameters
kp = 0.01
ki = 0.001
kd = 0.002
dt = 0.1  # Time step (seconds)

# Initialize PID Controller
pid = PIDController(kp, ki, kd, dt)

while True:
    success, video = webcam_video.read()  # Read frame from webcam
    if not success:
        break

    # Get the frame dimensions
    frame_height, frame_width, _ = video.shape
    frame_center_x = frame_width // 2

    # Draw a fixed vertical line at the center of the frame (for visual reference)
    cv2.line(video, (frame_center_x, 0), (frame_center_x, frame_height), (0, 0, 255), 2)

    # Convert BGR image to HSV
    img = cv2.cvtColor(video, cv2.COLOR_BGR2HSV)

    # Create a mask for green objects
    mask = cv2.inRange(img, lower, upper)

    # Clean up the mask with morphological operations
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Find contours in the mask
    mask_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If an object is detected
    if len(mask_contours) != 0:
        mask_contours = sorted(mask_contours, key=cv2.contourArea, reverse=True)
        largest_contour = mask_contours[0]

        if cv2.contourArea(largest_contour) > 500:  # Ignore small objects
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Draw the bounding box around the detected object
            cv2.rectangle(video, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green color, thickness 2

            # Calculate the center of the object
            cx = x + w // 2
            cy = y + h // 2

            # Draw a circle at the center of the object
            cv2.circle(video, (cx, cy), 5, (255, 0, 0), -1)  # Blue dot for object center

            # Calculate the error (distance from the center of the frame)
            error = frame_center_x - cx

            # Compute PID output
            pid_output = pid.compute(error)

            # Adjust motor velocities based on PID output
            base_speed = 0.05  # Base speed for both motors
            turn_speed = abs(pid_output) * 0.01  # Scale PID output to appropriate speed adjustment
            print(turn_speed)
            if pid_output < 0:  # Object is to the right of the center
                vl = base_speed - turn_speed
                vr = base_speed + turn_speed
            else:  # Object is to the left of the center
                vl = base_speed + turn_speed
                vr = base_speed - turn_speed

            # Send the velocities to Arduino
            print(f"vl: {vl}, vr: {vr}")
            # message = f"{vl} {vr}\n"
            # arduino.write(message.encode())
            # arduino.flush()

            # Display the error on the video feed
            cv2.putText(video, f"Error: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            # Show object center and error for debugging
            print(f"Object Center: x={cx}, y={cy}, Error: {error}")

    # Display the frames
    cv2.imshow("Mask Image", mask)
    cv2.imshow("Webcam Video", video)

    # Exit with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
webcam_video.release()
cv2.destroyAllWindows()
