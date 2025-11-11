import cv2
import numpy as np
import serial
import time

# Set up serial communication with Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600)  # Replace 'COM11' with your Arduino port
time.sleep(2)  # Wait for Arduino to initialize

# Color range for green in HSV
lower = np.array([36, 25, 25])
upper = np.array([70, 255, 255])

# Capture video from webcam
webcam_video = cv2.VideoCapture(0)


# Example usage of initial velocities
vl = 0.02
vr = 0.02

while True:
    success, video = webcam_video.read()  # Read frame from webcam
    if not success:
        break
    
    # Get the frame dimensions
    frame_height, frame_width, _ = video.shape

    # Calculate the horizontal center of the frame
    frame_center_x = frame_width // 2

    # Draw a fixed vertical line at the center of the frame (for visual reference)
    cv2.line(video, (frame_center_x, 0), (frame_center_x, frame_height), (0, 0, 255), 2)

    # Convert BGR image to HSV
    img = cv2.cvtColor(video, cv2.COLOR_BGR2HSV)

    # Create a mask for green objects
    mask = cv2.inRange(img, lower, upper)

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

            # Draw the bounding box around the detected object
            cv2.rectangle(video, (x, y), (x + w, y + h), (0, 255, 0), 2)  # Green color, thickness 2

            # Calculate the center of the object
            cx = x + w // 2
            cy = y + h // 2

            #Draw a circle at the center of the object
            cv2.circle(video, (cx, cy), 5, (255, 0, 0), -1)  # Blue dot for object center

            # Calculate the error (distance from the center of the frame)
            error = frame_center_x - cx

            # # Calculate the speed adjustment based on error
            # base_speed = 0.2  # Base speed for both motors
            # speed_adjustment = 0.05  # Speed adjustment for turning

            # Control the velocity based on the error
                # Example strategy: adjust velocities based on the error
            if error > 60:  # Object is to the right of the center
                vl = 0.05
                vr = 0.1
            elif error < -60:  # Object is to the left of the center
                vl = 0.1
                vr = 0.05
            else:  # Object is close to the center
                vl = 0.1
                vr = 0.1
            # Send the velocities to Arduino
            # print(f"vl: {vl}, vr: {vr}")
            # Function to send velocity to Arduino
            #"""Send the left and right wheel velocities to Arduino"""
            # message = f"{vl} {vr}\n"  # Format as 'vl vr' followed by newline
            arduino.write(f"{vr} {vl}\n".encode())  # Sending velocity for right and left motor
            arduino.flush() 


            # # Send speed values to Arduino
            # print(f"Sent to Arduino: Right Speed = {right_speed}, Left Speed = {left_speed}")
            # arduino.write(f"{right_speed},{left_speed}\n".encode())  # Sending velocity for right and left motor
            # arduino.flush() 

            # Display the error on the video feed
            cv2.putText(video, f"Error: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            # Show object center and error for debugging
            print(f"Object Center: x={cx}, y={cy}, Error: {error}")
            # print(f"Right Speed: {right_speed}, Left Speed: {left_speed}")  # Print right_speed and left_speed
    
    # Display the frames
    cv2.imshow("Mask Image", mask)
    cv2.imshow("Webcam Video", video)

    # Exit with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
webcam_video.release()
cv2.destroyAllWindows()
