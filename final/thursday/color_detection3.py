import cv2
import numpy as np
import serial
import time

# Set up serial communication with Arduino
arduino = serial.Serial('/dev/ttyUSB0', 9600)  # Replace 'COM11' with your Arduino port
time.sleep(2)  # Wait for Arduino to initialize

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

# Capture video from webcam
webcam_video = cv2.VideoCapture(0)


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
    mask = cv2.inRange(img, lower_limit, upper_limit)

    # Clean up the mask with morphological operations
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # Closing operation to merge nearby areas

    # Find contours in the mask
    mask_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # If an object is detected
    if len(mask_contours) != 0                                                                                                                  :
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

            Kp = 0.00001  # Proportional constant
            Ki = 0.0  # Integral constant
            Kd = 0.0  # Derivative constant

            # Initialize PID variables
            previous_error = 0
            integral = 0
         
            proportional = error
            integral += error
            derivative = error - previous_error
            
            # PID output
            pid_output = Kp * proportional + Ki * integral + Kd * derivative
            print(pid_output)
            
            # Adjust motor speeds based on PID output
            #if error > 0:  # Object is to the left
             #   vl = max(0.03, 0.05 - pid_output)  # Slow down left wheel
              #  vr = 0.05 + pid_output  # Speed up right wheel
            #elif error < 0:  # Object is to the right
             #   vl = 0.05 + abs(pid_output)  # Speed up left wheel
              #  vr = max(0.03, 0.05 - abs(pid_output))  # Slow down right wheel
            #else:  # Object is centered
             #   vl = vr = 0.03  # Move forward
            vl = 0.03 - pid_output
            vr = 0.03 + pid_output
            print(vl, vr)
          

            # Update previous error
            previous_error = error
           
            arduino.write(f"{vr} {vl}\n".encode())  # Sending velocity for right and left motor
            arduino.flush() 

            # Display the error on the video feed
            cv2.putText(video, f"Error: {error}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

            # Show object center and error for debugging
            # print(f"Object Center: x={cx}, y={cy}, Error: {error}")
           

                   
          
    # Display the frames
    cv2.imshow("Mask Image", mask)
    cv2.imshow("Webcam Video", video)

    # Exit with 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release resources
webcam_video.release()
cv2.destroyAllWindows()
