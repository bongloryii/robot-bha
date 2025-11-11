import cv2
import numpy as np
import serial
import time

# Thiết lập giao tiếp Serial với Arduino
arduino = serial.Serial('COM5', 9600)  # Cập nhật cổng Arduino nếu cần
time.sleep(2)  # Chờ Arduino khởi động

# Thiết lập màu sắc cho vật (màu xanh lá cây)
lower = np.array([36, 50, 50])
upper = np.array([70, 255, 255])

# Mở webcam
webcam_video = cv2.VideoCapture(0, cv2.CAP_DSHOW)

basespeed = 0.005

# Biến trạng thái
state = 1  # Trạng thái ban đầu: xoay tại chỗ
vl = -basespeed  
vr = basespeed  

# Gửi lệnh xoay tại chỗ cho Arduino
arduino.write("R\n".encode())
arduino.write(f"{vl} {vr}\n".encode())
arduino.flush()

while True:
    success, video = webcam_video.read()
    if not success:
        break

    # Lấy kích thước khung hình
    frame_height, frame_width, _ = video.shape
    frame_center_x = frame_width // 2

    # Vẽ đường thẳng ở giữa khung hình
    cv2.line(video, (frame_center_x, 0), (frame_center_x, frame_height), (0, 0, 255), 2)

    # Chuyển khung hình từ BGR sang HSV
    img = cv2.cvtColor(video, cv2.COLOR_BGR2HSV)

    # Lọc màu vật
    mask = cv2.inRange(img, lower, upper)

    # Xử lý mask để loại bỏ nhiễu
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Tìm đường viền của vật
    mask_contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(mask_contours) != 0:
        # Lấy đường viền lớn nhất
        largest_contour = max(mask_contours, key=cv2.contourArea)

        if cv2.contourArea(largest_contour) > 500:
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Vẽ khung bao quanh vật
            cv2.rectangle(video, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Tính tâm vật
            cx = x + w // 2

            if state == 1:
                # Nếu tâm vật nằm gần giữa khung hình, dừng xoay và chuyển trạng thái
                if abs(frame_center_x - cx) < 50: 
                    state = 4 # Stop
                    arduino.write("4\n".encode())  # Dừng xoay
                    vl = 0
                    vr = 0
                    arduino.write(f"{vl} {vr}\n".encode())
                    arduino.flush()
                    time.sleep(1)
                    state = 2  # Chuyển sang trạng thái di chuyển
                else:
                    # Nếu sai lệch lớn, tiếp tục xoay
                    vl = -basespeed  # Một bánh chạy ngược
                    vr = basespeed   # Một bánh chạy xuôi
                    arduino.write(f"{vl} {vr}\n".encode())
                    arduino.flush()

            elif state == 2:
                arduino.write("2\n".encode())
                # Tính toán sai lệch (error) để điều chỉnh tốc độ
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
                
                vl = 0.03 - pid_output
                vr = 0.03 + pid_output
                print(vl, vr)
                    
                # Gửi tốc độ đến Arduino
                arduino.write(f"{vl} {vr}\n".encode())
                arduino.flush()

                # # Nhận dữ liệu khoảng cách từ Arduino
                # if arduino.in_waiting > 0:
                #     distance = float(arduino.readline().decode().strip())
                #     print(f"Distance: {distance} cm")
                print(f"inwating", arduino.in_waiting)
                if arduino.in_waiting > 0:
                    try:
                        distance = float(arduino.readline().decode().strip())
                        print(f"Distance: {distance} cm")
                    except ValueError:
                     print("Invalid data received")

                    # Nếu khoảng cách đủ gần, dừng xe
                    if 0 < distance < 5:
                        arduino.write("4\n".encode())
                        vl = 0
                        vr = 0
                        arduino.write(f"{vl} {vr}\n".encode())
                        arduino.flush()
                        time.sleep(2)
                        state = 3  # Chuyển sang trạng thái gắp chai


            elif state == 3:
                # Gửi lệnh gắp chai đến Arduino
                arduino.write("3\n".encode())
                arduino.flush()
                time.sleep(3)  # Chờ servo gắp chai
             

    # Hiển thị video
    cv2.imshow("Mask", mask)
    cv2.imshow("Video", video)

    # Thoát bằng phím 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng tài nguyên
webcam_video.release()
cv2.destroyAllWindows()
