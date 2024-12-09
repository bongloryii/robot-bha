import serial
import csv
# import time
# import matplotlib.pyplot as plt
# Serial Port Configuration
port = "COM5"  # Replace with your Arduino's port (e.g., 'COM3' for Windows or '/dev/ttyUSB0' for Linux/Mac)
baudrate = 9600
output_file = 'robot_pose_data1.csv'

# Open Serial Port
ser = serial.Serial(port, baudrate, timeout=2)
# while (ser.in_waiting ==0):
#     pass
# Open CSV File for Writing
with open(output_file, 'w', newline='') as csvfile:
    fieldnames = ['X', 'Y', 'Theta']
    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
    writer.writeheader()  # Write CSV header

    print("Receiving data... Press Ctrl+C to stop.")
    try:
        while True:
            line = ser.readline().decode().strip()  # Read a line from serial
            print(line,"\n")
            if line:
                # Split the line into x, y, theta
                data = line.split(',')
                print("line is",data)
                if len(data) == 2:
                    x, y = map(float, data)
                    print(f"X: {x}, Y: {y}")
                    
                    # Write to CSV file
                    writer.writerow({'X': x, 'Y': y})
    except KeyboardInterrupt:
        print("\nData collection stopped.")
    finally:
        ser.close()
