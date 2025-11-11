
import serial
import time
arduino = serial.Serial('/dev/ttyUSB1', 9600)
time.sleep(2)  # Allow time for Arduino to initialize

"""Check for messages from Arduino."""
while True:
    if arduino.in_waiting > 0:
        message = arduino.readline().decode().strip()
        if message == "!":
            print("found")