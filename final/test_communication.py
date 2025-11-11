import serial
import time

# Configure the serial port
arduino = serial.Serial(port='COM6', baudrate=9600, timeout=1)  # Replace 'COM3' with your Arduino's port

time.sleep(2)  # Wait for the connection to establish

# Function to send data to Arduino
def send_to_arduino(command):
    arduino.write(f"{command}\n".encode())  # Send data encoded as bytes
    time.sleep(0.1)  # Small delay

# Function to read data from Arduino
def read_from_arduino():
    if arduino.in_waiting > 0:  # Check if data is available
        data = arduino.readline().decode().strip()  # Read and decode the data
        return data
    return None

# Main loop for duplex communication
try:
    while True:
        # Continuously read incoming data from Arduino
        response = read_from_arduino()
        if response:
            print("Arduino sent:", response)

        # Optionally send commands to Arduino
        # user_input = input("Enter command (or press Enter to skip): ").strip()
        # if user_input:
            # send_to_arduino(user_input)
        # else:
            # pass

except KeyboardInterrupt:
    print("Exiting...")
    arduino.close()
