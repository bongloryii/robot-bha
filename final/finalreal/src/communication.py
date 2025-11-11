import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time

class ArduinoCommNode(Node):
    def __init__(self):
        super().__init__('arduino_comm_node')

        # Serial port configuration
        self.serial_port = '/dev/ttyUSB0'  # Replace with your Arduino's port
        self.baud_rate = 9600

        # Initialize serial connection
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.serial_port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            raise

        # Publisher and subscriber
        self.publisher = self.create_publisher(String, 'arduino_response', 10)
        self.subscription = self.create_subscription(
            String,
            'arduino_command',
            self.command_callback,
            10
        )

        # Timer to read Arduino responses
        self.timer = self.create_timer(0.1, self.read_arduino)

    def command_callback(self, msg):
        command = msg.data
        self.get_logger().info(f"Sending command to Arduino: {command}")
        try:
            self.serial_conn.write((command + '\n').encode())
        except serial.SerialException as e:
            self.get_logger().error(f"Error writing to serial port: {e}")

    def read_arduino(self):
        try:
            if self.serial_conn.in_waiting > 0:
                response = self.serial_conn.readline().decode().strip()
                if response:
                    self.get_logger().info(f"Received from Arduino: {response}")
                    response_msg = String()
                    response_msg.data = response
                    self.publisher.publish(response_msg)
        except serial.SerialException as e:
            self.get_logger().error(f"Error reading from serial port: {e}")

    def destroy_node(self):
        # Close serial connection when shutting down
        if self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoCommNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
