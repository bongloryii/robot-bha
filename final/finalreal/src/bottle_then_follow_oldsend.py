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
from pathplanner import *
from objecttrack import *
# Initialize serial communication
arduino = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)  # Allow time for Arduino to initialize
print("arduino found")

isDebug=False
WHEEL_DISTANCE=0.182


current_state = 1

def main(args=None):
    rclpy.init(args=args)
    global current_state
    while (current_state<4):
        if current_state == 1:  # Finding bottle
            object_detect = ObjectTrackingNode()
            object_detect.spin()
            object_detect.destroy_node()
        elif current_state >= 2:  # Bottle found, generate and find path
            
            path_planner = PathPlanner()
            rclpy.spin(path_planner)
            path_planner.destroy_node()
            current_state == 4
            # time.sleep(1)            
    rclpy.shutdown()



if __name__ == '__main__':
    main()
