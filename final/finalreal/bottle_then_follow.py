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
# Initialize serial communication
arduino = serial.Serial('/dev/ttyUSB0', 9600)
time.sleep(2)  # Allow time for Arduino to initialize
print("arduino found")
arduino.write(f"1,{40},{-40}\n".encode())

isDebug=False
WHEEL_DISTANCE=0.182


current_state = 1

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.timer = self.create_timer(0.5, self.follow_path_callback)

        # Add subscriber to robot pose
        self.pose_subscriber = self.create_subscription(
            Pose,  # Pose message type
            '/robot_pose',  # Topic name
            self.pose_callback,  # Callback function
            10  # Queue size
        )
        # Initialize robot's current pose
        self.current_position = None  # To store the position
        # self.current_orientation = None  # To store the orientation
        self.current_yaw = None  # To store the yaw angle (optional)

        # Grid map metadata
        self.grid_resolution = 0.2  # Grid resolution in meters
        self.grid_origin = (0.0, 0.0)  # Origin of the grid in map coordinates

        # Grid settings
        self.x_min, self.x_max = -0.2, 4
        self.y_min, self.y_max = -0.2, 4
        self.num_x_cells = int((self.x_max - self.x_min) / self.grid_resolution)
        self.num_y_cells = int((self.y_max - self.y_min) / self.grid_resolution)
        # Define start and goal positions in map coordinates
        self.start_map = None  # Will be dynamically set
        self.goal_map = (2, 1)  # Map coordinates (x, y)

        # Initialize path-following variables
        self.current_waypoint_index = 0
        
        # Load map data and create a grid
        self.grid_map = self.load_and_preprocess_map('real_environment_map.csv')
        self.scale_and_save_grid_map()

        # Path points
        self.path_grid = []
        self.path_real = []
        self.all_x_points = np.array([])
        self.all_y_points = np.array([])

    def pose_callback(self, msg):
        """Callback to update the robot's current position."""
        # Store position and orientation as instance variables
        self.current_position = msg.position
        # self.current_orientation = msg.orientation
        self.current_yaw=msg.orientation.z
        # Set start position dynamically if not already set
        if self.start_map is None or not self.path_grid:
            self.start_map = self.current_position
            self.get_logger().info(f"Start position set to current position: {self.start_map[0]:.2f},{self.start_map[1]:.2f}")

            # Convert start and goal to grid coordinates
            self.start = self.map_to_grid(self.start_map)
            self.goal = self.map_to_grid(self.goal_map)

            # Generate path
            self.path_grid = self.generate_path(self.start, self.goal)
            self.path_real = [self.grid_to_map(point) for point in self.path_grid]
            self.path_real.append(self.goal_map)
            global current_state
            current_state=3
            # Debug: Print path points
            self.get_logger().info('Generated Path Points:')
            for point in self.path_real:
                self.get_logger().info(f'{point[0]:.2f}, {point[1]:.2f}')

            # Visualize the grid and path
            self.visualize_path()
    def get_ahead_point(self,ahead_distance=0.15):
       
        x_ap = self.current_position.x*(1+np.cos(self.current_yaw)*ahead_distance)
        y_ap = self.current_position.y*(1+np.sin(self.current_yaw)*ahead_distance)
        return np.array([x_ap, y_ap])

    def find_closest_path_point(self,ahead_point, path):
        """
        Finds the closest point on the path to the ahead_point in terms of lateral distance.
        Prioritizes points with a higher index in case of equal distances.

        :param ahead_point: A tuple (x, y) representing the ahead point.
        :param path: A list of tuples [(x1, y1), (x2, y2), ...] representing the path.
        :return: A tuple (closest_point, closest_distance, closest_index) where:
                - closest_point is the closest point on the path.
                - closest_distance is the lateral distance to that point.
                - closest_index is the index of the point on the path.
        """
        ahead_x, ahead_y = ahead_point
        closest_point = None
        closest_distance = float('inf')
        closest_index = -1

        for i in range(len(path) - 1):
            # Get the endpoints of the current segment
            x1, y1 = path[i]
            x2, y2 = path[i + 1]

            # Calculate the projection of ahead_point onto the line segment
            dx, dy = x2 - x1, y2 - y1
            if dx == 0 and dy == 0:
                # Handle degenerate case: segment is a single point
                distance = np.sqrt((ahead_x - x1) ** 2 + (ahead_y - y1) ** 2)
                point_on_segment = (x1, y1)
            else:
                # Parameterize the line segment
                t = max(0, min(1, ((ahead_x - x1) * dx + (ahead_y - y1) * dy) / (dx ** 2 + dy ** 2)))
                point_on_segment = (x1 + t * dx, y1 + t * dy)
                distance = np.sqrt((ahead_x - point_on_segment[0]) ** 2 + (ahead_y - point_on_segment[1]) ** 2)

            # Update the closest point and distance if necessary
            if distance < closest_distance or (distance == closest_distance and i + 1 > closest_index):
                closest_distance = distance
                closest_point = point_on_segment
                closest_index = i + 1

        return closest_point, closest_distance, closest_index

    def map_to_grid(self, map_coords):
        """Convert map coordinates to grid coordinates."""
        map_x, map_y = map_coords
        grid_x = int((map_x - self.x_min) / self.grid_resolution)
        grid_y = int((map_y - self.y_min) / self.grid_resolution)
        return (grid_x, grid_y)

    def grid_to_map(self, grid_coords):
        """Convert grid coordinates to map coordinates."""
        grid_x, grid_y = grid_coords
        map_x = (grid_x+0.5) * self.grid_resolution + self.x_min
        map_y = (grid_y+0.5) * self.grid_resolution + self.y_min
        return (map_x, map_y)
    def load_and_preprocess_map(self, file_path):
        
        x_values = []
        y_values = []

        # Read data from CSV file
        with open(file_path, mode='r') as csv_file:
            reader = csv.DictReader(csv_file)
            for row in reader:
                x_values.append(float(row['X']))  # Assuming X is in the first column
                y_values.append(float(row['Y']))  # Assuming Y is in the second column

        # Convert scatter points to NumPy arrays for easier comparison
        self.all_x_points = np.array(x_values)
        self.all_y_points = np.array(y_values)
        obstacle_grid = np.zeros((self.num_y_cells, self.num_x_cells), dtype=bool)

        # Shade grid cells for obstacles
        for i in range(self.num_x_cells):
            for j in range(self.num_y_cells):
                cell_x_min = self.x_min + i * self.grid_resolution
                cell_x_max = cell_x_min + self.grid_resolution
                cell_y_min = self.y_min + j * self.grid_resolution
                cell_y_max = cell_y_min + self.grid_resolution

                points_in_cell = ((self.all_x_points >= cell_x_min) & (self.all_x_points < cell_x_max) &
                                (self.all_y_points >= cell_y_min) & (self.all_y_points < cell_y_max))

                if np.any(points_in_cell):
                    obstacle_grid[j, i] = True
                    # plt.gca().add_patch(
                    #     plt.Rectangle((cell_x_min, cell_y_min), self.grid_resolution, self.grid_resolution,
                    #                 color='black', alpha=0.5)
                    # )

        return obstacle_grid
    

    def generate_path(self, start, goal, dilation_radius=1):
        def heuristic(a, b):
            # return 10*np.linalg.norm(np.array(a) - np.array(b))
            return 10*max(abs(a[0] - b[0]), abs(a[1] - b[1]))

        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),  # Horizontal and vertical
                    (-1, -1), (-1, 1), (1, -1), (1, 1)]  # Diagonal directions

        # Create a dilated obstacle grid using binary dilation
        # 1 is considered as obstacle and 0 as free space
        dilated_obstacles = binary_dilation(self.grid_map, structure=np.ones((2 * dilation_radius + 1, 2 * dilation_radius + 1)))
        self.grid_map=dilated_obstacles
        dilated_obstacles = dilated_obstacles.astype(int)
        # plt.figure(figsize=(6,6))
        # plt.imshow(self.grid_map, cmap='gray_r', origin='lower')
        # print(self.grid_map)
        # plt.show()
        # Priority queue (open set) for A* with (f_score, current_position)
        open_set = []
        heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))

        # Maps to track the shortest path
        came_from = {}
        g_score = {start: 0}

        # To track g_score and heuristic for plotting
        g_score_values = {start: 0}
        heuristic_values = {start: heuristic(start, goal)}

        # To track all visited nodes for displaying g_score and heuristic
        all_visited_nodes = []

        while open_set:
            _, current_g_score, current = heapq.heappop(open_set)

            # Record all visited nodes
            all_visited_nodes.append(current)

            # If we reach the goal, reconstruct the path
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                np.save("path.npy", path[::-1])
                return path[::-1]  # Return path

            # Check neighbors
            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])

                # Check bounds and obstacles in dilated grid
                if 0 <= neighbor[0] < self.num_x_cells and 0 <= neighbor[1] < self.num_y_cells and dilated_obstacles[neighbor[1], neighbor[0]] == 0:
                    # For diagonals, the cost is 14, for horizontal/vertical, it's 10
                    if direction in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:  # Diagonal directions
                        tentative_g_score = current_g_score + 14
                    else:  # Horizontal or vertical
                        tentative_g_score = current_g_score + 10

                    if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_g_score
                        g_score_values[neighbor] = tentative_g_score
                        heuristic_values[neighbor] = heuristic(neighbor, goal)
                        f_score = tentative_g_score + heuristic_values[neighbor]
                        heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))

        return []  # No path found
    def visualize_path(self):
        plt.figure(figsize=(8, 8))

        plt.scatter(self.all_x_points, self.all_y_points, marker='.', label=f'Data from map')
        # Shade grid cells for obstacles
        for i in range(self.num_x_cells):
            
            for j in range(self.num_y_cells):
                if self.grid_map[j, i]:  # Check if the cell is occupied
                    cell_x_min = self.x_min + i * self.grid_resolution
                    cell_y_min = self.y_min + j * self.grid_resolution
                    
                    plt.gca().add_patch(
                    plt.Rectangle(
                        (cell_x_min, cell_y_min),
                        self.grid_resolution,
                        self.grid_resolution,
                        color="black",
                        alpha=0.5,
                    )
                )
        # plt.imshow(self.grid_map, origin='lower', cmap='gray_r')
        path_x = [self.start_map[0]] + [self.x_min + p[0] * self.grid_resolution + self.grid_resolution / 2 for p in self.path_grid] + [self.goal_map[0]]
        path_y = [self.start_map[1]] + [self.y_min + p[1] * self.grid_resolution + self.grid_resolution / 2 for p in self.path_grid] + [self.goal_map[1]]
        # path_x = [self.start_map[0]] + [self.x_min + p[0] * self.grid_resolution for p in self.path_grid] + [self.goal_map[0]]
        # path_y = [self.start_map[1]] + [self.y_min + p[1] * self.grid_resolution for p in self.path_grid] + [self.goal_map[1]]
        
        plt.plot(path_x, path_y, marker='o', color='red', label='Shortest Path')
       
        plt.title('Path Planning on Grid Map')
        plt.xlabel('Grid X')
        plt.ylabel('Grid Y')
        plt.legend()
        # plt.grid()
        # Show the grid and path
        plt.savefig("path.png")
        # Adjust ticks: multiply by grid_resolution for real-world coordinates
        # plt.show()

    def scale_and_save_grid_map(self, output_file="grid_map.npy"):
        # Original grid dimensions
        grid_height, grid_width = self.grid_map.shape
        
        # Destination coordinate bounds
        x_range = self.x_max - self.x_min
        y_range = self.y_max - self.y_min
        
        # Compute scaling factors based on the destination size
        x_scale = x_range / grid_width
        y_scale = y_range / grid_height
        
        # Rescale the grid map
        scaled_grid_map = zoom(self.grid_map, (y_scale, x_scale), order=1)
        
        # Save scaled grid map to .npy
        np.save(output_file, scaled_grid_map)
        print(f"Scaled grid map saved to {output_file}")
    def follow_path_callback(self): #with pid and ahead point
        if self.current_position is None or self.current_waypoint_index >= len(self.path_real):
            return
        
        # Get the current position and orientation
        current_x = self.current_position.x
        current_y = self.current_position.y
        x, y = self.get_ahead_point()
        
        _,__, waypoint_index = self.find_closest_path_point(self.get_ahead_point(), self.path_real)
        if waypoint_index>self.current_waypoint_index:
            self.current_waypoint_index=waypoint_index
        target_waypoint = self.path_real[self.current_waypoint_index]
        x_g, y_g = target_waypoint

        if self.current_waypoint_index==len(self.path_real)-1:
            x=current_x
            y=current_y
        gamma =0.12 #linear
        lamda=0.008 #angular
        h=0.01 #phi
        theta = self.current_yaw

        if self.current_waypoint_index==len(self.path_real)-1:
            theta_g = math.atan2(y_g-self.path_real[self.current_waypoint_index-1][1],x_g-self.path_real[self.current_waypoint_index-1][0])
        else:
            theta_g = math.atan2(self.path_real[self.current_waypoint_index+1][1]-y_g,self.path_real[self.current_waypoint_index+1][0]-x_g)
        
        deltaX = x_g - x
        deltaY = y_g - y
        print(f"current pursuit: [{self.current_waypoint_index}] {x_g:.2f},{y_g:.2f},{theta_g:.2f}")

        rho = math.sqrt(pow(deltaX, 2) + pow(deltaY, 2));
        phi = math.atan2(deltaY, deltaX) - theta_g;
        alpha = math.atan2(deltaY, deltaX) - theta;
        v = gamma * math.cos(alpha) * rho;
        w = lamda * alpha + gamma * math.cos(alpha) * math.sin(alpha) * (alpha + h * phi) / alpha;
        
        if rho>0.12:
            
            print(f"Publishing: rho: {rho:.2f}; v: {v:.2f}; w {w:.2f}")
            self.vr = v + WHEEL_DISTANCE * w / 2;
            self.vl = v - WHEEL_DISTANCE * w / 2;
            
            # arduino.write(f"3,{self.vr},{self.vl}\n".encode())
            # arduino.flush() 
        else:
            self.current_waypoint_index += 1
            if self.current_waypoint_index==len(self.path_real):
                print("GOAL REACHED!")
                
                arduino.write(f"3,{0},{0}\n".encode())


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
        self.timer = self.create_timer(0.5, self.send_command)
        self.detected=False
        # Capture video from webcam
        self.webcam_video = cv2.VideoCapture(0,cv2.CAP_V4L2)
        self.vr=-40
        self.vl=40
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
            arduino.write(f"{current_state},{self.vr},{self.vl}\n".encode())
            
            time.sleep(0.1)  # Allow some time for Arduino to process
            # Listen for Arduino response
            response = arduino.readline().decode().strip()
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
                # arduino.write(f"1,{0},{0}\n".encode())
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
                # arduino.write(f"1,{self.vr},{self.vl}\n".encode())
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
                # arduino.write(f"1,{self.vr},{self.vl}\n".encode())
                # self.send_command

        if isDebug:
            # Display the frames
            cv2.imshow("Mask Image", mask)
            cv2.imshow("Webcam Video", video)
    
    def bottle_found(self):
        """Check for messages from Arduino."""
        if arduino.in_waiting > 0:
            message = arduino.readline().decode().strip()
            if message == "X":
                self.get_logger().info("Bottle found!")

                # print("Bottle found!")
                global current_state
                current_state= 2
                print("Stopping motors and generating path...")
                arduino.write(f"2,{0},{0}\n".encode())
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
