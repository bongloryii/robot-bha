import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import csv
import matplotlib.pyplot as plt
import math
import numpy as np
import heapq
from geometry_msgs.msg import Pose  # Assuming PoseStamped is used
from scipy.ndimage import zoom

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
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
        self.grid_resolution = 0.35  # Grid resolution in meters
        self.grid_origin = (0.0, 0.0)  # Origin of the grid in map coordinates

        # Grid settings
        self.x_min, self.x_max = -0.2, 4
        self.y_min, self.y_max = -0.2, 4
        self.num_x_cells = int((self.x_max - self.x_min) / self.grid_resolution)
        self.num_y_cells = int((self.y_max - self.y_min) / self.grid_resolution)
        # Define start and goal positions in map coordinates
        self.start_map = (1, 3)  # Map coordinates (x, y)
        self.goal_map = (3.1, 1.5)  # Map coordinates (x, y)

        # Convert map coordinates to grid coordinates
        self.start = self.map_to_grid(self.start_map)
        self.goal = self.map_to_grid(self.goal_map)
        self.all_x_points = np.array([])
        self.all_y_points = np.array([])

        # Load map data and create a grid
        self.grid_map = self.load_and_preprocess_map('Sim_WorldFinalV3.csv')
        self.scale_and_save_grid_map()

        # Generate path using A*
        self.path_grid = self.generate_path(self.start, self.goal)
        self.path_real = [self.grid_to_map(point) for point in self.path_grid]
        print(self.path_grid)
        print(self.path_real)

        # Debug: Print path points
        self.get_logger().info('Generated Path Points:')
        for point in self.path_grid:
            self.get_logger().info(f'{self.grid_to_map(point)}')  # Convert back to map coordinates for debugging

        # Visualize the grid and path
        self.visualize_path()

        # Initialize path-following variables
        self.current_waypoint_index = 0

    def pose_callback(self, msg):
        # Store position and orientation as instance variables
        self.current_position = msg.position
        # self.current_orientation = msg.orientation
        self.current_yaw=msg.orientation.z
        # Convert quaternion to Euler angles and store yaw
        # _, _, self.current_yaw = self.quaternion_to_euler(self.current_orientation)

        # Log the pose data
        # self.get_logger().info(f"ICP: x={self.current_position.x}, y={self.current_position.y}, yaw={self.current_yaw}")
        # self.get_logger().info(f"Orientation updated (Yaw): yaw={self.current_yaw}")

    # def use_pose_data(self):
    #     # Check if data is available
    #     if self.current_position and self.current_orientation:
    #         # Access the stored position and orientation
    #         x = self.current_position.x
    #         y = self.current_position.y
    #         yaw = self.current_yaw

    #         # Perform actions using the pose data
    #         self.get_logger().info(f"Using stored position: x={x}, y={y}")
    #         self.get_logger().info(f"Using stored yaw: {yaw}")
    #     else:
    #         self.get_logger().warn("Pose data is not available yet.")
    def get_ahead_point(self,ahead_distance=0.15):
       
        x_ap = self.pose[0]*(1+np.cos(self.pose[2])*ahead_distance)
        y_ap = self.pose[1]*(1+np.sin(self.pose[2])*ahead_distance)
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
    def quaternion_to_euler(self, quaternion):
        # Convert a quaternion to Euler angles (roll, pitch, yaw)
        q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
        euler = self.quaternion_to_euler_np(q)  # Assuming you will use numpy for conversion
        return euler[0], euler[1], euler[2]

    def quaternion_to_euler_np(self, q):
        # Conversion logic from quaternion to Euler angles using numpy
        w, x, y, z = q
        roll = math.atan2(2*(w*x + y*z), 1 - 2*(x**2 + y**2))
        pitch = math.asin(2*(w*y - z*x))
        yaw = math.atan2(2*(w*z + x*y), 1 - 2*(y**2 + z**2))
        return np.array([roll, pitch, yaw])

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
    def generate_path(self, start, goal):
        # def heuristic(a, b):
        #     return abs(a[0] - b[0]) + abs(a[1] - b[1])  # Manhattan distance

        # frontier = PriorityQueue()
        # frontier.put((0, start))
        # came_from = {}
        # cost_so_far = {start: 0}

        # while not frontier.empty():
        #     _, current = frontier.get()

        #     if current == goal:
        #         break

        #     # Define valid neighbors (4-connected grid)
        #     neighbors = [
        #         (current[0] + dx, current[1] + dy)
        #         for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
        #     ]

        #     for neighbor in neighbors:
        #         # Check if within bounds and not an obstacle
        #         if (
        #             0 <= neighbor[0] < self.grid_map.shape[0]
        #             and 0 <= neighbor[1] < self.grid_map.shape[1]
        #             and self.grid_map[neighbor[0], neighbor[1]] == 0
        #         ):
        #             new_cost = cost_so_far[current] + 1  # Uniform cost
        #             if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
        #                 cost_so_far[neighbor] = new_cost
        #                 priority = new_cost + heuristic(goal, neighbor)
        #                 frontier.put((priority, neighbor))
        #                 came_from[neighbor] = current

        # # Reconstruct path
        # path = []
        # current = goal
        # while current != start:
        #     path.append(current)
        #     current = came_from.get(current, start)
        # path.append(start)
        # path.reverse()
        # return path
        # Directions for moving: left, right, down, up, and diagonals
        # Function to calculate the Euclidean distance (heuristic for A*)
        def heuristic(a, b):
            # return 10*np.linalg.norm(np.array(a) - np.array(b))
            return 10*max(abs(a[0] - b[0]), abs(a[1] - b[1]))
       
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1),  # Horizontal and vertical
                    (-1, -1), (-1, 1), (1, -1), (1, 1)]  # Diagonal directions
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
                # return path[::-1], g_score_values, heuristic_values, all_visited_nodes  # Return path, g_score, heuristic, visited nodes
                return path[::-1] # Return path, g_score, heuristic, visited nodes
            
            # Check neighbors
            for direction in directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                # Check bounds and obstacles
                if 0 <= neighbor[0] < self.num_x_cells and 0 <= neighbor[1] < self.num_y_cells and not self.grid_map[neighbor[1], neighbor[0]]:
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
        
        # return [], g_score_values, heuristic_values, all_visited_nodes  # No path found
        return []
    def visualize_path(self):
        plt.figure(figsize=(8, 8))

        plt.scatter(self.all_x_points, self.all_y_points, marker='.', label=f'Data from map')
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
                    self.grid_map[j, i] = True
                    plt.gca().add_patch(
                        plt.Rectangle((cell_x_min, cell_y_min), self.grid_resolution, self.grid_resolution,
                                    color='black', alpha=0.5)
                    )
        # plt.imshow(self.grid_map, origin='lower', cmap='gray_r')
        path_x = [self.start_map[0]] + [self.x_min + p[0] * self.grid_resolution + self.grid_resolution / 2 for p in self.path_grid] + [self.goal_map[0]]
        path_y = [self.start_map[1]] + [self.y_min + p[1] * self.grid_resolution + self.grid_resolution / 2 for p in self.path_grid] + [self.goal_map[1]]
        # path_x = [self.start_map[0]] + [self.x_min + p[0] * self.grid_resolution for p in self.path_grid] + [self.goal_map[0]]
        # path_y = [self.start_map[1]] + [self.y_min + p[1] * self.grid_resolution for p in self.path_grid] + [self.goal_map[1]]
        
        plt.plot(path_x, path_y, marker='o', color='red', label='Shortest Path')
       
        # Plot the path
        # path = np.array(self.path_grid)
        # plt.plot(path[:, 0], path[:, 1], c='blue', linewidth=2, label='Path', marker='o')

        # Start and Goal
        # plt.scatter(self.start[0], self.start[1], c='green', s=100, label='Start')
        # plt.scatter(self.goal[0], self.goal[1], c='red', s=100, label='Goal')
        
        plt.title('Path Planning on Grid Map')
        plt.xlabel('Grid X')
        plt.ylabel('Grid Y')
        plt.legend()
        # plt.grid()
        # Show the grid and path

        # Adjust ticks: multiply by grid_resolution for real-world coordinates
        plt.show()

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
    def follow_path_callback(self):
        # Check if the path and pose data are available
        if self.current_position is None or self.current_waypoint_index > len(self.path_real):
            return
        if self.current_waypoint_index==len(self.path_real):
            target_waypoint = self.goal_map
        else:
            # Get the current waypoint
            target_waypoint = self.path_real[self.current_waypoint_index]
        target_x, target_y = target_waypoint

        # Get the current position and orientation
        current_x = self.current_position.x
        current_y = self.current_position.y
        current_yaw = self.current_yaw

        # Compute the distance and angle to the target waypoint
        distance_to_target = math.sqrt((target_x - current_x)**2 + (target_y - current_y)**2)
        if self.current_waypoint_index==0:
            target_angle = math.atan2(target_y -self.start_map[1],target_x-self.start_map[0])
        else:
            target_angle = math.atan2(target_y -self.path_real[self.current_waypoint_index-1][1],target_x-self.path_real[self.current_waypoint_index-1][0])
        print(f"current pursuit: [{self.current_waypoint_index}] {target_x},{target_y}, {target_angle}")

        # target_angle = math.atan2(target_y - current_y, target_x - current_x)
        
        # Calculate the angle difference
        angle_difference = target_angle - current_yaw
        angle_difference = math.atan2(math.sin(angle_difference), math.cos(angle_difference))  # Normalize to [-pi, pi]
        print(f"angle difference{angle_difference}")
        # Initialize Twist message
        cmd_vel = Twist()

        # Define thresholds for proximity and angular alignment
        proximity_threshold = 0.14  # meters
        angular_threshold = 0.4  # radians

        # Control logic
        if distance_to_target > proximity_threshold:
            if abs(angle_difference) > angular_threshold:
                # Rotate towards the target
                cmd_vel.linear.x = -0.05

                cmd_vel.angular.z = 0.45 * angle_difference  # Proportional control
            else:
                # Move forward if aligned
                cmd_vel.linear.x = 0.12
                cmd_vel.angular.z =0.0
        else:
            # Move to the next waypoint if close enough
            self.current_waypoint_index += 1

        # Publish the velocity command
        self.publisher.publish(cmd_vel)
        print(f"Publishing: linear velocity: {cmd_vel.linear.x}; angular velocity {cmd_vel.angular.z}")
    # def follow_path_callback(self):
    #     if self.current_position is None or self.current_waypoint_index > len(self.path_real):
    #         return
    #     gamma =0.5 #linear
    #     lamda=0.15 #angular
    #     h=0.1 #phi
    #     x = self.current_position.x
    #     y = self.current_position.y
    #     theta = self.current_yaw
    #     if self.current_waypoint_index==len(self.path_grid):
    #         x_g,y_g= self.goal_map
    #         theta_g = math.atan2(y_g-self.path_real[self.current_waypoint_index-1][1],x_g-self.path_real[self.current_waypoint_index-1][0])
    #     else:
    #         x_g,y_g = self.path_real[self.current_waypoint_index]
    #         if self.current_waypoint_index==(len(self.path_grid)-1):
    #             theta_g = math.atan2(self.goal_map[1]-y_g,self.goal_map[0]-x_g)
    #         else:
    #             theta_g = math.atan2(self.path_real[self.current_waypoint_index+1][1]-y_g,self.path_real[self.current_waypoint_index+1][0]-x_g)
        
    #     # theta_g = math.atan2(self.path_real[self.current_waypoint_index][1]-y_g,self.path_real[self.current_waypoint_index][0]-x_g)
    #     deltaX = x_g - x;
    #     deltaY = y_g - y;
    #     # if self.current_waypoint_index==0:
    #     #     theta_g = math.atan2(y_g-self.start_map[1],x_g-self.start_map[0])
    #     # else:
    #     #     theta_g = math.atan2(y_g-self.path_real[self.current_waypoint_index-1][1],x_g-self.path_real[self.current_waypoint_index-1][0])
        
    #     print(f"current pursuit: [{self.current_waypoint_index}] {x_g},{y_g},{theta_g}")

    #     rho = math.sqrt(pow(deltaX, 2) + pow(deltaY, 2));
    #     phi = math.atan2(deltaY, deltaX) - theta_g;
    #     alpha = math.atan2(deltaY, deltaX) - theta;
    #     v = gamma * math.cos(alpha) * rho;
    #     w = lamda * alpha + gamma * math.cos(alpha) * math.sin(alpha) * (alpha + h * phi) / alpha;
    #     # vr = v + WHEEL_DISTANCE * w / 2;
    #     # vl = v - WHEEL_DISTANCE * w / 2;
        
    #     # Move toward the target
    #     twist = Twist()
    #     if rho>0.2:
    #         twist.angular.z = w
    #         twist.linear.x = v
    #     else:
    #         self.current_waypoint_index += 1
    #         if self.current_waypoint_index==len(self.path_real):
    #             print("GOAL REACHED!")
    #             twist.angular.z = 0.0
    #             twist.linear.x = 0.0
    #     # Publish commands
    #     self.publisher.publish(twist)
    #     print(f"Publishing: rho: {rho}; v: {v}; w {w}")

def main(args=None):
    rclpy.init(args=args)
    path_planner = PathPlanner()
    rclpy.spin(path_planner)
    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
