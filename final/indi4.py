import matplotlib.pyplot as plt
import csv
import math
import numpy as np
import heapq

# Function to process data from a CSV file (no changes here)
def process_csv(file_path, x_offset, y_offset, rotation_angle):
    x_values = []
    y_values = []

    # Read data from CSV file
    with open(file_path, mode='r') as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            x_values.append(float(row['x']))  # Assuming 'x' is the column name
            y_values.append(float(row['y']))  # Assuming 'y' is the column name

    # Convert the rotation angle to radians
    angle_rad = math.radians(rotation_angle)

    # Rotate by the specified angle
    x_rotated = [x * math.cos(angle_rad) - y * math.sin(angle_rad) for x, y in zip(x_values, y_values)]
    y_rotated = [x * math.sin(angle_rad) + y * math.cos(angle_rad) for x, y in zip(x_values, y_values)]

    # Apply the offsets
    x_shifted = [x + x_offset for x in x_rotated]
    y_shifted = [y + y_offset for y in y_rotated]

    return x_shifted, y_shifted

# Function to calculate the Euclidean distance (heuristic for A*)
def heuristic(a, b):
    return 10*np.linalg.norm(np.array(a) - np.array(b))
    # return 10*max(abs(a[0] - b[0]), abs(a[1] - b[1]))
def diagonal_heuristic(x1, y1, x2, y2):
    return max(abs(x2 - x1), abs(y2 - y1))
def to_grid_coords(x, y):
    return int((x - x_min) // cell_width), int((y - y_min) // cell_height)
def astar(start, goal, grid, cell_width, cell_height):
    # Directions for moving: left, right, down, up, and diagonals
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
            return path[::-1], g_score_values, heuristic_values, all_visited_nodes  # Return path, g_score, heuristic, visited nodes
        
         # Check neighbors
        for direction in directions:
            neighbor = (current[0] + direction[0], current[1] + direction[1])
            # Check bounds and obstacles
            if 0 <= neighbor[0] < num_x_cells and 0 <= neighbor[1] < num_y_cells and not grid[neighbor[1], neighbor[0]]:
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
    
    return [], g_score_values, heuristic_values, all_visited_nodes  # No path found


# File paths, offsets, and rotation angles (same as before)
files = [
    ('lidar_pos1.csv', 1.3, 0.38, 230),
    ('lidar_pos2.csv', 1.25, 1.47, 70),
    ('lidar_pos3.csv', 0.95, 0.88, 95)
]

# Collecting scatter points
all_x_points = []
all_y_points = []

plt.figure(figsize=(10, 10))
for file, x_offset, y_offset, rotation_angle in files:
    x_shifted, y_shifted = process_csv(file, x_offset, y_offset, rotation_angle)
    all_x_points.extend(x_shifted)
    all_y_points.extend(y_shifted)
    plt.scatter(x_shifted, y_shifted, marker='.', label=f'Data from {file} (Rotated {rotation_angle}°)')

# Convert scatter points to NumPy arrays for easier comparison
all_x_points = np.array(all_x_points)
all_y_points = np.array(all_y_points)

# Grid settings
x_min, x_max = 0, 3
y_min, y_max = 0, 1.85
num_x_cells, num_y_cells = 10, 10
cell_width = (x_max - x_min) / num_x_cells
cell_height = (y_max - y_min) / num_y_cells
obstacle_grid = np.zeros((num_x_cells, num_y_cells), dtype=bool)

# Shade grid cells for obstacles
for i in range(num_x_cells):
    for j in range(num_y_cells):
        cell_x_min = x_min + i * cell_width
        cell_x_max = cell_x_min + cell_width
        cell_y_min = y_min + j * cell_height
        cell_y_max = cell_y_min + cell_height

        points_in_cell = ((all_x_points >= cell_x_min) & (all_x_points < cell_x_max) &
                          (all_y_points >= cell_y_min) & (all_y_points < cell_y_max))

        if np.any(points_in_cell):
            obstacle_grid[j, i] = True
            plt.gca().add_patch(
                plt.Rectangle((cell_x_min, cell_y_min), cell_width, cell_height,
                              color='black', alpha=0.5)
            )

# Define start and goal in world coordinates
start = (0.5, 0.8)  # Example start point
goal = (2.3,0.8)   # Example goal point

# Convert start and goal to grid coordinates
start_grid = to_grid_coords(start[0], start[1])
goal_grid = to_grid_coords(goal[0], goal[1])

# Find the shortest path using A*
# path = astar(start_grid, goal_grid, obstacle_grid, cell_width, cell_height)
path, g_scores, heuristics, all_visited_nodes = astar(start_grid, goal_grid, obstacle_grid, cell_width, cell_height)
# Draw the path
# if path:
path_x = [start[0]] + [x_min + p[0] * cell_width + cell_width / 2 for p in path] + [goal[0]]
path_y = [start[1]] + [y_min + p[1] * cell_height + cell_height / 2 for p in path] + [goal[1]]
plt.plot(path_x, path_y, marker='o', color='red', label='Shortest Path')

# Plot g_score and heuristic values for all visited nodes
for node in all_visited_nodes:
    grid_x = x_min + node[0] * cell_width + cell_width / 2
    grid_y = y_min + node[1] * cell_height + cell_height / 2
    g_val = g_scores[node]
    heuristic_val = heuristics[node]
    
    # Annotate g_score and heuristic on the plot
    plt.annotate(f'G: {g_val:.0f}\nH: {heuristic_val:.0f}', (grid_x, grid_y), textcoords="offset points", xytext=(-23, 2), ha='left')

plt.plot(path_x, path_y, marker='o', color='red', label='Shortest Path')

# Customize the plot
plt.title('Robot Pathfinding with A* Algorithm', fontsize=16)
plt.xlabel('X Values', fontsize=14)
plt.ylabel('Y Values', fontsize=14)

# Show the grid and path
plt.gca().set_xticks([i * 0.3 for i in range(11)], minor=False)
plt.gca().set_yticks([i * 0.185 for i in range(11)], minor=False)
plt.grid(which='major', linestyle='--', linewidth=0.5)

plt.show()
