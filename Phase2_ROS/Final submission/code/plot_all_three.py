import matplotlib.pyplot as plt
import csv

# Parameters for the first plot (translated dead reckoning)
output_file1 = 'robot_pose_data1.csv'
a_x = 0.93
a_y = 0.28
x_dead = []
y_dead = []

with open(output_file1, mode='r') as csv_file:
    reader = csv.DictReader(csv_file)
    for row in reader:
        x_dead.append(float(row['X']))
        y_dead.append(float(row['Y']))

x_rotated = [-y for y in y_dead]
y_rotated = [x for x in x_dead]

x_shifted = [x + a_x for x in x_rotated]
y_shifted = [y + a_y for y in y_rotated]

# Parameters for the second plot (ground truth ABCDE)
points = {
    "A": (0.93, 0.28),
    "B": (0.83, 1.19),
    "C": (1.41, 1.38),
    "D": (1.75, 0.44),
    "E": (1.29, 0.22)
}

landmarks = {
    # "LM1": (0.38, 0.42),
    # "LM2": (0.38, 1.45)
    "LM1": (0.31, 0.40),
    "LM2": (0.32, 1.4)
}

x_abcde = [points[key][0] for key in points]
y_abcde = [points[key][1] for key in points]

x_landmarks = [landmarks[key][0] for key in landmarks]
y_landmarks = [landmarks[key][1] for key in landmarks]

# Parameters for the third plot (lidar data)
# output_file2 = 'landmark_data9_nov21_real.csv'
output_file2 = 'landmark_data9_nov21_2ndmeasure.csv'

x_lidar = []
y_lidar = []

with open(output_file2, mode='r') as csv_file:
    reader = csv.reader(csv_file)
    for row in reader:
        x_lidar.append(float(row[0]))
        y_lidar.append(float(row[1]))

# Plot all data
plt.figure(figsize=(10, 10))

# Plot translated dead reckoning
plt.scatter(x_shifted, y_shifted, marker='o', color='r', label='Dead Reckoning (Translated)')

# Plot ground truth ABCDE
plt.plot(x_abcde, y_abcde, marker='o', color='b', label='Ground Truth ABCDE Path')

# # Annotate points A, B, C, D, E
# for label, (x, y) in points.items():
#     plt.scatter(x, y, color='blue')  # Mark the point
#     plt.text(x, y + 0.02, f'{label}', fontsize=20, color='black')  # Label above the point

plt.scatter(
    x_landmarks[0], y_landmarks[0],  # Coordinates of LM1
    color='g', s=150, label='Landmark 1 '
)
plt.scatter(
    x_landmarks[1], y_landmarks[1],  # Coordinates of LM2
    color='g', s=30, label='Landmark 2'
)
# Plot lidar data
plt.scatter(x_lidar, y_lidar, marker='x', color='purple', label='Lidar Data')

# Formatting
plt.title('Combined Plot of Dead Reckoning, Ground Truth, and Lidar Data', fontsize=16)
plt.xlabel('X Values', fontsize=14)
plt.ylabel('Y Values', fontsize=14)
plt.xlim(0, 2)
plt.ylim(0, 2)
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.legend(fontsize=12)
plt.grid(True)

# Show the plot
plt.show()
