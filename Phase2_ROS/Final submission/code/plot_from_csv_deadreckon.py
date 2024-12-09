import matplotlib.pyplot as plt

import csv
output_file = 'robot_pose_data1.csv'
a_x = 0.93  # Replace with your desired x offset
a_y = 0.28 
x_values = []
y_values = []

# Read data from CSV file
with open(output_file, mode='r') as csv_file:
    reader = csv.DictReader(csv_file)
    for row in reader:
        x_values.append(float(row['X']))  # Assuming X is in the first column
        y_values.append(float(row['Y']))  # Assuming Y is in the second column

# Rotate counterclockwise by 90 degrees
x_rotated = [-y for y in y_values]
y_rotated = [x for x in x_values]

x_shifted = [x + a_x for x in x_rotated]
y_shifted = [y + a_y for y in y_rotated]

# Plot x and y values
plt.figure(figsize=(10, 10))
# plt.scatter(x_values, y_values, marker='o', linestyle='-', color='b', label='x vs y')
plt.scatter(x_shifted, y_shifted, marker='o', linestyle='-', color='r', label='robot position')

# Customize the plot
plt.title('Plot of robot position by dead reckoning', fontsize=16)
plt.xlabel('X Values', fontsize=14)
plt.ylabel('Y Values', fontsize=14)
plt.xlim(0,2)
plt.ylim(0,2)
plt.grid(True)
plt.legend(fontsize=12)


# Display the plot
plt.show()