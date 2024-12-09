import matplotlib.pyplot as plt

import csv
output_file = 'landmark_data3_nov21_real2.csv'

x_values = []
y_values = []

# Read data from CSV file
with open(output_file, mode='r') as csv_file:
    reader = csv.reader(csv_file)
    for row in reader:
        x_values.append(float(row[0]))  # Assuming X is in the first column
        y_values.append(float(row[1]))  # Assuming Y is in the second column
# Plot x and y values
plt.figure(figsize=(10, 6))
plt.scatter(x_values, y_values, marker='o', linestyle='-', color='b', label='x vs y')

# Customize the plot
plt.title('Plot of X vs Y', fontsize=16)
plt.xlabel('X Values', fontsize=14)
plt.ylabel('Y Values', fontsize=14)
plt.xlim(0,2)
plt.ylim(0,2)
plt.grid(True)
plt.legend(fontsize=12)


# Display the plot
plt.show()