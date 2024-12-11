import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
data = pd.read_csv('real_environment_map.csv')

# Plot x vs. y
plt.scatter(data['X'], data['Y'])
plt.xlabel('X-axis Label')
plt.ylabel('Y-axis Label')
plt.title('X vs Y Plot')
plt.grid(True)
plt.show()
