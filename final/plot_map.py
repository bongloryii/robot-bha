import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
data = pd.read_csv('Sim_WorldFinalV3.csv')

# Plot x vs. y
plt.plot(data['x'], data['y'], marker='o')
plt.xlabel('X-axis Label')
plt.ylabel('Y-axis Label')
plt.title('X vs Y Plot')
plt.grid(True)
plt.show()
