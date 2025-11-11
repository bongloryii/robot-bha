import numpy as np

# Load the CSV file
input_csv_file = 'Sim_WorldFinalV3.csv'
data = np.loadtxt(input_csv_file, delimiter=',', skiprows=1)  # Skip the header row if present

# Save to .npy file
output_npy_file = 'path.npy'
np.save(output_npy_file, data)

print(f"Data from {input_csv_file} saved to {output_npy_file}")
