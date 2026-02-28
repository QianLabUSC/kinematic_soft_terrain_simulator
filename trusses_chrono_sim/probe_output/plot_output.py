import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

# Load data
df = pd.read_csv('./probe_output/output.dat', delimiter=' ')

# Extract columns
time = df.iloc[:, 0].values
force_x = df.iloc[:, 1].values
force_y = df.iloc[:, 2].values
force_z = df.iloc[:, 3].values
probe_dist = df.iloc[:, 4].values
probe_shear = None
try:
    probe_shear = df.iloc[:, 5].values
except IndexError:
    print("No shear data available")

# Find the first non-zero index in force_y
first_non_zero_index = np.nonzero(force_y)[0][0]

# Slice the data from the first non-zero index
time = time[first_non_zero_index:]
force_y = force_y[first_non_zero_index:]
force_z = force_z[first_non_zero_index:]
probe_dist = probe_dist[first_non_zero_index:] - probe_dist[first_non_zero_index]
if probe_shear is not None:
    probe_shear = probe_shear[first_non_zero_index:]

# Apply Savitzky-Golay filter to smooth the data
window_length = 100  # Choose an odd number, typically much less than the data size
polyorder = 1       # Choose a polynomial order
force_y_filtered = savgol_filter(force_y, window_length, polyorder)
force_z_filtered = savgol_filter(force_z, window_length, polyorder)

# Plot the filtered data
plt.figure(figsize=[14, 12])
plt.rcParams.update({'font.size': 21})
plt.rcParams['font.family'] = "Times New Roman"

# Plot force_y_filtered vs. probe_dist
if(np.isnan(probe_shear[0])):
    plt.subplot(1,1,1)
else:
    plt.subplot(2, 1, 1)
plt.plot(probe_dist, force_y_filtered)
plt.xlabel('Probe Displacement (m)')
plt.ylabel('Force (N)')
plt.title('Filtered Probe Vertical Displacement vs Force')

# If probe_shear is available, plot it with force_z_filtered
if np.isnan(probe_shear[0]) == False:
    plt.subplot(2, 1, 2)
    plt.plot(probe_shear, force_z_filtered)
    plt.xlabel('Probe Displacement (m)')
    plt.ylabel('Force (N)')
    plt.title('Filtered Probe Shear Displacement vs Force')


slope, intercept = np.polyfit(probe_dist, force_y_filtered, 1)

plt.tight_layout()
plt.show()
print(slope, intercept)