#!/usr/bin/env python3
import sys
import os
import warnings

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from datasets_path import datasets_path

plt.rcParams.update({'font.size': 20})

# if number of arguments is invalid
if len(sys.argv) != 1:
    warnings.warn("""Wrong number of arguments!\n
                Usage: rosrun graphics_generation distance_setups.py""")

# Data axis
height_values = [0.623, 0.715, 0.818, 0.931, 1.032, 1.144, 1.277]
folder_values = ["0.6", "0.7", "0.8", "0.9", "1.0", "1.1", "1.2"]
test_codename = "height"

# Subtract Velodyne LiDAR height to get HESAI height relative to it
height_values = [value - 0.918 for value in height_values]

# Numpy arrays to hold data
errors_normalized  = np.zeros((len(height_values)), dtype=np.double)

for i in range(0, len(height_values), 1):
    test_scenario = test_codename + "_" + folder_values[i] + "m"
    print(test_scenario)

    filename = datasets_path.constructFullPathToResults(test_scenario, datasets_path.INTERFERENCE_BOX_FILTER_FILE_NAME)
    print(filename)

    test_data = np.genfromtxt(filename, delimiter=", ", dtype=np.double)
    #print(test_data)

    # number of outliers / number of points
    errors_normalized[i] = test_data.item(4) / test_data.item(1)
    print(errors_normalized[i])

print("Done! Plotting Bar Graph... "),
fig, ax = plt.subplots(figsize=(12,9))

# Remove the plot frame lines
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)

# Axis ticks only show up on the bottom and left of the plot.
ax.get_xaxis().tick_bottom()
ax.get_yaxis().tick_left()

plt.grid(True, which="minor", color='grey', linestyle='-', linewidth=0.5, alpha=0.5)
rects = ax.barh(range(0, len(height_values), 1), errors_normalized, log=True, label=r'$\frac{\#\ of\ outliers}{\#\ of\ points}$', tick_label=height_values)
ax.legend(prop={'size': 28})
ax.set_xlabel('Normalized relative number of Outliers', size=22, fontstyle='italic')
ax.set_ylabel('Height difference between the LiDARs (m)', size=22, fontstyle='italic')
#ax.set_title("Variation of outliers with Height between LiDARs")
#ax.set_xticks(range(0, len(height_values), 1), height_values)
fig.tight_layout()
plt.show(block=False)

print("Done! Saving Bar Chart figure... "),
graphic_filename = datasets_path.constructFullPathToTestScenario(test_codename, datasets_path.INTERFERENCE_BOX_FILTER_BAR_FILE)
plt.savefig(graphic_filename)
print("Done! Bar Graph saved on: " + graphic_filename)
