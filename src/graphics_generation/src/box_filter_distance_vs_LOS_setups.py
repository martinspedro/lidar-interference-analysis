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
distance_values = range(2, 13, 2)
test_codename = "LOS"

# Numpy arrays to hold data
errors_normalized_distance  = np.zeros((len(distance_values)), dtype=np.double)
errors_normalized_los  = np.zeros((len(distance_values)), dtype=np.double)

for i in range(0, len(distance_values), 1):
    test_scenario = "LOS" + "_" + str(distance_values[i]) + "m"
    print(test_scenario)

    filename = datasets_path.constructFullPathToResults(test_scenario, datasets_path.INTERFERENCE_BOX_FILTER_FILE_NAME)
    print(filename)

    test_data = np.genfromtxt(filename, delimiter=", ", dtype=np.double)
    #print(test_data)

    # number of outliers / number of points
    errors_normalized_los[i] = test_data.item(4) / test_data.item(1)
    print(errors_normalized_los[i])

for i in range(0, len(distance_values), 1):
    test_scenario = "distance" + "_" + str(distance_values[i]) + "m"
    print(test_scenario)

    filename = datasets_path.constructFullPathToResults(test_scenario, datasets_path.INTERFERENCE_BOX_FILTER_FILE_NAME)
    print(filename)

    test_data = np.genfromtxt(filename, delimiter=", ", dtype=np.double)
    #print(test_data)

    # number of outliers / number of points
    errors_normalized_distance[i] = test_data.item(4) / test_data.item(1)
    print(errors_normalized_distance[i])

print("Done! Plotting Bar Graph... "),
fig, ax = plt.subplots(figsize=(12,9))

# Remove the plot frame lines
ax.spines["top"].set_visible(False)
ax.spines["right"].set_visible(False)

# Axis ticks only show up on the bottom and left of the plot.
ax.get_xaxis().tick_bottom()
ax.get_yaxis().tick_left()

plt.grid(True, which="minor", color='grey', linestyle='-', linewidth=0.5, alpha=0.5)

width = 0.4

distance_values_ticks = np.arange(2, 13, 2)
rects = ax.bar(distance_values_ticks - width/2, errors_normalized_distance, width, log=True, label="With Direct Interference")
rects = ax.bar(distance_values_ticks + width/2, errors_normalized_los, width, log=True, label="Without Direct Interference")
ax.legend(prop={'size': 24})
ax.set_xlabel('Distance between LiDARs (m)', size=22, fontstyle='italic')
ax.set_ylabel('Normalized relative number of Outliers', size=22, fontstyle='italic')
#ax.set_title("Variation of outliers with distance between LiDARs")
#ax.set_xticks(range(0, len(distance_values), 1), distance_values)
fig.tight_layout()
plt.show(block=False)

print("Done! Saving Bar Chart figure... "),
graphic_filename = datasets_path.constructFullPathToTestScenario(test_codename, datasets_path.INTERFERENCE_BOX_FILTER_BAR_FILE)
plt.savefig(graphic_filename)
print("Done! Bar Graph saved on: " + graphic_filename)
