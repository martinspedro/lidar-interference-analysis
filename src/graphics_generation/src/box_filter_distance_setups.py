#!/usr/bin/env python3
import sys
import os
import warnings

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from datasets_path import datasets_path


# if number of arguments is invalid
if len(sys.argv) != 1:
    warnings.warn("""Wrong number of arguments!\n
                Usage: rosrun graphics_generation distance_setups.py""")

# Data axis
distance_values = range(1, 13, 1)
test_codename = "distance"

# Numpy arrays to hold data
errors_normalized  = np.zeros((len(distance_values)), dtype=np.double)

for i in range(0, len(distance_values), 1):
    test_scenario = test_codename + "_" + str(i+1) + "m"
    print(test_scenario)

    filename = datasets_path.constructFullPathToResults(test_scenario, datasets_path.INTERFERENCE_BOX_FILTER_FILE_NAME)
    print(filename)

    test_data = np.genfromtxt(filename, delimiter=", ", dtype=np.double)
    #print(test_data)

    # number of outliers / number of points
    errors_normalized[i] = test_data.item(4) / test_data.item(1)
    print(errors_normalized[i])

print("Done! Plotting Bar Graph... "),
fig, ax = plt.subplots(figsize=(16,9))
rects = ax.bar(distance_values, errors_normalized, log=True, label=r'$\frac{\#\ of\ outliers}{\#\ of\ points}$')
ax.legend(prop={'size': 24})
ax.set_xlabel('Distance between LiDARs (m)')
ax.set_ylabel('Normalized relative number of Outliers')
ax.set_title("Variation of outliers with distance between LiDARs")
ax.set_xticks(distance_values)
fig.tight_layout()
plt.show(block=False)

print("Done! Saving Bar Chart figure... "),
graphic_filename = datasets_path.constructFullPathToTestScenario(test_codename, datasets_path.INTERFERENCE_BOX_FILTER_BAR_FILE)
plt.savefig(graphic_filename)
print("Done! Bar Graph saved on: " + graphic_filename)
