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
direction_values = range(0, 360, 30)
test_codename = "direction"

# Numpy arrays to hold data
errors_normalized  = np.zeros((len(direction_values)), dtype=np.double)

for i in range(0, len(direction_values), 1):
    test_scenario = test_codename + "_" + str(direction_values[i])
    print(test_scenario)

    filename = datasets_path.constructFullPathToResults(test_scenario, datasets_path.INTERFERENCE_BOX_FILTER_FILE_NAME)
    print(filename)

    test_data = np.genfromtxt(filename, delimiter=", ", dtype=np.double)
    #print(test_data)

    # number of outliers / number of points
    errors_normalized[i] = test_data.item(4) / test_data.item(1)
    print(errors_normalized[i])

print("Done! Plotting Bar Graph... "),

direction_values_radian = 2 * np.pi * np.array(direction_values) / 360
width = np.pi / (len(direction_values) + 1)
colors = plt.cm.coolwarm(np.array(errors_normalized) / np.max(np.array(errors_normalized)))

rows = 1
cols = 1
fig, ax = plt.subplots(rows, cols, sharex=False, sharey=False, figsize=(8,8))
ax = plt.subplot(111, projection='polar')#, figsize=(16,9))
rects = ax.bar(direction_values_radian, errors_normalized, width=width, bottom=0.0, color=colors, alpha=1.0, label=r'$\frac{\#\ of\ outliers}{\#\ of\ points}$')
ax.legend(loc=0, prop={'size': 24})
ax.set_title("Variation of outliers with the orientation between LiDARs")
ax.set_xticks(direction_values_radian)
ax.set_xticklabels(direction_values)
#ax.set_yticklabels(errors_normalized)
fig.tight_layout()
plt.show(block=False)

print("Done! Saving Polar Bar Chart figure... "),
graphic_filename = datasets_path.constructFullPathToTestScenario(test_codename, datasets_path.INTERFERENCE_BOX_FILTER_BAR_FILE)
plt.savefig(graphic_filename)
print("Done! Polar Bar Graph saved on: " + graphic_filename)