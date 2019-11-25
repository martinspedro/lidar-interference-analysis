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
distance_strings = ["closer", "halfway", "further"]
height_strings = ["below", "aligned", "above"]

# Values on Velodyne coordinate frame, gathered from Physical Setup Measures folder
distance_values = [1.54, 3.02, 4.54]
height_values = [-0.15, 0, 0.14]

test_codename = "it2_dark_room"

# Numpy arrays to hold data
errors_normalized  = np.zeros((len(height_strings),len(distance_strings) ), dtype=np.double)

for i in range(0, len(height_strings), 1):
    for j in range(0, len(distance_strings), 1):
        test_scenario = str(distance_strings[j]) + "_" + str(height_strings[i])
        print(test_scenario)

        filename = datasets_path.constructFullPathToResults(test_scenario, datasets_path.INTERFERENCE_BOX_FILTER_FILE_NAME)
        print(filename)

        if os.path.isfile(filename):
            test_data = np.genfromtxt(filename, delimiter=", ", dtype=np.double)
            #print(test_data)

            # number of outliers / number of points
            errors_normalized[i, j] = test_data.item(4) / test_data.item(1)
            print(errors_normalized[i, j])
        else:
            errors_normalized[i, j] = np.nan
            print (filename + " does not exist. Skipping!")


print("Plotting Colored Mesh Graph..."),
fig, ax = plt.subplots(figsize=(16,14))
cmap = plt.get_cmap('jet')
im = ax.pcolormesh(errors_normalized, cmap=cmap)
fig.colorbar(im, ax=ax)
plt.xticks(range(0, len(distance_values)), distance_values)  # Set locations and labels
plt.yticks(range(0, len(height_values)), height_values)      # Set locations and labels
ax.set_xlabel('Distance difference between the LiDARs (m) ')
ax.set_ylabel('Height difference between the LiDARs (m)')
ax.set_title("Variation of outliers with the distance and height between LiDARs")
fig.tight_layout()
plt.show(block=False)

print("Done! Saving Colored Mesh Graph figure... "),
graphic_filename = datasets_path.constructFullPathToTestScenario(test_codename, datasets_path.INTERFERENCE_BOX_FILTER_BAR_FILE)
plt.savefig(graphic_filename)
print("Done! Colored Mesh Graph figure saved on: " + graphic_filename)
