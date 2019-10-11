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
error_thresholds = np.arange(0.05, 1.55, 0.05, dtype=np.double)
distance_values = range(2, 13, 2)
distance_test_codename = "distance"
LOS_test_codename = "LOS"

distance_errors_absolute = np.zeros((len(distance_values), len(error_thresholds)), dtype=np.uint32)
distance_number_points  = np.zeros((len(distance_values), len(error_thresholds)), dtype=np.uint32)
distance_errors_normalized  = np.zeros((len(distance_values), len(error_thresholds)), dtype=np.double)

distance_errors_absolute_distance_filename = datasets_path.constructFullPathToTestScenario(distance_test_codename, datasets_path.INTERFERENCE_DISTANCE_ABSOLUTE_ERRORS_FILE_NAME)
distance_number_points_distance_filename = datasets_path.constructFullPathToTestScenario(distance_test_codename, datasets_path.INTERFERENCE_DISTANCE_TOTAL_POINTS_FILE_NAME)
distance_errors_normalized_distance_filename = datasets_path.constructFullPathToTestScenario(distance_test_codename, datasets_path.INTERFERENCE_DISTANCE_NORMALIZED_ERRORS_FILE_NAME)


if os.path.isfile(distance_errors_absolute_distance_filename) and os.path.isfile(distance_number_points_distance_filename) and os.path.isfile(distance_errors_normalized_distance_filename):
    print("Loading older data..." ),
    distance_errors_absolute = np.genfromtxt(distance_errors_absolute_distance_filename, delimiter=", ", dtype=np.double)
    distance_number_points = np.genfromtxt(distance_number_points_distance_filename, delimiter=", ", dtype=np.double)
    distance_errors_normalized = np.genfromtxt(distance_errors_normalized_distance_filename, delimiter=", ", dtype=np.double)
    print("Done!"),
else:
    sys.exit("No data for distance dataset. Please run rosrun graphics_generation distance_setups.py")

LOS_errors_absolute = np.zeros((len(distance_values), len(error_thresholds)), dtype=np.uint32)
LOS_number_points  = np.zeros((len(distance_values), len(error_thresholds)), dtype=np.uint32)
LOS_errors_normalized  = np.zeros((len(distance_values), len(error_thresholds)), dtype=np.double)

LOS_errors_absolute_distance_filename = datasets_path.constructFullPathToTestScenario(LOS_test_codename, datasets_path.INTERFERENCE_DISTANCE_ABSOLUTE_ERRORS_FILE_NAME)
LOS_number_points_distance_filename = datasets_path.constructFullPathToTestScenario(LOS_test_codename, datasets_path.INTERFERENCE_DISTANCE_TOTAL_POINTS_FILE_NAME)
LOS_errors_normalized_distance_filename = datasets_path.constructFullPathToTestScenario(LOS_test_codename, datasets_path.INTERFERENCE_DISTANCE_NORMALIZED_ERRORS_FILE_NAME)

if os.path.isfile(LOS_errors_absolute_distance_filename) and os.path.isfile(LOS_number_points_distance_filename) and os.path.isfile(LOS_errors_normalized_distance_filename):
    print("Loading older data..." ),
    LOS_errors_absolute = np.genfromtxt(LOS_errors_absolute_distance_filename, delimiter=", ", dtype=np.double)
    LOS_number_points = np.genfromtxt(LOS_number_points_distance_filename, delimiter=", ", dtype=np.double)
    LOS_errors_normalized = np.genfromtxt(LOS_errors_normalized_distance_filename, delimiter=", ", dtype=np.double)
    print("Done!"),
else:
    sys.exit("No data for LOS dataset. Please run rosrun graphics_generation LOS_setups.py")




# label all data
distance_label = "LIDARs in LOS"
los_label = "Obstacle between LIDARs"

rows = 3
cols = 2
fig1, ax1 = plt.subplots(rows, cols, sharex=False, sharey=False, figsize=(16,9))

width = 0.02

print("Done! Plotting Bar Graph... "),
for i in range(0, len(distance_values), 1):
    rows_idx = int(i / cols)
    cols_idx = int(i % cols)
    rects_ground_truth = ax1[rows_idx, cols_idx].bar(error_thresholds - width/2, distance_errors_normalized[i*2, :], width, label=distance_label)
    rects_interference = ax1[rows_idx, cols_idx].bar(error_thresholds + width/2, LOS_errors_normalized[i, :], width, label=los_label)
    ax1[rows_idx, cols_idx].legend()
    ax1[rows_idx, cols_idx].set_title(f"Distance: {distance_values[i]}m")

fig1.suptitle("Interference Errors with varying distance between LiDARs")
fig1.tight_layout()
plt.show(block=False)
print("Done! Saving Bar Chart figure... "),

graphic_filename = datasets_path.constructFullPathToTestScenario(LOS_test_codename, datasets_path.INTERFERENCE_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_COMPARISON_BAR_FILE)
plt.savefig(graphic_filename)
print("Done! Bar Chart saved on: " + graphic_filename)

# Cpmute and save normalized errors difference
errors_normalized_difference =  distance_errors_normalized[range(0, 12, 2), :] - LOS_errors_normalized
errors_normalized_difference_filename = datasets_path.constructFullPathToTestScenario(LOS_test_codename, datasets_path.INTERFERENCE_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_FILE_NAME)
np.savetxt(errors_normalized_difference_filename, errors_normalized_difference, delimiter=", ")


print("Plotting Colored Mesh Graph..."),
fig2, ax2 = plt.subplots(figsize=(16,9))
cmap = plt.get_cmap('jet')
im = ax2.pcolormesh(errors_normalized_difference, cmap=cmap)
fig2.colorbar(im, ax=ax2)
plt.xticks(range(0, len(error_thresholds), 1), error_thresholds)
plt.yticks(range(0, len(distance_values), 1), distance_values)  # Set locations and labels
ax2.set_xlabel('Interference distance Threshold (m)')
ax2.set_ylabel('Distance between LiDARs (m)')
ax2.set_title("Interfered points")
fig2.tight_layout()
plt.show(block=False)

print("Done! Saving Colored Mesh Graph... "),
colored_mesh_filename = datasets_path.constructFullPathToTestScenario(LOS_test_codename, datasets_path.INTERFERENCE_LOS_VS_DISTANCE_ERRORS_NORMALIZED_DIFFERENCE_COMPARISON_COLOR_MESH_FILE)
plt.savefig(colored_mesh_filename)
