#!/usr/bin/env python3
import sys
import os
import warnings
import csv

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from datasets_path import datasets_path


# if number of arguments is invalid
if len(sys.argv) != 1:
    warnings.warn("""Wrong number of arguments!\n
                Usage: rosrun graphics_generation distance_setups.py""")

# Data axis
error_thresholds = np.arange(0.05, 0.45+0.05, 0.05, dtype=float)
direction_values = range(0, 360, 30)
test_codename = "direction"

# Numpy arrays to hold data
resolution   = np.zeros((len(error_thresholds), len(direction_values)), dtype=float)
ground_truth = np.zeros((len(error_thresholds), len(direction_values)), dtype=float)
interference = np.zeros((len(error_thresholds), len(direction_values)), dtype=float)
difference   = np.zeros((len(error_thresholds), len(direction_values)), dtype=float)



for i in range(0, len(direction_values), 1):
    test_scenario = test_codename + "_" + str(direction_values[i])
    print(test_scenario)

    filename = datasets_path.constructFullPathToResults(test_scenario, datasets_path.INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BIN_NAME)
    print(filename)

    print("Opening " + filename + "... "),
    data = []
    with open(filename) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        for row in csv_reader:
            data.append(row)

    # Convert list to numpy array and cast string to float
    data_ordered = np.array(data).astype(np.float)


    # split numpy array in several 1D arrays with data
    resolution[:, i] = np.array(data_ordered[:, 0], dtype=float)
    ground_truth[:, i] = np.array(data_ordered[:, 1], dtype=float)
    interference[:, i] = np.array(data_ordered[:, 2], dtype=float)
    difference[:, i] = np.array(data_ordered[:, 3], dtype=float)


print("Plotting Colored Mesh Graph..."),
fig1, ax1 = plt.subplots(figsize=(16,9))
cmap = plt.get_cmap('jet')
im = ax1.pcolormesh(ground_truth, cmap=cmap)
fig1.colorbar(im, ax=ax1)
plt.xticks(range(0, len(direction_values), 1), direction_values)  # Set locations and labels
plt.yticks(range(0, len(error_thresholds), 1), error_thresholds)
ax1.set_xlabel('Direction between LiDARs (º)')
ax1.set_ylabel('Voxel Edge Length (m)')
ax1.set_title("Interfered points")
fig1.tight_layout()
plt.show(block=False)

print("Done! Saving Colored Mesh Graph... "),
colored_mesh_filename = datasets_path.constructFullPathToTestScenario(test_codename, datasets_path.OCTREE_GROUND_TRUTH_COLOR_MESH)
plt.savefig(colored_mesh_filename)

print("Done! Ground Truth Colored Mesh saved on: " + colored_mesh_filename)


print("Plotting Colored Mesh Graph..."),
fig1, ax1 = plt.subplots(figsize=(16,9))
cmap = plt.get_cmap('jet')
im = ax1.pcolormesh(interference, cmap=cmap)
fig1.colorbar(im, ax=ax1)
plt.xticks(range(0, len(direction_values), 1), direction_values)  # Set locations and labels
plt.yticks(range(0, len(error_thresholds), 1), error_thresholds)
ax1.set_xlabel('Direction between LiDARs (º)')
ax1.set_ylabel('Voxel Edge Length (m)')
ax1.set_title("Interfered points")
fig1.tight_layout()
plt.show(block=False)

print("Done! Saving Colored Mesh Graph... "),
colored_mesh_filename = datasets_path.constructFullPathToTestScenario(test_codename, datasets_path.OCTREE_INTERFERENCE_COLOR_MESH)
plt.savefig(colored_mesh_filename)

print("Done! Interference Colored Mesh saved on: " + colored_mesh_filename)

print("Plotting Colored Mesh Graph..."),
fig1, ax1 = plt.subplots(figsize=(16,9))
cmap = plt.get_cmap('jet')
im = ax1.pcolormesh(difference, cmap=cmap)
fig1.colorbar(im, ax=ax1)
plt.xticks(range(0, len(direction_values), 1), direction_values)  # Set locations and labels
plt.yticks(range(0, len(error_thresholds), 1), error_thresholds)
ax1.set_xlabel('Direction between LiDARs (º)')
ax1.set_ylabel('Voxel Edge Length (m)')
ax1.set_title("Interfered points")
fig1.tight_layout()
plt.show(block=False)

print("Done! Saving Colored Mesh Graph... "),
colored_mesh_filename = datasets_path.constructFullPathToTestScenario(test_codename, datasets_path.OCTREE_DIFFERENCE_COLOR_MESH)
plt.savefig(colored_mesh_filename)

print("Done! Difference Colored Mesh saved on: " + colored_mesh_filename)
