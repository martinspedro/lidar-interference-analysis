#!/usr/bin/env python3
import sys
import warnings

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

#from datasets_path import datasets_path
from datasets_path import *


# if number of arguments is invalid
if len(sys.argv) != 1:
    warnings.warn("""Wrong number of arguments!\n
                Usage: rosrun graphics_generation distance_setups.py""")

# Data axis
error_thresholds = np.arange(0.05, 1.55, 0.05, dtype=np.double)
human_values = ["3m", "4m", "5m", "6m", "direct"]
#folder_values = ["human_3m", "human_4m", "human_5m", "human_6m", "human_direct"]
test_codename = "human"
#
#   DISTANCE
#

# Numpy arrays to hold data
errors_absolute = np.zeros((len(human_values), len(error_thresholds)), dtype=np.uint32)
number_points  = np.zeros((len(human_values), len(error_thresholds)), dtype=np.uint32)
errors_normalized  = np.zeros((len(human_values), len(error_thresholds)), dtype=np.double)

errors_absolute_distance_filename = constructFullPathToTestScenario(test_codename, INTERFERENCE_DISTANCE_ABSOLUTE_ERRORS_FILE_NAME)
number_points_distance_filename = constructFullPathToTestScenario(test_codename, INTERFERENCE_DISTANCE_TOTAL_POINTS_FILE_NAME)
errors_normalized_distance_filename = constructFullPathToTestScenario(test_codename, INTERFERENCE_DISTANCE_NORMALIZED_ERRORS_FILE_NAME)

if os.path.isfile(errors_absolute_distance_filename) and os.path.isfile(number_points_distance_filename) and os.path.isfile(errors_normalized_distance_filename):
    print("Loading older data..." ),
    errors_absolute = np.genfromtxt(errors_absolute_distance_filename, delimiter=", ", dtype=np.double)
    number_points = np.genfromtxt(number_points_distance_filename, delimiter=", ", dtype=np.double)
    errors_normalized = np.genfromtxt(errors_normalized_distance_filename, delimiter=", ", dtype=np.double)
    print("Done!")
else:
    for i in range(0, len(human_values), 1):
        test_scenario =  test_codename + "_" + human_values[i]
        print(test_scenario)

        filename = constructFullPathToResults(test_scenario, INTERFERENCE_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME)
        f = open(filename, "rb")
        print(filename)

        print("Loading Data... "),
        data = np.fromfile(f, dtype=np.double) # read floats with big endian
        data_without_nan =  data[~np.isnan(data)]

        print("Done! Computing results for every distance value ... "),
        for j in range(0, len(error_thresholds), 1):
            errors_absolute[i, j] = np.size( np.where( data_without_nan >= error_thresholds[j] ) )
            number_points[i, j] = np.size(data_without_nan)
            errors_normalized[i, j] = np.double(errors_absolute[i, j]) / np.double(number_points[i, j])

    print("Done!Saving results for latter use ... "),
    np.savetxt(errors_absolute_distance_filename, errors_absolute, delimiter=", ")
    np.savetxt(number_points_distance_filename, number_points, delimiter=", ")
    np.savetxt(errors_normalized_distance_filename, errors_normalized, delimiter=", ")



print("Plotting Colored Mesh Graph..."),
fig1, ax1 = plt.subplots(figsize=(16,9))
cmap = plt.get_cmap('jet')
im = ax1.pcolormesh(errors_normalized, cmap=cmap)
fig1.colorbar(im, ax=ax1)
plt.xticks(range(0, len(error_thresholds), 1), error_thresholds)
plt.yticks(range(0, len(human_values), 1), human_values)  # Set locations and labels
ax1.set_xlabel('Interference distance Threshold (m)')
ax1.set_ylabel('Distance between LiDARs (m)')
ax1.set_title("Interfered points")
fig1.tight_layout()
plt.show(block=False)

print("Done! Saving Colored Mesh Graph... "),
colored_mesh_filename = constructFullPathToTestScenario(test_codename, INTERFERENCE_DISTANCE_ERRORS_COLOR_MESH_FILE_NAME)

plt.savefig(colored_mesh_filename)
print("Done! Colored Mesh saved on: " + colored_mesh_filename)

# GROUND TRUTH
# Numpy arrays to hold data
errors_absolute = np.zeros((len(human_values), len(error_thresholds)), dtype=np.uint32)
number_points  = np.zeros((len(human_values), len(error_thresholds)), dtype=np.uint32)
errors_normalized  = np.zeros((len(human_values), len(error_thresholds)), dtype=np.double)

errors_absolute_distance_filename = constructFullPathToTestScenario(test_codename, GROUND_TRUTH_DISTANCE_ABSOLUTE_ERRORS_FILE_NAME)
number_points_distance_filename = constructFullPathToTestScenario(test_codename, GROUND_TRUTH_DISTANCE_TOTAL_POINTS_FILE_NAME)
errors_normalized_distance_filename = constructFullPathToTestScenario(test_codename, GROUND_TRUTH_DISTANCE_NORMALIZED_ERRORS_FILE_NAME)

if os.path.isfile(errors_absolute_distance_filename) and os.path.isfile(number_points_distance_filename) and os.path.isfile(errors_normalized_distance_filename):
    print("Loading older data..." ),
    errors_absolute = np.genfromtxt(errors_absolute_distance_filename, delimiter=", ", dtype=np.double)
    number_points = np.genfromtxt(number_points_distance_filename, delimiter=", ", dtype=np.double)
    errors_normalized = np.genfromtxt(errors_normalized_distance_filename, delimiter=", ", dtype=np.double)
    print("Done!")
else:
    for i in range(0, len(human_values), 1):
        test_scenario =  test_codename + "_" + human_values[i]
        print(test_scenario)

        filename = constructFullPathToResults(test_scenario, GROUND_TRUTH_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME)
        f = open(filename, "rb")
        print(filename)

        print("Loading Data... "),
        data = np.fromfile(f, dtype=np.double) # read floats with big endian
        data_without_nan =  data[~np.isnan(data)]

        print("Done! Computing results for every distance value ... "),
        for j in range(0, len(error_thresholds), 1):
            errors_absolute[i, j] = np.size( np.where( data_without_nan >= error_thresholds[j] ) )
            number_points[i, j] = np.size(data_without_nan)
            errors_normalized[i, j] = np.double(errors_absolute[i, j]) / np.double(number_points[i, j])

    print("Done!Saving results for latter use ... "),
    np.savetxt(errors_absolute_distance_filename, errors_absolute, delimiter=", ")
    np.savetxt(number_points_distance_filename, number_points, delimiter=", ")
    np.savetxt(errors_normalized_distance_filename, errors_normalized, delimiter=", ")



print("Plotting Colored Mesh Graph..."),
fig1, ax1 = plt.subplots(figsize=(16,9))
cmap = plt.get_cmap('jet')
im = ax1.pcolormesh(errors_normalized, cmap=cmap)
fig1.colorbar(im, ax=ax1)
plt.xticks(range(0, len(error_thresholds), 1), error_thresholds)
plt.yticks(range(0, len(human_values), 1), human_values)  # Set locations and labels
ax1.set_xlabel('Interference distance Threshold (m)')
ax1.set_ylabel('Distance between LiDARs (m)')
ax1.set_title("Interfered points")
fig1.tight_layout()
plt.show(block=False)

print("Done! Saving Colored Mesh Graph... "),
colored_mesh_filename = constructFullPathToTestScenario(test_codename, GROUND_TRUTH_DISTANCE_ERRORS_COLOR_MESH_FILE_NAME)

plt.savefig(colored_mesh_filename)
print("Done! Colored Mesh saved on: " + colored_mesh_filename)
