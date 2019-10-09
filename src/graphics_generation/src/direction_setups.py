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
direction_values = range(0, 360, 30)
test_codename = "direction"


#
# Image holds a Direction. Each Polar Plot a Laser, Colors represent different frames
#
rows = 3
cols = 6

for i in range(0, len(direction_values), 1):
    test_scenario = test_codename + "_" + str(direction_values[i])
    print(test_scenario)

    filename = constructFullPathToResults(test_scenario, INTERFERENCE_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME)
    f = open(filename, "rb")
    print(filename)

    print("Loading Data... "),
    data = np.fromfile(f, dtype=np.double) # read floats with big endian
    data_without_nan = data #np.nan_to_num(data, copy=True)
    data_without_nan[np.isnan(data_without_nan)] = 0
    data_without_nan.resize(int(len(data_without_nan)/16), 16)

    print("Done! Plotting Polar Graph..."),
    theta = 2 * np.pi * np.arange(0, 1800, 1) / 1800

    fig1, ax1 = plt.subplots(rows, cols, sharex=False, sharey=False, figsize=(16,9))
    for j in range(0, 16, 1):
        rows_idx = int(j / cols)
        cols_idx = int(j % cols)
        ax = plt.subplot(rows, cols, j+1, projection="polar")
        #ax1[rows_idx, cols_idx]
        for k in range(0, int(len(data_without_nan)/(1800*16)), 1):
            ax.plot(theta, data_without_nan[range(1800*k, 1800*(k+1), 1), j])  # average intensity is on the first column
        ax.set_title(f"Laser {j}")
    #ax1.set_title("Average Intensity of point measures in ground_truth_model by azimuth")
    fig1.suptitle(f"{direction_values[i]}º")
    fig1.tight_layout()
    plt.show(block=False)
    graphic_filename = constructFullPathToTestScenario(test_codename, INTERFERENCE_DIRECTION_DISTANCE_ERRORS_POLAR_PLOT_DIRECTION_BASE_NAME + f"{direction_values[i]}.png")
    print(graphic_filename)
    plt.savefig(graphic_filename)
    plt.close()



#
# Image holds a Laser. Each Polar Plot a Direction, Colors represent different frames
#

rows = 3
cols = 4

for j in range(0, 16, 1):
    rows_idx = int(j / cols)
    cols_idx = int(j % cols)
    fig2, ax2 = plt.subplots(rows, cols, sharex=False, sharey=False, figsize=(16,9))

    for i in range(0, len(direction_values), 1):
        test_scenario = test_codename + "_" + str(direction_values[i])
        print(test_scenario)

        filename = constructFullPathToResults(test_scenario, INTERFERENCE_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME)
        f = open(filename, "rb")
        print(filename)

        print("Loading Data... "),
        data = np.fromfile(f, dtype=np.double) # read floats with big endian
        data_without_nan = data #np.nan_to_num(data, copy=True)
        data_without_nan[np.isnan(data_without_nan)] = 0
        data_without_nan.resize(int(len(data_without_nan)/16), 16)

        print("Done! Plotting Polar Graph..."),
        theta = 2 * np.pi * np.arange(0, 1800, 1) / 1800


        ax = plt.subplot(rows, cols, i+1, projection="polar")
        for k in range(0, int(len(data_without_nan)/(1800*16)), 1):
            ax.plot(theta, data_without_nan[range(1800*k, 1800*(k+1), 1), j])  # average intensity is on the first column
        ax.set_title(f"Direction: {direction_values[i]}")

        #ax1.set_title("Average Intensity of point measures in ground_truth_model by azimuth")
    fig2.suptitle(f"Laser ID: {j}")
    fig2.tight_layout()
    plt.show(block=False)
    graphic_filename = constructFullPathToTestScenario(test_codename, INTERFERENCE_DIRECTION_DISTANCE_ERRORS_POLAR_PLOT_LASER_BASE_NAME + f"{j}.png")
    print(graphic_filename)
    plt.savefig(graphic_filename)
    plt.close()
