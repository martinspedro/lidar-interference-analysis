#!/usr/bin/env python3
import sys
import csv

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from datasets_path import datasets_path
from datasets_path import graphic_utilities


# if number of arguments is invalid
if len(sys.argv) != 2:
    sys.exit("""Wrong number of arguments!\n
                Usage: rosrun graphics_generation ground_truth_laser_intensity.py <test scenario codename>""")

graphics_folder = datasets_path.makeGraphicsDirectory(sys.argv[1], datasets_path.GRAPHICS_FOLDER_RELATIVE_PATH)
filename = datasets_path.constructFullPathToResults(sys.argv[1], datasets_path.INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BIN_NAME)

print("Opening " + filename + "... "),
data = []
with open(filename) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    for row in csv_reader:
        data.append(row)

# Convert list to numpy array and cast string to float
data_ordered = np.array(data).astype(np.float)

# Compute width of bars from the minimum resolution value contained on the dataset
width = np.amin(data_ordered[:, 0]) / 4

# split numpy array in several 1D arrays with data
resolution = np.array(data_ordered[:, 0], dtype=float)
ground_truth = np.array(data_ordered[:, 1], dtype=float)
interference = np.array(data_ordered[:, 2], dtype=float)
difference = np.array(data_ordered[:, 3], dtype=float)

# label all data
ground_truth_label = "ground truth bag"
interference_label = "interference bag"
difference_label = "|ground truth bag - interference bag|"

print("Done! Plotting Bar Graph... "),
fig1, ax1 = plt.subplots(figsize=(16,9))
rects_ground_truth = ax1.bar(resolution - width/2, ground_truth, width, label=ground_truth_label)
rects_interference = ax1.bar(resolution + width/2, interference, width, label=interference_label)
ax1.legend()
ax1.set_xlabel('Voxel Edge Length (m)')
ax1.set_ylabel('Normalized Mismatch between ground_truth_model and dataset')
ax1.set_title("Octree voxels mismatch")
ax1.set_xticks(resolution)
#graphic_utilities.autolabel(rects_ground_truth, ax1, 45)
#graphic_utilities.autolabel(rects_interference, ax1, 45)
fig1.tight_layout()
plt.show(block=False)

print("Done! Saving Bar Chart figure... "),

graphic_filename = datasets_path.constructFullPathToGraphics(sys.argv[1], datasets_path.INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_COMPARISON_BAR_FILE_NAME)
plt.savefig(graphic_filename)
print("Done! Bar Graph saved on: " + graphic_filename)


print("Done! Plotting Bar Graph... "),
fig2, ax2 = plt.subplots(figsize=(16,9))
rects_difference = ax2.bar(resolution, difference, width*2, label=difference_label)
ax2.legend()
ax2.set_xlabel('Voxel Edge Length (m)')
ax2.set_ylabel('Normalized Mismatch between ground_truth_model and dataset')
ax2.set_title("Difference between octree voxels mismatch")
ax2.set_xticks(resolution)
#graphic_utilities.autolabel(rects_difference, ax2)
fig2.tight_layout()
plt.show(block=False)

print("Done! Saving Bar Chart figure... "),

graphic_filename = datasets_path.constructFullPathToGraphics(sys.argv[1], datasets_path.INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BAR_FILE_NAME)
plt.savefig(graphic_filename)
print("Done! Bar Graph saved on: " + graphic_filename)
