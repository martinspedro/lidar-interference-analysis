#!/usr/bin/env python3
import sys
import csv

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from datasets_path import *

# from https://matplotlib.org/3.1.1/gallery/lines_bars_and_markers/barchart.html#sphx-glr-gallery-lines-bars-and-markers-barchart-py
def autolabel(rects, axes, text_rotation=0):
    """Attach a text label above each bar in *rects*, displaying its height."""
    for rect in rects:
        height = rect.get_height()
        axes.annotate('{}'.format(height),
                    xy=(rect.get_x() + rect.get_width() / 2, height),
                    xytext=(rect.get_width(), 3),  # 3 points vertical offset
                    textcoords="offset points",
                    rotation=text_rotation,
                    ha='center', va='bottom')



# if number of arguments is invalid
if len(sys.argv) != 2:
    sys.exit("""Wrong number of arguments!\n
                Usage: rosrun graphics_generation ground_truth_laser_intensity.py <test scenario codename>""")

graphics_folder = makeGraphicsDirectory(sys.argv[1], GRAPHICS_FOLDER_RELATIVE_PATH)
filename = constructFullPathToResults(sys.argv[1], INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BIN_NAME)

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
fig1, ax1 = plt.subplots()
rects_ground_truth = ax1.bar(resolution - width/2, ground_truth, width, label=ground_truth_label)
rects_interference = ax1.bar(resolution + width/2, interference, width, label=interference_label)
ax1.legend()
ax1.set_xlabel('Voxel Edge Length (m)')
ax1.set_ylabel('Normalized Mismatch between ground_truth_model and dataset')
ax1.set_title("Octree voxels mismatch")
ax1.set_xticks(resolution)
autolabel(rects_ground_truth, ax1, 45)
autolabel(rects_interference, ax1, 45)
fig1.tight_layout()
plt.show(block=False)

print("Done! Saving Bar Chart figure... "),

graphic_filename = constructFullPathToGraphics(sys.argv[1], INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_COMPARISON_BAR_FILE_NAME)
plt.savefig(graphic_filename, dpi=200)
print("Done! Bar Graph saved on: " + graphic_filename)


print("Done! Plotting Bar Graph... "),
fig2, ax2 = plt.subplots()
rects_difference = ax2.bar(resolution, difference, width, label=difference_label)
ax2.legend()
ax2.set_xlabel('Voxel Edge Length (m)')
ax2.set_ylabel('Normalized Mismatch between ground_truth_model and dataset')
ax2.set_title("Difference between octree voxels mismatch")
ax2.set_xticks(resolution)
autolabel(rects_difference, ax2)
fig2.tight_layout()
plt.show(block=False)

print("Done! Saving Bar Chart figure... "),

graphic_filename = constructFullPathToGraphics(sys.argv[1], INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BAR_FILE_NAME)
plt.savefig(graphic_filename, dpi=200)
print("Done! Bar Graph saved on: " + graphic_filename)
