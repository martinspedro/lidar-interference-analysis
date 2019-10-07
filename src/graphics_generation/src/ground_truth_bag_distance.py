#!/usr/bin/env python3
import sys

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from datasets_path import *

# if number of arguments is invalid
if len(sys.argv) != 2 and len(sys.argv) != 3:
    sys.exit("""Wrong number of arguments!\n
                Usage: rosrun point_cloud_statistics ground_truth_point_distance_variance.py <test scenario codename>
                Usage: rosrun point_cloud_statistics ground_truth_point_distance_variance.py <test scenario codename> <number of bins>""")

graphics_folder = makeGraphicsDirectory(sys.argv[1], GRAPHICS_FOLDER_RELATIVE_PATH)
filename = constructFullPathToResults(sys.argv[1], GROUND_TRUTH_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME)

if len(sys.argv) == 3:
    number_of_bins = sys.argv[2]
else:
    number_of_bins = 130 # one bin per meter

data_label = "ground truth intensity difference"

print("Opening  " + filename + "... "),
f = open(filename, "rb")

print("Done. Loading and casting data to numpy array... "),
data = np.fromfile(f, dtype=np.double) # read double with big endian
data_clean = data[~np.isnan(data)]

print("Done! Plotting Histogram..."),
fig1, ax = plt.subplots()
n, bins, patches = ax.hist(data_clean, number_of_bins, density=False, histtype='bar', align='mid', log=True, label=data_label)
#ax.legend()
ax.set_xlabel('Distance difference')
ax.set_ylabel('Normalized probability density')
ax.set_title("Distance difference between ground truth bag and ground_truth_model")
fig1.tight_layout()
plt.show(block=False)

print("Done! Saving histogram figure... ")

hist_filename = constructFullPathToGraphics(sys.argv[1], GROUND_TRUTH_BAG_DISTANCE_HIST_FILE_NAME)
plt.savefig(hist_filename, dpi=200)
print("Done! Histogram saved on: " + hist_filename)
