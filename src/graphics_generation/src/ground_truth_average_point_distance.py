#!/usr/bin/env python3
import sys

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from datasets_path import datasets_path

# if number of arguments is invalid
if len(sys.argv) != 2 and len(sys.argv) != 3:
    sys.exit("""Wrong number of arguments!\n
                Usage: rosrun point_cloud_statistics ground_truth_average_point_distance.py <test scenario codename>
                Usage: rosrun point_cloud_statistics ground_truth_average_point_distance.py <test scenario codename> <number of bins>""")

graphics_folder = datasets_path.makeGraphicsDirectory(sys.argv[1], datasets_path.GRAPHICS_FOLDER_RELATIVE_PATH)
filename = datasets_path.constructFullPathToResults(sys.argv[1], datasets_path.GROUND_TRUTH_AVERAGE_POINT_DISTANCE_BIN_NAME)

if len(sys.argv) == 3:
    number_of_bins = sys.argv[2]
else:
    number_of_bins = 130 # one bin per meter

data_label = "ground truth model average distance"

print("Opening  " + filename + "... "),
f = open(filename, "rb")

print("Done. Loading and casting data to numpy array... "),
data = np.fromfile(f, dtype=np.float32) # read floats with big endian


print("Done! Plotting Histogram..."),
fig1, ax = plt.subplots(figsize=(16,9))
n, bins, patches = ax.hist(data, number_of_bins, density=True, histtype='bar', align='mid', log=False, label=data_label)
ax.legend()
ax.set_xlabel('Distance from Origin (m)')
ax.set_ylabel('Normalized probability density')
ax.set_title("Average distance per point used to generate the ground_truth_model")
fig1.tight_layout()
plt.show(block=False)

print("Done! Saving histogram figure... ")

hist_filename = datasets_path.constructFullPathToGraphics(sys.argv[1], datasets_path.GROUND_TRUTH_AVERAGE_POINT_DISTANCE_HIST_FILE_NAME)
plt.savefig(hist_filename)
print("Done! Histogram saved on: " + hist_filename)

data.resize(1800, 16)

print("Plotting Colored Mesh Graph..."),
fig2, ax = plt.subplots(figsize=(16,9))
cmap = plt.get_cmap('plasma')
im = ax.pcolormesh(data, cmap=cmap)
fig2.colorbar(im, ax=ax)
ticks = range(0, 1800, 150)
plt.yticks(ticks, np.arange(-180, 180, 30))  # Set locations and labels
ax.set_xlabel('Laser ID/Ring Number')
ax.set_ylabel('Azimuthal angle (º)')
ax.set_title("Average distance per point used to generate the ground_truth_model")

fig2.tight_layout()
plt.show(block=False)

print("Done! Saving Colored Mesh Graph... "),
colored_mesh_filename = datasets_path.constructFullPathToGraphics(sys.argv[1], datasets_path.GROUND_TRUTH_AVERAGE_POINT_DISTANCE_COLOR_MESH_FILE_NAME)
plt.savefig(colored_mesh_filename)
print("Done! Colored Mesh saved on: " + colored_mesh_filename)
