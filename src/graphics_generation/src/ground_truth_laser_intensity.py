#!/usr/bin/env python3
import sys

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from datasets_path import datasets_path

# if number of arguments is invalid
if len(sys.argv) != 2:
    sys.exit("""Wrong number of arguments!\n
                Usage: rosrun graphics_generation ground_truth_laser_intensity.py <test scenario codename>""")

graphics_folder = datasets_path.makeGraphicsDirectory(sys.argv[1], datasets_path.GRAPHICS_FOLDER_RELATIVE_PATH)
filename = datasets_path.constructFullPathToResults(sys.argv[1], datasets_path.GROUND_TRUTH_LASER_INTENSITY_BIN_NAME)


data_label = "ground truth laser intensity"

print("Opening  " + filename + "... "),
f = open(filename, "rb")

print("Done. Loading and casting data to numpy array... "),
data = np.fromfile(f, dtype=np.float32) # read floats with big endian
data.resize(16, 2)

print("Done! Plotting Bar Graph... "),
fig1, ax = plt.subplots(figsize=(16,9))
rects = ax.bar(np.arange(0, 16, 1), data[:, 0])  # average intensity is on the first column
ticks = range(0, 16, 1)
plt.xticks(ticks, ticks)
ax.set_xlabel('Laser/Ring ID')
ax.set_ylabel('Intensity Value')
ax.set_title("Average Intensity of point measures in ground_truth_model by Laser ID")
plt.show(block=False)

print("Done! Saving Bar Chart figure... "),

graphic_filename = datasets_path.constructFullPathToGraphics(sys.argv[1], datasets_path.GROUND_TRUTH_LASER_INTENSITY_BAR_FILE_NAME)
plt.savefig(graphic_filename)
print("Done! Bar Graph saved on: " + graphic_filename)
