#!/usr/bin/env python3
import sys

import matplotlib
import numpy as np
import matplotlib.pyplot as plt

from datasets_path import datasets_path

# if number of arguments is invalid
if len(sys.argv) != 2:
    sys.exit("""Wrong number of arguments!\n
                Usage: rosrun point_cloud_statistics ground_truth_azimuth_intensity.py <test scenario codename>""")

graphics_folder = datasets_path.makeGraphicsDirectory(sys.argv[1], datasets_path.GRAPHICS_FOLDER_RELATIVE_PATH)
filename = datasets_path.constructFullPathToResults(sys.argv[1], datasets_path.GROUND_TRUTH_AZIMUTH_INTENSITY_BIN_NAME)


data_label = "ground truth azimuth intensity"

print("Opening  " + filename + "... "),
f = open(filename, "rb")

print("Done. Loading and casting data to numpy array... "),
data = np.fromfile(f, dtype=np.float32) # read floats with big endian
data.resize(1800, 2)

print("Done! Plotting Polar Graph..."),
theta = np.linspace(-np.pi, np.pi,1800)    # Theta in polar coordinates
theta_labels = range(180, -180, -30)       # Theta in degrees (inverted because of velodyne referential frame)
ticks = np.linspace(0, 2*np.pi, len(theta_labels) + 1) #plt.xticks()[0]     # get ticks

plt.subplots(figsize=(16,9))
ax = plt.subplot(111, projection='polar')
ax.plot(theta, data[:, 0])  # average intensity is on the first column
plt.xticks(ticks,theta_labels)
ax.set_title("Average Intensity of point measures in ground_truth_model by azimuth")
plt.show(block=False)

print("Done! Saving histogram figure... ")

graphic_filename = datasets_path.constructFullPathToGraphics(sys.argv[1], datasets_path.GROUND_TRUTH_AZIMUTH_INTENSITY_POLAR_FILE_NAME)
plt.savefig(graphic_filename)
print("Done! Polar Graph saved on: " + graphic_filename)
