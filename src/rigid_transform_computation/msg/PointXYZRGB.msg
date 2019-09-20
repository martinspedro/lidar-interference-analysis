# Message definition for a pcl::PointXYZRGB
#
# PointXYZRGB data assumes PCL orientation:
# - x axis points forward
# - y axis points to the right
# - z axis poinst upwards
#
# Color information is conveyed in a RGB triplet

# Position
float64 x   # point x coordinate
float64 y   # point y coordinate
float64 z   # point z coordinate

# Color Information
uint8 r     # Red   value of the (x, y, z) point cloud point
uint8 g     # Green value of the (x, y, z) point cloud point
uint8 b     # Blue  value of the (x, y, z) point cloud point
