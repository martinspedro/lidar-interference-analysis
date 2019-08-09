# Message definition for a pcl::PointXYZ
#
# PointXYZ data assumes PCL orientation:
# - x axis points forward
# - y axis points to the right
# - z axis poinst upwards
#
# The intensity field is to store the intensity of the reflected luminance of the LiDAR ray
#

# Position
float64 x         # point x coordinate
float64 y         # point y coordinate
float64 z         # point z coordinate

# Point Characteristics
float64 intensity # point intensity measure
