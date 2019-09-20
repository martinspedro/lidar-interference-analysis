# Message definition for a datatype representing an OpenCV pixel
#
# Pixel data assumes OpenCV image orientation:
# - Images are row-major
# - rows are indexed with y value
# - columns are indexed with x value
#
# The positive y axis points downwards and the positive x axis points right
#

# Position information
uint32 x    # column coordinate of an image
uint32 y    # row    coordinate of an image

# Color Information
uint8 r     # Red   value of the (y, x) pixel
uint8 g     # Green value of the (y, x) pixel
uint8 b     # Blue  value of the (y, x) pixel
