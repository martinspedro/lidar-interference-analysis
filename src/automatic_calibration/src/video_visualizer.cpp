/*!
 * \file   video_visualizer.cpp
 * \brief  Simple ROS node to visualize the camera feed on OpenCV
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

#include "automatic_calibration/automatic_calibration.hpp"
#include "automatic_calibration/image_visualizer.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "image_visualizer");

  // Create Image Visualizer object to handle incoming video stream and display
  ImageVisualizer image_visualizer_object;

  ros::spin();

  return EXIT_SUCCESS;
}
