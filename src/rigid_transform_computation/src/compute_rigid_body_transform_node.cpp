/*!
 * \file   compute_rigid_body_transform_node.cpp
 * \brief  Node implementation that computes and publishes a static rigib body transform between a moncular camera and a
 3D LiDAR
 *
 * ROS Node implementation file that reads the follwoing files:
 * - CSV file (CSV_manager for format) with 2D <-> 3D correspondences a
 * - Camera Info YAML file
 *
 * And computes the rigid body transform using computeRigidBodyTransform, that is after published under /tf_static
 *
 *  \remark Requires that the solvePnP algorithm to given as argument
 *  \parblock
 *  Available methods:
 *  - 0: SOLVEPNP_ITERATIVE
 *  - 1: SOLVEPNP_P3P
 *  - 2: SOLVEPNP_EPNP
 *  - 3: SOLVEPNP_DLS
 *  - 4: SOLVEPNP_UPNP
 *  \endparblock
 */

#include <iostream>
#include <sstream>

#include "rigid_transform_computation/rigid_body_transform.hpp"
#include "rigid_transform_computation/CSV_manager.hpp"

using namespace rigid_body_transform;

int main(int argc, char* argv[])
{
  // Initialize ROS
  ros::init(argc, argv, "compute_rigid_body_transform");
  ROS_INFO("ROS Node init succesfully!");

  // Node must be called giving the correspondences CSV filenam, the camera info YAML file and the solvePnP_algorithm
  if (argc != 4)
  {
    ROS_INFO("Usage: rosrun rigid_transform_computation compute_rigid_body_transform_node csv_filename"
             "camera_info_yaml_filename solvePnP_algorithm");
    return EXIT_FAILURE;
  }

  std::vector<cv::Point3f> point_cloud_points;
  std::vector<cv::Point2f> image_pixels;

  std::cout << "Loading 2D <-> 3D correspondences for calibration... ";
  csv_file_manager::read(THESIS_FILE_PATH + argv[1], &image_pixels, &point_cloud_points);

  std::cout << "Done!" << std::endl << "Creating LiDAR & Camera Calibration Object... ";
  LiDARCameraCalibrationData calibration_data_object("velo_link", "camera_link", point_cloud_points, image_pixels);

  std::cout << "Done!" << std::endl << "Loading Camera Info YAML file... ";
  calibration_data_object.readCameraInfoFromYAML(argv[2]);

  std::cout << "Done!" << std::endl << "Computing Rigid Body Transform... ";
  calibration_data_object.computeRigidBodyTransform(atoll(argv[3]));

  std::cout << "Done! " << std::endl << "Create Static Transform Message to be published... ";
  calibration_data_object.publishStaticTransformMsg();

  std::cout << "Done! " << std::endl << "Static Transform is going to be published under /tf_static. " << std::endl;

  ros::Rate r(1);
  while (ros::ok())
  {
    ros::spinOnce();  // Spin even if publishStaticTransformMsg latches messages
    r.sleep();        // loop at 1 Hz
  }

  return EXIT_SUCCESS;
}
