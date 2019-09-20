/*!
 * \file   correspondences_data_listener_node.cpp
 * \brief  Listens to 2D and 3D points selected by the ImageVisualizer and point_cloud::PointCloudVisualizer,
 respectively
 *
 * Implementation of a ROS node that listens to Pixel and PointXYZI custom messages, published by the ImageVisualizer
 and point_cloud::PointCloudVisualizer classes, respectively.
 * This correspondences are 2d <-> 3D that can be used to compute the rigid body transform between the two sensors
 *
 * The node can subscribe to the camera_info topic to obtain the Camera parameters or can read them from a YAML file, if
 provided as argument
 *
 * Also runs two service servers:
 * - save_correspondences: saves the 2D <-> 3D correspondences
 * - compute_rigid_body_transform: given the solvePnP_algorithm, compute the 6 DoF transform between the LiDAR and the
 camera
 *
 */

#include <iostream>
#include "rigid_transform_computation/rigid_body_transform.hpp"

using namespace rigid_body_transform;  // use namespace defined in class file

int main(int argc, char* argv[])
{
  // Initialize ROS
  ros::init(argc, argv, "correspondences_data_listener");
  std::cout << "Node init succesfully" << std::endl;

  ros::NodeHandle nh;
  LiDARCameraCalibrationData calibration_data_object("velo_link", "camera_link");
  ros::Subscriber camera_info_sub;

  // if the camera parameters yaml file was provided as argument, load it. Otherwises instatiate a subscriber to
  // the camera_info topic
  if (argc == 2)
  {
    calibration_data_object.readCameraInfoFromYAML(argv[1]);
    ROS_INFO("Loading Camera Info from YAML file: %s", argv[1]);
  }
  else
  {
    camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>(
        "camera/camera_info", 10, &LiDARCameraCalibrationData::cameraInfoCallback, &calibration_data_object);
    ROS_INFO("Subscribing to Camera Info topic.");
  }

  // Subscribers to Pixel and PointXYZI custom messages
  ros::Subscriber image_sub = nh.subscribe<rigid_transform_computation::Pixel>(
      "camera/clicked_pixel", 10, &LiDARCameraCalibrationData::pixelCallback, &calibration_data_object);
  ros::Subscriber point_cloud_sub = nh.subscribe<rigid_transform_computation::PointXYZI>(
      "velo/picked_point", 10, &LiDARCameraCalibrationData::pointCloudCallback, &calibration_data_object);

  // Services Server
  ros::ServiceServer computationSrv =
      nh.advertiseService("computeRigidBodyTransform", &LiDARCameraCalibrationData::computeRigidTransformSrvCallback,
                          &calibration_data_object);
  ros::ServiceServer saveSrv = nh.advertiseService(
      "save_correspondences", &LiDARCameraCalibrationData::saveCorrespondencesSrvCallback, &calibration_data_object);

  ros::Rate r(20);
  while (ros::ok())
  {
    ros::spinOnce();  // Spin only one time and exits function after spinning
    r.sleep();        // loop at 20 Hz
  }
  return EXIT_SUCCESS;
}
