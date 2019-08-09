#include <iostream>
#include "rigid_transform_computation/rigid_body_transform.hpp"

using namespace rigid_body_transform;

int main(int argc, char* argv[])
{
  // Initialize ROS
  ros::init(argc, argv, "Rigid_Transform_Computation_Node");
  std::cout << "Node init succesfully" << std::endl;

  ros::NodeHandle nh;
  LiDARCameraCalibrationData calibration_data_object("velo_link", "camera_link");

  ros::Subscriber camera_info_sub;
  if (argc == 2)
  {
    std::cout << "Loading Camera Info from YAML file... ";
    calibration_data_object.readCameraInfoFromYAML(argv[1]);
    std::cout << "Done!" << std::endl;
  }
  else
  {
    std::cout << "Subscribing to Camera Info topic... ";
    camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>(
        "camera/camera_info", 10, &LiDARCameraCalibrationData::cameraInfoCallback, &calibration_data_object);
    std::cout << "Done!" << std::endl;
  }

  ros::Subscriber image_sub = nh.subscribe<rigid_transform_computation::Pixel>(
      "camera/clicked_pixel", 10, &LiDARCameraCalibrationData::pixelCallback, &calibration_data_object);
  ros::Subscriber point_cloud_sub = nh.subscribe<rigid_transform_computation::PointXYZI>(
      "velo/picked_point", 10, &LiDARCameraCalibrationData::pointCloudCallback, &calibration_data_object);

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
