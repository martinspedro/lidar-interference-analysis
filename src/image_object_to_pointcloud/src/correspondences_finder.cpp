/**
 * @file   correspondences.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 */

#define PCL_NO_PRECOMPILE        // must be included before any PCL include on this CPP file or HPP included before
#define EIGEN_RUNTIME_NO_MALLOC  // Define this symbol to enable runtime tests for allocations
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>

#include <Eigen/Core>  // Needs to be included before the opencv eigen interface!
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>  // Needs to be included before other opencv headers!
// OPENCV
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"

//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>

//#include "sensor_fusion/color.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <std_msgs/Int8.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stdint.h>
#include <image_geometry/pinhole_camera_model.h>
#include <typeinfo>

#include <exception>
#include <list>

#include <pcl/filters/frustum_culling.h>

#include <point_cloud_statistics/velodyne_point_type.h>
#include "point_cloud_statistics/point_cloud_statistics.hpp"

#include "image_object_to_pointcloud/pinhole_camera_model_utilities.hpp"

#include <geometry_msgs/TransformStamped.h>

#include <cmath>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>

#include "tf2/LinearMath/Matrix3x3.h"
#include <geometry_msgs/TransformStamped.h>
#include "tf2/LinearMath/Quaternion.h"

#include <string>

const float NEAR_PLANE_DISTANCE = 1.0f;
const float FAR_PLANE_DISTANCE = 30.0f;
const double PIXEL_SIZE = 3.45e-6;

// Define types to be used depending on the dataset
#define USE_WITH_KITTI

#ifdef USE_WITH_KITTI
typedef point_cloud::PointCloudXYZ PointCloudType;
typedef pcl::PointXYZ PointType;
#else
typedef velodyne::VelodynePointCloud PointCloudType;
typedef velodyne::PointXYZIR PointType;
#endif

// Eigen::Quaterniond tf_quaternion;
// Eigen::Vector3d tf_translation;
Eigen::Affine3d transform_eigen;

ros::Publisher pub, pub_camera, pub_camera_lidar, pub_lidar, pub_final;  //!< ROS Publisher

Eigen::Matrix4f getLiDARPose(const PointCloudType::ConstPtr point_cloud_ptr)
{
  Eigen::Matrix4f lidar_pose_ = Eigen::Matrix4f::Identity();

  lidar_pose_.block<3, 3>(0, 0) = point_cloud_ptr->sensor_orientation_.matrix();  // Quaternion to Rot Matrix
  lidar_pose_.block<4, 1>(0, 3) = point_cloud_ptr->sensor_origin_;                // Vector 4f

  /*
  // lidar_pose.row(3).head(3) =.head(3);
  lidar_pose(3, 0) = point_cloud_ptr->sensor_orientation_.x();
  lidar_pose(3, 1) = point_cloud_ptr->sensor_orientation_.y();
  lidar_pose(3, 2) = point_cloud_ptr->sensor_orientation_.z();
  lidar_pose(3, 3) = point_cloud_ptr->sensor_orientation_.w();
  */

  return lidar_pose_;
}

void callback(const darknet_ros_msgs::ObjectCountConstPtr& object_count,
              const darknet_ros_msgs::BoundingBoxesConstPtr& b_boxes, const sensor_msgs::CameraInfoConstPtr& cam_info,
              const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
  std::cout << "Callback" << std::endl;

  // Create Datatype dependent if operating with KITTI datasets or Experimental Dataype
  PointCloudType point_cloud_velodyne, point_cloud, target;
  PointCloudType::Ptr cloudPtr, point_cloud_ptr(new PointCloudType);
  std::cout << "Init" << std::endl;
  /* Convert ROS msg to Point Cloud
    sensor_msgs::PointCloud2 point_cloud_camera_msg;
    try
    {
      tf2::doTransform(*point_cloud_msg, point_cloud_camera_msg, transformStamped);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    fromROSMsg(*point_cloud_camera_msg, point_cloud);
  */
  fromROSMsg(*point_cloud_msg, point_cloud);
  *point_cloud_ptr = point_cloud;
  std::cout << "Message" << std::endl;

  /*
  std::cout << "Sensor Origin:" << point_cloud.sensor_origin_ << std::endl;
  std::cout << "Sensor Orientation: " << point_cloud.sensor_orientation_.x() << ", "
            << point_cloud.sensor_orientation_.y() << ", " << point_cloud.sensor_orientation_.z() << ", "
            << point_cloud.sensor_orientation_.w() << ", " << std::endl;
*/

  // Get Camera Information from msg
  image_geometry::PinholeCameraModel cam_model_;
  cam_model_.fromCameraInfo(cam_info);
  std::cout << "Camera Model" << std::endl;

  cv::Size image_resolution = cam_model_.fullResolution();
  FOV image_fov = getImageFOV(cam_model_);
  std::cout << "FOV" << std::endl;

  double aperture_width = image_resolution.width * PIXEL_SIZE;
  double aperture_height = image_resolution.height * PIXEL_SIZE;
  double fov_x, fov_y, focal_length, aspect_ratio;

  cv::Point2d principal_point;
  cv::calibrationMatrixValues(cam_model_.fullIntrinsicMatrix(), image_resolution, aperture_width, aperture_height,
                              fov_x, fov_y, focal_length, principal_point, aspect_ratio);
  std::cout << "OpenCV" << std::endl;

  FOV bounding_box_fov;
  Eigen::Vector3f camera_rotation;
  computeBoundingBoxFOV(cam_model_, b_boxes->bounding_boxes[0], &bounding_box_fov, camera_rotation);
  std::cout << "Compute BBOxFOV" << std::endl;

  // Pose Rotations

  // Compute Camera Affine Transform from Angle Axis vector
  cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1);
  cv::Vec3f camera_rotation_cv;
  Eigen::Matrix3f rotation_matrix_eigen;

  cv::eigen2cv<float, 3, 1>(camera_rotation, camera_rotation_cv);
  cv::Rodrigues(camera_rotation_cv, rotation_matrix);  // Rodrigues overloads matrix type from float to double!
  cv::cv2eigen<float, 3, 3>(rotation_matrix, rotation_matrix_eigen);

  std::cout << "Compute Camera Rotation" << std::endl;
  Eigen::Quaternionf q_camera(rotation_matrix_eigen);
  Eigen::Matrix4f camera_transform = Eigen::Matrix4f::Identity();
  camera_transform.block<3, 3>(0, 0) = q_camera.matrix();  // Quaternion to Rot Matrix
  std::cout << "Compute Rotation Matrix Done" << std::endl;
  // lidar_pose_.block<4, 1>(0, 4) = point_cloud_ptr->sensor_origin_;  // Vector 4f

  // Get Initial LiDAR Pose
  Eigen::Matrix4f camera_to_lidar_6DOF = (transform_eigen.matrix()).cast<float>();  // Convert Eigen Affine3d Trans
  std::cout << "6DOF" << std::endl;
  Eigen::Matrix4f lidar_pose = getLiDARPose(point_cloud_ptr);
  std::cout << "LiDAR Pose Done" << std::endl;

  // Order is computed on the inverse order!
  Eigen::Matrix4f frustrum_filter_pose = camera_to_lidar_6DOF * camera_transform * lidar_pose;
  /* Note: This assumes a coordinate system where X is forward, Y is up, and Z is right. To convert from the
  traditional
  // camera coordinate system (X right, Y down, Z forward), one can use:
  // clang-format off

  Eigen::Matrix4f cam2robot;
  cam2robot << 0, 0, 1, 0
               0, -1, 0, 0
               1, 0, 0, 0
               0, 0, 0, 1;
  // clang-format on
*/

  // Print Stuff
  float fov_X =
      atan((b_boxes->bounding_boxes[0].xmax - b_boxes->bounding_boxes[0].xmin) / (cam_model_.fx())) * 180.0f / M_PI;
  float fov_Y =
      atan((b_boxes->bounding_boxes[0].ymax - b_boxes->bounding_boxes[0].ymin) / (cam_model_.fy())) * 180.0f / M_PI;

  std::cout << "(" << b_boxes->bounding_boxes[0].xmin << ", " << b_boxes->bounding_boxes[0].xmax << ") e ("
            << b_boxes->bounding_boxes[0].ymin << ", " << b_boxes->bounding_boxes[0].ymax << ") " << std::endl;
  std::cout << b_boxes->bounding_boxes[0].Class << ": " << b_boxes->bounding_boxes[0].probability << " (" << fov_X
            << " ," << fov_Y << ") with Principial Point: " << principal_point << std::endl;
  std::cout << b_boxes->bounding_boxes[0].Class << ": " << b_boxes->bounding_boxes[0].probability << " ("
            << bounding_box_fov.x << " ," << bounding_box_fov.y << ") with Principial Point: " << principal_point
            << std::endl;
  std::cout << "Rotation: " << camera_rotation << std::endl;

  pcl::FrustumCulling<PointType> fc;
  fc.setInputCloud(point_cloud_ptr);
  fc.setVerticalFOV(bounding_box_fov.y);
  fc.setHorizontalFOV(bounding_box_fov.x);
  fc.setNearPlaneDistance(NEAR_PLANE_DISTANCE);
  fc.setFarPlaneDistance(FAR_PLANE_DISTANCE);
  std::cout << "FrustumCulling" << std::endl;

  // std::cout << "Pose: " << camera_pose << std::endl;
  // .. read or input the camera pose from a registration algorithm.
  fc.setCameraPose(frustrum_filter_pose);
  fc.filter(target);
  std::cout << "Filtred. Target size = " << target.size() << std::endl;

  sensor_msgs::PointCloud2 out_cloud;
  *point_cloud_ptr = target;
  std::cout << "Atributed" << std::endl;
  // pcl::toROSMsg(target, *out_cloud);
  std::cout << "toROSMsg Done" << std::endl;
  pub.publish(point_cloud_ptr);  // PointCloud Object is automacally serialized by ROS and there is no need to call
                                 // toROSMsg
  std::cout << "Published" << std::endl;

  geometry_msgs::Pose pose_camera, pose_lidar_camera, pose_lidar, pose_final;
  geometry_msgs::PoseStamped pose_camera_s, pose_lidar_s, pose_lidar_camera_s, pose_final_s;
  Eigen::Affine3d camera_affine_transform, lidar_camera_affine_transform, lidar_affine_transform,
      final_affine_transform;

  camera_affine_transform.matrix() = camera_transform.cast<double>();
  lidar_affine_transform.matrix() = lidar_pose.cast<double>();
  lidar_camera_affine_transform.matrix() = camera_to_lidar_6DOF.cast<double>();
  final_affine_transform.matrix() = frustrum_filter_pose.cast<double>();

  pose_camera = Eigen::toMsg(camera_affine_transform);
  pose_lidar = Eigen::toMsg(lidar_affine_transform);
  pose_lidar_camera = Eigen::toMsg(lidar_camera_affine_transform);
  pose_final = Eigen::toMsg(final_affine_transform);

  pose_camera_s.header.stamp = ros::Time::now();
  pose_lidar_s.header.stamp = ros::Time::now();
  pose_lidar_camera_s.header.stamp = ros::Time::now();
  pose_final_s.header.stamp = ros::Time::now();

  pose_camera_s.header.frame_id = "camera_color_left";
  pose_lidar_s.header.frame_id = "velo_link";
  pose_lidar_camera_s.header.frame_id = "camera_color_left";
  pose_final_s.header.frame_id = "velo_link";

  pose_camera_s.pose = pose_camera;
  pose_lidar_s.pose = pose_lidar;
  pose_lidar_camera_s.pose = pose_lidar_camera;
  pose_final_s.pose = pose_final;
  // msg.header.frame_id = "detection";

  pub_camera.publish(pose_camera_s);
  pub_lidar.publish(pose_lidar_s);
  pub_camera_lidar.publish(pose_lidar_camera_s);
  pub_final.publish(pose_final_s);
}

const std::string LIDAR_TF2_REFERENCE_FRAME = "velo_link";
const std::string CAMERA_TF2_REFERENCE_FRAME = "camera_color_left";

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "correspondences_finder");

  std::cout << "Estou" << std::endl;
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(
      LIDAR_TF2_REFERENCE_FRAME, CAMERA_TF2_REFERENCE_FRAME, ros::Time(0), ros::Duration(20.0));

  // get Affine3d matrix that defines the 6DOF transformation between LiDAR and Camera
  transform_eigen = tf2::transformToEigen(transformStamped);
  std::cout << transform_eigen.matrix() << std::endl;

  // geometry_msgs::Vector3 translation = transformStamped.transform.translation;
  // geometry_msgs::Quaternion rotation_q = transformStamped.transform.rotation;

  // tf2::convert<geometry_msgs::Quaternion, Eigen::Quaterniond>(rotation_q, tf_quaternion);
  // Eigen::fromMsg(rotation_q, tf_quaternion);
  // tf2::convert<geometry_msgs::Vector3, Eigen::Vector3d>(translation, tf_translation);
  // CHECK AFFINE 3D

  // tf::quaternionMsgToEigen(rotation_q, tf_quaternion);
  // tf::vectorMsgToEigen(translation, tf_translation);

  pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1);
  pub_camera = nh.advertise<geometry_msgs::PoseStamped>("/rotated_camera_pose", 1);
  pub_camera_lidar = nh.advertise<geometry_msgs::PoseStamped>("/camera_pose_in_lidar", 1);
  pub_lidar = nh.advertise<geometry_msgs::PoseStamped>("/lidar_pose", 1);
  pub_final = nh.advertise<geometry_msgs::PoseStamped>("/final_pose", 1);

  message_filters::Subscriber<darknet_ros_msgs::ObjectCount> object_count_sub(nh, "/darknet_ros/found_object", 2);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub(nh, "/darknet_ros/bounding_boxes", 2);
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub(nh, "/camera/camera_info", 20);
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub(nh, "/velodyne_points", 20);

  typedef message_filters::sync_policies::ApproximateTime<
      darknet_ros_msgs::ObjectCount, darknet_ros_msgs::BoundingBoxes, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2>
      MySyncPolicy;

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), object_count_sub, bounding_boxes_sub, cam_info_sub,
                                                   point_cloud_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::Rate r(5);  // 10 hz
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return EXIT_SUCCESS;
}
