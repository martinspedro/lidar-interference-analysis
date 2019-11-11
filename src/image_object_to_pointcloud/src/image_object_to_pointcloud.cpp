
#define EIGEN_RUNTIME_NO_MALLOC  // Define this symbol to enable runtime tests for allocations

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>  // Needs to be included before other opencv headers!
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"

#include "image_object_to_pointcloud/image_object_to_pointcloud.hpp"
#include "image_object_to_pointcloud/pinhole_camera_model_utilities.hpp"

#include <ros/ros.h>

/*
 * http://docs.pointclouds.org/trunk/frustum__culling_8hpp_source.html#l00047 cv::Size im_dimensions =
 * cam_model_.fullResolution(); This assumes a coordinate system where X is forward, Y is up, and Z is right. To
 * convert from the traditional camera coordinate system (X right, Y down, Z forward), one can use:
 *
 * The original matrix has (2,1) = -1 and (1, 2) = 1. This matrix, with switched coefficients allows for a flip on the
 * axis, rendering more visually accurate results, despite might not being correct
 */
// clang-format off
const Eigen::Matrix4f LIDAR_POSE_TO_FRUSTRUM_POSE = (Eigen::Matrix4f() <<  1, 0, 0, 0,
                                                                           0, 0,-1, 0,
                                                                           0, 1, 0, 0,
                                                                           0, 0, 0, 1).finished();
// clang-format on

/**
 * \brief Compute Camera Affine Transform from Angle Axis vector
 *
 * \remark Conversions and transforms verified using https://www.andre-gaschler.com/rotationconverter/
 */
Eigen::Matrix4f angleAxisVecToTransform(const Eigen::Vector3f angle_axis_rotation)
{
  cv::Matx33f cv_rotation_matrix = cv::Matx33f(3, 3, CV_32FC1);
  cv::Vec3f cv_rotation_vec;
  Eigen::Matrix3f eigen_rotation_matrix;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

  /* Converts from Eigen::Vector to Opencv Vector
   * Opencv Rodrigues converts the Angle Axis Vector Rotation to a Rotation Matrix
   * The Rotation Matrix is converted to a Eigen Representation
   */
  cv::eigen2cv<float, 3, 1>(angle_axis_rotation, cv_rotation_vec);
  cv::Rodrigues(cv_rotation_vec, cv_rotation_matrix);  // Rodrigues overloads matrix type from float to double!
  cv::cv2eigen<float, 3, 3>(cv_rotation_matrix, eigen_rotation_matrix);  // convertion keeps datatype on float

  transform.block<3, 3>(0, 0) = eigen_rotation_matrix;  // Add rotation to affine transform

  ROS_DEBUG_STREAM("Eigen Angle Axis Vec: " << std::endl
                                            << angle_axis_rotation << std::endl
                                            << "CV Angle Axis Vec: " << std::endl
                                            << cv_rotation_vec << std::endl
                                            << "Eigen Rotation Matrix: " << std::endl
                                            << eigen_rotation_matrix << std::endl
                                            << "CV Rotation Matrix: " << std::endl
                                            << cv_rotation_matrix << std::endl
                                            << "Eigen Transform: " << std::endl
                                            << transform << std::endl);

  return transform;
}

void getLiDARPose(const PointCloudType::ConstPtr point_cloud_ptr, Eigen::Matrix4f& lidar_pose)
{
  lidar_pose = Eigen::Matrix4f::Identity();

  lidar_pose.block<3, 3>(0, 0) = point_cloud_ptr->sensor_orientation_.matrix();  // Quaternion to Rot Matrix
  lidar_pose.block<4, 1>(0, 3) = point_cloud_ptr->sensor_origin_;                // Vector 4f
}

void getCameraRotation(image_geometry::PinholeCameraModel cam_model_, darknet_ros_msgs::BoundingBox bounding_box,
                       Eigen::Matrix4f& fov_bbox)
{
  // Get middle point of the bounding box
  cv::Point2d bounding_box_center;
  bounding_box_center.x = (bounding_box.xmin + bounding_box.xmax) / 2.0f;
  bounding_box_center.y = (bounding_box.ymin + bounding_box.ymax) / 2.0f;

  cv::Point3d bbox_ray = cam_model_.projectPixelTo3dRay(bounding_box_center);

  cv::Point2d image_center = pinhole_camera::getImageCenterPoint(cam_model_.fullIntrinsicMatrix());
  cv::Point3d center_ray = cam_model_.projectPixelTo3dRay(image_center);

  // Convert CV points to Eigen data type
  Eigen::Vector3d bbox_ray_eigen(bbox_ray.x, bbox_ray.y, bbox_ray.z);
  Eigen::Vector3d center_ray_eigen(center_ray.x, center_ray.y, center_ray.z);

  // Computes the rotation that sends a line in the direction of the bbox_ray to the direction of center ray. Both Lines
  // pass throught the origin. See https://eigen.tuxfamily.org/dox/classEigen_1_1QuaternionBase.html#title26
  Eigen::Quaterniond fov_lidar_quaternion = Eigen::Quaterniond().setFromTwoVectors(bbox_ray_eigen, center_ray_eigen);
  fov_lidar_quaternion.normalize();

  // Create Pure Rotation Affine Transform
  fov_bbox = Eigen::Matrix4f::Identity();
  fov_bbox.block<3, 3>(0, 0) = (fov_lidar_quaternion.toRotationMatrix()).cast<float>();
}
