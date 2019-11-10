
#ifndef IMAGE_OBJECT_TO_POINT_CLOUD_H
#define IMAGE_OBJECT_TO_POINT_CLOUD_H

#include <Eigen/Core>  // Needs to be included before the opencv eigen interface!
#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <image_geometry/pinhole_camera_model.h>
#include <point_cloud_statistics/velodyne_point_type.h>
#include "point_cloud_statistics/point_cloud_statistics.hpp"
#include "image_object_to_pointcloud/FOV.hpp"

// Define types to be used depending on the dataset
#define USE_WITH_KITTI

#ifdef USE_WITH_KITTI
typedef point_cloud::PointCloudXYZ PointCloudType;
typedef pcl::PointXYZ PointType;
#else
typedef velodyne::VelodynePointCloud PointCloudType;
typedef velodyne::PointXYZIR PointType;
#endif

extern const Eigen::Matrix4f LIDAR_POSE_TO_FRUSTRUM_POSE;

Eigen::Matrix4f angleAxisVecToTransform(const Eigen::Vector3f angle_axis_rotation);
void getLiDARPose(const PointCloudType::ConstPtr point_cloud_ptr, Eigen::Matrix4f& lidar_pose);
void getCameraRotation(image_geometry::PinholeCameraModel cam_model_, darknet_ros_msgs::BoundingBox bounding_box,
                       Eigen::Matrix4f& fov_bbox);
#endif  // IMAGE_OBJECT_TO_POINT_CLOUD_H
