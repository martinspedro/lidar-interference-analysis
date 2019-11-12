
#ifndef IMAGE_OBJECT_TO_POINT_CLOUD_H
#define IMAGE_OBJECT_TO_POINT_CLOUD_H

#include <Eigen/Core>  // Needs to be included before the opencv eigen interface!
#include <sensor_msgs/PointCloud2.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <boost/shared_ptr.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <point_cloud_statistics/velodyne_point_type.h>
#include "point_cloud_statistics/point_cloud_statistics.hpp"
#include "image_object_to_pointcloud/FOV.hpp"
#include "image_object_to_pointcloud/pinhole_camera_model_utilities.hpp"

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

inline bool pixelInsideBoundingBox(cv::Point2i pixel, darknet_ros_msgs::BoundingBox bounding_box)
{
  return (pixel.x >= bounding_box.xmin) && (pixel.y >= bounding_box.ymin) && (pixel.x <= bounding_box.xmax) &&
         (pixel.y <= bounding_box.ymax);
}

inline cv::Point3d PCLToOpenCvPoint(pcl::PointXYZ pcl_point)
{
  return cv::Point3d(pcl_point.x, pcl_point.y, pcl_point.z);
}

/**
 * \param[in] camera_point_cloud_ptr Point Cloud in the camera coordinate frame
 *
 * \remark The function is inline due to speed requirements
 */
inline void filterPointCloudFromCameraROIs(const unsigned int& b_boxes_object_count,
                                           const darknet_ros_msgs::BoundingBoxesConstPtr& b_boxes,
                                           const PointCloudType::ConstPtr camera_point_cloud_ptr,
                                           const image_geometry::PinholeCameraModel& camera_model,
                                           const cv::Size& image_dimensions, const float& near_plane_distance,
                                           const float& far_plane_distance,
                                           boost::shared_ptr<std::vector<int>> index_shr_ptr)
{
  for (int i = 0; i < camera_point_cloud_ptr->points.size(); i++)
  {
    cv::Point2d uv = camera_model.project3dToPixel(PCLToOpenCvPoint(camera_point_cloud_ptr->points[i]));
    cv::Point2i uv_int = cv::Point2i((int)(uv.x), (int)(uv.y));

    if (pinhole_camera::pixelInsideImageDimensions(uv_int, image_dimensions) &&
        (camera_point_cloud_ptr->points[i].z > near_plane_distance) &&
        (camera_point_cloud_ptr->points[i].z < far_plane_distance))
    {
      for (int j = 0; j < b_boxes_object_count; ++j)
      {
        if (pixelInsideBoundingBox(uv_int, b_boxes->bounding_boxes[j]))
        {
          index_shr_ptr->push_back(i);
        }
      }
    }
  }
}

Eigen::Matrix4f angleAxisVecToTransform(const Eigen::Vector3f angle_axis_rotation);
void getLiDARPose(const PointCloudType::ConstPtr point_cloud_ptr, Eigen::Matrix4f& lidar_pose);
void getCameraRotation(image_geometry::PinholeCameraModel cam_model_, darknet_ros_msgs::BoundingBox bounding_box,
                       Eigen::Matrix4f& fov_bbox);
#endif  // IMAGE_OBJECT_TO_POINT_CLOUD_H
