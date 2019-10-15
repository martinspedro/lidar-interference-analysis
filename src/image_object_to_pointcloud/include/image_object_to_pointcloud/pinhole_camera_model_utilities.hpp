

#ifndef PINHOLE_CAMERA_MODEL_UTILITIES_H
#define PINHOLE_CAMERA_MODEL_UTILITIES_H

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>

#include <darknet_ros_msgs/BoundingBox.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <boost/shared_ptr.hpp>

struct FOV
{
  union
  {
    float data[2];
    struct
    {
      float x;
      float y;
    };
  };
};

inline cv::Point2f getImageCenterPoint(cv::Size dimensions);
inline float getFovRadian(const float length, const float distance_to_focal_point);
inline float getFovDegree(const float length, const float distance_to_focal_point);

FOV getImageFOV(image_geometry::PinholeCameraModel cam_model);
FOV getImageFOV(cv::Size image_dimensions, cv::Matx33d intrinsic_matrix);

void computeBoundingBoxFOV(image_geometry::PinholeCameraModel cam_model, darknet_ros_msgs::BoundingBox bounding_box,
                           FOV* bounding_box_fov, Eigen::Vector3f& camera_rotation);

#endif  // PINHOLE_CAMERA_MODEL_UTILITIES_H
