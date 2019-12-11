/*!
 * \file   pinhole_camera_model_utilities.hpp
 * \brief  General utilities header file for pinhole camera model operations
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

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

#include "image_object_to_pointcloud/FOV.hpp"

/*!
 * \namespace pinhole_camera
 * \brief namespace to enclose all the operations that use the pinhole camera model
 */
namespace pinhole_camera
{
/*!
 * \brief Check if a pixel is inside the image dimensions
 * \param[in] pixel Integer coordinates of the pixel on the image using a cv::Point2i object
 * \param[in] image_dimensions Dimensions of the image using cv::Size
 * \retval True if pixel index is inside image dimensions
 * \retval False otherwise
 */
inline bool pixelInsideImageDimensions(cv::Point2i pixel, cv::Size image_dimensions)
{
  return (pixel.x >= 0) && (pixel.y >= 0) && (pixel.x <= image_dimensions.width) &&
         (pixel.y <= image_dimensions.height);
}

/*!
 * \brief Get image center in pixel coordinates
 * \param[in] dimensions Dimensions of the image using cv::Size
 * \return The pixel coordinates in a cv::Point2f
 */
inline cv::Point2f getImageCenterPoint(cv::Size dimensions)
{
  // image is indexed (x, y) despite matrices/Mat being indexed (y, x)
  // Point constructor is (x, y)
  return cv::Point2f(dimensions.width / 2.0f, dimensions.height / 2.0f);
};

/*!
 * \brief Get image center in pixel coordinates
 * \param[in] intrinsic_matrix Intrinsic Calibration Matrix
 * \return The pixel coordinates in a cv::Point2d
 */
inline cv::Point2d getImageCenterPoint(cv::Matx33d intrinsic_matrix)
{
  // Retrieve cx and cy from camera intrinsic matrix
  // Point constructor is (x, y)
  // Matx are indexed using curly braces
  return cv::Point2d(intrinsic_matrix(0, 2), intrinsic_matrix(1, 2));
};

/*!
 * \brief Compute Field Of View along on eimage dimension in Radians
 * \param[in] length Length of the ROI along the desired axis
 * \param[in] distance_to_focal_point Distance form the image plan to the focal point. Also know as the \f$f\f$
 * parameter
 * \return the FOV along the axis in radians
 * \remark Assumes a pinhole camera model and that the FOV of the ROI is centered on the principal image axis
 */
inline float getFovRadian(const float length, const float distance_to_focal_point)
{
  return atan2(length, distance_to_focal_point);
};

/*!
 * \brief Compute Field Of View along on image dimension in Degrees
 * \param[in] length Length of the ROI along the desired axis
 * \param[in] distance_to_focal_point Distance form the image plan to the focal point. Also know as the \f$f\f$
 * parameter
 * \return the FOV along the axis in radians
 * \remark Assumes a pinhole camera model and that the FOV of the ROI is centered on the principal image axis
 * \remark Uses getFovRadian and converts iyts output to radian
 */
inline float getFovDegree(const float length, const float distance_to_focal_point)
{
  return getFovRadian(length, distance_to_focal_point) * 180.0f / M_PI;
};

/*!
 * \brief Computes the Field Of View of an Image in degrees
 * \param[in] cam_model Pinhole Camera Model object from ROS image_geometry package
 * \return the FOV object that defines the field of view of the image along the two image axis in degrees
 * \remark Assumes a pinhole camera model
 * \remark ROS image_geometry package compatible
 */
FOV getImageFOV(image_geometry::PinholeCameraModel cam_model);

/*!
 * \brief Computes the Field Of View of an Image in degrees
 * \param[in] image_dimensions Dimensions of the image using cv::Size
 * \param[in] intrinsic_matrix Intrinsic Calibration Matrix
 * \return the FOV object that defines the field of view of the image along the two image axis in degrees
 * \remark Assumes a pinhole camera model
 * \remark OpenCV compatible
 */
FOV getImageFOV(cv::Size image_dimensions, cv::Matx33d intrinsic_matrix);

/*!
 * \brief Computes the Field Of View of an Image Bounding Box
 * \param[in] cam_model Pinhole Camera Model object from ROS image_geometry package
 * \param[in] bounding_box bounding box object from darknet_ros package
 * \param[out] bounding_box_fov FOV object pointer that defines the field of view of the image along the two image axis
 * in degrees
 * \param[out] camera_rotation OpenCV Axis Rotation Vector
 * \remark Assumes a pinhole camera model
 * \remark camera_rotation can be converted to a Rotation Matrix using cv::Rodrigues
 *
 * Computes the FOV for the bounding box. The bounding box must no be centered in the image principal axis. If the
 * boudning box is not centered, it computes the rotation required to apply on the camera coordinate frame so that the
 * new principal axis passes through the bounding center
 */
void computeBoundingBoxFOV(image_geometry::PinholeCameraModel cam_model, darknet_ros_msgs::BoundingBox bounding_box,
                           FOV* bounding_box_fov, Eigen::Vector3f& camera_rotation);

}  // namespace pinhole_camera

#endif  // PINHOLE_CAMERA_MODEL_UTILITIES_H
