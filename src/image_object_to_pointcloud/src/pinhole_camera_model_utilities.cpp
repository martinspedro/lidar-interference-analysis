
#include <cmath>

#include "image_object_to_pointcloud/pinhole_camera_model_utilities.hpp"

inline cv::Point2f getImageCenterPoint(cv::Size dimensions)
{
  // image is indexed (x, y) despite matrices/Mat being indexed (y, x)
  return cv::Point2f(dimensions.width / 2.0f, dimensions.height / 2.0f);
}

inline float getFovRadian(const float length, const float distance_to_focal_point)
{
  return atan(length / distance_to_focal_point);
}

inline float getFovDegree(const float length, const float distance_to_focal_point)
{
  return getFovRadian(length, distance_to_focal_point) * 180.0f / M_PI;
}

// Oriented for ROS image_geometry
FOV getImageFOV(image_geometry::PinholeCameraModel cam_model)
{
  cv::Size im_dimensions = cam_model.fullResolution();

  FOV im_fov;

  /* FOV is computed using a rectangular triangle, with half of the image dimensions
   * Therefore the result is equivalent to half of the Image field of view and requireds that in length is half of the
   * full image dimension along the desired dimensions
   */
  im_fov.x = 2 * getFovDegree(im_dimensions.width / 2, cam_model.fx());
  im_fov.y = 2 * getFovDegree(im_dimensions.height / 2, cam_model.fy());

  return im_fov;
}

// For full OpenCV
FOV getImageFOV(cv::Size image_dimensions, cv::Matx33d intrinsic_matrix)
{
  FOV im_fov;

  /* FOV is computed using a rectangular triangle, with half of the image dimensions
   * Therefore the result is equivalent to half of the Image field of view and requireds that in length is half of the
   * full image dimension along the desired dimensions
   */
  im_fov.x = 2 * getFovDegree(image_dimensions.width / 2, intrinsic_matrix(0, 0));
  im_fov.y = 2 * getFovDegree(image_dimensions.height / 2, intrinsic_matrix(1, 1));

  return im_fov;
}

void computeBoundingBoxFOV(image_geometry::PinholeCameraModel cam_model, darknet_ros_msgs::BoundingBox bounding_box,
                           FOV* bounding_box_fov, Eigen::Vector3f& camera_rotation)
{
  cv::Size im_dimensions = cam_model.fullResolution();
  std::cout << "Camera" << std::endl;

  float x_min_fov = getFovDegree(bounding_box.xmin, cam_model.fx());
  float x_max_fov = getFovDegree(bounding_box.xmax, cam_model.fx());

  float y_min_fov = getFovDegree(bounding_box.ymin, cam_model.fy());
  float y_max_fov = getFovDegree(bounding_box.ymax, cam_model.fy());
  std::cout << "FOV Degree" << std::endl;
  bounding_box_fov->x = x_min_fov - x_max_fov;
  bounding_box_fov->y = y_min_fov - y_max_fov;
  std::cout << "FOV access" << std::endl;
  cv::Point2f image_center = getImageCenterPoint(im_dimensions);
  std::cout << "Center Point" << std::endl;
  cv::Point2f bounding_box_center;
  bounding_box_center.x = bounding_box.xmin + (bounding_box.xmax - bounding_box.xmin) / 2;
  bounding_box_center.y = bounding_box.ymin + (bounding_box.ymax - bounding_box.ymin) / 2;

  // FOV image_fov = computeImageFOV(cam_model);

  // Distance in pixel between bounding box center and image center
  float dx = bounding_box_center.x - image_center.x;
  float dy = bounding_box_center.y - image_center.y;

  // Hypotenuse of the rectangular triangle formed by the line from the focal point to the image center, with an
  // opposite side defined by the bounding box center
  float hyp_x = sqrt(pow(dx, 2) + pow(cam_model.fx(), 2));
  float hyp_y = sqrt(pow(dy, 2) + pow(cam_model.fy(), 2));

  /* The rotation only occurs in 2D and affects the other image axis
   * The rotation needs to be scaled
   */
  std::cout << "Before Eigen" << std::endl;
  camera_rotation[0] = asin(dy / hyp_y) * 180.0f / M_PI;
  camera_rotation[1] = asin(dx / hyp_x) * 180.0f / M_PI;
  camera_rotation[2] = 0.0f;
}
