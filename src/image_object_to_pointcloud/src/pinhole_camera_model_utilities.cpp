/*!
 * \file   pinhole_camera_model_utilities.cpp
 * \brief  General utilities header file for pinhole camera model operations
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

#include <cmath>
#include <ros/ros.h>

#include "image_object_to_pointcloud/pinhole_camera_model_utilities.hpp"

namespace pinhole_camera
{
FOV getImageFOV(image_geometry::PinholeCameraModel cam_model)
{
  cv::Size im_dimensions = cam_model.fullResolution();

  FOV im_fov;

  /* FOV is computed using a rectangular triangle, with half of the image dimensions
   * Therefore the result is equivalent to half of the Image field of view and requireds that in length is half of the
   * full image dimension along the desired dimensions
   */
  im_fov.x = 2.0f * getFovDegree(im_dimensions.width / 2.0f, (float)(cam_model.fx()));
  im_fov.y = 2.0f * getFovDegree(im_dimensions.height / 2.0f, (float)(cam_model.fy()));

  return im_fov;
}

FOV getImageFOV(cv::Size image_dimensions, cv::Matx33d intrinsic_matrix)
{
  FOV im_fov;

  /* FOV is computed using a rectangular triangle, with half of the image dimensions
   * Therefore the result is equivalent to half of the Image field of view and requireds that in length is half of the
   * full image dimension along the desired dimensions
   */
  im_fov.x = 2.0f * getFovDegree(image_dimensions.width / 2.0f, intrinsic_matrix(0, 0));
  im_fov.y = 2.0f * getFovDegree(image_dimensions.height / 2.0f, intrinsic_matrix(1, 1));

  return im_fov;
}

void computeBoundingBoxFOV(image_geometry::PinholeCameraModel cam_model, darknet_ros_msgs::BoundingBox bounding_box,
                           FOV* bounding_box_fov, Eigen::Vector3f& camera_rotation)
{
  cv::Size im_dimensions = cam_model.fullResolution();

  float x_min_fov = getFovDegree(bounding_box.xmin, (float)(cam_model.fx()));
  float x_max_fov = getFovDegree(bounding_box.xmax, (float)(cam_model.fx()));

  float y_min_fov = getFovDegree(bounding_box.ymin, (float)(cam_model.fy()));
  float y_max_fov = getFovDegree(bounding_box.ymax, (float)(cam_model.fy()));

  bounding_box_fov->x = x_max_fov - x_min_fov;
  bounding_box_fov->y = y_max_fov - y_min_fov;

  ROS_ASSERT_MSG(bounding_box_fov->x > 0, "FOV in width must be bigger than 0!");
  ROS_ASSERT_MSG(bounding_box_fov->y > 0, "FOV in height must be bigger than 0!");

  cv::Point2f image_center = getImageCenterPoint(cam_model.fullIntrinsicMatrix());

  // Get middle point of the bounding box
  cv::Point2f bounding_box_center;
  bounding_box_center.x = (bounding_box.xmin + bounding_box.xmax) / 2.0f;
  bounding_box_center.y = (bounding_box.ymin + bounding_box.ymax) / 2.0f;

  /* Distance in pixel between bounding box center and image center
   * Coordinates follow an almost Left-hand system (yy axis is inverted), which means the rotation is positive in the
   * clockwise direction, with the left hand rule applied on the fixed rotation axis
   *
   * If the bounding box center is further away from the matrix coordinates (0,0) in the (u, v) frame than the image
   * center (on the same referential - (u, v)), then the rotation is positive on that axis. This is mapped in a
   * negative offset distance On the x axis this results on a positive rotation if the the x coordinate of the
   * bounding box center is further away from origin that the image center, but on the y axis (due to its inverted
   * notation), the positive rotation occurs to values closer to matrix origina than the image center
   */

  const float XX_ROTATION_FACTOR = 1.0f;
  const float YY_ROTATION_FACTOR = -1.0f;

  float dx = bounding_box_center.x - image_center.x;
  float dy = bounding_box_center.y - image_center.y;

  // Hypotenuse of the rectangular triangle formed by the line from the focal point to the image center, with an
  // opposite side defined by the bounding box center
  float hyp_x = sqrt(pow(dx, 2) + pow((float)(cam_model.fx()), 2));
  float hyp_y = sqrt(pow(dy, 2) + pow((float)(cam_model.fy()), 2));

  /* The rotation only occurs in 2D and affects the other image axis
   * The rotation of the yy axis creates an angular offset between the x and x' coordinates of the bounding box
   * The rotation of the xx axis creates an angular offset between the y and y' coordinates of the bounding box
   *
   * Therefore, the angles of rotation muste be inverted
   */
  camera_rotation[0] = YY_ROTATION_FACTOR * asin(dy / hyp_y) * 180.0f / M_PI;
  camera_rotation[1] = XX_ROTATION_FACTOR * asin(dx / hyp_x) * 180.0f / M_PI;
  camera_rotation[2] = 0.0f;

  // Normalize the vector to be used by Rodrigues formula (which takes data Angle Axis vector divided by the vector
  // norm)
  float theta = camera_rotation.norm();
  camera_rotation[0] /= theta;
  camera_rotation[1] /= theta;
  camera_rotation[2] /= theta;

  // Convert from angle axis rotation (x, y) to (u,v)
  /*
  float u = (float)(cam_model.fx() * (double)(bounding_box_center.x) + cam_model.cx());
  float v = (float)(cam_model.fy() * (double)(bounding_box_center.y) + cam_model.cy());

  hyp_x = sqrt(pow(u, 2) + pow((float)(cam_model.fx()), 2));
  hyp_y = sqrt(pow(v, 2) + pow((float)(cam_model.fy()), 2));

  camera_rotation[0] = asin(v / hyp_y);  //* 180.0f / M_PI;
  camera_rotation[1] = asin(u / hyp_x);  //* 180.0f / M_PI;
  camera_rotation[2] = 0.0f;
  */
}

}  // namespace pinhole_camera
