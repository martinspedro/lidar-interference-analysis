/*!
 * \file   image_object_to_pointcloud.hpp
 * \brief  Header file of the utilities to compute the bounding box on the point cloud
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

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
#include "jsk_recognition_msgs/BoundingBox.h"

// Define types to be used depending on the dataset
//#define USE_WITH_KITTI

/*!
 * \brief Define if the code is to be runt KITTI or our Experimental Dataset
 *
 * If KITTI, the Point Clouds are defined to be point_cloud::PointCloudXYZ and the points pcl::PointXYZ, with only
 * Euclidean Coordinates
 *
 * If our experimental dataset is to be used, then the Point Clouds used are velodyne::VelodynePointCloud and the points
 * are velodyne::PointXYZIR containing Intensity and Ring Measurements, besides XYZ
 */
#ifdef USE_WITH_KITTI
typedef point_cloud::PointCloudXYZ PointCloudType;
typedef pcl::PointXYZ PointType;
#else
typedef velodyne::VelodynePointCloud PointCloudType;
typedef velodyne::PointXYZIR PointType;
#endif

extern const Eigen::Matrix4f LIDAR_POSE_TO_FRUSTRUM_POSE;  //!< Affine Transformation to LiDAR to Frustum

/*!
 * \brief Check if a pixel is inside the bounding box
 * \param[in] pixel Integer coordinates of the pixel on the image using a cv::Point2i object
 * \param[in] bounding_box bounding box object from darknet_ros package
 * \retval True if pixel index is inside the image bounding box
 * \retval False otherwise
 */
inline bool pixelInsideBoundingBox(cv::Point2i pixel, darknet_ros_msgs::BoundingBox bounding_box)
{
  return (pixel.x >= bounding_box.xmin) && (pixel.y >= bounding_box.ymin) && (pixel.x <= bounding_box.xmax) &&
         (pixel.y <= bounding_box.ymax);
}

/*!
 * \brief Convert a Point Cloud Point to an Opencv Point3D
 * \param[in] pcl_point PointType to be converted
 * \return cv::Point3d object containing the same point defined in OpenCv
 */
inline cv::Point3d PCLToOpenCvPoint(PointType pcl_point)
{
  return cv::Point3d(pcl_point.x, pcl_point.y, pcl_point.z);
}

/*!
 * \brief Convert a Point Cloud Point to an Eigen Vector
 * \param[in] pcl_point XYZ Point Cloud to be converted
 * \return Eigen::Vector3f containing the point coordinates
 */
inline Eigen::Vector3f PCLPointXYZToEigenVector3f(pcl::PointXYZ pcl_point)
{
  Eigen::Vector3f temp_vec;
  temp_vec.x() = pcl_point.x;
  temp_vec.y() = pcl_point.y;
  temp_vec.z() = pcl_point.z;
  return temp_vec;
}

/*!
 * \brief Convert a Point Cloud Point to a ROS geometry messages vector
 * \param[in] pcl_point PointType to be converted
 * \return geometry_msgs::Vector3 containing the point coordinates
 */
inline geometry_msgs::Vector3 PCLPointXYZToGeometryMsgsVector(PointType pcl_point)
{
  geometry_msgs::Vector3 temp_vec;
  temp_vec.x = (double)(pcl_point.x);
  temp_vec.y = (double)(pcl_point.y);
  temp_vec.z = (double)(pcl_point.z);
  return temp_vec;
}

/*!
 * \brief Convert a Point Cloud Point to a ROS geometry messages point
 * \param[in] pcl_point PointType to be converted
 * \return geometry_msgs::Point containing the point coordinates
 */
inline geometry_msgs::Point PCLPointXYZToGeometryMsgsPoint(PointType pcl_point)
{
  geometry_msgs::Point temp_point;
  temp_point.x = (double)(pcl_point.x);
  temp_point.y = (double)(pcl_point.y);
  temp_point.z = (double)(pcl_point.z);
  return temp_point;
}

/*!
 * \brief Convert an Eigen Vector to a ROS geometry messages point
 * \param[in] vector_eigen tridimensional Vector from Eigen
 * \return geometry_msgs::Point containing the vector coordinates
 */
inline geometry_msgs::Point eigenVectorToGeometryMsgsPoint(Eigen::Vector3f vector_eigen)
{
  geometry_msgs::Point temp_point;
  temp_point.x = (double)(vector_eigen.x());
  temp_point.y = (double)(vector_eigen.y());
  temp_point.z = (double)(vector_eigen.z());
  return temp_point;
}

/*!
 * \brief Get the indexes of the points that are contained in the camera ROIs
 * \param[in] b_boxes_object_count number of bounding box objects detected on the image
 * \param[in] b_boxes bounding box array from darknet_ros package containing all the bounding boxes detected
 * \param[in] camera_point_cloud_ptr Point Cloud in the camera coordinate frame
 * \param[in] camera_model Pinhole Camera Model object from ROS image_geometry package
 * \param[in] image_dimensions Dimensions of the image using cv::Size
 * \param[in] near_plane_distance Cutoff near distance for correspondences establishing
 * \param[in] far_plane_distance Cutoff far distance for correspondences establishing
 * \param[out] index_shr_ptr boost shared pointer to a vector of indexes for store the point cloud poimnts insde the
 * bounding box
 *

 * This method iterates over all the points in the point cloud and projects them to the corresponding pixels on the
 image plane. If the pixels is inside the image dimensions and the distance cutoff planes, it is then compared with all
 the bounding boxes. If the projected 3D point corresponds to a detected object (hence inside its bounding boxes
 dimension), its index is stored on a vector
 *
 * \pre The point Cloud camera_point_cloud_ptr must be already defined on the camera coordinate frame
 * \remark The function is inline due to speed requirements \todo Allow the selection of which ROIs/ROI IDs
 * are to be used
 * \todo Implement multithreading
 * \todo Break cycle if one coorespendence is found
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

/*!
 * \brief Extract the rotation around the Z-axis from a generic rotation matrix
 * \param[in] original_rotation_matrix Generic Rotation Matrix
 * \return The Z-axis rotation matrix from the original_rotation_matrix
 */
Eigen::Matrix3d degenerateRotationMatrixToZ(Eigen::Matrix3d original_rotation_matrix);

/*!
 * \brief Compute a rotation matrix using the Eigen Vectors of an object coordinate axis
 * \param[in] eigen_vectors Eigen vector defining the axis on the objects coordinate frame
 * \return The rotation matrix
 *
 * \see https://en.wikipedia.org/wiki/Principal_component_analysis
 * \warning Seems to be correct, but needs more testing
 */
Eigen::Matrix3d computeRotationMatrixFromEigenVectors(const Eigen::Matrix3d eigen_vectors);

/*!
 * \brief Computes the bounding box Pose using Principal Component Analysis
 * \param[in] point_cloud_cluster_ptr Point Cloud cluster that is the Bounding Box source data
 * \param[in] lockRotationToZAxis If true, degenerates the Rotation to be only on Z-Axis, using
 * degenerateRotationMatrixToZ
 * \param[out] bounding_box_pose the vector of poses for each bounding box
 *
 * \see https://en.wikipedia.org/wiki/Principal_component_analysis
 */
void computeBoundingBoxPosePCA(const PointCloudType::ConstPtr point_cloud_cluster_ptr, bool lockRotationToZAxis,
                               geometry_msgs::Pose::Ptr& bounding_box_pose);

/*!
 * \brief Compute Camera Affine Transform from Angle Axis vector
 * \param[in] angle_axis_rotation Angle Axis Vector in order [X, Y, Z]
 * \return Rotation Matrix in an Affine Transform Matrix, so dimensions are [4 x 4]
 *
 * \remark Conversions and transforms verified using https://www.andre-gaschler.com/rotationconverter/
 */
Eigen::Matrix4f angleAxisVecToTransform(const Eigen::Vector3f angle_axis_rotation);

/*!
 * \brief Computes the LiDAR Pose
 * \param[in] point_cloud_ptr Point cloud defined on the LiDAR coordinate frame
 * \param[out] lidar_pose the LiDAR pose, defined using a joint rotation and translation matrix
 */
void getLiDARPose(const PointCloudType::ConstPtr point_cloud_ptr, Eigen::Matrix4f& lidar_pose);

/*!
 * \brief Computes the Camera rotation for the given bounding box
 * \param[in] cam_model_ Pinhole Camera Model object from ROS image_geometry package
 * \param[in] bounding_box darknet bounding box object
 * \param[out] rotation_bbox Affine Transform to rotate the Camera Coordinate frame to the bounding box coordinate frame
 */
void getCameraRotation(image_geometry::PinholeCameraModel cam_model_, darknet_ros_msgs::BoundingBox bounding_box,
                       Eigen::Matrix4f& rotation_bbox);

/*!
 * \brief Computes the Cluster bounding box parameters
 * \param[in] point_cloud_cluster Point Cloud cluster that is the Bounding Box source data
 * \param[out] rviz_bbox_ptr Bounding box object to be visualized in RViz with jsk_recognition_msgs plugin
 *
 + Uses pcl::getMinMax3D to obtain the limits of the bounding box and computes all of its parameters using geometry and
 trignometry proporties. The determined bounding box is an Axis Aligned Bounding Box (AABB)
 */
void computeClusterBoundingBox(const PointCloudType::ConstPtr point_cloud_cluster,
                               jsk_recognition_msgs::BoundingBox::Ptr& rviz_bbox_ptr);

/*!
 * \brief Computes the given bounding box pose
 * \param[in] rviz_bbox_ptr Bounding box object to be visualized in RViz with jsk_recognition_msgs plugin
 * \param[out] bounding_box_pose the vector of poses for each bounding box
 */
void computeClusterBoundingBoxPose(jsk_recognition_msgs::BoundingBox::Ptr& rviz_bbox_ptr,
                                   geometry_msgs::Pose::Ptr& bounding_box_pose);

/*!
 * \brief Computes the Cluster bounding box parameters
 * \param[in] point_cloud_cluster_ptr Point Cloud cluster that is the Bounding Box source data
 * \param[out] rviz_bbox_ptr Bounding box object to be visualized in RViz with jsk_recognition_msgs plugin
 *
 * Uses pcl::MomentOfInertiaEstimation to determine the Object Bounding Box (OBB)
 */
void computeOBB(const PointCloudType::ConstPtr point_cloud_cluster_ptr,
                jsk_recognition_msgs::BoundingBox::Ptr& rviz_bbox_ptr);

/*!
 * \brief Computes the Cluster bounding box parameters
 * \param[in] point_cloud_cluster_ptr Point Cloud cluster that is the Bounding Box source data
 * \param[out] rviz_bbox_ptr Bounding box object to be visualized in RViz with jsk_recognition_msgs plugin
 *
 * Uses pcl::MomentOfInertiaEstimation to determine the  Axis Aligned Bounding Box (AABB)
 */
void computeAABB(const PointCloudType::ConstPtr point_cloud_cluster_ptr,
                 jsk_recognition_msgs::BoundingBox::Ptr& rviz_bbox_ptr);

#endif  // IMAGE_OBJECT_TO_POINT_CLOUD_H
