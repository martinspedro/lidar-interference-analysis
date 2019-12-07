/*!
 * \file   rangeImageVisualizer.hpp
 * \brief  header file for RangeImageVisualizer class
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

#ifndef RANGE_IMAGE_VISUALIZER_H
#define RANGE_IMAGE_VISUALIZER_H

#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>
#include "rigid_transform_computation/pointCloudVisualizer.hpp"
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Pose.h>

/*!
 * \class RangeImageVisualizer
 * \brief PCL Rnage Visualier with ROS integration
 *
 */
class RangeImageVisualizer
{
public:
  /*!
   * \brief Constructor that creates an PCL Rnage Visualier Window to visualize Rnage Images generated from the point
   * cloud
   *
   * \param[in] point_cloud_topic string containing the Point Cloud topic name to be visualized
   * \param[in] node_handler_name the ROS node handler name to be created
   */
  RangeImageVisualizer(std::string point_cloud_topic, std::string node_handler_name);

  /*!
   * \brief Constructor that creates an PCL Rnage Visualier Window to visualize Rnage Images generated from the point
   * cloud
   *
   * \param[in] point_cloud_topic string containing the Point Cloud topic name to be visualized
   * \param[in] viewer_pose_topic string contains the topic for the viewers pose to be published
   * \param[in] node_handler_name the ROS node handler name to be created
   */
  RangeImageVisualizer(std::string point_cloud_topic, std::string viewer_pose_topic, std::string node_handler_name);

  /*!
   * \brief Destructor
   *
   * Deallocates dynamic memory
   */
  ~RangeImageVisualizer();

  // void attachPoseTrigger(point_cloud::PointCloudVisualizer pcl_viewer);

private:
  /*!
   * \brief Generates range imag for the point cloud and updates viewer
   * \param[in] point_cloud_msg ROS Point Cloud 2 Mesage
   *
   * The angular steps are pre-defined using constant values and the origin referential used is the LiDAR
   * The point cloud is converted to a RangeImage using this parameters and the point cloud viewer pose
   *
   * \remark See documentation on
   http://pointclouds.org/documentation/tutorials/range_image_creation.php#range-image-creation
   */
  void rangeImageCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);

  /*!
   * \brief ROS callback for pose messages
   * \param[in] viewer_pose_msg ROS Pose message
   *
   * This callback is triggered when the PointCloudVisualizer viewer object is enabled to publish its pose and there is
   * a pose change by the user when mainpulating the visualization
   * This callback updates the internal pose of the Range Visualizer, which uses this new pose when it computes the new
   * range image from a new point cloud message
   */
  void viewerPoseCallback(const geometry_msgs::Pose::ConstPtr& viewer_pose_msg);

  std::string node_handler_name;  //!< Node Handler Name
  std::string point_cloud_topic;  //!< Point Cloud Topic
  std::string viewer_pose_topic;  //!< Viewer Pose Topic

  ros::NodeHandlePtr nh_;           //!< ROS Node Handler Object
  ros::Subscriber point_cloud_sub;  //!< ROS Point Cloud Subscriber
  ros::Subscriber pose_sub;         //!< ROS Pose Publisher
  ros::Publisher range_image_pub;   //!< Range Image ROS Publisher

  point_cloud::PointCloud point_cloud;    //!< Point Cloud object
  point_cloud::PointCloud::Ptr cloudPtr;  //!< Point Cloud Pointer

  pcl::visualization::RangeImageVisualizer::Ptr range_image_widget;  //!< PCL Visualizer Range Image Object

  Eigen::Affine3f sensorPose;  //!< Current Pose of th Range Viewer
};

#endif  // RANGE_IMAGE_VISUALIZER_H
