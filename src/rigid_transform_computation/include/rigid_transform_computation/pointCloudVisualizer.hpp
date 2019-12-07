/*!
 * \file   pointCloudVisualizer.hpp
 * \brief  Header file for Point Cloud Visualizer Class
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

#ifndef POINT_CLOUD_VISUALIZER_H
#define POINT_CLOUD_VISUALIZER_H

#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_ros/point_cloud.h>

/*!
 * \namespace point_cloud
 * \brief Namespace for point cloud related operations
 *
 * It defines variables, typedefs, methods, classes and structures that are usefull for point cloud manipulation
 */
namespace point_cloud
{
typedef pcl::PointXYZI PointI;      //!< PCL point with Intensity
typedef pcl::PointXYZRGB PointRGB;  //!< PCL point with Color

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;        //!< Point Cloud with only Euclidean Coordinates
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;      //!< Point Cloud with Intensity
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;  //!< Point Cloud with Color

/*!
 * \class PointCloudVisualizer
 * \brief PCL Visualier with ROS integration and added functionalities for correspondences selection
 *
 * Class that uses an PCL visualizer to show Point Cloud 2 message.
 * It is integrated with ROS but is detach from ROS simulated time for callback operations, supporting freeze time
 */
class PointCloudVisualizer
{
public:
  /*!
   * \brief Construct that creates an PCL window to visualize the Point Cloud feed to the topics given
   *
   * \param[in] point_cloud_topic string containing the Point Cloud topic name to be visualized
   * \param[in] node_handler_name the ROS node handler name to be created
   */
  PointCloudVisualizer(std::string point_cloud_topic, std::string node_handler_name);

  /*!
   * \brief Constructor that creates an PCL window to visualize the Point Cloud feed to the topics given
   *
   * \param[in] point_cloud_topic string containing the Point Cloud topic name to be visualized
   * \param[in] pose_topic string containing the topic name for the viewers pose to be published
   * \param[in] node_handler_name the ROS node handler name to be created
   */
  PointCloudVisualizer(std::string point_cloud_topic, std::string pose_topic, std::string node_handler_name);

  /*!
   * \brief Destructor
   *
   * Call PCLCloudVisualizer close method and deallocates dynamic memory
   */
  ~PointCloudVisualizer();

  /*!
   * \brief Allows the selection of points in the visualizer
   *
   * Enables the callback associated to mouse clicks on the PCL_VISUALIZER
   */
  void registerPointPickingCallback(const uint8_t MODE);

  /*!
   * \brief Spins PCL Visualizer and ROS node
   */
  void spin();

  /*!
   * \brief returns the current PCL Visualizer viewer Pose
   *
   * \return The tridimensional affine transform that contains the Pose of the viewer regarding th point cloud
   * coordinate frame
   */
  Eigen::Affine3f getViewerPose();

  static const uint8_t SINGLE_POINT_MODE = 0;  //!< Mode for a single point to be selected, on a Mouse event

private:
  /*!
   * \brief Initializes point Cloud Visualizer with pre defined parameters
   */
  void initPointCloudVisualizer();

  /*!
   * \brief Adds a Point Cloud to an already exisiting point cloud on the visualizer
   * \param[in] point_cloud_msg ROS Point Cloud 2 message object
   * \param[in] point_cloud_name Point Cloud Object name
   */
  void addNewPointCloudToVisualizer(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg,
                                    std::string point_cloud_name);

  /*!
   * \brief Adds a Point Cloud to a already exisiting point cloud on the visualizer
   * \param[in] pickingEvent PointPickingEvent object
   * \param[in] viewerVoidPtr void pointer for user data. Used to pass a pointer for the ros::Publisher
   *
   * Gets the selected point coordinates in relation to the point cloud coordinate frame that
   *
   * \remark: See http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_picking_event.html
   */
  static void onPointPickingEvent(const pcl::visualization::PointPickingEvent& pickingEvent, void* viewerVoidPtr);

  /*!
   * \brief Callback for every PCL Point Image message
   *
   * \param[in] point_cloud_msg Point Cloud Message
   *
   * Updates the Point Cloud on the visualizer and computes the Pose
   */
  void viewerCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);

  /*!
   * \brief Callback for every PCL Point Image message
   *
   * \param[in] point_cloud_msg Point Cloud Message
   *
   * Updates the Point Cloud on the visualizer, computes the Pose and publishes the pose using pose_pub
   */
  void viewerWithPoseCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);

  /*!
   * \brief Sets the pose for a PCL Viewer
   *
   * \param[in] viewer Point Cloud viewer
   * \param[in] viewer_pose 3D Pose to be set
   */
  void setViewerPose(pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose);

  std::string node_handler_name;  //!< Node Handler Name
  std::string point_cloud_topic;  //!< Point Cloud Topic
  std::string pose_topic;         //!< Pose Topic

  ros::NodeHandlePtr nh_;           //!< ROS Node Handler Object
  ros::Subscriber point_cloud_sub;  //!< ROS Point Cloud Subscriber
  ros::Publisher pose_pub;          //!< ROS Pose Publisher
  ros::Publisher point_pub;         //!< ROS selected Point Publisher (PointXYZI message)

  Eigen::Affine3f viewerPose;  //!< Point Cloud Visualizer Viewer Pose

  PointCloud point_cloud;    //!< Point Cloud object
  PointCloud::Ptr cloudPtr;  //!< Point Cloud Pointer

  pcl::visualization::PCLVisualizer::Ptr pcl_viewer;  //!< PCL Visualizer Object
};

}  // namespace point_cloud

#endif  // POINT_CLOUD_VISUALIZER_H
