/**
 * @file   correspondences.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 */
#define PCL_NO_PRECOMPILE  // must be included before any PCL include on this CPP file or HPP included before

#include <ros/ros.h>

#include <iostream>
#include <string>

/* The next two headers MUST BE INCLUDED BEFORE OPENCV headers.
 * OpenCv leaks a macro #define USE_UNORDERED_MAP that is picked up by FLANN library (included by KDTree from PCL)
 * This causes FLANN to not define a couple of stuff that causes KDTree to not compile
 *
 * See the following links:
 * https://stackoverflow.com/questions/42504592/flann-util-serialization-h-class-stdunordered-mapunsigned-int-stdvectorun
 * https://github.com/mariusmuja/flann/issues/214
 */
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
// OPENCV
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

// PCL
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseArray.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>

#include <image_geometry/pinhole_camera_model.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>

#include "image_object_to_pointcloud/image_object_to_pointcloud.hpp"
#include "jsk_recognition_msgs/BoundingBox.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <dynamic_reconfigure/server.h>
#include "image_object_to_pointcloud/euclidian_cluster_filterConfig.h"

// If defined, the node will publish a PoseArray containing the BBox major direction
// #define PUBLISH_BBOX_POSES

const float NEAR_PLANE_DISTANCE = 1.0f;
const float FAR_PLANE_DISTANCE = 40.0f;

const double PIXEL_SIZE = 3.45e-6;

const unsigned int SYNC_POLICY_QUEUE_SIZE = 20;  // queue size for syncPolicyForCallback;

const std::string LIDAR_TF2_REFERENCE_FRAME = "velo_link";
const std::string CAMERA_TF2_REFERENCE_FRAME = "camera_color_left";

const bool EXTRACT_INDEX_FROM_POINT_CLOUD = true;
const bool REMOVE_INDEX_FROM_POINT_CLOUD = false;

const float CLUSTER_L2_EUCLIDEAN_NORM_TOLERANCE = 0.10;  // in meters
const unsigned int MIN_CLUSTER_SIZE = 100;
const unsigned int MAX_CLUSTER_SIZE = 25000;

float dyn_rec_cluster_tolerance;        // = CLUSTER_L2_EUCLIDEAN_NORM_TOLERANCE;
unsigned int dyn_rec_min_cluster_size;  // = MIN_CLUSTER_SIZE;
unsigned int dyn_rec_max_cluster_size;  // = MAX_CLUSTER_SIZE;

const float VOXEL_GRID_LEAF_SIZE = 0.04f;  //!< Voxel length used for the Voxel Grid filter

geometry_msgs::TransformStamped transformStamped;

ros::Publisher pub, pub_voxelized, pub_bboxes;  //!< ROS Publisher

#ifdef PUBLISH_BBOX_POSES
ros::Publisher pub_bboxes_poses;
#endif

void callback(const darknet_ros_msgs::ObjectCountConstPtr& object_count,
              const darknet_ros_msgs::BoundingBoxesConstPtr& b_boxes, const sensor_msgs::CameraInfoConstPtr& cam_info,
              const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
  PointCloudType point_cloud_camera, point_cloud;
  PointCloudType::Ptr point_cloud_camera_ptr(new PointCloudType);
  PointCloudType::Ptr point_cloud_ptr(new PointCloudType);
  PointCloudType::Ptr point_cloud_b_boxes_ptr(new PointCloudType);
  PointCloudType::Ptr point_cloud_voxelized(new PointCloudType);

  PointCloudType point_cloud_all_clusters;

  sensor_msgs::PointCloud2 point_cloud_camera_msg;  // LiDAR Point cloud message in camera coordinate frame

  try
  {
    tf2::doTransform(*point_cloud_msg, point_cloud_camera_msg, transformStamped);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
  }

  // Convert ROS msg to Point Cloud
  fromROSMsg(point_cloud_camera_msg, point_cloud_camera);
  fromROSMsg(*point_cloud_msg, point_cloud);

  // Initialize pointer to point cloud data
  *point_cloud_camera_ptr = point_cloud_camera;
  *point_cloud_ptr = point_cloud;

  // Camera Information from Message and get image dimensions
  image_geometry::PinholeCameraModel cam_model_;
  cam_model_.fromCameraInfo(cam_info);
  cv::Size im_dimensions = cam_model_.fullResolution();

  // vector and shared pointer to store the point cloud points corresponding to the image bounding boxes
  std::vector<int> indices_in;
  boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indices_in);

#ifdef PUBLISH_BBOX_POSES
  geometry_msgs::PoseArray bboxes_poses;
#endif

  // Note that Kitti inverts the image dimensions, therefore we invert them when creating a new cv::Size object
  filterPointCloudFromCameraROIs(object_count->count, b_boxes, point_cloud_camera_ptr, cam_model_,
                                 cv::Size(im_dimensions.height, im_dimensions.height), NEAR_PLANE_DISTANCE,
                                 FAR_PLANE_DISTANCE, index_ptr);

  // EXTRACT_INDEX_FROM_POINT_CLOUD sets the filter to output a point cloud with the points of the given indexes
  pcl::ExtractIndices<PointType> point_cloud_idx_filter(EXTRACT_INDEX_FROM_POINT_CLOUD);
  point_cloud_idx_filter.setInputCloud(point_cloud_ptr);
  point_cloud_idx_filter.setIndices(index_ptr);
  point_cloud_idx_filter.filter(*point_cloud_b_boxes_ptr);

  ROS_INFO_STREAM("Image bounding boxes correspondent Point Cloud points number: " << index_ptr->size());

  // Reduce number of points in point cloud
  pcl::VoxelGrid<PointType> voxel_grid_xyz_filter;
  voxel_grid_xyz_filter.setInputCloud(point_cloud_b_boxes_ptr);
  voxel_grid_xyz_filter.setLeafSize(VOXEL_GRID_LEAF_SIZE, VOXEL_GRID_LEAF_SIZE, VOXEL_GRID_LEAF_SIZE);
  voxel_grid_xyz_filter.filter(*point_cloud_voxelized);
  ROS_INFO_STREAM("Voxelized Cloud Point: " << point_cloud_voxelized->points.size() << " @" << VOXEL_GRID_LEAF_SIZE
                                            << "m");

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointType>::Ptr kd_tree(new pcl::search::KdTree<PointType>);
  kd_tree->setInputCloud(point_cloud_voxelized);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> euclidian_cluster_extraction_filter;
  euclidian_cluster_extraction_filter.setClusterTolerance(dyn_rec_cluster_tolerance);
  euclidian_cluster_extraction_filter.setMinClusterSize(dyn_rec_min_cluster_size);
  euclidian_cluster_extraction_filter.setMaxClusterSize(dyn_rec_max_cluster_size);
  euclidian_cluster_extraction_filter.setSearchMethod(kd_tree);
  euclidian_cluster_extraction_filter.setInputCloud(point_cloud_voxelized);
  euclidian_cluster_extraction_filter.extract(cluster_indices);

  int j = 0;  // Cluster counter ID
  jsk_recognition_msgs::BoundingBoxArray rviz_bboxes;

  // Adapted from http://pointclouds.org/documentation/tutorials/cluster_extraction.php
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    PointCloudType::Ptr point_cloud_current_cluster(new PointCloudType);

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      point_cloud_current_cluster->points.push_back(point_cloud_voxelized->points[*pit]);
    }

    point_cloud_current_cluster->header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
    point_cloud_current_cluster->width = point_cloud_current_cluster->points.size();
    point_cloud_current_cluster->height = 1;
    point_cloud_current_cluster->is_dense = true;

    ROS_INFO_STREAM("Cluster #" << j << " point cloud size: " << point_cloud_current_cluster->points.size());

    // Store all the individual clusters into a point cloud object
    point_cloud_all_clusters += *point_cloud_current_cluster;

    // Compute Bounding Box for current cluster and add it to the vector of  bounding boxes detected on this frame
    jsk_recognition_msgs::BoundingBox::Ptr rviz_bbox_ptr(new jsk_recognition_msgs::BoundingBox);
    computeClusterBoundingBox(point_cloud_current_cluster, rviz_bbox_ptr);
    rviz_bboxes.boxes.push_back(*rviz_bbox_ptr);

#ifdef PUBLISH_BBOX_POSES
    // Compute the Bounding Box Pose for the current cluster bounding boxes and add it to the vector of bounding boxes
    // poses for the current LiDAR frame
    geometry_msgs::Pose::Ptr bboxes_pose_ptr(new geometry_msgs::Pose);
    computeClusterBoundingBoxPose(rviz_bbox_ptr, bboxes_pose_ptr);
    bboxes_poses.poses.push_back(*bboxes_pose_ptr);
#endif
    j++;  // increment bounding box counter
  }

  // Published Cloud with Points Corresponding to Image Bouding Boxes
  point_cloud_b_boxes_ptr->header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
  pub.publish(point_cloud_b_boxes_ptr);

  // Published Cloud with the Clusters for the Points Corresponding to Image Bouding Boxes
  point_cloud_all_clusters.header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
  pub_voxelized.publish(point_cloud_all_clusters);

  // Publish BoundingBoxes and respective Poses
  rviz_bboxes.header.stamp = ros::Time::now();
  rviz_bboxes.header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
  pub_bboxes.publish(rviz_bboxes);

#ifdef PUBLISH_BBOX_POSES
  bboxes_poses.header.stamp = ros::Time::now();
  bboxes_poses.header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
  pub_bboxes_poses.publish(bboxes_poses);
#endif
}

void euclidianClusterFilterReconfigureCallback(image_object_to_pointcloud::euclidian_cluster_filterConfig& config,
                                               uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f %f %f", config.L2_norm_tolerance, config.min_cluster_size, config.max_cluster_size);

  dyn_rec_cluster_tolerance = config.L2_norm_tolerance;
  dyn_rec_min_cluster_size = config.min_cluster_size;
  dyn_rec_max_cluster_size = config.max_cluster_size;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "correspondences_finder");

  std::cout << "Node init successfull" << std::endl;
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  /*
   * lookupTransform(target_frame, source_frame)
   * target_frame	The frame to which data should be transformed
   * source_frame	The frame where the data originated
   *
   * If one wants to transform the data from one referential to another, the target frame most be the final on which the
   * data will be processed.
   * The function construction might seem counter intuitive , but notice that the transform from the Referential Frame A
   * -> B allows the transformation of data from the frame B -> A
   *
   * On this case, since no data will be manipulated, onluy referentials, we want the transfrom from the Camera to the
   * LiDAR
   *
   * Another perspective is: we want to transform from <target_frame> from this <source_frame> frame
   */

  transformStamped = tfBuffer.lookupTransform(CAMERA_TF2_REFERENCE_FRAME, LIDAR_TF2_REFERENCE_FRAME, ros::Time(0),
                                              ros::Duration(20.0));

  // Publish filtered point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 5);
  pub_voxelized = nh.advertise<sensor_msgs::PointCloud2>("voxelized_point_cloud", 5);
  pub_bboxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("bboxes", 5);

#ifdef PUBLISH_BBOX_POSES
  pub_bboxes_poses = nh.advertise<geometry_msgs::PoseArray>("bboxes_poses", 5);
#endif
  // Subscribers list to be synchronized
  message_filters::Subscriber<darknet_ros_msgs::ObjectCount> object_count_sub(nh, "/darknet_ros/found_object", 2);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub(nh, "/darknet_ros/bounding_boxes", 2);
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub(nh, "/camera/camera_info", 20);
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub(nh, "/velodyne_points", 20);

  typedef message_filters::sync_policies::ApproximateTime<
      darknet_ros_msgs::ObjectCount, darknet_ros_msgs::BoundingBoxes, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2>
      syncPolicyForCallback;

  // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(20)
  message_filters::Synchronizer<syncPolicyForCallback> sync(syncPolicyForCallback(SYNC_POLICY_QUEUE_SIZE),
                                                            object_count_sub, bounding_boxes_sub, cam_info_sub,
                                                            point_cloud_sub);

  // Bind Callback to function to each of the subscribers
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  // Dynamic Reconfigure
  dynamic_reconfigure::Server<image_object_to_pointcloud::euclidian_cluster_filterConfig> server;
  dynamic_reconfigure::Server<image_object_to_pointcloud::euclidian_cluster_filterConfig>::CallbackType
      reconfigure_callback_function;

  // Bind Callback to function
  reconfigure_callback_function = boost::bind(&euclidianClusterFilterReconfigureCallback, _1, _2);
  server.setCallback(reconfigure_callback_function);

  ros::Rate r(100);  // 100 hz
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return EXIT_SUCCESS;
}
