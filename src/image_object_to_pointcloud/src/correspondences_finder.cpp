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

const float NEAR_PLANE_DISTANCE = 1.0f;
const float FAR_PLANE_DISTANCE = 40.0f;
const double PIXEL_SIZE = 3.45e-6;

const unsigned int SYNC_POLICY_QUEUE_SIZE = 20;  // queue size for syncPolicyForCallback;

const std::string LIDAR_TF2_REFERENCE_FRAME = "velo_link";
const std::string CAMERA_TF2_REFERENCE_FRAME = "camera_color_left";

geometry_msgs::TransformStamped transformStamped;

ros::Publisher pub, pub_voxelized, pub_bboxes, pub_bboxes_poses;  //!< ROS Publisher

void callback(const darknet_ros_msgs::ObjectCountConstPtr& object_count,
              const darknet_ros_msgs::BoundingBoxesConstPtr& b_boxes, const sensor_msgs::CameraInfoConstPtr& cam_info,
              const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
  PointCloudType point_cloud_camera, point_cloud;
  PointCloudType::Ptr cloudCameraPtr(new PointCloudType);
  PointCloudType::Ptr cloudPtr(new PointCloudType);
  PointCloudType::Ptr cloudFilteredPtr(new PointCloudType);

  sensor_msgs::PointCloud2 point_cloud_camera_msg;

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
  *cloudCameraPtr = point_cloud_camera;
  *cloudPtr = point_cloud;

  image_geometry::PinholeCameraModel cam_model_;
  cam_model_.fromCameraInfo(cam_info);
  cv::Size im_dimensions = cam_model_.fullResolution();

  std::vector<int> indices_in;
  boost::shared_ptr<std::vector<int>> index_ptr = boost::make_shared<std::vector<int>>(indices_in);
  geometry_msgs::PoseArray bboxes_poses;

  for (int i = 0; i < cloudCameraPtr->points.size(); i++)
  {
    cv::Point2d uv = cam_model_.project3dToPixel(
        cv::Point3d(cloudCameraPtr->points[i].x, cloudCameraPtr->points[i].y, cloudCameraPtr->points[i].z));

    // Note that Kitti width and height are swapped so we swap them here also
    if (((int)(uv.x) >= 0) && ((int)uv.y >= 0) && ((int)(uv.x) <= im_dimensions.height) &&
        ((int)uv.y <= im_dimensions.width) && (cloudCameraPtr->points[i].z >= NEAR_PLANE_DISTANCE) &&
        (cloudCameraPtr->points[i].z <= FAR_PLANE_DISTANCE))
    {
      for (int j = 0; j < object_count->count; ++j)
      {
        if (((int)(uv.x) >= b_boxes->bounding_boxes[j].xmin) && ((int)uv.y >= b_boxes->bounding_boxes[j].ymin) &&
            ((int)(uv.x) <= b_boxes->bounding_boxes[j].xmax) && ((int)uv.y <= b_boxes->bounding_boxes[j].ymax))
        {
          index_ptr->push_back(i);
        }
      }
    }
  }

  pcl::ExtractIndices<PointType> point_cloud_idx_filter(true);  // Initializing with true will allow us to extract the
                                                                // removed indices
  point_cloud_idx_filter.setInputCloud(cloudPtr);
  point_cloud_idx_filter.setIndices(index_ptr);
  point_cloud_idx_filter.filter(*cloudFilteredPtr);

  ROS_INFO_STREAM("Numbered of points in filtered cloud: " << index_ptr->size());

  cloudFilteredPtr->header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
  pub.publish(cloudFilteredPtr);

  pcl::VoxelGrid<PointType> vg;
  PointCloudType::Ptr cloud_filtered(new PointCloudType);
  vg.setInputCloud(cloudFilteredPtr);
  vg.setLeafSize(0.04f, 0.04f, 0.04f);
  vg.filter(*cloud_filtered);
  std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size() << " data points." << std::endl;

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
  tree->setInputCloud(cloud_filtered);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance(0.25);  // L2 Euclidean Norm distance in meters
  ec.setMinClusterSize(300);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_filtered);
  ec.extract(cluster_indices);

  int j = 0;
  PointCloudType final_cloud;
  jsk_recognition_msgs::BoundingBoxArray rviz_bboxes;
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
  {
    PointCloudType::Ptr cloud_cluster(new PointCloudType);

    for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
    {
      cloud_cluster->points.push_back(cloud_filtered->points[*pit]);
    }
    cloud_cluster->width = cloud_cluster->points.size();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster " << j << ": " << cloud_cluster->points.size() << " data points."
              << std::endl;
    final_cloud += *cloud_cluster;

    pcl::PCA<PointType> pca;
    pca.setInputCloud(cloud_cluster);

    // Rotation of PCA
    Eigen::Matrix3f rot_mat = pca.getEigenVectors();
    Eigen::Quaterniond rot_quat(rot_mat.cast<double>());
    geometry_msgs::Quaternion temp_quat = Eigen::toMsg(rot_quat);

    // translation of PCA
    Eigen::Vector3f cl_translation = pca.getMean().head(3);
    geometry_msgs::Point temp_vec;
    temp_vec.x = (double)(cl_translation.x());
    temp_vec.y = (double)(cl_translation.y());
    temp_vec.z = (double)(cl_translation.z());

    // Reordering of principal components
    Eigen::Matrix3f affine_trans;
    affine_trans.col(0) << (rot_mat.col(0).cross(rot_mat.col(1))).normalized();
    affine_trans.col(1) << rot_mat.col(0);
    affine_trans.col(2) << rot_mat.col(1);

    rot_quat = Eigen::Quaterniond(affine_trans.cast<double>());
    temp_quat = Eigen::toMsg(rot_quat);

    PointType min_pt, max_pt;
    pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

    geometry_msgs::Point mid_pt;
    mid_pt.x = (max_pt.x + min_pt.x) / 2;
    mid_pt.y = (max_pt.y + min_pt.y) / 2;
    mid_pt.z = (max_pt.z + min_pt.z) / 2;

    geometry_msgs::Quaternion pose_orientation;
    pose_orientation.w = 1;

    jsk_recognition_msgs::BoundingBox temp_bbox;
    geometry_msgs::Pose temp_cluster_pose;
    temp_cluster_pose.position = mid_pt;
    temp_cluster_pose.orientation = pose_orientation;
    temp_bbox.pose = temp_cluster_pose;

    geometry_msgs::Vector3 bbox_dimensions;
    bbox_dimensions.x = max_pt.x - min_pt.x;
    bbox_dimensions.y = max_pt.y - min_pt.y;
    bbox_dimensions.z = max_pt.z - min_pt.z;
    temp_bbox.dimensions = bbox_dimensions;

    temp_bbox.header.stamp = ros::Time::now();
    temp_bbox.header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
    temp_bbox.value = 0.0f;
    temp_bbox.label = 0;

    rviz_bboxes.boxes.push_back(temp_bbox);

    // copy coordinates of mid_point and replace it with the leading dimension. The if cascade prioretizes x, y and z
    // dimensions, on this order
    geometry_msgs::Pose bboxes_pose_arrow = temp_cluster_pose;
    if (bbox_dimensions.x >= bbox_dimensions.y)
    {
      bboxes_pose_arrow.position.x = max_pt.x;
    }
    else if (bbox_dimensions.y >= bbox_dimensions.z)
    {
      bboxes_pose_arrow.position.y = max_pt.y;
      geometry_msgs::Quaternion yy_rotation;
      yy_rotation.z = 0.7071068;
      yy_rotation.w = 0.7071068;

      bboxes_pose_arrow.orientation = yy_rotation;
    }
    else
    {
      bboxes_pose_arrow.position.z = max_pt.z;
      geometry_msgs::Quaternion zz_rotation;
      zz_rotation.y = -0.7071068;
      zz_rotation.w = 0.7071068;

      bboxes_pose_arrow.orientation = zz_rotation;
    }

    bboxes_poses.poses.push_back(bboxes_pose_arrow);

    j++;
  }

  final_cloud.header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
  pub_voxelized.publish(final_cloud);

  rviz_bboxes.header.stamp = ros::Time::now();
  rviz_bboxes.header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
  pub_bboxes.publish(rviz_bboxes);

  bboxes_poses.header.stamp = ros::Time::now();
  bboxes_poses.header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
  pub_bboxes_poses.publish(bboxes_poses);
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
  pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1);
  pub_voxelized = nh.advertise<sensor_msgs::PointCloud2>("voxelized_point_cloud", 1);
  pub_bboxes = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("bboxes", 1);
  pub_bboxes_poses = nh.advertise<geometry_msgs::PoseArray>("bboxes_poses", 1);

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

  ros::Rate r(100);  // 100 hz
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return EXIT_SUCCESS;
}
