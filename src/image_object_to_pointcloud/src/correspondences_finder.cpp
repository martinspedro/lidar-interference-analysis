/**
 * @file   correspondences.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 */

#include <ros/ros.h>

#include <iostream>
#include <string>

// OPENCV
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>

#include <geometry_msgs/TransformStamped.h>

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

const float NEAR_PLANE_DISTANCE = 1.0f;
const float FAR_PLANE_DISTANCE = 30.0f;
const double PIXEL_SIZE = 3.45e-6;

const unsigned int SYNC_POLICY_QUEUE_SIZE = 20;  // queue size for syncPolicyForCallback;

const std::string LIDAR_TF2_REFERENCE_FRAME = "velo_link";
const std::string CAMERA_TF2_REFERENCE_FRAME = "camera_color_left";

geometry_msgs::TransformStamped transformStamped;

ros::Publisher pub;  //!< ROS Publisher

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

  for (int i = 0; i < cloudCameraPtr->points.size(); i++)
  {
    cv::Point2d uv = cam_model_.project3dToPixel(
        cv::Point3d(cloudCameraPtr->points[i].x, cloudCameraPtr->points[i].y, cloudCameraPtr->points[i].z));

    // Note that Kitti width and height are swapped so we swap them here also
    if (((int)(uv.x) >= 0) && ((int)uv.y >= 0) && ((int)(uv.x) <= im_dimensions.height) &&
        ((int)uv.y <= im_dimensions.width) && (cloudCameraPtr->points[i].z >= 0))
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
