/**
 * @file   correspondences.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 */

#define PCL_NO_PRECOMPILE  // must be included before any PCL include on this CPP file or HPP included before

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>

// OPENCV
//#include <opencv2/opencv.hpp>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>

//#include "sensor_fusion/color.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <std_msgs/Int8.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stdint.h>
#include <image_geometry/pinhole_camera_model.h>
#include <typeinfo>

#include <exception>
#include <list>

#include <pcl/filters/frustum_culling.h>

#include <point_cloud_statistics/velodyne_point_type.h>
#include "point_cloud_statistics/point_cloud_statistics.hpp"

const float NEAR_PLANE_DISTANCE = 1.0f;
const float FAR_PLANE_DISTANCE = 30.0f;

#define KITTI

ros::Publisher pub;  //!< ROS Publisher

void callback(const darknet_ros_msgs::ObjectCountConstPtr& object_count,
              const darknet_ros_msgs::BoundingBoxesConstPtr& b_boxes, const sensor_msgs::CameraInfoConstPtr& cam_info,
              const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
  std::cout << "Callback" << std::endl;
  /*
    for (int i = 0; i < object_count->count; ++i)
    {
       std::cout << b_boxes->bounding_boxes[i].Class << ": " << b_boxes->bounding_boxes[i].probability << std::endl;
    }
    */
  point_cloud::PointCloudXYZ point_cloud_velodyne, point_cloud, target;
  point_cloud::PointCloudXYZ::Ptr cloudPtr, point_cloud_ptr(new point_cloud::PointCloudXYZ);
  /*
#ifdef KITTI

#else
 velodyne::VelodynePointCloud point_cloud_velodyne, point_cloud, target;
 velodyne::VelodynePointCloud::Ptr cloudPtr, point_cloud_ptr(new velodyne::VelodynePointCloud);
#endif
*/
  std::cout << "Ok" << std::endl;
  /*
    sensor_msgs::PointCloud2 point_cloud_camera_msg;

    try
    {
      tf2::doTransform(*point_cloud_msg, point_cloud_camera_msg, transformStamped);
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
  */
  // Convert ROS msg to Point Cloud
  fromROSMsg(*point_cloud_msg, point_cloud);
  std::cout << "Converted" << std::endl;
  *point_cloud_ptr = point_cloud;
  std::cout << "Stored" << std::endl;
  /*
#ifdef KITTI

#else
  pcl::FrustumCulling<velodyne::PointXYZIR> fc;
#endif
*/
  pcl::FrustumCulling<pcl::PointXYZ> fc;
  fc.setInputCloud(point_cloud_ptr);
  fc.setVerticalFOV(45);
  fc.setHorizontalFOV(60);
  fc.setNearPlaneDistance(NEAR_PLANE_DISTANCE);
  fc.setFarPlaneDistance(FAR_PLANE_DISTANCE);

  // image_geometry::PinholeCameraModel cam_model_;
  // cam_model_.fromCameraInfo(cam_info);

  Eigen::Matrix4f camera_pose = Eigen::Matrix4f::Identity();
  // .. read or input the camera pose from a registration algorithm.
  fc.setCameraPose(camera_pose);
  fc.filter(target);
  std::cout << "Filtred. Target size = " << target.size() << std::endl;

  sensor_msgs::PointCloud2 out_cloud;
  *point_cloud_ptr = target;
  std::cout << "Atributed" << std::endl;
  // pcl::toROSMsg(target, *out_cloud);
  std::cout << "toROSMsg Done" << std::endl;
  pub.publish(point_cloud_ptr);  // PointCloud Object is automacally serialized by ROS and there is no need to call
                                 // toROSMsg
  std::cout << "Published" << std::endl;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "correspondences_finder");

  std::cout << "Estou" << std::endl;
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped =
      tfBuffer.lookupTransform("camera_color_left", "velo_link", ros::Time(0), ros::Duration(20.0));

  geometry_msgs::Vector3 translation = transformStamped.transform.translation;
  geometry_msgs::Quaternion rotation_q = transformStamped.transform.rotation;

  pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1);

  message_filters::Subscriber<darknet_ros_msgs::ObjectCount> object_count_sub(nh, "/darknet_ros/found_object", 2);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub(nh, "/darknet_ros/bounding_boxes", 2);
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub(nh, "/kitti/camera_color_left/camera_info", 20);
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub(nh, "/velodyne_points", 20);

  typedef message_filters::sync_policies::ApproximateTime<
      darknet_ros_msgs::ObjectCount, darknet_ros_msgs::BoundingBoxes, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2>
      MySyncPolicy;

  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(20), object_count_sub, bounding_boxes_sub, cam_info_sub,
                                                   point_cloud_sub);

  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin();

  return EXIT_SUCCESS;
}
