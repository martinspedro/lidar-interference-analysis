/**
 * @file   correspondences.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 */

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
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/registration/icp.h>
//#include <pcl_ros/point_cloud.h>
//#include <sensor_msgs/PointCloud2.h>
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

ros::Publisher pub;  //!< ROS Publisher

std::list<int> g_number_of_occurrences;
bool stuck = false;

void number_of_objects_callback(const std_msgs::Int8ConstPtr& number_of_occurrences)
{
  if (stuck)  // this callback as run twice, but not the other
  {
    g_number_of_occurrences.clear();
  }

  std::cout << "Callback - Number of number_of_occurrences" << std::endl;
  std::cout << "Number of occurrences" << (int)(number_of_occurrences->data) << std::endl;
  g_number_of_occurrences.push_back((int)(number_of_occurrences->data));
  stuck = true;
}

void callback(const darknet_ros_msgs::ObjectCountConstPtr& object_count, const darknet_ros_msgs::BoundingBoxesConstPtr& b_boxes, const sensor_msgs::CameraInfoConstPtr& cam_info,
              const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
  std::cout << "Callback" << std::endl;
  /*
  if (!g_number_of_occurrences.empty())
  {
    std::cout << "Callback with number of occurrences: " << g_number_of_occurrences.front() << std::endl;
    std::list<int>::iterator it;
    for (it = g_number_of_occurrences.begin(); it != g_number_of_occurrences.end(); it++)
    {
      std::cout << *it << " ";
    }

    std::cout << std::endl;
*/
    for (int i = 0; i < object_count->count; ++i)
    {
      try
      {
        std::cout << b_boxes->bounding_boxes[i].Class << ": " << b_boxes->bounding_boxes[i].probability << std::endl;
      }
      catch (std::exception& e)
      {
        std::cout << "Catched " << e.what() << std::endl;
      }
    }
    /* g_number_of_occurrences.pop_front();  // remove the first element
  }
  else
  {
    std::cout << "List is empty" << std::endl;
  }
  stuck = false;
  */
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "correspondences_finder");

  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  geometry_msgs::TransformStamped transformStamped =
      tfBuffer.lookupTransform("camera_color_left", "velo_link", ros::Time(0), ros::Duration(20.0));

  geometry_msgs::Vector3 translation = transformStamped.transform.translation;
  geometry_msgs::Quaternion rotation_q = transformStamped.transform.rotation;

  pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1);

  //ros::Subscriber detected_objects_sub = nh.subscribe("/darknet_ros/found_object", 10, number_of_objects_callback);

  message_filters::Subscriber<darknet_ros_msgs::ObjectCount> object_count_sub(nh, "/darknet_ros/found_object", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub(nh, "/darknet_ros/bounding_boxes", 1);
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, "/kitti/camera_color_left/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub(nh, "/velodyne_points", 1);

  typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::ObjectCount, darknet_ros_msgs::BoundingBoxes, sensor_msgs::CameraInfo,
                                                          sensor_msgs::PointCloud2>
      MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), object_count_sub, bounding_boxes_sub, info_sub, point_cloud_sub);

  // TimeSynchronizer<Image, sensor_msgs::PointCloud2> sync(image_sub, point_cloud_sub,  100);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin();

  return EXIT_SUCCESS;
}
