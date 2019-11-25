#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <string>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <vector>
#include <iostream>

#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"

#include "pcl/common/common.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_box_filter");

  std::string full_bag_path = datasets_path::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_BAG_NAME);
  std::cout << "Opening rosbag" << full_bag_path << std::endl;

  PointCloud::Ptr cloudPtr(new PointCloud);

  rosbag::Bag ground_truth_bag;
  ground_truth_bag.open(full_bag_path);

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View view(ground_truth_bag, rosbag::TopicQuery(topics));

  pcl::PointXYZ min_point_all, max_point_all;
  foreach (rosbag::MessageInstance const m, view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      PointCloud point_cloud;
      fromROSMsg(*msg, point_cloud);
      pcl::PointXYZ min_point, max_point;

      pcl::getMinMax3D(point_cloud, min_point, max_point);
      if (min_point.x < min_point_all.x)
      {
        min_point_all.x = min_point.x;
      }
      if (min_point.y < min_point_all.y)
      {
        min_point_all.y = min_point.y;
      }
      if (min_point.z < min_point_all.z)
      {
        min_point_all.z = min_point.z;
      }

      if (max_point.x > max_point_all.x)
      {
        max_point_all.x = max_point.x;
      }
      if (max_point.y > max_point_all.y)
      {
        max_point_all.y = max_point.y;
      }
      if (max_point.z > max_point_all.z)
      {
        max_point_all.z = max_point.z;
      }

      // std::cout << min_point << " - " << max_point << std::endl;
    }
  }

  ground_truth_bag.close();
  std::cout << "Min_Point: " << min_point_all << " | Max_point: " << max_point_all << std::endl;

  return EXIT_SUCCESS;
}
