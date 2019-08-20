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
#include "point_cloud_statistics/box_filter.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

const std::string bags_folder_path = "/media/martinspedro/Elements/mine/IT2 Dark Room/Scenario B/";

inline const std::string constructFullBagPath(const std::string bag_type, std::string dataset_name)
{
  return datasets_path::IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + dataset_name + "/" + bag_type;
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_histogram");

  PointCloud::Ptr cloudPtr(new PointCloud);

  std::cout << "Opening rosbag" << constructFullBagPath(datasets_path::INTERFERENCE_BAG_NAME, argv[1]) << std::endl;
  rosbag::Bag bag;
  bag.open(constructFullBagPath(datasets_path::INTERFERENCE_BAG_NAME, argv[1]));

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  long int msg_count = 0, point_count = 0, out_points_count = 0, in_points_count = 0;
  BoxFilter dark_room(-2.0, 7.0, -3.0, 4.5, -1.9, 3.5);  // added 1 meter in every direction

  foreach (rosbag::MessageInstance const m, view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      PointCloud point_cloud;
      fromROSMsg(*msg, point_cloud);
      ++msg_count;
      for (int j = 0; j < point_cloud.points.size(); j++)
      {
        point_count++;
        if (dark_room.pointInsideBox(point_cloud.points[j]))
        {
          ++in_points_count;
        }
        else
        {
          ++out_points_count;
        }
      }
    }
  }

  bag.close();

  float relative_percentage_out_points = (float)(out_points_count) / point_count * 100;
  float relative_percentage_in_points = (float)(in_points_count) / point_count * 100;

  std::cout << "Number of received Point Cloud Messages: " << msg_count << std::endl
            << "Number of received Point Cloud 3D Points: " << point_count << std::endl
            << "From which " << out_points_count << "(" << relative_percentage_out_points
            << "%) are outside the room limits and " << in_points_count << "(" << relative_percentage_in_points
            << "%) are inside room dimensions" << std::endl;

  return EXIT_SUCCESS;
}
