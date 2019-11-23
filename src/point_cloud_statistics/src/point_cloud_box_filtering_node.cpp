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
#include "point_cloud_statistics/cloud_statistical_data.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_box_filter");

  // Get Private parameters
  ros::NodeHandle nh_("~");

  double min_x, min_y, min_z, max_x, max_y, max_z, tolerance;
  nh_.getParam("min_point/x", min_x);
  nh_.getParam("min_point/y", min_y);
  nh_.getParam("min_point/z", min_z);
  nh_.getParam("max_point/x", max_x);
  nh_.getParam("max_point/y", max_y);
  nh_.getParam("max_point/z", max_z);
  nh_.getParam("point_distance_tolerance", tolerance);

  tolerance += 1.0d;  // Add 1 to tolerance to keep the current value

  BoxFilter room((float)(min_x * tolerance), (float)(max_x * tolerance), (float)(min_y * tolerance),
                 (float)(max_y * tolerance), (float)(min_z * tolerance), (float)(max_z * tolerance));

  point_cloud::statistics::CloudStatisticalData interference_bag_stats =
      point_cloud::statistics::CloudStatisticalData();

  std::string full_bag_path = datasets_path::constructFullPathToDataset(argv[1], datasets_path::INTERFERENCE_BAG_NAME);
  ROS_INFO_STREAM("Opening rosbag file on: " << full_bag_path);

  PointCloud::Ptr cloudPtr(new PointCloud);

  rosbag::Bag ground_truth_bag;
  ground_truth_bag.open(full_bag_path);

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View view(ground_truth_bag, rosbag::TopicQuery(topics));

  foreach (rosbag::MessageInstance const m, view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      PointCloud point_cloud;
      fromROSMsg(*msg, point_cloud);

      ++interference_bag_stats.point_cloud_msg_count;
      for (int j = 0; j < point_cloud.points.size(); j++)
      {
        interference_bag_stats.point_count++;
        if (room.pointInsideBox(point_cloud.points[j]))
        {
          ++interference_bag_stats.inliers_points_count;
        }
        else
        {
          ++interference_bag_stats.outliers_points_count;
        }
      }
    }
  }

  ground_truth_bag.close();

  interference_bag_stats.computeStats();
  interference_bag_stats.printStatistics();

  /*********************************************************************************************************************
   *                                                   Data Saving
   ********************************************************************************************************************/

  std::fstream fout;  // file pointer
  std::string interference_csv_stats =
      datasets_path::constructFullPathToResults(argv[1], datasets_path::INTERFERENCE_BOX_FILTER_FILE_NAME);
  fout.open(interference_csv_stats, std::ios::out | std::ios::trunc);  // creates a new csv file with writing permission

  std::stringstream full_statistics;
  full_statistics << interference_bag_stats.point_cloud_msg_count << ", " << interference_bag_stats.point_count << ", "
                  << interference_bag_stats.points_average_per_msg << ", "
                  << interference_bag_stats.inliers_points_count << ", " << interference_bag_stats.outliers_points_count
                  << std::endl;

  fout << full_statistics.str();
  fout.close();

  ROS_INFO_STREAM("Interference Results saved on csv file on: " << interference_csv_stats);
  return EXIT_SUCCESS;
}
