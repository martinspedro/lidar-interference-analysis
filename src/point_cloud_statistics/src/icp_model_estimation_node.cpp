/**
 * \file   ground_truth_model_estimation_node.cpp
 * \brief
 *
 */
#define PCL_NO_PRECOMPILE  // must be included before any PCL include on this CPP file or HPP included before

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>

// Datasets related includes
#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"
#include "point_cloud_statistics/point_cloud_statistics.hpp"

#include <point_cloud_statistics/velodyne_point_type.h>
#include "point_cloud_statistics/vlp_16_utilities.hpp"

#include "point_cloud_statistics/organized_pointcloud.hpp"
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"
#include "point_cloud_statistics/point_register.hpp"

#include "matplotlib-cpp/matplotlibcpp.h"

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "ground_truth_model_estimation_node");

  if (argc != 2)
  {
    ROS_ERROR("USAGE: rosrun point_cloud_statistics ground_truth_model_estimation_node <IT2 folder for test scenario>");

    return EXIT_FAILURE;
  }

  std::string ground_truth_full_bag_path =
      datasets_path::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_BAG_NAME);

  ROS_INFO_STREAM("\nGROUND TRUTH GENERATION: " << std::endl
                                                << "- Test folder name is: " << argv[1] << std::endl
                                                << "- Ground Truth Full path: " << ground_truth_full_bag_path
                                                << std::endl);

  rosbag::Bag ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));

  int count = 0;

  velodyne::VelodynePointCloud::Ptr ground_truth_ptr(new velodyne::VelodynePointCloud),  // Ground Truth Model
      source(new velodyne::VelodynePointCloud),                                          // Update Model of Ground Truth
      target(new velodyne::VelodynePointCloud);                                          // Current iteration pointcloud

  foreach (rosbag::MessageInstance const m, ground_truth_view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      velodyne::VelodynePointCloud ground_truth_point_cloud;
      fromROSMsg(*msg, ground_truth_point_cloud);

      *target = ground_truth_point_cloud;
      if (count > 0)
      {
        // Custom ICP wrapper gives back the merged point clouds
        *source = point_cloud::statistics::icp(source, target, true);
      }
      else
      {
        *source = ground_truth_point_cloud;
      }

      std::cout << "Message number: " << count << std::endl;
    }
  }

  ground_truth_bag.close();  // close ground truth bag file

  // Voxel Grid Filtering of result
  pcl::VoxelGrid<velodyne::PointXYZIR> sor_voxel;
  sor_voxel.setInputCloud(source);
  sor_voxel.setLeafSize(0.05f, 0.05f, 0.05f);
  sor_voxel.filter(*ground_truth_ptr);

  std::stringstream ss;
  ss << datasets_path::constructFullPathToDataset(argv[1], "ground_truth_model_voxel.pcd");
  pcl::io::savePCDFile(ss.str(), *ground_truth_ptr, true);

  std::cout << "Ground Truth Model saved on " << ss.str() << std::endl;

  return EXIT_SUCCESS;
}
