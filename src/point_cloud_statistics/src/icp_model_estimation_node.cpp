/**
 * \file   icp_model_estimation_node.cpp
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

#include "point_cloud_statistics/velodyne_point_type.h"
#include "point_cloud_statistics/vlp_16_utilities.hpp"

#include "matplotlib-cpp/matplotlibcpp.h"

const float VOXEL_EDGE_LENGTH = 0.05f;
int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "icp_model_estimation_node");

  if (!((argc != 2) || (argc != 3)))
  {
    ROS_ERROR("Invalid number of arguments provided\n"
              "USAGE: rosrun point_cloud_statistics ground_truth_model_estimation_node <test scenario codename>\n"
              "USAGE: rosrun point_cloud_statistics ground_truth_model_estimation_node <test scenario codename> <voxel "
              "edge length>");

    return EXIT_FAILURE;
  }

  // Use voxel edge length provided by cli or use the default
  float voxel_edge_length = ((argc == 3) ? atof(argv[2]) : VOXEL_EDGE_LENGTH);

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

  std::ofstream logger_file;
  std::string icp_logger_file_name =
      datasets_path::constructFullPathToResults(argv[1], datasets_path::ICP_LOGGER_FILE_NAME);
  logger_file.open(icp_logger_file_name, std::ios::out | std::ios::trunc);
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
        *source = point_cloud::statistics::icp(source, target, logger_file, true);
      }
      else
      {
        *source = ground_truth_point_cloud;
      }

      logger_file << "Message number: " << count << std::endl;
      std::cout << "Message number: " << count << std::endl;
      ++count;
    }
  }
  logger_file.close();
  ground_truth_bag.close();  // close ground truth bag file

  ROS_INFO_STREAM("ICP Computation for ICP Ground Truth Model saved on " << icp_logger_file_name);

  // Voxel Grid Filtering of result
  pcl::VoxelGrid<velodyne::PointXYZIR> sor_voxel;
  sor_voxel.setInputCloud(source);
  sor_voxel.setLeafSize(voxel_edge_length, voxel_edge_length, voxel_edge_length);
  sor_voxel.filter(*ground_truth_ptr);

  /*********************************************************************************************************************
   *                                                   Data Saving
   ********************************************************************************************************************/
  std::string ground_truth_results_folder =
      datasets_path::makeResultsDirectory(argv[1], datasets_path::GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH);

  ROS_INFO_STREAM(datasets_path::GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH << " folder created on "
                                                                         << ground_truth_results_folder);

  /*
   * Unvoxelized ICP Ground Truth PCD Model
   */
  std::stringstream ss;
  ss << datasets_path::constructFullPathToResults(argv[1], datasets_path::ICP_GROUND_TRUTH_MODEL_PCD_NAME);
  pcl::io::savePCDFile(ss.str(), *source, true);
  ROS_INFO_STREAM("ICP Unvoxelized Ground Truth Model saved on " << ss.str());

  /*
   * Voxelized  ICP Ground Truth PCD Model
   */
  ss.str(std::string());  // clear stringstream object contents
  ss << datasets_path::constructFullPathToResults(argv[1], datasets_path::ICP_GROUND_TRUTH_MODEL_PCD_NAME);
  pcl::io::savePCDFile(ss.str(), *ground_truth_ptr, true);
  ROS_INFO_STREAM("ICP Ground Truth Model saved on " << ss.str());

  return EXIT_SUCCESS;
}
