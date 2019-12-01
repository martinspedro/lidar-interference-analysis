/**
 * \file   ground_truth_model_estimation_node.cpp
 * \brief Ground truth model estimation for pcl Point Clouds
 *
 */
#define PCL_NO_PRECOMPILE

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Datasets related includes
#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"
#include "point_cloud_statistics/point_cloud_statistics.hpp"

#include <point_cloud_statistics/velodyne_point_type.h>
#include "point_cloud_statistics/vlp_16_utilities.hpp"

#include "point_cloud_statistics/organized_point_cloud.hpp"
#include "point_cloud_statistics/organized_velodyne_point_cloud.hpp"
#include "point_cloud_statistics/organized_point_cloud_with_container.hpp"

#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"
#include "point_cloud_statistics/point_statistics_container.hpp"

#include "matplotlib-cpp/matplotlibcpp.h"

#include <fstream>

#include <sys/types.h>
#include <sys/stat.h>

/**
 * \struct intensity characteristics
 */
struct intensity
{
  float mean;      //!< average of intensity measures
  float variance;  //!< variance of intensity measures
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "ground_truth_model_estimation_node");

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  if (argc != 2)
  {
    ROS_ERROR("USAGE: rosrun point_cloud_statistics ground_truth_model_estimation_node <IT2 folder for test scenario>");

    return EXIT_FAILURE;
  }

  std::string ground_truth_full_bag_path =
      datasets_path::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_ROI_BAG_NAME);

  ROS_INFO_STREAM("\nGROUND TRUTH GENERATION: " << std::endl
                                                << "- Test folder name is: " << argv[1] << std::endl
                                                << "- Ground Truth Full path: " << ground_truth_full_bag_path
                                                << std::endl);

  point_cloud::organized::OrganizedPointCloudWithContainer<PointStatisticsContainer<velodyne::PointXYZIR>,
                                                           velodyne::PointXYZIR>
      ground_truth_dataset(velodyne::vlp16::AZIMUTHAL_UNIQUE_ANGLES_COUNT, velodyne::vlp16::VLP16_LASER_COUNT);

  point_cloud::organized::OrganizedVelodynePointCloud* ground_truth_model_ptr(
      new point_cloud::organized::OrganizedVelodynePointCloud(velodyne::vlp16::AZIMUTHAL_UNIQUE_ANGLES_COUNT,
                                                              velodyne::vlp16::VLP16_LASER_COUNT));

  velodyne::VelodynePointCloud::Ptr ground_truth_ptr(new velodyne::VelodynePointCloud);

  rosbag::Bag ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file

  std::vector<std::string> topics;
  topics.push_back(std::string("/point_cloud_clusters"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));

  foreach (rosbag::MessageInstance const m, ground_truth_view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      velodyne::VelodynePointCloud ground_truth_point_cloud;
      fromROSMsg(*msg, ground_truth_point_cloud);
      *ground_truth_ptr = ground_truth_point_cloud;

      ground_truth_dataset.registerPointCloud(ground_truth_point_cloud);
    }
  }

  ground_truth_bag.close();  // close ground truth bag file

  std::vector<intensity> azimuth(velodyne::vlp16::AZIMUTHAL_UNIQUE_ANGLES_COUNT);
  std::vector<intensity> laser(velodyne::vlp16::VLP16_LASER_COUNT);

  ground_truth_dataset.computeStats<intensity>(azimuth, laser);
  ground_truth_dataset.generateModel(ground_truth_model_ptr);

  ROS_INFO_STREAM("Data computation ended. Ground Truth Model and related statistics caluldated.");

  /*********************************************************************************************************************
   *                                                   Data Saving
   ********************************************************************************************************************/
  std::string ground_truth_results_folder =
      datasets_path::makeResultsDirectory(argv[1], datasets_path::GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH);

  ROS_INFO_STREAM(datasets_path::GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH << " folder created on "
                                                                         << ground_truth_results_folder);

  /*
   * Organized PointCloud PCD Model
   */
  std::stringstream ss;
  ss << datasets_path::constructFullPathToResults(argv[1], datasets_path::GROUND_TRUTH_ROI_MODEL_PCD_NAME);
  pcl::io::savePCDFile(ss.str(), *ground_truth_model_ptr, true);
  ROS_INFO_STREAM("Ground Truth ROI Model saved on " << ss.str());

  return EXIT_SUCCESS;
}
