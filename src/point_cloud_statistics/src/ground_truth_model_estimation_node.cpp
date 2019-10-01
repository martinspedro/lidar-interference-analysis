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

#include "point_cloud_statistics/organized_pointcloud.hpp"
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"
#include "point_cloud_statistics/point_register.hpp"

#include "matplotlib-cpp/matplotlibcpp.h"

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

  if (argc != 2)
  {
    ROS_ERROR("USAGE: rosrun point_cloud_statistics ground_truth_model_estimation_node <IT2 folder for test scenario>");

    return EXIT_FAILURE;
  }

  std::string ground_truth_full_bag_path =
      point_cloud_statistics::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_BAG_NAME);

  ROS_INFO_STREAM("\nGROUND TRUTH GENERATION: " << std::endl
                                                << "- Test folder name is: " << argv[1] << std::endl
                                                << "- Ground Truth Full path: " << ground_truth_full_bag_path
                                                << std::endl);

  point_cloud::organized::OrganizedPointCloud<PointRegister<velodyne::PointXYZIR> > ground_truth_dataset(
      velodyne::vlp16::AZIMUTHAL_UNIQUE_ANGLES_COUNT, velodyne::vlp16::VLP16_LASER_COUNT);
  point_cloud::organized::OrganizedPointCloud<velodyne::PointXYZIR> ground_truth_model(
      velodyne::vlp16::AZIMUTHAL_UNIQUE_ANGLES_COUNT, velodyne::vlp16::VLP16_LASER_COUNT);

  velodyne::VelodynePointCloud::Ptr ground_truth_ptr(new velodyne::VelodynePointCloud);

  rosbag::Bag ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));

  foreach (rosbag::MessageInstance const m, ground_truth_view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      velodyne::VelodynePointCloud ground_truth_point_cloud;
      fromROSMsg(*msg, ground_truth_point_cloud);
      *ground_truth_ptr = ground_truth_point_cloud;

      ground_truth_dataset.registerVelodynePointCloud(*ground_truth_ptr);
    }
  }

  ground_truth_bag.close();  // close ground truth bag file

  std::vector<intensity> azimuth(velodyne::vlp16::AZIMUTHAL_UNIQUE_ANGLES_COUNT);
  std::vector<intensity> laser(velodyne::vlp16::VLP16_LASER_COUNT);

  ground_truth_dataset.computeStats<velodyne::PointXYZIR, intensity>(azimuth, laser);
  ground_truth_model.generateModel<PointRegister<velodyne::PointXYZIR> >(ground_truth_dataset);

  for (int i = 0; i < azimuth.size(); ++i)
  {
    std::cout << i << ": " << azimuth[i].mean << std::endl;
  }

  for (int i = 0; i < laser.size(); ++i)
  {
    std::cout << i << ": " << laser[i].mean << std::endl;
  }

  std::stringstream ss;
  ss << point_cloud_statistics::constructFullPathToDataset(argv[1], "ground_truth_model_new.pcd");
  pcl::io::savePCDFile(ss.str(), ground_truth_model, true);

  std::cout << "Ground Truth Model saved on "
            << point_cloud_statistics::constructFullPathToDataset(argv[1], "ground_truth_model_new.pcd") << std::endl;

  return EXIT_SUCCESS;
}
