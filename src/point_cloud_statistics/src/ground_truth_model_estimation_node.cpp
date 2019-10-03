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
      datasets_path::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_BAG_NAME);

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
  topics.push_back(std::string("/velodyne_points"));
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

  std::stringstream ss;
  ss << datasets_path::constructFullPathToResults(argv[1], datasets_path::ORGANIZED_GROUND_TRUTH_MODEL_PCD_NAME);
  pcl::io::savePCDFile(ss.str(), *ground_truth_model_ptr, true);
  std::cout << "Ground Truth Model saved on " << ss.str() << std::endl;

  /**
   *  Data Saving
   */
  /*
    for (int i = 0; i < azimuth.size(); ++i)
    {
      std::cout << i << ": " << azimuth[i].mean << std::endl;
    }

    for (int i = 0; i < laser.size(); ++i)
    {
      std::cout << i << ": " << laser[i].mean << std::endl;
    }
  */
  int num_elem_written = 0;
  std::ofstream fout;  // file pointer

  std::string average_intensity_filename =
      datasets_path::constructFullPathToResults(argv[1], datasets_path::GROUND_TRUTH_AVERAGE_POINT_INTENSITY_BIN_NAME);

  fout.open(average_intensity_filename, std::ios::out | std::ios::trunc | std::ios::binary);

  for (int i = 0; i < ground_truth_model_ptr->width; ++i)
  {
    for (int j = 0; j < ground_truth_model_ptr->height; ++j)
    {
      if (fout.is_open())
      {
        fout.write(reinterpret_cast<const char*>(&ground_truth_model_ptr->at(i, j).intensity), sizeof(float));
        ++num_elem_written;
      }
      else
      {
        ROS_ERROR("File is not Open! Aborting!");
      }
    }
  }
  fout.close();

  ROS_ASSERT_MSG(num_elem_written == ground_truth_model_ptr->size(),
                 "Average Intensity Data could not be fully saved! Written: %d of %lu.", num_elem_written,
                 ground_truth_model_ptr->size());
  std::cout << "Ground Truth Model Average Intensity saved on " << average_intensity_filename << std::endl;

  /*
    std::ifstream fin;  // file pointer
    fin.open(mean_intensity_filename, std::ios::in | std::ios::binary);

    fin.seekg(0, fin.end);
    int length = fin.tellg() / sizeof(float);
    fin.seekg(0, fin.beg);
    unsigned int count = 0;


  point_cloud::organized::OrganizedVelodynePointCloud test(velodyne::vlp16::AZIMUTHAL_UNIQUE_ANGLES_COUNT,
                                                           velodyne::vlp16::VLP16_LASER_COUNT);
  int num_elements_read = 0;
  float temp_azimuth[velodyne::vlp16::VLP16_LASER_COUNT];
  while (fin.read(reinterpret_cast<char*>(&temp_azimuth[0]), velodyne::vlp16::VLP16_LASER_COUNT * sizeof(float)))
  {
    for (int i = 0; i < velodyne::vlp16::VLP16_LASER_COUNT; ++i)
    {
      test.at(count, i).intensity = temp_azimuth[i];
      ++num_elements_read;
    }
    ++count;
  }
  fin.close();
  std::cout << "Count: " << count << std::endl;

  std::cout << "Interference Results saved on binary file on: " << mean_intensity_filename << std::endl;

  for (int i = 0; i < ground_truth_model_ptr->width; ++i)
  {
    for (int j = 0; j < ground_truth_model_ptr->height; ++j)
    {
      if (ground_truth_model_ptr->at(i, j).intensity != test.at(i, j).intensity)
      {
        ROS_ERROR("Data mismatch");
      }
      std::cout << ground_truth_model_ptr->at(i, j).intensity << " vs " << test.at(i, j).intensity << std::endl;
    }
  }
  std::cout << num_elements_write << " vs " << num_elements_read << std::endl;
  */

  return EXIT_SUCCESS;
}
