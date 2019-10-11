/**
 * \file   point_cloud_interference_analysis_node.cpp
 * \brief
 *
 */

// ROS Bag related includes
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <string>
#include <vector>
#include <array>
#include <iostream>
#include <algorithm>

// Bar Graph related includes
#include <vtkColorSeries.h>
#include "point_cloud_statistics/bar_chart_plotter.hpp"
#include "point_cloud_statistics/cloud_statistical_data.hpp"

// Datasets related includes
#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"
#include "point_cloud_statistics/point_cloud_statistics.hpp"

// writing to CSV file
#include <fstream>

#include <pcl/visualization/pcl_plotter.h>

#include "point_cloud_statistics/organized_point_cloud.hpp"
#include "point_cloud_statistics/organized_velodyne_point_cloud.hpp"
#include "point_cloud_statistics/velodyne_point_type.h"
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"

#include "matplotlib-cpp/matplotlibcpp.h"

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_interference_analysis_node");

  if (argc != 2)
  {
    ROS_ERROR("Invalid number of arguments provided\n"
              "USAGE: rosrun point_cloud_statistics point_cloud_interference_analysis_node <test scenario codename>");

    return EXIT_FAILURE;
  }

  // Get full path to bags, giving the codename and the type of bag
  std::string ground_truth_full_bag_path =
      datasets_path::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_BAG_NAME);
  std::string interference_full_bag_path =
      datasets_path::constructFullPathToDataset(argv[1], datasets_path::INTERFERENCE_BAG_NAME);

  ROS_INFO_STREAM("\nTEST CONDITIONS: " << std::endl
                                        << "- Test folder name is: " << argv[1] << std::endl
                                        << "- Ground Truth Full path: " << ground_truth_full_bag_path << std::endl
                                        << "- Interference Full path: " << interference_full_bag_path << std::endl);

  std::vector<double> ground_truth_bag_distance, interference_bag_distance, ground_truth_bag_intensity,
      interference_bag_intensity;
  velodyne::VelodynePointCloud::Ptr current_msg_cloud_ptr(new velodyne::VelodynePointCloud);
  velodyne::VelodynePointCloud::Ptr ground_truth_model_ptr(new velodyne::VelodynePointCloud);

  point_cloud::organized::OrganizedVelodynePointCloud ground_truth_model(velodyne::vlp16::AZIMUTHAL_UNIQUE_ANGLES_COUNT,
                                                                         velodyne::vlp16::VLP16_LASER_COUNT);
  point_cloud::organized::OrganizedVelodynePointCloud ground_truth_cloud(velodyne::vlp16::AZIMUTHAL_UNIQUE_ANGLES_COUNT,
                                                                         velodyne::vlp16::VLP16_LASER_COUNT);
  point_cloud::organized::OrganizedVelodynePointCloud interference_cloud(velodyne::vlp16::AZIMUTHAL_UNIQUE_ANGLES_COUNT,
                                                                         velodyne::vlp16::VLP16_LASER_COUNT);

  // Load Ground Truth Model for desired Test Scenario
  std::string ground_truth_pcd =
      datasets_path::constructFullPathToResults(argv[1], datasets_path::ORGANIZED_GROUND_TRUTH_MODEL_PCD_NAME);
  pcl::io::loadPCDFile<velodyne::PointXYZIR>(ground_truth_pcd, *ground_truth_model_ptr);

  ground_truth_model.organizePointCloud(*ground_truth_model_ptr);

  // Load Bags
  rosbag::Bag interference_bag, ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file
  interference_bag.open(interference_full_bag_path);  // Open interference bag

  // Create Viewers to bags topics
  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));
  rosbag::View interference_view(interference_bag, rosbag::TopicQuery(topics));

  point_cloud::statistics::CloudStatisticalData ground_truth_statistics;
  point_cloud::statistics::CloudStatisticalData interference_statistics;

  std::ofstream logger_file;
  std::string logger_file_name =
      datasets_path::constructFullPathToResults(argv[1], datasets_path::INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME);
  logger_file.open(logger_file_name, std::ios::out | std::ios::trunc);

  foreach (rosbag::MessageInstance const m, ground_truth_view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      velodyne::VelodynePointCloud point_cloud;
      fromROSMsg(*msg, point_cloud);
      *current_msg_cloud_ptr = point_cloud;

      ground_truth_statistics.point_count += current_msg_cloud_ptr->size();
      ++ground_truth_statistics.point_cloud_msg_count;

      ground_truth_cloud.organizePointCloud(*current_msg_cloud_ptr);
      ground_truth_cloud.computeDistanceBetweenPointClouds(ground_truth_model, ground_truth_bag_distance,
                                                           ground_truth_bag_intensity);
      ground_truth_cloud.clearPointsFromPointcloud();
    }
  }

  std::stringstream ground_truth_statistics_text;
  ground_truth_statistics_text << "GROUND TRUTH: " << std::endl
                               << ground_truth_statistics.outputStringFormattedPointStatistics().str();
  logger_file << ground_truth_statistics_text.str();
  std::cout << ground_truth_statistics_text.str();

  foreach (rosbag::MessageInstance const m, interference_view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      velodyne::VelodynePointCloud point_cloud;
      fromROSMsg(*msg, point_cloud);
      *current_msg_cloud_ptr = point_cloud;

      interference_statistics.point_count += current_msg_cloud_ptr->size();
      ++interference_statistics.point_cloud_msg_count;

      interference_cloud.organizePointCloud(*current_msg_cloud_ptr);
      interference_cloud.computeDistanceBetweenPointClouds(ground_truth_model, interference_bag_distance,
                                                           interference_bag_intensity);
      interference_cloud.clearPointsFromPointcloud();
    }
  }

  ground_truth_bag.close();  // close ground truth bag file
  interference_bag.close();  // close interference bag

  std::stringstream interference_statistics_text;
  interference_statistics_text << "INTERFERENCE: " << std::endl
                               << interference_statistics.outputStringFormattedPointStatistics().str();
  logger_file << interference_statistics_text.str();
  std::cout << interference_statistics_text.str();

  logger_file.close();
  ROS_INFO_STREAM("Logger File saved on " << logger_file_name);

  /*********************************************************************************************************************
   *                                                   Data Saving
   ********************************************************************************************************************/
  std::string ground_truth_results_folder =
      datasets_path::makeResultsDirectory(argv[1], datasets_path::GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH);

  ROS_INFO_STREAM(datasets_path::GROUND_TRUTH_MODEL_FOLDER_RELATIVE_PATH << " folder created on "
                                                                         << ground_truth_results_folder);

  std::string interference_results_folder =
      datasets_path::makeResultsDirectory(argv[1], datasets_path::INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH);

  ROS_INFO_STREAM(datasets_path::INTERFERENCE_ANALYSIS_FOLDER_RELATIVE_PATH << " folder created on "
                                                                            << interference_results_folder);

  int num_elem_written = 0;
  std::ofstream fout;  // file pointer

  /*
   * Save the Ground Truth Bag Distance between Ground Truth Model and the Ground Truth Bag
   */
  std::string ground_truth_distance_vector_filename = datasets_path::constructFullPathToResults(
      argv[1], datasets_path::GROUND_TRUTH_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME);

  fout.open(ground_truth_distance_vector_filename, std::ios::out | std::ios::trunc | std::ios::binary);

  for (int i = 0; i < ground_truth_bag_distance.size() && fout.is_open(); ++i)
  {
    if (fout.good())
    {
      fout.write(reinterpret_cast<const char*>(&ground_truth_bag_distance[i]), sizeof(double));
      ++num_elem_written;
    }
    else
    {
      ROS_ERROR("Good bit no set!");
    }
  }

  fout.close();

  ROS_ASSERT_MSG(num_elem_written == ground_truth_bag_distance.size(),
                 "Laser Intensity Data could not be fully saved! Written: %d of %lu.", num_elem_written,
                 ground_truth_bag_distance.size());
  ROS_INFO_STREAM("Ground Truth Bag Distance from Ground Truth Model saved on "
                  << ground_truth_distance_vector_filename);

  /*
   * Save the Ground Truth Bag Intensity Difference between Ground Truth Model and the Ground Truth Bag
   */
  num_elem_written = 0;
  std::string ground_truth_intensity_vector_filename = datasets_path::constructFullPathToResults(
      argv[1], datasets_path::GROUND_TRUTH_BAG_POINTS_INTENSITY_VECTOR_BIN_NAME);

  fout.open(ground_truth_intensity_vector_filename, std::ios::out | std::ios::trunc | std::ios::binary);

  for (int i = 0; i < ground_truth_bag_intensity.size() && fout.is_open(); ++i)
  {
    if (fout.good())
    {
      fout.write(reinterpret_cast<const char*>(&ground_truth_bag_intensity[i]), sizeof(double));
      ++num_elem_written;
    }
    else
    {
      ROS_ERROR("Good bit no set!");
    }
  }

  fout.close();

  ROS_ASSERT_MSG(num_elem_written == ground_truth_bag_intensity.size(),
                 "Laser Intensity Data could not be fully saved! Written: %d of %lu.", num_elem_written,
                 ground_truth_bag_intensity.size());
  ROS_INFO_STREAM("Ground Truth Bag Intensity Difference from Ground Truth Model saved on "
                  << ground_truth_intensity_vector_filename);

  /*
   * Save the Interference Bag Distance between Ground Truth Model and the Interference Bag
   */
  num_elem_written = 0;
  std::string interference_distance_vector_filename = datasets_path::constructFullPathToResults(
      argv[1], datasets_path::INTERFERENCE_BAG_POINTS_DISTANCE_VECTOR_BIN_NAME);

  fout.open(interference_distance_vector_filename, std::ios::out | std::ios::trunc | std::ios::binary);

  for (int i = 0; i < interference_bag_distance.size() && fout.is_open(); ++i)
  {
    if (fout.good())
    {
      fout.write(reinterpret_cast<const char*>(&interference_bag_distance[i]), sizeof(double));
      ++num_elem_written;
    }
    else
    {
      ROS_ERROR("Good bit no set!");
    }
  }

  fout.close();

  ROS_ASSERT_MSG(num_elem_written == interference_bag_distance.size(),
                 "Laser Intensity Data could not be fully saved! Written: %d of %lu.", num_elem_written,
                 interference_bag_distance.size());
  ROS_INFO_STREAM("Interference Bag Distance from Ground Truth Model saved on  "
                  << interference_distance_vector_filename);

  /*
   * Save the Interference Bag Distance between Ground Truth Model and the Interference Bag
   */
  num_elem_written = 0;
  std::string interference_intensity_vector_filename = datasets_path::constructFullPathToResults(
      argv[1], datasets_path::INTERFERENCE_BAG_POINTS_INTENSITY_VECTOR_BIN_NAME);

  fout.open(interference_intensity_vector_filename, std::ios::out | std::ios::trunc | std::ios::binary);

  for (int i = 0; i < interference_bag_intensity.size() && fout.is_open(); ++i)
  {
    if (fout.good())
    {
      fout.write(reinterpret_cast<const char*>(&interference_bag_intensity[i]), sizeof(double));
      ++num_elem_written;
    }
    else
    {
      ROS_ERROR("Good bit no set!");
    }
  }

  fout.close();

  ROS_ASSERT_MSG(num_elem_written == interference_bag_intensity.size(),
                 "Laser Intensity Data could not be fully saved! Written: %d of %lu.", num_elem_written,
                 interference_bag_intensity.size());
  ROS_INFO_STREAM("Interference Bag Intensity Difference from Ground Truth Model saved on  "
                  << interference_intensity_vector_filename);
  /*
  *
  +
  +
  +
  +
  +
  +
  +
  +
  */

  /*
    // Statistical part of Code
    const double DISTANCE_RESOLUTION = 0.1d;    // in meters
    const unsigned int MAXIMUM_DISTANCE = 130;  // in meters
    const unsigned int UNIQUE_DISTANCES_COUNT = (unsigned int)(ceil(MAXIMUM_DISTANCE / DISTANCE_RESOLUTION));

    std::fstream fout;  // file pointer
    std::string interference_distance_errors = datasets_path::constructFullPathToDataset(argv[1], "interference_"
                                                                                                  "distance_"
                                                                                                  "errors.csv");
    fout.open(interference_distance_errors, std::ios::out);  // creates a new csv file with writing permission
    for (int i = 0; i < interference_errors.size(); i += VLP16_LASER_COUNT)
    {
      for (int j = i; j < i + VLP16_LASER_COUNT; ++j)
      {
        fout << interference_errors[j] << ", ";
      }
      fout << "\n";
    }
    fout.close();

    std::cout << "Interference Results saved on csv file on: " << interference_distance_errors << std::endl;

    std::string ground_truth_distance_errors = datasets_path::constructFullPathToDataset(argv[1], "ground_truth_"
                                                                                                  "distance_"
                                                                                                  "errors.csv");
    fout.open(ground_truth_distance_errors, std::ios::out);  // creates a new csv file with writing permission
    for (int i = 0; i < ground_truth_errors.size(); i += VLP16_LASER_COUNT)
    {
      for (int j = i; j < i + VLP16_LASER_COUNT; ++j)
      {
        fout << ground_truth_errors[j] << ", ";
      }
      fout << "\n";
    }
    fout.close();

    std::cout << "Interference Results saved on csv file on: " << ground_truth_distance_errors << std::endl;

    // create arrays to count the frequency of interference and use default value initialization
    std::array<int, UNIQUE_DISTANCES_COUNT> ground_truth_freq_count{}, interference_freq_count{};
    std::array<double, UNIQUE_DISTANCES_COUNT> ground_truth_relative_freq_count{}, interference_relative_freq_count{};

    // Create array to hold x_axis values and initialize it
    std::array<double, UNIQUE_DISTANCES_COUNT> x_axis{};
    for (int i = 0; i < UNIQUE_DISTANCES_COUNT; ++i)
    {
      x_axis[i] = (double)(i)*0.1;
    }

    double total_valid_points_ground = 0;
    double total_valid_points_interference = 0;

    for (int i = 0; i < ground_truth_errors.size(); ++i)
    {
      if (ground_truth_errors[i] != std::numeric_limits<double>::quiet_NaN())
      {
        ++ground_truth_freq_count[(unsigned int)(floor(ground_truth_errors[i] / DISTANCE_RESOLUTION))];
        ++total_valid_points_ground;
      }
    }

    for (int i = 0; i < interference_errors.size(); ++i)
    {
      if (interference_errors[i] != std::numeric_limits<double>::quiet_NaN())
      {
        unsigned int index = (unsigned int)(floor(interference_errors[i] / DISTANCE_RESOLUTION));
        // std::cout << index << std::endl;
        if (index < interference_freq_count.size())
        {
          ++interference_freq_count[index];
          ++total_valid_points_interference;
        }
        else
        {
          ROS_WARN("Index out of bounds: %d", index);
        }
      }
    }
    std::cout << "Relative Interference Computed" << std::endl;

    std::cout << "Total Valid Ground Points: " << total_valid_points_ground << std::endl;
    std::cout << "Total Valid Intereference Points: " << total_valid_points_interference << std::endl;

    std::vector<double> x_axis_v(x_axis.begin(), x_axis.end()),
        interference_freq_count_v(interference_freq_count.begin(), interference_freq_count.end()),
        ground_truth_freq_count_v(ground_truth_freq_count.begin(), ground_truth_freq_count.end());

    for (int i = 0; i < interference_freq_count_v.size(); ++i)
    {
      interference_freq_count_v[i] /= total_valid_points_interference;
    }

    for (int i = 0; i < ground_truth_freq_count_v.size(); ++i)
    {
      ground_truth_freq_count_v[i] /= total_valid_points_ground;
    }
    // Create Bar Plot object with Full HD resolution and the description for the data
    BarChartPlotter* plotter =
        new BarChartPlotter(1920, 1080, "Interference Analysis based on Change Detection using an octree structure",
                            "Voxel edge Resolution", "Outliers/Inliers");

    // Add the outliers of the interfered and ground truth datasets
    plotter->setColorScheme(vtkColorSeries::WARM);
    plotter->addBarPlotData(x_axis_v, interference_freq_count_v, "Interference Bag vs Ground Truth Model");
    plotter->setColorScheme(vtkColorSeries::BLUES);
    plotter->addBarPlotData(x_axis_v, ground_truth_freq_count_v, "Ground Truth Bag vs Ground Truth Model");

    plotter->plot();  // holds here until window is given the closing instruction

    // Saves the bar chart as a PNG file on the dataset directory
    std::string bar_chart_filename = datasets_path::constructFullPathToDataset(argv[1], "chart.png").c_str();
    plotter->saveBarChartPNG(bar_chart_filename);
    std::cout << "Saved bar chart on: " << bar_chart_filename << std::endl;

    plotter->close();  // Destroys bar chart object

    if (!matplotlibcpp::semilogy(x_axis_v, ground_truth_freq_count_v))
    {
      ROS_WARN("No graph produced");
    }
    matplotlibcpp::show();
    interference_errors.erase(
        std::remove(interference_errors.begin(), interference_errors.end(), std::numeric_limits<double>::quiet_NaN()),
        interference_errors.end());

    if (!matplotlibcpp::bar(x_axis_v, interference_freq_count_v))
    {
      ROS_WARN("No bar2 produced");
    }
    matplotlibcpp::show();
    if (!matplotlibcpp::hist(interference_errors))
    {
      ROS_WARN("No hist produced");
    }

    matplotlibcpp::show();
  */
  return EXIT_SUCCESS;
}
