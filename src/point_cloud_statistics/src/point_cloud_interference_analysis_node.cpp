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

#include <velodyne_pointcloud/point_types.h>

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

#include "point_cloud_statistics/organized_pointcloud.hpp"
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"

typedef pcl::PointCloud<velodyne_pointcloud::PointXYZIR> VelodynePointCloud;

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

  // define Velodyne related constants
  const unsigned int VLP16_LASER_COUNT = 16u;
  const float AZIMUTHAL_ANGULAR_RESOLUTION = 0.2F;
  const unsigned int AZIMUTHAL_UNIQUE_ANGLES_COUNT =
      (unsigned int)ceil(point_cloud::organized::FULL_REVOLUTION_DEGREE_F / AZIMUTHAL_ANGULAR_RESOLUTION);

  // Get full path to bags, giving the codename and the type of bag
  std::string ground_truth_full_bag_path =
      point_cloud_statistics::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_BAG_NAME);
  std::string interference_full_bag_path =
      point_cloud_statistics::constructFullPathToDataset(argv[1], datasets_path::INTERFERENCE_BAG_NAME);

  ROS_INFO_STREAM("\nTEST CONDITIONS: " << std::endl
                                        << "- Test folder name is: " << argv[1] << std::endl
                                        << "- Ground Truth Full path: " << ground_truth_full_bag_path << std::endl
                                        << "- Interference Full path: " << interference_full_bag_path << std::endl);

  std::vector<double> ground_truth_errors, interference_errors;
  VelodynePointCloud::Ptr current_msg_cloud_ptr(new VelodynePointCloud);
  VelodynePointCloud::Ptr ground_truth_model_ptr(new VelodynePointCloud);

  point_cloud::organized::OrganizedPointCloud<velodyne_pointcloud::PointXYZIR> ground_truth_model(
      AZIMUTHAL_UNIQUE_ANGLES_COUNT, VLP16_LASER_COUNT);
  point_cloud::organized::OrganizedPointCloud<velodyne_pointcloud::PointXYZIR> ground_truth_cloud(
      AZIMUTHAL_UNIQUE_ANGLES_COUNT, VLP16_LASER_COUNT);
  point_cloud::organized::OrganizedPointCloud<velodyne_pointcloud::PointXYZIR> interference_cloud(
      AZIMUTHAL_UNIQUE_ANGLES_COUNT, VLP16_LASER_COUNT);

  // Load Ground Truth Model for desired Test Scenario
  std::string ground_truth_pcd =
      point_cloud_statistics::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_MODEL_FILENAME);
  pcl::io::loadPCDFile<velodyne_pointcloud::PointXYZIR>(ground_truth_pcd, *ground_truth_model_ptr);

  ground_truth_model.organizeVelodynePointCloud(*ground_truth_model_ptr);

  // Load Bags
  rosbag::Bag interference_bag, ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file
  interference_bag.open(interference_full_bag_path);  // Open interference bag

  // Create Viewers to bags topics
  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));
  rosbag::View interference_view(interference_bag, rosbag::TopicQuery(topics));

  long long unsigned int mean = 0;
  long long unsigned int msg_num_interference = 0;

  foreach (rosbag::MessageInstance const m, ground_truth_view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      VelodynePointCloud point_cloud;
      fromROSMsg(*msg, point_cloud);
      *current_msg_cloud_ptr = point_cloud;

      mean += current_msg_cloud_ptr->size();

      ground_truth_cloud.organizeVelodynePointCloud(*current_msg_cloud_ptr);
      ground_truth_cloud.computeDistanceBetweenPointClouds(ground_truth_model, ground_truth_errors);
      ground_truth_cloud.clearPointsFromPointcloud();
    }
  }

  foreach (rosbag::MessageInstance const m, interference_view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      VelodynePointCloud point_cloud;
      fromROSMsg(*msg, point_cloud);
      *current_msg_cloud_ptr = point_cloud;

      mean += current_msg_cloud_ptr->size();
      ++msg_num_interference;

      interference_cloud.organizeVelodynePointCloud(*current_msg_cloud_ptr);
      interference_cloud.computeDistanceBetweenPointClouds(ground_truth_model, interference_errors);
      interference_cloud.clearPointsFromPointcloud();
    }
  }

  ground_truth_bag.close();  // close ground truth bag file
  interference_bag.close();  // close interference bag

  std::cout << "Nº messages received: " << msg_num_interference << std::endl
            << "Nº points measures: " << mean << std::endl
            << "Average Points per message: " << (1.0d * mean) / msg_num_interference << std::endl;

  // Statistical part of Code
  const double DISTANCE_RESOLUTION = 0.1d;    // in meters
  const unsigned int MAXIMUM_DISTANCE = 130;  // in meters
  const unsigned int UNIQUE_DISTANCES_COUNT = (unsigned int)(ceil(MAXIMUM_DISTANCE / DISTANCE_RESOLUTION));

  std::fstream fout;  // file pointer
  std::string interference_distance_errors = point_cloud_statistics::constructFullPathToDataset(argv[1], "interference_"
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

  std::string ground_truth_distance_errors = point_cloud_statistics::constructFullPathToDataset(argv[1], "ground_truth_"
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

  unsigned int total_valid_points_ground = 0;
  unsigned int total_valid_points_interference = 0;

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
  /*
    // Create Bar Plot object with Full HD resolution and the description for the data
    BarChartPlotter* plotter =
        new BarChartPlotter(1920, 1080, "Interference Analysis based on Change Detection using an octree structure",
                            "Voxel edge Resolution", "Outliers/Inliers");

    // Add the outliers of the interfered and ground truth datasets
    plotter->setColorScheme(vtkColorSeries::WARM);
    plotter->addBarPlotData(x_axis, interference_freq_count, 1000, "Interference Bag vs Ground Truth Model");
    plotter->setColorScheme(vtkColorSeries::BLUES);
    plotter->addBarPlotData(x_axis, ground_truth_freq_count, 1000, "Ground Truth Bag vs Ground Truth Model");

    plotter->plot();  // holds here until window is given the closing instruction

    // Saves the bar chart as a PNG file on the dataset directory
    std::string bar_chart_filename = point_cloud_statistics::constructFullPathToDataset(argv[1], "chart.png").c_str();
    plotter->saveBarChartPNG(bar_chart_filename);
    std::cout << "Saved bar chart on: " << bar_chart_filename << std::endl;

    plotter->close();  // Destroys bar chart object
  */
  std::vector<double> aux, x_axis_v;
  for (int i = 0; i < UNIQUE_DISTANCES_COUNT; ++i)
  {
    aux.push_back(ground_truth_freq_count[i]);
    x_axis_v.push_back(x_axis[i]);
  }

  if (!matplotlibcpp::semilogy(x_axis_v, aux))
  {
    ROS_WARN("No graph produced");
  }

  interference_errors.erase(
      std::remove(interference_errors.begin(), interference_errors.end(), std::numeric_limits<double>::quiet_NaN()),
      interference_errors.end());

  if (!matplotlibcpp::bar(x_axis_v, interference_errors))
  {
    ROS_WARN("No bar2 produced");
  }

  if (!matplotlibcpp::hist(interference_errors))
  {
    ROS_WARN("No hist produced");
  }

  matplotlibcpp::show();

  return EXIT_SUCCESS;
}
