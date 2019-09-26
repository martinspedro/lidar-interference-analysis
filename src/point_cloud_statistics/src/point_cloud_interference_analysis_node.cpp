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
#include <iostream>

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

#include "./matplotlibcpp.h"

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

  std::cout << "Ground Truth Model Loaded" << std::endl;

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

  std::cout << "Ground truth done " << std::endl;
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
            << "Nº points measures" << mean << std::endl
            << "Average Points per message: " << (1.0d * mean) / msg_num_interference << std::endl;

  // Statistical part of Code
  double relative_freq_ground[1000], relative_freq_interference[1000];
  double x_axis[1000];

  for (int i = 0; i < 1000; ++i)
  {
    relative_freq_ground[i] = 0;
    relative_freq_interference[i] = 0;
    x_axis[i] = (double)(i)*0.1;
  }

  std::cout << "Relative vectors" << std::endl;
  unsigned int total_valid_points_ground;
  unsigned int total_valid_points_interference;

  double relative_valid_points_ground[1000];
  double relative_valid_points_interference[1000];

  std::cout << "Ground Truth Errors" << std::endl;
  for (int i = 0; i < ground_truth_errors.size(); ++i)
  {
    if (ground_truth_errors[i] != std::numeric_limits<double>::quiet_NaN())
    {
      ++relative_freq_ground[(unsigned int)(floor(ground_truth_errors[i] * 10.0d))];
      ++total_valid_points_ground;
    }
  }

  std::cout << "Interfernce Errors SIze:" << interference_errors.size() << std::endl;
  for (int i = 0; i < interference_errors.size(); ++i)
  {
    if (interference_errors[i] != std::numeric_limits<double>::quiet_NaN())
    {
      unsigned int index = (unsigned int)(floor(interference_errors[i] * 10.0d));
      // std::cout << index << std::endl;
      if (index < (sizeof(relative_freq_interference) / sizeof(*relative_freq_interference)))
      {
        ++relative_freq_interference[index];
        ++total_valid_points_interference;
      }
      else
      {
        ROS_WARN("Index out of bounds: %d", index);
      }
    }
  }
  std::cout << "Relative Interference Computed" << std::endl;

  std::cout << "done" << std::endl;

  /*
    for (int i = 0; i < 1000; ++i)
    {
      relative_valid_points_ground[i] = relative_freq_ground[i] / total_valid_points_ground;
      relative_valid_points_interference[i] = relative_freq_interference[i] / total_valid_points_interference;

      if (i < 50)
      {
        std::cout << i * 0.1 << ": " << relative_valid_points_ground[i] << " | " <<
    relative_valid_points_interference[i]
                  << std::endl;
      }
    }
  */
  double interference = 0;
  for (int i = 1; i < 1000; ++i)
  {
    interference += (double)relative_freq_interference[i];
  }

  std::cout << "Interference: " << interference / total_valid_points_interference << std::endl;

  // Create Bar Plot object with Full HD resolution and the description for the data
  BarChartPlotter* plotter =
      new BarChartPlotter(1920, 1080, "Interference Analysis based on Change Detection using an octree structure",
                          "Voxel edge Resolution", "Outliers/Inliers");

  // Add the outliers of the interfered and ground truth datasets
  plotter->setColorScheme(vtkColorSeries::WARM);
  plotter->addBarPlotData(x_axis, relative_freq_interference, 1000, "Interference Bag vs Ground Truth Model");
  plotter->setColorScheme(vtkColorSeries::BLUES);
  plotter->addBarPlotData(x_axis, relative_freq_ground, 1000, "Ground Truth Bag vs Ground Truth Model");

  plotter->plot();  // holds here until window is given the closing instruction

  // Saves the bar chart as a PNG file on the dataset directory
  std::string bar_chart_filename = point_cloud_statistics::constructFullPathToDataset(argv[1], "chart.png").c_str();
  plotter->saveBarChartPNG(bar_chart_filename);
  std::cout << "Saved bar chart on: " << bar_chart_filename << std::endl;

  plotter->close();  // Destroys bar chart object

  std::vector<float> aux, x_axis_v;
  for (int i = 0; i < 1000; ++i)
  {
    aux.push_back(relative_freq_ground[i]);
    x_axis_v.push_back(x_axis[i]);
  }

  if (!matplotlibcpp::semilogy(x_axis_v, aux))
  {
    ROS_WARN("No graph produced");
  }

  matplotlibcpp::show();

  return EXIT_SUCCESS;
}
