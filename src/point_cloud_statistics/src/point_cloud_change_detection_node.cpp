/**
 * \file   point_cloud_change_detection_node.cpp
 * \brief
 *
 */
#define PCL_NO_PRECOMPILE

// ROS Bag related includes
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <string>
#include <vector>

// IO
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <fstream>

// Bar Graph related includes
#include <vtkColorSeries.h>
#include "point_cloud_statistics/bar_chart_plotter.hpp"
#include "point_cloud_statistics/cloud_statistical_data.hpp"

#include "point_cloud_statistics/velodyne_point_type.h"
#include "point_cloud_statistics/vlp_16_utilities.hpp"

// Datasets related includes
#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_change_detection_node");

  if (!((argc == 3) || (argc == 4) || (argc == 5)))
  {
    ROS_ERROR(
        "Invalid number of arguments providede\n"
        "USAGE: rosrun point_cloud_statistics point_cloud_change_detection_node <IT2 folder for test scenario> "
        "<voxel edge resolution>\n"  // 3 arguments
        "USAGE: rosrun point_cloud_statistics point_cloud_change_detection_node <IT2 folder for test scenario> "
        "<voxel edge intervarl minimum resolution> <voxel edge interval maximum resolution>\n"  // 4 arguments
        "USAGE: rosrun point_cloud_statistics point_cloud_change_detection_node <IT2 folder for test scenario> "
        "<voxel edge intervarl minimum resolution> <voxel edge interval maximum resolution> <step value>\n"  // 5 args
    );

    return EXIT_FAILURE;
  }

  float min_resolution = atof(argv[2]), step_value, max_resolution;

  switch (argc)
  {
    case 3:
      ROS_ASSERT_MSG(min_resolution > 0, "Resolution value must be positive!");
      max_resolution = min_resolution;
      step_value = 1;  // workaround to exit the for cycle for the interval
      break;
    case 4:
      ROS_ASSERT_MSG(min_resolution > 0, "Interval inferior bound must be positive!");
      max_resolution = atof(argv[3]);
      step_value =
          (max_resolution - min_resolution) / 10;  // make step value a order of magnitude below the interval range
      break;
    case 5:
      max_resolution = atof(argv[3]);
      step_value = atof(argv[4]);
      ROS_ASSERT_MSG(step_value > 0, "Step value cannot be negative!");
      break;
  }

  ROS_ASSERT_MSG(min_resolution <= max_resolution,
                 "Interval [%f, %f] is invalid! Inferior bound must be lower than upper bound", min_resolution,
                 max_resolution);

  std::string ground_truth_full_bag_path =
      datasets_path::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_BAG_NAME);
  std::string interference_full_bag_path =
      datasets_path::constructFullPathToDataset(argv[1], datasets_path::INTERFERENCE_BAG_NAME);

  std::cout << "TEST CONDITIONS:" << std::endl
            << "Voxel edge resolution interval: [" << min_resolution << ", " << max_resolution << "]" << std::endl
            << "With a step of: " << step_value << std::endl
            << "Test folder name is: " << argv[1] << std::endl
            << "Ground Truth Full path: " << ground_truth_full_bag_path << std::endl
            << "Interference Full path: " << interference_full_bag_path << std::endl;

  velodyne::VelodynePointCloud::Ptr current_msg_cloud_ptr(new velodyne::VelodynePointCloud),
      ground_truth_ptr(new velodyne::VelodynePointCloud);

  rosbag::Bag interference_bag, ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));

  std::string ground_truth_pcd =
      datasets_path::constructFullPathToResults(argv[1], datasets_path::ORGANIZED_GROUND_TRUTH_MODEL_PCD_NAME);
  pcl::io::loadPCDFile<velodyne::PointXYZIR>(ground_truth_pcd, *ground_truth_ptr);

  std::ofstream logger_file;
  std::string logger_file_name =
      datasets_path::constructFullPathToResults(argv[1], datasets_path::OCTREE_INTERFERENCE_ANALYSIS_LOGGER_FILE_NAME);
  logger_file.open(logger_file_name, std::ios::out | std::ios::trunc);

  std::vector<double> ground_truth_errors, interference_errors, real_errors, resolution_values;

  interference_bag.open(interference_full_bag_path);  // Open interference bag

  rosbag::View interference_view(interference_bag, rosbag::TopicQuery(topics));

  // \TODO Implement Multithreading here. Could speed up computation speeds but requires multiple copies of the octree
  // stucture
  for (float i = min_resolution; i <= max_resolution; i += step_value)
  {
    resolution_values.push_back(i);

    point_cloud::statistics::CloudStatisticalData ground_truth_bag_stats =
        point_cloud::statistics::CloudStatisticalData();
    point_cloud::statistics::CloudStatisticalData interference_bag_stats =
        point_cloud::statistics::CloudStatisticalData();

    // Instantiate octree-based point cloud change detection class
    pcl::octree::OctreePointCloudChangeDetector<velodyne::PointXYZIR> octree(i);

    logger_file << std::endl << "Voxel edge resolution: " << i << std::endl;
    std::cout << std::endl << "Voxel edge resolution: " << i << std::endl;

    // Ground Truth Bag
    foreach (rosbag::MessageInstance const m, ground_truth_view)
    {
      sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (msg != NULL)
      {
        velodyne::VelodynePointCloud point_cloud;
        fromROSMsg(*msg, point_cloud);
        *current_msg_cloud_ptr = point_cloud;

        octree.setInputCloud(ground_truth_ptr);
        octree.addPointsFromInputCloud();
        octree.switchBuffers();

        // Add points from cloudB to octree
        octree.setInputCloud(current_msg_cloud_ptr);
        octree.addPointsFromInputCloud();
        std::vector<int> newPointIdxVector;

        // Get vector of point indices from octree voxels which did not exist in previous buffer
        octree.getPointIndicesFromNewVoxels(newPointIdxVector);

        ++ground_truth_bag_stats.point_cloud_msg_count;
        ground_truth_bag_stats.point_count += point_cloud.size();
        ground_truth_bag_stats.outliers_points_count += newPointIdxVector.size();

        octree.deleteTree();
      }
    }

    ground_truth_bag_stats.computeOutliersRelativeValue();  // Compute Relative percentage of outliers

    std::stringstream octree_ground_truth_statistics_text;
    octree_ground_truth_statistics_text << "GROUND TRUTH: " << std::endl
                                        << ground_truth_bag_stats.outputStringFormattedStatistics().str();
    logger_file << octree_ground_truth_statistics_text.str();
    std::cout << octree_ground_truth_statistics_text.str();
    // Interference Bag
    foreach (rosbag::MessageInstance const m, interference_view)
    {
      sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (msg != NULL)
      {
        velodyne::VelodynePointCloud point_cloud;
        fromROSMsg(*msg, point_cloud);
        *current_msg_cloud_ptr = point_cloud;

        octree.setInputCloud(ground_truth_ptr);
        octree.addPointsFromInputCloud();
        octree.switchBuffers();

        // Add points from cloudB to octree
        octree.setInputCloud(current_msg_cloud_ptr);
        octree.addPointsFromInputCloud();
        std::vector<int> newPointIdxVector;

        // Get vector of point indices from octree voxels which did not exist in previous buffer
        octree.getPointIndicesFromNewVoxels(newPointIdxVector);

        ++interference_bag_stats.point_cloud_msg_count;
        interference_bag_stats.point_count += point_cloud.size();
        interference_bag_stats.outliers_points_count += newPointIdxVector.size();

        octree.deleteTree();
      }
    }

    interference_bag_stats.computeOutliersRelativeValue();  // Compute Relative percentage of outliers

    std::stringstream octree_interference_statistics_text;
    octree_interference_statistics_text << "INTERFERENCE: " << std::endl
                                        << interference_bag_stats.outputStringFormattedStatistics().str();
    logger_file << octree_interference_statistics_text.str();
    std::cout << octree_interference_statistics_text.str();

    // Generate Data for bar chart
    ground_truth_errors.push_back(ground_truth_bag_stats.getOutliersPercentage());
    interference_errors.push_back(interference_bag_stats.getOutliersPercentage());
    real_errors.push_back(interference_bag_stats.getOutliersPercentage() -
                          ground_truth_bag_stats.getOutliersPercentage());
  }

  ground_truth_bag.close();  // close ground truth bag file
  interference_bag.close();  // close interference bag

  logger_file.close();
  ROS_INFO_STREAM("Logger File saved on " << logger_file_name);

  /*********************************************************************************************************************
   *                                                   Data Saving
   ********************************************************************************************************************/

  std::fstream fout;  // file pointer
  std::string interference_csv_stats = datasets_path::constructFullPathToResults(
      argv[1], datasets_path::INTERFERENCE_ANALYSIS_OCTREE_OCUPATION_BIN_NAME);
  fout.open(interference_csv_stats, std::ios::out | std::ios::trunc);  // creates a new csv file with writing permission

  std::stringstream full_statistics;
  for (int i = 0; i < resolution_values.size(); i++)
  {
    full_statistics << resolution_values[i] << ", " << ground_truth_errors[i] / 100.0d << ", "
                    << interference_errors[i] / 100.0d << ", " << real_errors[i] / 100.0d << std::endl;
  }

  fout << full_statistics.str();
  fout.close();

  ROS_INFO_STREAM("Interference Results saved on csv file on: " << interference_csv_stats);

  std::cout << std::endl << full_statistics.str();
  /*
    // Create Bar Plot object with Full HD resolution and the description for the data
    BarChartPlotter* plotter =
        new BarChartPlotter(1920, 1080, "Interference Analysis based on Change Detection using an octree structure",
                            "Voxel edge Resolution", "Outliers/Inliers");

    // Add the outliers of the interfered and ground truth datasets
    plotter->setColorScheme(vtkColorSeries::WARM);
    plotter->addBarPlotData(resolution_values, interference_errors, "Interference Bag vs Ground Truth Model");
    plotter->setColorScheme(vtkColorSeries::BLUES);
    plotter->addBarPlotData(resolution_values, ground_truth_errors, "Ground Truth Bag vs Ground Truth Model");
    plotter->setColorScheme(vtkColorSeries::COOL);
    plotter->addBarPlotData(resolution_values, real_errors, "Ground Truth Bag vs Ground Truth Model");

    plotter->plot();  // holds here until window is given the closing instruction

    // Saves the bar chart as a PNG file on the dataset directory
    std::string bar_chart_filename =
        datasets_path::constructFullPathToDataset(argv[1], "interference_bar_chart.png").c_str();
    plotter->saveBarChartPNG(bar_chart_filename);
    std::cout << "Saved bar chart on: " << bar_chart_filename << std::endl;

    plotter->close();  // Destroys bar chart object
  */
  return EXIT_SUCCESS;
}
