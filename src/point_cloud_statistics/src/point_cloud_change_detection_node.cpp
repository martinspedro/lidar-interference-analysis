/**
 * \file   point_cloud_change_detection_node.cpp
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
#include <pcl/common/geometry.h>

#include <pcl/octree/octree_pointcloud_changedetector.h>

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

typedef pcl::PointCloud<velodyne_pointcloud::PointXYZIR> VelodynePointCloud;

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
      point_cloud_statistics::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_BAG_NAME);
  std::string interference_full_bag_path =
      point_cloud_statistics::constructFullPathToDataset(argv[1], datasets_path::INTERFERENCE_BAG_NAME);

  std::cout << "TEST CONDITIONS:" << std::endl
            << "Voxel edge resolution interval: [" << min_resolution << ", " << max_resolution << "]" << std::endl
            << "With a step of: " << step_value << std::endl
            << "Test folder name is: " << argv[1] << std::endl
            << "Ground Truth Full path: " << ground_truth_full_bag_path << std::endl
            << "Interference Full path: " << interference_full_bag_path << std::endl;

  PointCloud::Ptr current_msg_cloud_ptr(new PointCloud);
  PointCloud::Ptr ground_truth_ptr(new PointCloud);
  // velodyne_pointcloud::PointcloudXYZIR::Ptr ground_truth_ptr(new velodyne_pointcloud::PointcloudXYZIR);

  rosbag::Bag interference_bag, ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));

  std::string ground_truth_pcd = point_cloud_statistics::constructFullPathToDataset(argv[1], "ground_truth_model.pcd");
  pcl::io::loadPCDFile<pcl::PointXYZ>(ground_truth_pcd, *ground_truth_ptr);

  std::vector<double> ground_truth_errors, interference_errors, resolution_values;

  interference_bag.open(interference_full_bag_path);  // Open interference bag

  rosbag::View interference_view(interference_bag, rosbag::TopicQuery(topics));

  // \TODO Implement Multithreading here. Could speed up computation speeds but requires multiple copies of the octree
  // stucture
  for (float i = min_resolution; i <= max_resolution; i += step_value)
  {
    resolution_values.push_back(i);

    point_cloud_statistics::CloudStatisticalData ground_truth_bag_stats =
        point_cloud_statistics::CloudStatisticalData();
    point_cloud_statistics::CloudStatisticalData interference_bag_stats =
        point_cloud_statistics::CloudStatisticalData();

    // Instantiate octree-based point cloud change detection class
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(i);

    // Ground Truth Bag
    foreach (rosbag::MessageInstance const m, ground_truth_view)
    {
      sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (msg != NULL)
      {
        PointCloud point_cloud;
        fromROSMsg(*msg, point_cloud);
        *current_msg_cloud_ptr = point_cloud;

        ++ground_truth_bag_stats.point_cloud_msg_count;

        octree.setInputCloud(ground_truth_ptr);
        octree.addPointsFromInputCloud();
        octree.switchBuffers();

        // Add points from cloudB to octree
        octree.setInputCloud(current_msg_cloud_ptr);
        octree.addPointsFromInputCloud();
        std::vector<int> newPointIdxVector;

        // Get vector of point indices from octree voxels which did not exist in previous buffer
        octree.getPointIndicesFromNewVoxels(newPointIdxVector);

        ground_truth_bag_stats.point_count += point_cloud.size();
        ground_truth_bag_stats.outliers_points_count += newPointIdxVector.size();

        octree.deleteTree();
      }
    }

    // Interference Bag
    foreach (rosbag::MessageInstance const m, interference_view)
    {
      sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (msg != NULL)
      {
        PointCloud point_cloud;
        fromROSMsg(*msg, point_cloud);
        *current_msg_cloud_ptr = point_cloud;

        ++interference_bag_stats.point_cloud_msg_count;

        octree.setInputCloud(ground_truth_ptr);
        octree.addPointsFromInputCloud();
        octree.switchBuffers();

        // Add points from cloudB to octree
        octree.setInputCloud(current_msg_cloud_ptr);
        octree.addPointsFromInputCloud();
        std::vector<int> newPointIdxVector;

        // Get vector of point indices from octree voxels which did not exist in previous buffer
        octree.getPointIndicesFromNewVoxels(newPointIdxVector);

        interference_bag_stats.point_count += point_cloud.size();
        interference_bag_stats.outliers_points_count += newPointIdxVector.size();

        octree.deleteTree();
      }
    }

    // Compute Relative percentage of outliers
    ground_truth_bag_stats.computeOutliersRelativeValue();
    interference_bag_stats.computeOutliersRelativeValue();

    // Print statistical results
    std::cout << "\nVoxel edge resolution: " << i << std::endl << "Ground Truth: " << std::endl;
    ground_truth_bag_stats.printStatistics();
    std::cout << "Interference: " << std::endl;
    interference_bag_stats.printStatistics();

    // Generate Data for bar chart
    ground_truth_errors.push_back(ground_truth_bag_stats.getOutliersPercentage());
    interference_errors.push_back(interference_bag_stats.getOutliersPercentage());
  }

  ground_truth_bag.close();  // close ground truth bag file
  interference_bag.close();  // close interference bag

  std::cout << std::endl;

  std::fstream fout;  // file pointer
  std::string interference_csv_stats = point_cloud_statistics::constructFullPathToDataset(argv[1], "interference_"
                                                                                                   "results.csv");
  fout.open(interference_csv_stats, std::ios::out);  // creates a new csv file with writing permission
  for (int i = 0; i < resolution_values.size(); i++)
  {
    fout << resolution_values[i] << ", " << ground_truth_errors[i] / 100.0 << ", " << interference_errors[i] / 100.0
         << "\n";
    std::cout << resolution_values[i] << ", " << ground_truth_errors[i] / 100.0 << ", "
              << interference_errors[i] / 100.0 << std::endl;
  }
  fout.close();

  // Create Bar Plot object with Full HD resolution and the description for the data
  BarChartPlotter* plotter =
      new BarChartPlotter(1920, 1080, "Interference Analysis based on Change Detection using an octree structure",
                          "Voxel edge Resolution", "Outliers/Inliers");

  // Add the outliers of the interfered and ground truth datasets
  plotter->setColorScheme(vtkColorSeries::WARM);
  plotter->addBarPlotData(resolution_values, interference_errors, "Interference Bag vs Ground Truth Model");
  plotter->setColorScheme(vtkColorSeries::BLUES);
  plotter->addBarPlotData(resolution_values, ground_truth_errors, "Ground Truth Bag vs Ground Truth Model");

  plotter->plot();  // holds here until window is given the closing instruction

  // Saves the bar chart as a PNG file on the dataset directory
  std::string bar_chart_filename =
      point_cloud_statistics::constructFullPathToDataset(argv[1], "interference_bar_chart.png").c_str();
  plotter->saveBarChartPNG(bar_chart_filename);
  std::cout << "Saved bar chart on: " << bar_chart_filename << std::endl;

  plotter->close();  // Destroys bar chart object

  return EXIT_SUCCESS;
}
