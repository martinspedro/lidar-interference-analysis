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

#include <pcl/visualization/pcl_plotter.h>

#include "point_cloud_statistics/organized_pointcloud.hpp"
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"

typedef pcl::PointCloud<velodyne_pointcloud::PointXYZIR> VelodynePointCloud;

//  template class organized_pointcloud::OrganizedPointCloud<velodyne_pointcloud::PointXYZIR>;  // forward declaration
//  of

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_interference_analysis_node");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
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

  std::cout << "TEST CONDITIONS:" << std::endl
            << "Test folder name is: " << argv[1] << std::endl
            << "Ground Truth Full path: " << ground_truth_full_bag_path << std::endl
            << "Interference Full path: " << interference_full_bag_path << std::endl;

  std::vector<double> ground_truth_errors, interference_errors, resolution_values;
  std::vector<double>* ground_truth_errors_ptr;
  std::vector<double>* interference_errors_ptr;
  std::cout << "Vectors created:" << std::endl;

  //*ground_truth_errors_ptr = ground_truth_errors;
  //*interference_errors_ptr = interference_errors;
  std::cout << "Vectors assigned:" << std::endl;

  // organized_pointcloud::
  OrganizedPointCloud<velodyne_pointcloud::PointXYZIR> ground_truth_model(AZIMUTHAL_UNIQUE_ANGLES_COUNT,
                                                                          VLP16_LASER_COUNT);
  // organized_pointcloud::
  OrganizedPointCloud<velodyne_pointcloud::PointXYZIR> ground_truth_cloud(AZIMUTHAL_UNIQUE_ANGLES_COUNT,
                                                                          VLP16_LASER_COUNT);
  // organized_pointcloud::
  OrganizedPointCloud<velodyne_pointcloud::PointXYZIR> interference_cloud(AZIMUTHAL_UNIQUE_ANGLES_COUNT,
                                                                          VLP16_LASER_COUNT);

  VelodynePointCloud::Ptr current_msg_cloud_ptr(new VelodynePointCloud);
  VelodynePointCloud::Ptr ground_truth_ptr(new VelodynePointCloud);

  // Load Ground Truth Model
  std::string ground_truth_pcd = point_cloud_statistics::constructFullPathToDataset(argv[1], "ground_truth_model.pcd");
  pcl::io::loadPCDFile<velodyne_pointcloud::PointXYZIR>(ground_truth_pcd, *ground_truth_ptr);

  ground_truth_model.organizeVelodynePointCloud(*ground_truth_ptr);
  std::cout << "Finishung" << std::endl;
  rosbag::Bag interference_bag, ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));

  interference_bag.open(interference_full_bag_path);  // Open interference bag

  rosbag::View interference_view(interference_bag, rosbag::TopicQuery(topics));
  std::cout << "GROUND TRUTH" << std::endl;
  foreach (rosbag::MessageInstance const m, ground_truth_view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      VelodynePointCloud point_cloud;
      fromROSMsg(*msg, point_cloud);
      *current_msg_cloud_ptr = point_cloud;
      // std::cout << "Read" << std::endl;
      ground_truth_cloud.organizeVelodynePointCloud(*current_msg_cloud_ptr);
      // std::cout << "Organized" << std::endl;
      ground_truth_cloud.computeDistanceBetweenPointClouds(ground_truth_model, ground_truth_errors);
      // std::cout << "Computed" << std::endl;
      ground_truth_cloud.clearPointsFromPointcloud();
      // std::cout << "Cleared" << std::endl;
    }
  }
  std::cout << "Print Ground Truth " << std::endl;
  /*
    for (int i = 0; i < ground_truth_errors.size(); ++i)
    {
      std::cout << ground_truth_errors[i] << std::endl;
    }
  */
  std::cout << "Ground truth done " << std::endl;
  foreach (rosbag::MessageInstance const m, interference_view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      VelodynePointCloud point_cloud;
      fromROSMsg(*msg, point_cloud);
      *current_msg_cloud_ptr = point_cloud;

      // std::cout << "Read I" << std::endl;
      interference_cloud.organizeVelodynePointCloud(*current_msg_cloud_ptr);
      // std::cout << "Organized I" << std::endl;
      interference_cloud.computeDistanceBetweenPointClouds(ground_truth_model, interference_errors);
      // std::cout << "Computed I " << std::endl;
      interference_cloud.clearPointsFromPointcloud();
      // std::cout << "ClearedI " << std::endl;
    }
  }
  std::cout << "Bags Read" << std::endl;

  // std::vector<double> ground_truth_errors = ground_truth_errors, interference_errors = interference_errors;
  // std::cout << "Finishung" << std::endl;

  // long long unsigned int mean = 0;
  // long long unsigned int msg_num_interference = 0;

  // Instantiate octree-based point cloud change detection class
  /*
    for (int i = 0; i < ground_truth_ptr->size(); ++i)
    {
      ground_truth_cloud[ground_truth_ptr->points[i].ring]
                        [getAzimuthIndex(ground_truth_ptr->points[i].y, ground_truth_ptr->points[i].x)] =
                            computeEuclideanDistance(ground_truth_ptr->points[i].x, ground_truth_ptr->points[i].y,
                                                     ground_truth_ptr->points[i].z);
    }

    // Ground Truth Bag
    foreach (rosbag::MessageInstance const m, ground_truth_view)
    {
      sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
      if (msg != NULL)
      {
        VelodynePointCloud point_cloud;
        fromROSMsg(*msg, point_cloud);
        *current_msg_cloud_ptr = point_cloud;

        mean += current_msg_cloud_ptr->size();
        //++msg_num;

        for (int i = 0; i < current_msg_cloud_ptr->size(); ++i)
        {
          organized_cloud_ground_truth[current_msg_cloud_ptr->points[i].ring][getAzimuthIndex(
              current_msg_cloud_ptr->points[i].y, current_msg_cloud_ptr->points[i].x)] =
              computeEuclideanDistance(current_msg_cloud_ptr->points[i].x, current_msg_cloud_ptr->points[i].y,
                                       current_msg_cloud_ptr->points[i].z);
        }

        for (int i = 0; i < 15; ++i)
        {
          for (int j = 0; j < 1800; ++j)
          {
            // std::cout << abs(ground_truth_cloud[i][j] - organized_cloud_ground_truth[i][j]) << std::endl;
            ground_truth_errors.push_back(abs(ground_truth_cloud[i][j] - organized_cloud_ground_truth[i][j]));
          }
        }

        for (int i = 0; i < 15; ++i)
        {
          for (int j = 0; j < 1800; ++j)
          {
            organized_cloud_ground_truth[i][j] = std::numeric_limits<double>::quiet_NaN();
          }
        }
      }
    }

    // Interference Bag
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

        for (int i = 0; i < current_msg_cloud_ptr->size(); ++i)
        {
          organized_cloud_interference[current_msg_cloud_ptr->points[i].ring][getAzimuthIndex(
              current_msg_cloud_ptr->points[i].y, current_msg_cloud_ptr->points[i].x)] =
              computeEuclideanDistance(current_msg_cloud_ptr->points[i].x, current_msg_cloud_ptr->points[i].y,
                                       current_msg_cloud_ptr->points[i].z);
        }

        for (int i = 0; i < 15; ++i)
        {
          for (int j = 0; j < 1800; ++j)
          {
            // std::cout << abs(ground_truth_cloud[i][j] - organized_cloud_ground_truth[i][j]) << std::endl;
            interference_errors.push_back(abs(ground_truth_cloud[i][j] - organized_cloud_interference[i][j]));
          }
        }

        for (int i = 0; i < 15; ++i)
        {
          for (int j = 0; j < 1800; ++j)
          {
            organized_cloud_interference[i][j] = std::numeric_limits<double>::quiet_NaN();
          }
        }
      }
    }


    std::cout << "Nº messages received: " << msg_num_interference << std::endl
              << "Nº points measures" << mean << std::endl
              << "Average Points per message: " << (1.0d * mean) / msg_num_interference << std::endl;
  */
  /*
    for (int i = 0; i < 15; ++i)
    {
      for (int j = 0; j < 1800; ++j)
      {
        // std::cout << abs(ground_truth_cloud[i][j] - organized_cloud_ground_truth[i][j]) << std::endl;
        ground_truth_errors.push_back(abs(ground_truth_cloud[i][j] - organized_cloud_ground_truth[i][j]));
        std::cout << ground_truth_cloud[i][j] << " - " << organized_cloud_interference[i][j] << " = "
                  << abs(ground_truth_cloud[i][j] - organized_cloud_interference[i][j]) << std::endl;
        interference_errors.push_back(abs(ground_truth_cloud[i][j] - organized_cloud_interference[i][j]));
      }
    }
  */
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
    /*
  std::cout << interference_errors[i] << std::endl;
  std::cout << interference_errors[i] * 10.0d << std::endl;
  std::cout << floor(interference_errors[i] * 10.0d) << std::endl;
  std::cout << (unsigned int)(floor(interference_errors[i] * 10.0d)) << std::endl;
  */
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
  /*
  for (int i = 0; i < 1000; ++i)
  {
    relative_freq_ground[i] = relative_freq_ground[i] == 0 ? 0 : abs(log10(relative_freq_ground[i]));
    relative_freq_interference[i] = relative_freq_interference[i] == 0 ? 0 :
  abs(log10(relative_freq_interference[i]));
  }
  */

  std::cout << "done" << std::endl;
  ground_truth_bag.close();  // close ground truth bag file
  interference_bag.close();  // close interference bag

  std::cout << "Close" << std::endl;
  /*
    pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter("Ground");
    plotter->setTitle("Ground");  // global title
    plotter->setXTitle("Absolute Distance");
    plotter->setYTitle("Logarithmic");
    plotter->setShowLegend(true);  // show legends

    plotter->addHistogramData(ground_truth_errors, 200, "Ground_Truth_Comparison");  // number of bins are 10
    // plotter->setXRange(0, 20);
    plotter->plot();

    pcl::visualization::PCLPlotter* plotter2 = new pcl::visualization::PCLPlotter("Interference");
    plotter2->setTitle("Interference");  // global title
    plotter2->setXTitle("Absolute Distance");
    plotter2->setYTitle("Logarithmic");
    plotter2->setShowLegend(true);  // show legends

    plotter2->addHistogramData(interference_errors, 200, "Interference_Comparison");  // number of bins are 10
    // plotter2->setXRange(0, 20);

    plotter2->plot();
  */
  for (int i = 0; i < 1000; ++i)
  {
    relative_valid_points_ground[i] = relative_freq_ground[i] / total_valid_points_ground;
    relative_valid_points_interference[i] = relative_freq_interference[i] / total_valid_points_interference;

    if (i < 50)
    {
      std::cout << i * 0.1 << ": " << relative_valid_points_ground[i] << " | " << relative_valid_points_interference[i]
                << std::endl;
    }
  }

  double interference = 0;
  for (int i = 1; i < 1000; ++i)
  {
    interference += (double)relative_freq_interference[i];
  }

  std::cout << "Interference: " << interference / total_valid_points_interference << std::endl;

  // Create Bar Plot object with Full HD resolution and the description for the data
  BarChartPlotter* plotter3 =
      new BarChartPlotter(1920, 1080, "Interference Analysis based on Change Detection using an octree structure",
                          "Voxel edge Resolution", "Outliers/Inliers");

  // Add the outliers of the interfered and ground truth datasets
  plotter3->setColorScheme(vtkColorSeries::WARM);
  plotter3->addBarPlotData(x_axis, relative_freq_interference, 1000, "Interference Bag vs Ground Truth Model");
  plotter3->setColorScheme(vtkColorSeries::BLUES);
  plotter3->addBarPlotData(x_axis, relative_freq_ground, 1000, "Ground Truth Bag vs Ground Truth Model");

  plotter3->plot();  // holds here until window is given the closing instruction

  // Saves the bar chart as a PNG file on the dataset directory
  std::string bar_chart_filename = point_cloud_statistics::constructFullPathToDataset(argv[1], "chart.png").c_str();
  plotter3->saveBarChartPNG(bar_chart_filename);
  std::cout << "Saved bar chart on: " << bar_chart_filename << std::endl;

  plotter3->close();  // Destroys bar chart object

  return EXIT_SUCCESS;
}
