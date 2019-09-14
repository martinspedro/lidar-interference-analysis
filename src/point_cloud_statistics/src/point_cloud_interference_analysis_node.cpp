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

#define _USE_MATH_DEFINES  // Define to use PI
#include <math.h>

#include <pcl/visualization/pcl_plotter.h>

typedef pcl::PointCloud<velodyne_pointcloud::PointXYZIR> VelodynePointCloud;

double getAzimuth(double y, double x)
{
  return atan2(y, x) * 180.0d / (double)(M_PI);
}

unsigned int getAzimuthIndex(double y, double x)
{
  double shifted_azimuth = getAzimuth(y, x) + 180.0d;
  double fixed_point_shifted_azimuth = floor(shifted_azimuth * 10.0d) / 10.0d;
  double index = fixed_point_shifted_azimuth * 1800 / 360;
  return (unsigned int)(index);

  // std::cout << azimuth * 1800 / 360.0 << std::endl;
}

double computeEuclideanDistance(double x, double y, double z)
{
  return std::sqrt(x * x + y * y + z * z);
}

double computeEuclideanDistance(double x_1, double y_1, double z_1, double x_2, double y_2, double z_2)
{
  double x_diff = x_2 - x_1;
  double y_diff = y_2 - y_1;
  double z_diff = z_2 - z_1;

  return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_change_detection_node");
  /*
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
  */
  std::string ground_truth_full_bag_path =
      point_cloud_statistics::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_BAG_NAME);
  std::string interference_full_bag_path =
      point_cloud_statistics::constructFullPathToDataset(argv[1], datasets_path::INTERFERENCE_BAG_NAME);

  std::cout << "TEST CONDITIONS:"
            << std::endl
            //<< "Voxel edge resolution interval: [" << min_resolution << ", " << max_resolution << "]" << std::endl
            //<< "With a step of: " << step_value << std::endl
            << "Test folder name is: " << argv[1] << std::endl
            << "Ground Truth Full path: " << ground_truth_full_bag_path << std::endl
            << "Interference Full path: " << interference_full_bag_path << std::endl;

  double organized_cloud_ground_truth[16][1800];
  double organized_cloud_interference[16][1800];
  double ground_truth_cloud[16][1800];
  std::vector<double> distance_ground, distance_interference;

  VelodynePointCloud::Ptr current_msg_cloud_ptr(new VelodynePointCloud);
  VelodynePointCloud::Ptr ground_truth_ptr(new VelodynePointCloud);
  // velodyne_pointcloud::PointcloudXYZIR::Ptr ground_truth_ptr(new velodyne_pointcloud::PointcloudXYZIR);

  rosbag::Bag interference_bag, ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));

  std::string ground_truth_pcd = point_cloud_statistics::constructFullPathToDataset(argv[1], "ground_truth_model.pcd");
  pcl::io::loadPCDFile<velodyne_pointcloud::PointXYZIR>(ground_truth_pcd, *ground_truth_ptr);

  std::vector<double> ground_truth_errors, interference_errors, resolution_values;

  interference_bag.open(interference_full_bag_path);  // Open interference bag

  rosbag::View interference_view(interference_bag, rosbag::TopicQuery(topics));

  // \TODO Implement Multithreading here. Could speed up computation speeds but requires multiple copies of the octree
  // stucture
  /// for (float i = min_resolution; i <= max_resolution; i += step_value)
  //{
  // resolution_values.push_back(i);

  point_cloud_statistics::CloudStatisticalData ground_truth_bag_stats = point_cloud_statistics::CloudStatisticalData();
  point_cloud_statistics::CloudStatisticalData interference_bag_stats = point_cloud_statistics::CloudStatisticalData();

  long long unsigned int mean = 0;
  long long unsigned int msg_num = 0;

  // Instantiate octree-based point cloud change detection class
  // pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(i);

  for (int i = 0; i < ground_truth_ptr->size(); ++i)
  {
    ground_truth_cloud[ground_truth_ptr->points[i].ring]
                      [getAzimuthIndex(ground_truth_ptr->points[i].y, ground_truth_ptr->points[i].x)] =
                          computeEuclideanDistance(ground_truth_ptr->points[i].x, ground_truth_ptr->points[i].y,
                                                   ground_truth_ptr->points[i].z);
    /*
    std::cout << "Laser ID: " << current_msg_cloud_ptr->points[i].ring << " - "
              << atan(current_msg_cloud_ptr->points[i].y / current_msg_cloud_ptr->points[i].x) * 180.0d / M_PI
              << std::endl;
              */
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

      // std::cout << "Ground Truth" << std::endl;
      // for (int i = 0; i < current_msg_cloud_ptr->size(); ++i)

      // std::cout << current_msg_cloud_ptr->size() << std::endl;
      mean += current_msg_cloud_ptr->size();
      ++msg_num;

      for (int i = 0; i < current_msg_cloud_ptr->size(); ++i)
      {
        organized_cloud_ground_truth[current_msg_cloud_ptr->points[i].ring][getAzimuthIndex(
            current_msg_cloud_ptr->points[i].y, current_msg_cloud_ptr->points[i].x)] =
            computeEuclideanDistance(current_msg_cloud_ptr->points[i].x, current_msg_cloud_ptr->points[i].y,
                                     current_msg_cloud_ptr->points[i].z);
        /*
        std::cout << "Laser ID: " << current_msg_cloud_ptr->points[i].ring << " - "
                  << atan(current_msg_cloud_ptr->points[i].y / current_msg_cloud_ptr->points[i].x) * 180.0d / M_PI
                  << std::endl;
                  */
      }
      /*
      for (int i = 0; i < 30; ++i)
      {


                  std::cout << "Laser ID: " << current_msg_cloud_ptr->points[i].ring << " - "
                            << atan(current_msg_cloud_ptr->points[i].y / current_msg_cloud_ptr->points[i].x) * 180 /
           M_PI
                            << std::endl;

      }
      */
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

      ++interference_bag_stats.point_cloud_msg_count;
      // std::cout << current_msg_cloud_ptr->size() << std::endl;
      // std::cout << "Interference" << std::endl;
      // for (int i = 0; i < current_msg_cloud_ptr->size(); ++i)
      mean += current_msg_cloud_ptr->size();
      ++msg_num;

      for (int i = 0; i < current_msg_cloud_ptr->size(); ++i)
      {
        organized_cloud_interference[current_msg_cloud_ptr->points[i].ring][getAzimuthIndex(
            current_msg_cloud_ptr->points[i].y, current_msg_cloud_ptr->points[i].x)] =
            computeEuclideanDistance(current_msg_cloud_ptr->points[i].x, current_msg_cloud_ptr->points[i].y,
                                     current_msg_cloud_ptr->points[i].z);
        /*
        std::cout << "Laser ID: " << current_msg_cloud_ptr->points[i].ring << " - "
                  << atan(current_msg_cloud_ptr->points[i].y / current_msg_cloud_ptr->points[i].x) * 180.0d / M_PI
                  << std::endl;
                  */
      }
    }
  }

  // break;
  // }
  /*
    for (int i = 0; i < 16; ++i)
    {
      std::cout << "---------------------- Ring " << i << "-----------------------" << std::endl;
      // sort(organized_cloud[i].begin(), organized_cloud[i].end());
      for (int j = 0; j < 1800; ++j)
      {
        std::cout << "#" << j << ": " << organized_cloud[i][j] << std::endl;
        // std::cout << "(x, y, z, i): (" << organized_cloud[i][j] << ", " << organized_cloud[i][j].y << ", "
        //            << organized_cloud[i][j].z << ", " << organized_cloud[i][j].intensity << ") " << std::endl;
      }
    }
*/
  std::cout << "Nº messages received: " << msg_num << std::endl
            << "Nº points measures" << mean << std::endl
            << "Average Points per message: " << (1.0d * mean) / msg_num << std::endl;

  for (int i = 0; i < 15; ++i)
  {
    for (int j = 0; j < 1800; ++j)
    {
      // std::cout << abs(ground_truth_cloud[i][j] - organized_cloud_ground_truth[i][j]) << std::endl;
      distance_ground.push_back(abs(ground_truth_cloud[i][j] - organized_cloud_ground_truth[i][j]));
      distance_interference.push_back(abs(ground_truth_cloud[i][j] - organized_cloud_interference[i][j]));
    }
  }
  std::cout << "done" << std::endl;
  ground_truth_bag.close();  // close ground truth bag file
  interference_bag.close();  // close interference bag

  pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter("Ground");
  plotter->setTitle("Groubnd");  // global title
  plotter->setXTitle("Absolute Distance");
  plotter->setYTitle("Logarithmic");
  plotter->setShowLegend(true);  // show legends

  plotter->addHistogramData(distance_ground, 200, "Ground_Truth_Comparison");  // number of bins are 10
  plotter->plot();

  pcl::visualization::PCLPlotter* plotter2 = new pcl::visualization::PCLPlotter("Interference");
  plotter2->setTitle("Interference");  // global title
  plotter2->setXTitle("Absolute Distance");
  plotter2->setYTitle("Logarithmic");
  plotter2->setShowLegend(true);  // show legends

  plotter2->addHistogramData(distance_interference, 200, "Interference_Comparison");  // number of bins are 10
  plotter2->plot();

  return EXIT_SUCCESS;
}
