/**
 * \file   ground_truth_model_estimation_node.cpp
 * \brief
 *
 */

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/features/normal_3d.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

// Datasets related includes
#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"
#include "point_cloud_statistics/point_cloud_statistics.hpp"

#include <velodyne_pointcloud/point_types.h>
typedef pcl::PointCloud<velodyne_pointcloud::PointXYZIR> VelodynePointCloud;

#include "point_cloud_statistics/organized_pointcloud.hpp"
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"
#include "point_cloud_statistics/point_register.hpp"

#include "matplotlib-cpp/matplotlibcpp.h"

struct intensity
{
  float mean;
  float variance;
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
  // define Velodyne related constants
  const unsigned int VLP16_LASER_COUNT = 16u;
  const float AZIMUTHAL_ANGULAR_RESOLUTION = 0.2F;
  const unsigned int AZIMUTHAL_UNIQUE_ANGLES_COUNT =
      (unsigned int)ceil(point_cloud::organized::FULL_REVOLUTION_DEGREE_F / AZIMUTHAL_ANGULAR_RESOLUTION);

  std::string ground_truth_full_bag_path =
      point_cloud_statistics::constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_BAG_NAME);

  ROS_INFO_STREAM("\nGROUND TRUTH GENERATION: " << std::endl
                                                << "- Test folder name is: " << argv[1] << std::endl
                                                << "- Ground Truth Full path: " << ground_truth_full_bag_path
                                                << std::endl);

  point_cloud::organized::OrganizedPointCloud<PointRegister<velodyne_pointcloud::PointXYZIR> > ground_truth_dataset(
      AZIMUTHAL_UNIQUE_ANGLES_COUNT, VLP16_LASER_COUNT);
  point_cloud::organized::OrganizedPointCloud<velodyne_pointcloud::PointXYZIR> ground_truth_model(
      AZIMUTHAL_UNIQUE_ANGLES_COUNT, VLP16_LASER_COUNT);

  VelodynePointCloud::Ptr ground_truth_ptr(new VelodynePointCloud);
  // PointCloud::Ptr ground_truth_ptr(new PointCloud);

  // velodyne_pointcloud::PointcloudXYZIR::Ptr ground_truth_ptr(new velodyne_pointcloud::PointcloudXYZIR);

  rosbag::Bag ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));

  int count = 0;

  // Tutorial
  VelodynePointCloud::Ptr result(new VelodynePointCloud), source(new VelodynePointCloud),
      target(new VelodynePointCloud);
  // PointCloud::Ptr result(new PointCloud), source(new PointCloud), target(new PointCloud);
  // Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

  foreach (rosbag::MessageInstance const m, ground_truth_view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      VelodynePointCloud ground_truth_point_cloud;
      // PointCloud ground_truth_point_cloud;
      fromROSMsg(*msg, ground_truth_point_cloud);
      *ground_truth_ptr = ground_truth_point_cloud;

      ground_truth_dataset.registerVelodynePointCloud(*ground_truth_ptr);

      // ground_truth_model.computeDistanceBetweenPointClouds(ground_truth_model, ground_truth_errors);
      // ground_truth_model.clearPointsFromPointcloud();

      //*result = ground_truth_point_cloud;
      // std::cout << "(" << msg->width << ", " << msg->height << ")" << std::endl;
      /*
      VelodynePointCloud::Ptr filtered_point_cloud(new VelodynePointCloud);
      *filtered_point_cloud = ground_truth_point_cloud;
      */

      /*
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_out;
      sor_out.setInputCloud(filtered_point_cloud);
      sor_out.setMeanK(50);
      sor_out.setStddevMulThresh(1);
      sor_out.filter(*filtered_point_cloud);
      */
      /*
      *target = ground_truth_point_cloud;
      if (count > 0)
      {
        // Add visualization data
        // showCloudsLeft(source, target);

        // VelodynePointCloud::Ptr temp(new VelodynePointCloud);
        // point_cloud_statistics::pairAlign(source, target, temp, pairTransform, true);
        *result = point_cloud_statistics::icp(source, target);
        // transform current pair into the global transform
        // pcl::transformPointCloud(*temp, *result, pairTransform);

        // update the global transform
        // GlobalTransform *= pairTransform;

        *source = *result;
      }
      else
      {
        *source = ground_truth_point_cloud;
      }

      std::cout << "Message number: " << count << std::endl;
      if (++count > 10)
      {
        break;
      }
      */
    }
  }

  ground_truth_bag.close();  // close ground truth bag file

  std::vector<intensity> azimuth(AZIMUTHAL_UNIQUE_ANGLES_COUNT);
  std::vector<intensity> laser(VLP16_LASER_COUNT);
  ground_truth_dataset.computeStats<velodyne_pointcloud::PointXYZIR, intensity>(azimuth, laser);

  ground_truth_model.generateModel<PointRegister<velodyne_pointcloud::PointXYZIR> >(ground_truth_dataset);

  for (int i = 0; i < azimuth.size(); ++i)
  {
    std::cout << i << ": " << azimuth[i].mean << std::endl;
  }

  for (int i = 0; i < laser.size(); ++i)
  {
    std::cout << i << ": " << laser[i].mean << std::endl;
  }

  // save aligned pair, transformed into the first cloud's frame
  std::stringstream ss;
  // ss << point_cloud_statistics::constructFullPathToDataset(argv[1], "ground_truth_model.pcd");
  // pcl::io::savePCDFile(ss.str(), *result, true);

  /*
    float average = (double)sum / count;
    float variance = ((double)sum_squared - sum * sum) / count;
    std::cout << "Sum: " << sum << std::endl;
    std::cout << "Average: " << average << std::endl;
    std::cout << "Variance: " << variance << std::endl;
    std::cout << "Number of clouds: " << count << std::endl;
    std::cout << result->size() << std::endl;
  */

  // VOXEL GRID FILTER
  // Create the filtering object
  /*
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_out;
    sor_out.setInputCloud(result);
    sor_out.setMeanK(50);
    sor_out.setStddevMulThresh(1);
    sor_out.filter(*ground_truth_ptr);

    ss.str(std::string());  // clear stringstream buffer
    ss << point_cloud_statistics::constructFullPathToDataset(argv[1], "filtered_ground_truth_model.pcd");
    pcl::io::savePCDFile(ss.str(), *ground_truth_ptr, true);
  */
  // Create the filtering object

  /* pcl::VoxelGrid<velodyne_pointcloud::PointXYZIR> sor_voxel;
  pcl::VoxelGrid<pcl::PointXYZ> sor_voxel;
  sor_voxel.setInputCloud(result);
  sor_voxel.setLeafSize(0.05f, 0.05f, 0.05f);
  sor_voxel.filter(*ground_truth_ptr);

  ss.str(std::string());  // clear stringstream buffer
  ss << point_cloud_statistics::constructFullPathToDataset(argv[1], "voxelized_ground_truth_model.pcd");
  pcl::io::savePCDFile(ss.str(), *ground_truth_ptr, true);
*/
  ss.str(std::string());  // clear stringstream buffer
  ss << point_cloud_statistics::constructFullPathToDataset(argv[1], "ground_truth_model_new.pcd");
  pcl::io::savePCDFile(ss.str(), ground_truth_model, true);

  std::cout << "Ground Truth Model saved on "
            << point_cloud_statistics::constructFullPathToDataset(argv[1], "ground_truth_model_new.pcd") << std::endl;

  std::vector<std::vector<float> > x, y, z;

  for (int i = 0; i < AZIMUTHAL_UNIQUE_ANGLES_COUNT; ++i)
  {
    std::vector<float> x_row, y_row, z_row;
    for (int j = 0; j < VLP16_LASER_COUNT; ++j)
    {
      x_row.push_back(i);
      y_row.push_back(j);
      z_row.push_back(ground_truth_dataset.at(i, j).distance_var);
    }
    x.push_back(x_row);
    y.push_back(y_row);
    z.push_back(z_row);
  }

  matplotlibcpp::plot_surface(x, y, z);
  matplotlibcpp::show();

  return EXIT_SUCCESS;
}
