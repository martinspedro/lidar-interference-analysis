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

  std::cout << "GROUND TRUTH GENERATION" << std::endl
            << "Test folder name is: " << argv[1] << std::endl
            << "Ground Truth Full path: " << ground_truth_full_bag_path << std::endl;

  PointCloud::Ptr ground_truth_ptr(new PointCloud);
  // velodyne_pointcloud::PointcloudXYZIR::Ptr ground_truth_ptr(new velodyne_pointcloud::PointcloudXYZIR);

  rosbag::Bag ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));

  int count = 0;

  // Tutorial
  PointCloud::Ptr result(new PointCloud), source(new PointCloud), target(new PointCloud);
  Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity(), pairTransform;

  foreach (rosbag::MessageInstance const m, ground_truth_view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      // VelodynePointCloud ground_truth_point_cloud;
      PointCloud ground_truth_point_cloud;
      fromROSMsg(*msg, ground_truth_point_cloud);
      //*result = ground_truth_point_cloud;

      // std::cout << "(" << msg->width << ", " << msg->height << ")" << std::endl;

      PointCloud::Ptr filtered_point_cloud(new PointCloud);
      *filtered_point_cloud = ground_truth_point_cloud;
      /*
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_out;
      sor_out.setInputCloud(filtered_point_cloud);
      sor_out.setMeanK(50);
      sor_out.setStddevMulThresh(1);
      sor_out.filter(*filtered_point_cloud);
      */

      *target = ground_truth_point_cloud;
      if (count > 0)
      {
        // Add visualization data
        // showCloudsLeft(source, target);

        PointCloud::Ptr temp(new PointCloud);
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
        *source = *filtered_point_cloud;
      }

      std::cout << "Message number: " << count << std::endl;
      if (++count > 10)
      {
        break;
      }
    }
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

  pcl::VoxelGrid<pcl::PointXYZ> sor_voxel;
  sor_voxel.setInputCloud(result);
  sor_voxel.setLeafSize(0.01f, 0.01f, 0.01f);
  sor_voxel.filter(*ground_truth_ptr);

  ss.str(std::string());  // clear stringstream buffer
  ss << point_cloud_statistics::constructFullPathToDataset(argv[1], "voxelized_ground_truth_model.pcd");
  pcl::io::savePCDFile(ss.str(), *ground_truth_ptr, true);

  ss.str(std::string());  // clear stringstream buffer
  ss << point_cloud_statistics::constructFullPathToDataset(argv[1], "ground_truth_model.pcd");
  pcl::io::savePCDFile(ss.str(), *ground_truth_ptr, true);

  ground_truth_bag.close();  // close ground truth bag file

  return EXIT_SUCCESS;
}
