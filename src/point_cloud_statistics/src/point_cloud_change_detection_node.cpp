#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <string>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <vector>
#include <iostream>
#include <utility>

#define VIEWER_NAME "Simple Cloud Visualizer"

#include <pcl/octree/octree_pointcloud_changedetector.h>

#include <iostream>
#include <vector>
#include <ctime>

#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_histogram");

  PointCloud::Ptr interference_cloud_ptr(new PointCloud);
  PointCloud::Ptr ground_truth_ptr(new PointCloud);

  // define the edge
  float resolution = atof(argv[2]);
  std::cout << "Octree voxel edge resolution (m): " << resolution << std::endl;

  // Instantiate octree-based point cloud change detection class
  pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);

  rosbag::Bag bag;
  std::cout << "Opening rosbag"
            << datasets_path::IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + argv[1] + "/" +
                   datasets_path::GROUND_TRUTH_BAG_NAME
            << std::endl;
  bag.open(datasets_path::IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + argv[1] + "/" +
           datasets_path::GROUND_TRUTH_BAG_NAME);

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  foreach (rosbag::MessageInstance const m, view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      PointCloud ground_truth_point_cloud;
      fromROSMsg(*msg, ground_truth_point_cloud);
      *ground_truth_ptr = ground_truth_point_cloud;
      break;
    }
  }

  bag.close();

  std::cout << "Opening rosbag"
            << datasets_path::IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + argv[1] + "/" +
                   datasets_path::INTERFERENCE_BAG_NAME
            << std::endl;
  bag.open(datasets_path::IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + argv[1] + "/" +
           datasets_path::INTERFERENCE_BAG_NAME);

  rosbag::View view2(bag, rosbag::TopicQuery(topics));
  long int msg_count = 0, point_count = 0, out_points_count = 0, in_points_count = 0;

  foreach (rosbag::MessageInstance const m, view2)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();

    if (msg != NULL)
    {
      PointCloud point_cloud;
      fromROSMsg(*msg, point_cloud);
      *interference_cloud_ptr = point_cloud;

      ++msg_count;

      octree.setInputCloud(ground_truth_ptr);
      octree.addPointsFromInputCloud();
      octree.switchBuffers();

      // Add points from cloudB to octree
      octree.setInputCloud(interference_cloud_ptr);
      octree.addPointsFromInputCloud();
      std::vector<int> newPointIdxVector;

      // Get vector of point indices from octree voxels which did not exist in previous buffer
      octree.getPointIndicesFromNewVoxels(newPointIdxVector);

      // Output points
      /*
      std::cout << "Output from getPointIndicesFromNewVoxels:" << std::endl;
      for (size_t i = 0; i < newPointIdxVector.size(); ++i)
        std::cout << i << "# Index:" << newPointIdxVector[i] << "  Point:" << point_cloud.points[newPointIdxVector[i]].x
                  << " " << point_cloud.points[newPointIdxVector[i]].y << " "
                  << point_cloud.points[newPointIdxVector[i]].z << std::endl;
      */
      point_count += point_cloud.size();

      out_points_count += newPointIdxVector.size();
      octree.deleteTree();
    }
  }
  bag.close();

  float relative_percentage_out_points = (float)(out_points_count) / point_count * 100;

  std::cout << "Number of received Point Cloud Messages: " << msg_count << std::endl
            << "Number of received Point Cloud 3D Points: " << point_count << std::endl
            << "From which " << out_points_count << "(" << relative_percentage_out_points << "%) are interfered"
            << std::endl;

  return EXIT_SUCCESS;
}
