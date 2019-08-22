#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_plotter.h>
#include <string>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <vector>
#include <iostream>
#include <utility>

#include <pcl/octree/octree_pointcloud_changedetector.h>

#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"

#include <vtkColorSeries.h>
#include <vtkChart.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

inline const std::string constructFullBagPath(const std::string bag_type, std::string dataset_name)
{
  return datasets_path::IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + dataset_name + "/" + bag_type;
}

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
        "<voxel edge intervarl minimum resolution> <voxel edge intervarl maximum resolution>\n"  // 4 arguments
        "USAGE: rosrun point_cloud_statistics point_cloud_change_detection_node <IT2 folder for test scenario> "
        "<voxel edge intervarl minimum resolution> <voxel edge intervarl maximum resolution> <step value>\n"  // 5 args
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

  std::string ground_truth_full_bag_path = constructFullBagPath(datasets_path::GROUND_TRUTH_BAG_NAME, argv[1]);
  std::string interference_full_bag_path = constructFullBagPath(datasets_path::INTERFERENCE_BAG_NAME, argv[1]);

  std::cout << "TEST CONDITIONS:" << std::endl
            << "Voxel edge resolution interval: [" << min_resolution << ", " << max_resolution << "]" << std::endl
            << "With a step of: " << step_value << std::endl
            << "Test folder name is: " << argv[1] << std::endl
            << "Ground Truth Full path: " << ground_truth_full_bag_path << std::endl
            << "Interference Full path: " << interference_full_bag_path << std::endl;

  PointCloud::Ptr interference_cloud_ptr(new PointCloud);
  PointCloud::Ptr ground_truth_ptr(new PointCloud);

  rosbag::Bag bag;
  bag.open(ground_truth_full_bag_path);  // open ground truth bag file

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

  bag.close();  // close ground truth bag file

  std::vector<double> freqdata, resolution_values;

  // \TODO Implement Multithreading here. Could speed up computation speeds but requires multiple copies of the octree
  // stucture
  for (float i = min_resolution; i <= max_resolution; i += step_value)
  {
    resolution_values.push_back(i);
    long int msg_count = 0, point_count = 0, out_points_count = 0, in_points_count = 0;

    // Instantiate octree-based point cloud change detection class
    pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(i);

    bag.open(interference_full_bag_path);  // Open interference bag

    rosbag::View view2(bag, rosbag::TopicQuery(topics));

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

        point_count += point_cloud.size();
        out_points_count += newPointIdxVector.size();

        octree.deleteTree();
      }
    }

    bag.close();  // close interference bag

    float relative_percentage_out_points = (float)(out_points_count) / point_count * 100;

    std::cout << "\nVoxel edge resolution: " << i << std::endl
              << "Number of received Point Cloud Messages: " << msg_count << std::endl
              << "Number of received Point Cloud 3D Points: " << point_count << std::endl
              << "From which " << out_points_count << "(" << relative_percentage_out_points << "%) are interfered"
              << std::endl;

    freqdata.push_back((double)(out_points_count) / point_count);
  }

  std::cout << std::endl;
  for (int i = 0; i < freqdata.size(); ++i)
  {
    std::cout << resolution_values[i] << ", " << freqdata[i] << std::endl;
  }

  pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter("My Plotter");

  plotter->setWindowSize(900, 600);
  plotter->setYTitle("Y axis");
  plotter->setXTitle("X axis");
  plotter->setTitle("My plot");
  plotter->setShowLegend(true);  // show legends
  plotter->setColorScheme(vtkColorSeries::WARM);

  // plotter->addHistogramData(freqdata, (int)((max_resolution - min_resolution) / step_value) + 1,
  //                            "Interference points relative to the number of registered points");
  plotter->addPlotData(resolution_values, freqdata, "Interference points relative to the number of registered points",
                       vtkChart::BAR);
  plotter->plot();
  plotter->spinOnce(2000);

  return EXIT_SUCCESS;
}
