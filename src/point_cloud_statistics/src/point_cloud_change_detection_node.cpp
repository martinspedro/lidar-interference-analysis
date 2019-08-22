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

#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>

#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

inline const std::string constructFullPathToDataset(const std::string dataset_name, const std::string file_name)
{
  return datasets_path::IT2_DARK_ROOM_SCENARIO_B1_INTERFERENCE_FOLDER_FULL_PATH + dataset_name + "/" + file_name;
}

struct StatisticalData
{
  long int point_cloud_msg_count;
  long int point_count;
  long int outliers_points_count;
  long int inliers_points_count;

  double relative_percentage_out_points;

  StatisticalData()
  {
    point_cloud_msg_count = 0;
    point_count = 0;
    outliers_points_count = 0;
    inliers_points_count = 0;

    relative_percentage_out_points = 0.0;
  }

  void printStatistics()
  {
    std::cout << "Number of received Point Cloud Messages: " << point_cloud_msg_count << std::endl
              << "Number of received Point Cloud 3D Points: " << point_count << std::endl
              << "From which " << outliers_points_count << " (" << relative_percentage_out_points << "%) are interfered"
              << std::endl;
  }
};

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

  std::string ground_truth_full_bag_path = constructFullPathToDataset(argv[1], datasets_path::GROUND_TRUTH_BAG_NAME);
  std::string interference_full_bag_path = constructFullPathToDataset(argv[1], datasets_path::INTERFERENCE_BAG_NAME);

  std::cout << "TEST CONDITIONS:" << std::endl
            << "Voxel edge resolution interval: [" << min_resolution << ", " << max_resolution << "]" << std::endl
            << "With a step of: " << step_value << std::endl
            << "Test folder name is: " << argv[1] << std::endl
            << "Ground Truth Full path: " << ground_truth_full_bag_path << std::endl
            << "Interference Full path: " << interference_full_bag_path << std::endl;

  PointCloud::Ptr current_msg_cloud_ptr(new PointCloud);
  PointCloud::Ptr ground_truth_ptr(new PointCloud);

  rosbag::Bag interference_bag, ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));

  // generate ground truth model for test scenario
  foreach (rosbag::MessageInstance const m, ground_truth_view)
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

  std::vector<double> ground_truth_errors, interference_errors, resolution_values;

  interference_bag.open(interference_full_bag_path);  // Open interference bag

  rosbag::View interference_view(interference_bag, rosbag::TopicQuery(topics));

  // \TODO Implement Multithreading here. Could speed up computation speeds but requires multiple copies of the octree
  // stucture
  for (float i = min_resolution; i <= max_resolution; i += step_value)
  {
    resolution_values.push_back(i);

    StatisticalData ground_truth_bag_stats = StatisticalData();
    StatisticalData interference_bag_stats = StatisticalData();

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
    ground_truth_bag_stats.relative_percentage_out_points =
        (double)(ground_truth_bag_stats.outliers_points_count) / ground_truth_bag_stats.point_count * 100;
    interference_bag_stats.relative_percentage_out_points =
        (double)(interference_bag_stats.outliers_points_count) / interference_bag_stats.point_count * 100;

    // Print statistical results
    std::cout << "\nVoxel edge resolution: " << i << std::endl << "Ground Truth: " << std::endl;
    ground_truth_bag_stats.printStatistics();
    std::cout << "Interference: " << std::endl;
    interference_bag_stats.printStatistics();

    // Generate Data for bar chart
    ground_truth_errors.push_back((double)(ground_truth_bag_stats.outliers_points_count) /
                                  ground_truth_bag_stats.point_count);
    interference_errors.push_back((double)(interference_bag_stats.outliers_points_count) /
                                  interference_bag_stats.point_count);
  }

  ground_truth_bag.close();  // close ground truth bag file
  interference_bag.close();  // close interference bag

  std::cout << std::endl;
  for (int i = 0; i < ground_truth_errors.size(); ++i)
  {
    std::cout << resolution_values[i] << ", " << ground_truth_errors[i] << ", " << interference_errors[i] << std::endl;
  }

  pcl::visualization::PCLPlotter* plotter = new pcl::visualization::PCLPlotter();

  plotter->setWindowSize(1920, 1080);
  plotter->setTitle("Interference Analysis based on Change Detection using an octree structure");
  plotter->setXTitle("Voxel edge Resolution");
  plotter->setYTitle("Outliers/Inliers");

  plotter->setShowLegend(true);  // show legends

  plotter->setColorScheme(vtkColorSeries::WARM);
  plotter->addPlotData(resolution_values, interference_errors, "Interference Bag vs Ground Truth Model", vtkChart::BAR);

  plotter->setColorScheme(vtkColorSeries::COOL);
  plotter->addPlotData(resolution_values, ground_truth_errors, "Ground Truth Bag vs Ground Truth Model", vtkChart::BAR);

  plotter->plot();  // holds here until window is given the closing instruction

  vtkSmartPointer<vtkRenderWindow> plotter_render_window_ptr = plotter->getRenderWindow();

  vtkSmartPointer<vtkWindowToImageFilter> window_to_image_filter = vtkSmartPointer<vtkWindowToImageFilter>::New();
  window_to_image_filter->SetInput(plotter_render_window_ptr);
  window_to_image_filter->SetInputBufferTypeToRGBA();  // also record the alpha (transparency) channel
  window_to_image_filter->ReadFrontBufferOff();        // read from the back buffer
  window_to_image_filter->Update();

  vtkSmartPointer<vtkPNGWriter> writer = vtkSmartPointer<vtkPNGWriter>::New();
  writer->SetFileName(constructFullPathToDataset(argv[1], std::string(argv[1]) + ".png").c_str());
  writer->SetInputConnection(window_to_image_filter->GetOutputPort());
  writer->Write();

  std::cout << "Saved bar chart on: " << constructFullPathToDataset(argv[1], std::string(argv[1]) + ".png")
            << std::endl;

  plotter->close();

  return EXIT_SUCCESS;
}
