#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <string>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <vector>
#include <iostream>
#include <utility>

#include <vtkColorSeries.h>
#include "point_cloud_statistics/bar_chart_plotter.hpp"

#include <pcl/octree/octree_pointcloud_changedetector.h>

#include "multiple_lidar_interference_mitigation_bringup/datasets_info.hpp"
/*
#include <vtkColorSeries.h>
#include <vtkChart.h>

#include <vtkSmartPointer.h>
#include <vtkRenderWindow.h>

#include <vtkWindowToImageFilter.h>
#include <vtkPNGWriter.h>
*/
#include <velodyne_pointcloud/point_types.h>

#include <pcl/point_representation.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>

//#include "lz4.h"
//#include "lz4hc.h"
#include <pcl/features/normal_3d.h>

typedef pcl::PointCloud<velodyne_pointcloud::PointXYZIR> VelodynePointCloud;

using pcl::visualization::PointCloudColorHandlerCustom;
using pcl::visualization::PointCloudColorHandlerGenericField;

// convenient typedefs
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

struct PCD
{
  PointCloud::Ptr cloud;
  std::string f_name;

  PCD() : cloud(new PointCloud){};
};

// Define a new point representation for < x, y, z, curvature >
class MyPointRepresentation : public pcl::PointRepresentation<PointNormalT>
{
  using pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
  MyPointRepresentation()
  {
    // Define the number of dimensions
    nr_dimensions_ = 4;
  }

  // Override the copyToFloatArray method to define our feature vector
  virtual void copyToFloatArray(const PointNormalT& p, float* out) const
  {
    // < x, y, z, curvature >
    out[0] = p.x;
    out[1] = p.y;
    out[2] = p.z;
    out[3] = p.curvature;
  }
};

////////////////////////////////////////////////////////////////////////////////
/** \brief Align a pair of PointCloud datasets and return the result
 * \param cloud_src the source PointCloud
 * \param cloud_tgt the target PointCloud
 * \param output the resultant aligned source PointCloud
 * \param final_transform the resultant transform between source and target
 */
void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output,
               Eigen::Matrix4f& final_transform, bool downsample = false)
{
  //
  // Downsample for consistency and speed
  // \note enable this for large datasets
  PointCloud::Ptr src(new PointCloud);
  PointCloud::Ptr tgt(new PointCloud);
  pcl::VoxelGrid<PointT> grid;
  if (downsample)
  {
    grid.setLeafSize(0.01, 0.01, 0.01);
    grid.setInputCloud(cloud_src);
    grid.filter(*src);

    grid.setInputCloud(cloud_tgt);
    grid.filter(*tgt);
  }
  else
  {
    src = cloud_src;
    tgt = cloud_tgt;
  }

  // Compute surface normals and curvature
  PointCloudWithNormals::Ptr points_with_normals_src(new PointCloudWithNormals);
  PointCloudWithNormals::Ptr points_with_normals_tgt(new PointCloudWithNormals);

  pcl::NormalEstimation<PointT, PointNormalT> norm_est;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  norm_est.setSearchMethod(tree);
  norm_est.setKSearch(30);

  norm_est.setInputCloud(src);
  norm_est.compute(*points_with_normals_src);
  pcl::copyPointCloud(*src, *points_with_normals_src);

  norm_est.setInputCloud(tgt);
  norm_est.compute(*points_with_normals_tgt);
  pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

  //
  // Instantiate our custom point representation (defined above) ...
  MyPointRepresentation point_representation;
  // ... and weight the 'curvature' dimension so that it is balanced against x, y, and z
  float alpha[4] = { 1.0, 1.0, 1.0, 1.0 };
  point_representation.setRescaleValues(alpha);

  //
  // Align
  pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;
  reg.setTransformationEpsilon(1e-6);
  // Set the maximum distance between two correspondences (src<->tgt) to 10cm
  // Note: adjust this based on the size of your datasets
  reg.setMaxCorrespondenceDistance(0.1);
  // Set the point representation
  reg.setPointRepresentation(boost::make_shared<const MyPointRepresentation>(point_representation));

  reg.setInputSource(points_with_normals_src);
  reg.setInputTarget(points_with_normals_tgt);

  //
  // Run the same optimization in a loop and visualize the results
  Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
  PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
  reg.setMaximumIterations(50);
  reg.align(*reg_result);
  Ti = reg.getFinalTransformation();
  /*
  for (int i = 0; i < 30; ++i)
  {
    PCL_INFO("Iteration Nr. %d.\n", i);

    // save cloud for visualization purpose
    points_with_normals_src = reg_result;

    // Estimate
    reg.setInputSource(points_with_normals_src);
    reg.align(*reg_result);

    // accumulate transformation between each Iteration
    Ti = reg.getFinalTransformation() * Ti;

    // if the difference between this transformation and the previous one
    // is smaller than the threshold, refine the process by reducing
    // the maximal correspondence distance
    if (std::abs((reg.getLastIncrementalTransformation() - prev).sum()) < reg.getTransformationEpsilon())
      reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

    prev = reg.getLastIncrementalTransformation();

    // visualize current state
    // showCloudsRight(points_with_normals_tgt, points_with_normals_src);
  }
*/
  //
  // Get the transformation from target to source
  targetToSource = Ti.inverse();

  //
  // Transform target back in source frame
  pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);

  // p->removePointCloud("source");
  // p->removePointCloud("target");

  // PointCloudColorHandlerCustom<PointT> cloud_tgt_h(output, 0, 255, 0);
  // PointCloudColorHandlerCustom<PointT> cloud_src_h(cloud_src, 255, 0, 0);
  // p->addPointCloud(output, cloud_tgt_h, "target", vp_2);
  // p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

  // PCL_INFO("Press q to continue the registration.\n");
  // p->spin();

  // p->removePointCloud("source");
  // p->removePointCloud("target");

  // add the source to the transformed target
  *output += *cloud_src;

  final_transform = targetToSource;
}

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
  // velodyne_pointcloud::PointcloudXYZIR::Ptr ground_truth_ptr(new velodyne_pointcloud::PointcloudXYZIR);

  rosbag::Bag interference_bag, ground_truth_bag;
  ground_truth_bag.open(ground_truth_full_bag_path);  // open ground truth bag file

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View ground_truth_view(ground_truth_bag, rosbag::TopicQuery(topics));

  std::cout << "Ground Truth" << std::endl;
  int count = 0;
  long int sum = 0, sum_squared = 0;
  // generate ground truth model for test scenario

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

      std::cout << "(" << msg->width << ", " << msg->height << ")" << std::endl;
      sum += msg->width;
      sum_squared += msg->width * msg->width;

      PointCloud::Ptr filtered_point_cloud(new PointCloud);
      *filtered_point_cloud = ground_truth_point_cloud;
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_out;
      sor_out.setInputCloud(filtered_point_cloud);
      sor_out.setMeanK(50);
      sor_out.setStddevMulThresh(1);
      sor_out.filter(*filtered_point_cloud);
      // break;
      /*
      for (int i = 0; i < msg->fields.size(); ++i)
      {
        std::cout << "Field " << i << ": (" << msg->fields[i].name << ", " << msg->fields[i].offset << ", "
                  << msg->fields[i].datatype << ", " << msg->fields[i].count << ")" << std::endl;
      }
      // std::cout << msg->fields << std::endl;
      std::cout << "(" << ground_truth_point_cloud.width << ", " << ground_truth_point_cloud.height << ")" << std::endl;

      std::cout << pcl::getFieldsList(ground_truth_point_cloud) << std::endl << std::endl;
      */
      *target = *filtered_point_cloud;
      // Tutorial starts here
      if (count > 0)
      {
        // for (size_t i = 1; i < data.size(); ++i)
        // {
        // source = data[i - 1].cloud;
        // target = data[i].cloud;

        // Add visualization data
        // showCloudsLeft(source, target);

        PointCloud::Ptr temp(new PointCloud);
        // PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i - 1].f_name.c_str(), source->points.size(),
        //         data[i].f_name.c_str(), target->points.size());
        pairAlign(source, target, temp, pairTransform, true);

        // transform current pair into the global transform
        pcl::transformPointCloud(*temp, *result, GlobalTransform);

        // update the global transform
        GlobalTransform *= pairTransform;

        *source = *result;
      }
      else
      {
        *source = *filtered_point_cloud;
      }
      /*
      std::vector<int> field_sizes;
      std::vector<sensor_msgs::PointField> point_fields;

      pcl::getFields(ground_truth_point_cloud, point_fields);
      pcl::getFieldsSizes(point_fields, field_sizes);

      for (int i = 0; i < field_sizes.size(); ++i)
      {
        std::cout << field_sizes[i] << " ";
      }

      std::cout << std::endl;
      */
      /*
      for (int i = 0; i < ground_truth_point_cloud.size(); ++i)
      {
        ground_truth_point_cloud.poins[i].
      }
      */
      ++count;
      std::cout << "Message number: " << count << std::endl;
      if (count > 3)
      {
        break;
      }
    }
  }
  // save aligned pair, transformed into the first cloud's frame
  std::stringstream ss;
  ss << constructFullPathToDataset(argv[1], "ground_truth_model.pcd");
  pcl::io::savePCDFile(ss.str(), *result, true);

  //}
  float average = (double)sum / count;
  float variance = ((double)sum_squared - sum * sum) / count;
  std::cout << "Sum: " << sum << std::endl;
  std::cout << "Average: " << average << std::endl;
  std::cout << "Variance: " << variance << std::endl;
  std::cout << "Number of clouds: " << count << std::endl;
  std::cout << result->size() << std::endl;
  // return 0;

  // VOXEL GRID FILTER
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_out;
  sor_out.setInputCloud(result);
  sor_out.setMeanK(50);
  sor_out.setStddevMulThresh(1);
  sor_out.filter(*ground_truth_ptr);

  std::stringstream ss1;
  ss1 << constructFullPathToDataset(argv[1], "filtered_ground_truth_model.pcd");
  pcl::io::savePCDFile(ss1.str(), *ground_truth_ptr, true);

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor_voxel;
  sor_voxel.setInputCloud(ground_truth_ptr);
  sor_voxel.setLeafSize(0.01f, 0.01f, 0.01f);
  sor_voxel.filter(*ground_truth_ptr);

  std::stringstream ss2;
  ss2 << constructFullPathToDataset(argv[1], "voxelized_ground_truth_model.pcd");
  pcl::io::savePCDFile(ss2.str(), *ground_truth_ptr, true);

  // ground_truth_ptr = result;

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
  const char* bar_chart_filename = constructFullPathToDataset(argv[1], std::string(argv[1]) + ".png").c_str();
  plotter->saveBarChartPNG(bar_chart_filename);
  std::cout << "Saved bar chart on: " << bar_chart_filename << std::endl;

  plotter->close();  // Destroys bar chart object

  return EXIT_SUCCESS;
}
