/**
 * \file   point_cloud_statistics.cpp
 * \brief
 *
 */

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include "point_cloud_statistics/point_cloud_statistics.hpp"
#include "point_cloud_statistics/velodyne_point_type.h"
#include "point_cloud_statistics/vlp_16_utilities.hpp"

namespace point_cloud
{
namespace statistics
{
PointCloudXYZ icp(PointCloudXYZ::Ptr source, PointCloudXYZ::Ptr target, bool downsample = false)
{
  if (downsample)
  {
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize(0.05f, 0.05f, 0.05f);
    grid.setInputCloud(source);
    grid.filter(*source);

    grid.setInputCloud(target);
    grid.filter(*target);
  }

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(0.02);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(50);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon(0.05);

  PointCloudXYZ final;
  icp.align(final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  return *source + final;
}

velodyne::VelodynePointCloud icp(velodyne::VelodynePointCloud::Ptr source, velodyne::VelodynePointCloud::Ptr target,
                                 bool downsample = false)
{
  if (downsample)
  {
    pcl::VoxelGrid<velodyne::PointXYZIR> grid;
    grid.setLeafSize(0.05f, 0.05f, 0.05f);
    grid.setInputCloud(source);
    grid.filter(*source);

    grid.setInputCloud(target);
    grid.filter(*target);
  }

  pcl::IterativeClosestPoint<velodyne::PointXYZIR, velodyne::PointXYZIR> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);

  // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
  icp.setMaxCorrespondenceDistance(0.02);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(50);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  icp.setEuclideanFitnessEpsilon(0.05);

  velodyne::VelodynePointCloud final;
  icp.align(final);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  return *source + final;
}
}  // namespace statistics

}  // namespace point_cloud
