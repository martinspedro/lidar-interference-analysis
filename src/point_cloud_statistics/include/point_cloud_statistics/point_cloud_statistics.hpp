/**
 * \file   point_cloud_statistics.hpp
 * \brief
 *
 */

#ifndef POINT_CLOUD_STATISTICS_H
#define POINT_CLOUD_STATISTICS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>

#include "point_cloud_statistics/vlp_16_utilities.hpp"

namespace point_cloud
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

namespace statistics
{
PointCloudXYZ icp(PointCloudXYZ::Ptr source, PointCloudXYZ::Ptr target, std::ofstream& logger_file, bool downsample);
velodyne::VelodynePointCloud icp(velodyne::VelodynePointCloud::Ptr source, velodyne::VelodynePointCloud::Ptr target,
                                 std::ofstream& logger_file, bool downsample);

}  // namespace statistics

}  // namespace point_cloud

#endif  // POINT_CLOUD_STATISTICS_H
