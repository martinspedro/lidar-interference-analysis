/**
 * \file   point_cloud_statistics.hpp
 * \brief
 *
 */

#ifndef POINT_CLOUD_STATISTICS_H
#define POINT_CLOUD_STATISTICS_H

#include <string>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace point_cloud
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;

namespace statistics
{
PointCloudXYZ icp(PointCloudXYZ::Ptr source, PointCloudXYZ::Ptr target, bool downsample);

}  // namespace statistics

}  // namespace point_cloud

#endif  // POINT_CLOUD_STATISTICS_H
