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

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

namespace point_cloud_statistics
{
const std::string constructFullPathToDataset(const std::string dataset_name, const std::string file_name);

void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output,
               Eigen::Matrix4f& final_transform, bool downsample);
}  // namespace point_cloud_statistics

#endif  // POINT_CLOUD_STATISTICS_H
