/**
 * \file  point_cloud_statistics.hpp
 * \brief header file of point cloud statistics method
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

#ifndef POINT_CLOUD_STATISTICS_H
#define POINT_CLOUD_STATISTICS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>

#include "point_cloud_statistics/vlp_16_utilities.hpp"

namespace point_cloud
{
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;    //!< Point Cloud with ZYZ points
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;  //!< Point Cloud with points with intensity

namespace statistics
{
/*!
 * \brief iteratve closest point wrap up for XYZ point clouds
 * \param[in] source point cloud which the data is to be aligned
 * \param[in] target point cloud which has the data to be alinged
 * \param[in] logger_file pointer to the ofstream object for which the data is to be logged
 * \param[in] downsample Enables the downsampling of the source and target point cloud
 * \return The source + the aligned point cloud
 *
 * \remarks uses pcl::IterativeClosestPoint
 */
PointCloudXYZ icp(PointCloudXYZ::Ptr source, PointCloudXYZ::Ptr target, std::ofstream& logger_file, bool downsample);

/*!
 * \brief iteratve closest point wrap up for XYZ point clouds
 * \param[in] source point cloud which the data is to be aligned
 * \param[in] target point cloud which has the data to be alinged
 * \param[in] logger_file pointer to the ofstream object for which the data is to be logged
 * \param[in] downsample Enables the downsampling of the source and target point cloud
 * \return The source + the aligned point cloud
 *
 * \remarks uses pcl::IterativeClosestPoint
 */
velodyne::VelodynePointCloud icp(velodyne::VelodynePointCloud::Ptr source, velodyne::VelodynePointCloud::Ptr target,
                                 std::ofstream& logger_file, bool downsample);

}  // namespace statistics

}  // namespace point_cloud

#endif  // POINT_CLOUD_STATISTICS_H
