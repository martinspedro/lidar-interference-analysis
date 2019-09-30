/**
 * \file  cloud_statistical_data.hpp
 * \brief Header file for Statistical Cloud data class
 *
 */

#ifndef CLOUD_STATISTICAL_DATA_H
#define CLOUD_STATISTICAL_DATA_H

namespace point_cloud
{
namespace statistics
{
struct CloudStatisticalData
{
  long int point_cloud_msg_count;
  long int point_count;
  long int outliers_points_count;
  long int inliers_points_count;

  double points_average_per_msg;
  double points_variance_per_msg;

  double relative_out_points;

  CloudStatisticalData();

  void printStatistics();
  void computeOutliersRelativeValue();
  inline double getOutliersPercentage();
};

}  // namespace statistics

}  // namespace point_cloud

#endif  // CLOUD_STATISTICAL_DATA_H
