/**
 * \file  cloud_statistical_data.hpp
 * \brief Header file for Statistical Cloud data class
 *
 */

#ifndef CLOUD_STATISTICAL_DATA_H
#define CLOUD_STATISTICAL_DATA_H

#include <string>
#include <sstream>

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
  double relative_in_points;

  CloudStatisticalData();

  void printStatistics();
  void printPointStatistics();
  std::stringstream outputStringFormattedPointStatistics();
  std::stringstream outputStringFormattedStatistics();

  void computeStats();
  void computeOutliersRelativeValue();
  void computeInliersRelativeValue();

  inline double getOutliersPercentage()
  {
    return this->relative_out_points * 100.0d;
  }

  inline double getInliersPercentage()
  {
    return this->relative_in_points * 100.0d;
  }
};

}  // namespace statistics

}  // namespace point_cloud

#endif  // CLOUD_STATISTICAL_DATA_H
