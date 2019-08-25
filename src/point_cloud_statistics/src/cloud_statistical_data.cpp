/**
 * \file   cloud_statistical_data.cpp
 * \brief
 *
 */

#include "point_cloud_statistics/cloud_statistical_data.hpp"
#include <iostream>

namespace point_cloud_statistics
{
CloudStatisticalData::CloudStatisticalData()
{
  this->point_cloud_msg_count = 0;
  this->point_count = 0;
  this->outliers_points_count = 0;
  this->inliers_points_count = 0;

  this->points_average_per_msg = 0.0;
  this->points_variance_per_msg = 0.0;

  this->relative_out_points = 0.0;
}

void CloudStatisticalData::printStatistics()
{
  std::cout << "Number of received Point Cloud Messages: " << this->point_cloud_msg_count << std::endl
            << "Number of received Point Cloud 3D Points: " << this->point_count << std::endl
            << "From which " << this->outliers_points_count << " (" << this->getOutliersPercentage()
            << "%) are interfered" << std::endl;
}

void CloudStatisticalData::computeOutliersRelativeValue()
{
  this->relative_out_points = (double)(this->outliers_points_count) / this->point_count;
}

inline double CloudStatisticalData::getOutliersPercentage()
{
  return this->relative_out_points * 100;
}

}  // namespace point_cloud_statistics