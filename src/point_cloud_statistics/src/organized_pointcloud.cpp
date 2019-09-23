/**
 * \file   organized_pointcloud.cpp
 * \brief
 *
 */

#include "point_cloud_statistics/organized_pointcloud.hpp"

namespace organized_pointcloud
{
template <class T>
OrganizedPointCloud<T>::OrganizedPointCloud(unsigned int width, unsigned int height, const T& value)
  : pcl::PointCloud<T>(width, height, value)
{
}

//{
// PointCloud (uint32_t width_, uint32_t height_, const PointT& value_ = PointT ())
// this->cloud = new pcl::PointCloud<pcl::PointXYZ>(this->_width, this->_height);

// this->points = &_points;
// this->_init_data_structure();
/*
void OrganizedPointCloud<T>::_init_data_structure()
{
  for (int i = 0; i < this->_height; ++i)
  {
    for (int j = 0; j < this->_width; ++j)
    {
      // this->points[i][j] = std::numeric_limits<double>::quiet_NaN();
    }
  }
}
*/
}  // namespace organized_pointcloud
