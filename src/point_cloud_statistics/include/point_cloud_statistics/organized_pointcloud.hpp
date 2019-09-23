/**
 * \file   organized_pointcloud.hpp
 * \brief
 *
 */

#ifndef ORGANIZED_POINTCLOUD_H
#define ORGANIZED_POINTCLOUD_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <pcl/PCLHeader.h>
#include <pcl/exceptions.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_traits.h>

#include <algorithm>
#include <utility>
#include <vector>
#include <pcl/impl/point_types.hpp>

namespace organized_pointcloud
{
template <class T>
class OrganizedPointCloud : public pcl::PointCloud<T>
{
public:
  OrganizedPointCloud<T>(unsigned int width, unsigned int height, const T& value);

  ~OrganizedPointCloud<T>() = default;

  // void _init_data_structure();

private:
  pcl::PointCloud<pcl::PointXYZ>* cloud;

  unsigned int _height;
  unsigned int _width;
};

}  // namespace organized_pointcloud

#endif  // ORGANIZED_POINTCLOUD_H
