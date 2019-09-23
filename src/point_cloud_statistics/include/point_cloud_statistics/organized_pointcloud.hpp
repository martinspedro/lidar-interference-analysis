/**
 * \file   organized_pointcloud.hpp
 * \brief
 *
 */

#ifndef ORGANIZED_POINTCLOUD_H
#define ORGANIZED_POINTCLOUD_H

//#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace organized_pointcloud
{
template <class T>
class OrganizedPointCloud : public pcl::PointCloud<T>
{
public:
  OrganizedPointCloud(unsigned int width, unsigned int height, const T& init_value);
  ~OrganizedPointCloud<T>() = default;

  void clearPointsFromPointcloud();
  void organizeVelodynePointCloud(pcl::PointCloud<T> unorganized_cloud);
  OrganizedPointCloud<T> computeDistanceBetweenPointClouds(OrganizedPointCloud<T> point_cloud_1,
                                                           OrganizedPointCloud<T> point_cloud_2);

private:
  float getAzimuth(T point);
  unsigned int getAzimuthIndex(T point);
  float computeEuclideanDistanceToOrigin(T point);
  float computeEuclideanDistance(T point1, T point2);
};

}  // namespace organized_pointcloud

#endif  // ORGANIZED_POINTCLOUD_H
