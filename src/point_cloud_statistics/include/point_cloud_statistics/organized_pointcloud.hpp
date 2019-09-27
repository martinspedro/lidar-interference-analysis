/**
 * \file   organized_pointcloud.hpp
 * \brief
 *
 */

#ifndef ORGANIZED_POINT_CLOUD_H
#define ORGANIZED_POINT_CLOUD_H

#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"
#include <pcl/point_cloud.h>

#include <ros/ros.h>
#include <ros/console.h>

namespace point_cloud
{
namespace organized
{
template <class PointT>
class OrganizedPointCloud : public pcl::PointCloud<PointT>
{
public:
  OrganizedPointCloud(unsigned int width, unsigned int height) : pcl::PointCloud<PointT>(width, height)
  {
    ROS_DEBUG_NAMED("call_stack", "[OrganizedPointCloud] with (%d, %d)", width, height);
  }

  void clearPointsFromPointcloud()
  {
    ROS_DEBUG_NAMED("call_stack", "[clearPointsFromPointcloud]");

    PointT aux;
    aux.x = std::numeric_limits<float>::quiet_NaN();
    aux.y = std::numeric_limits<float>::quiet_NaN();
    aux.z = std::numeric_limits<float>::quiet_NaN();
    aux.intensity = std::numeric_limits<float>::quiet_NaN();
    aux.ring = -1;

    for (int i = 0; i < this->width; ++i)
    {
      for (int j = 0; j < this->height; ++j)
      {
        this->at(i, j) = aux;
      }
    }
  }

  float getAzimuth(const PointT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getAzimuth]");
    return atan2(point.y, point.x) * point_cloud::organized::RADIAN_TO_DEGREE_F;
  }

  unsigned int getAzimuthIndex(const PointT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getAzimuthIndex]");
    float shifted_azimuth =
        OrganizedPointCloud<PointT>::getAzimuth(point) + point_cloud::organized::DEGREE_OFFSET_TO_POSITIVE_ANGLE_F;
    float fixed_point_shifted_azimuth = floor(shifted_azimuth * 10.0f) / 10.0f;
    float index =
        fixed_point_shifted_azimuth * (float)(this->width - 1) / point_cloud::organized::FULL_REVOLUTION_DEGREE_F;

    return (unsigned int)(index);
  }

  float computeEuclideanDistanceToOrigin(PointT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeEuclideanDistanceToOrigin]");
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  }

  float computeEuclideanDistance(PointT point1, PointT point2)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeEuclideanDistance]");
    ROS_DEBUG("P1->(%f, %f, %f), P2->(%f, %f, %f)", point1.x, point1.y, point1.z, point2.x, point2.y, point2.z);
    float x_diff = point2.x - point1.x;
    float y_diff = point2.y - point1.y;
    float z_diff = point2.z - point1.z;

    return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
  }

  void organizeVelodynePointCloud(pcl::PointCloud<PointT> unorganized_cloud)
  {
    ROS_DEBUG_NAMED("call_stack", "[organizeVelodynePointCloud]");
    this->header = unorganized_cloud.header;  // use sequence number, stamp and frame_id from the unorganized cloud
    this->is_dense = false;                   // Organized Point Cloud is not dense

    PointT aux;
    aux.x = std::numeric_limits<float>::quiet_NaN();
    aux.y = std::numeric_limits<float>::quiet_NaN();
    aux.z = std::numeric_limits<float>::quiet_NaN();
    aux.intensity = std::numeric_limits<float>::quiet_NaN();
    aux.ring = -1;

    for (int i = 0; i < unorganized_cloud.size(); ++i)
    {
      unsigned int azimuth_index = getAzimuthIndex(unorganized_cloud.points[i]);

      ROS_ASSERT_MSG(azimuth_index < this->width, "Azimuth index %d vs size %d", azimuth_index, this->width);

      if (this->at(azimuth_index, unorganized_cloud.points[i].ring).intensity !=
          std::numeric_limits<float>::quiet_NaN())
      {
        this->at(azimuth_index, unorganized_cloud.points[i].ring) = unorganized_cloud.points[i];
      }
      else
      {
        ROS_WARN("Point would erase previous point! (%f, %f, %d) vs (%f, %f, %d) and azimuth_index: %d. Point not "
                 "added!",
                 this->at(azimuth_index, unorganized_cloud.points[i].ring).x,
                 this->at(azimuth_index, unorganized_cloud.points[i].ring).y,
                 this->at(azimuth_index, unorganized_cloud.points[i].ring).ring, unorganized_cloud.points[i].x,
                 unorganized_cloud.points[i].y, unorganized_cloud.points[i].ring, azimuth_index);
      }
    }
  }

  OrganizedPointCloud<PointT> computeDistanceBetweenPointClouds(OrganizedPointCloud<PointT> point_cloud_1,
                                                                OrganizedPointCloud<PointT> point_cloud_2)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeDistanceBetweenPointClouds]");

    ROS_ASSERT_MSG((point_cloud_1.width == point_cloud_2.width) && (point_cloud_1.height == point_cloud_2.height),
                   "Point cloud dimensions disagree: (%i, %i) vs (%i, %i)", point_cloud_1.width, point_cloud_1.height,
                   point_cloud_2.width, point_cloud_2.height);

    OrganizedPointCloud<PointT> distance_map(point_cloud_1.width, point_cloud_1.height);

    for (int i = 0; i < point_cloud_1.width; ++i)
    {
      for (int j = 0; j < point_cloud_1.height; ++j)
      {
        distance_map.at(i, j) = computeEuclideanDistance(point_cloud_1.at(i, j), point_cloud_2.at(i, j));
      }
    }

    return distance_map;
  }

  void computeDistanceBetweenPointClouds(OrganizedPointCloud<PointT> point_cloud, std::vector<double>& distance_vector)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeDistanceBetweenPointClouds] (%i, %i) vs (%i, %i)", this->width, this->height,
                    point_cloud.width, point_cloud.height);
    ROS_ASSERT_MSG((this->width == point_cloud.width) && (this->height == point_cloud.height),
                   "Point cloud dimensions disagree: (%i, %i) vs (%i, %i)", this->width, this->height,
                   point_cloud.width, point_cloud.height);

    for (int i = 0; i < this->width; ++i)
    {
      for (int j = 0; j < this->height; ++j)
      {
        double distance = (double)(computeEuclideanDistance(this->at(i, j), point_cloud.at(i, j)));
        ROS_DEBUG("Distance: %f", distance);
        distance_vector.push_back(distance);
      }
    }
  }
};

}  // namespace organized
}  // namespace point_cloud

#endif  // ORGANIZED_POINT_CLOUD_H
