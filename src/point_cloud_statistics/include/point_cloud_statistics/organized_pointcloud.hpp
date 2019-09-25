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

// namespace organized_pointcloud
//{

template <class T>
class OrganizedPointCloud : public pcl::PointCloud<T>
{
public:
  /*
OrganizedPointCloud(unsigned int width, unsigned int height);
~OrganizedPointCloud() = default;
void clearPointsFromPointcloud();

void organizeVelodynePointCloud(pcl::PointCloud<T> unorganized_cloud);

OrganizedPointCloud<T> computeDistanceBetweenPointClouds(OrganizedPointCloud<T> point_cloud_1,
                                                         OrganizedPointCloud<T> point_cloud_2);

void computeDistanceBetweenPointClouds(OrganizedPointCloud<T> point_cloud, std::vector<float> distance_vector);

private:
float getAzimuth(T point);
unsigned int getAzimuthIndex(T point);
float computeEuclideanDistanceToOrigin(T point);
float computeEuclideanDistance(T point1, T point2);
};
*/
  // template <class T>
  OrganizedPointCloud(unsigned int width, unsigned int height) : pcl::PointCloud<T>(width, height)
  {
    ROS_DEBUG_NAMED("call_stack", "[OrganizedPointCloud] with (%d, %d)", width, height);
  }

  // template <typename T>
  void clearPointsFromPointcloud()
  {
    ROS_DEBUG_NAMED("call_stack", "[clearPointsFromPointcloud]");

    T aux;
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

  // template <typename T>
  float getAzimuth(const T point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getAzimuth]");
    return atan2(point.y, point.x) * point_cloud::organized::RADIAN_TO_DEGREE_F;
  }

  // template <typename T>
  unsigned int getAzimuthIndex(const T point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getAzimuthIndex]");
    float shifted_azimuth =
        OrganizedPointCloud<T>::getAzimuth(point) + point_cloud::organized::DEGREE_OFFSET_TO_POSITIVE_ANGLE_F;
    float fixed_point_shifted_azimuth = floor(shifted_azimuth * 10.0f) / 10.0f;
    float index =
        fixed_point_shifted_azimuth * (float)(this->width - 1) / point_cloud::organized::FULL_REVOLUTION_DEGREE_F;

    return (unsigned int)(index);
  }

  // template <typename T>
  float computeEuclideanDistanceToOrigin(T point)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeEuclideanDistanceToOrigin]");
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  }

  // template <typename T>
  float computeEuclideanDistance(T point1, T point2)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeEuclideanDistance]");
    ROS_INFO("P1->(%f, %f, %f), P2->(%f, %f, %f)", point1.x, point1.y, point1.z, point2.x, point2.y, point2.z);
    float x_diff = point2.x - point1.x;
    float y_diff = point2.y - point1.y;
    float z_diff = point2.z - point1.z;

    return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
  }

  // template <typename T>
  void organizeVelodynePointCloud(pcl::PointCloud<T> unorganized_cloud)
  {
    ROS_DEBUG_NAMED("call_stack", "[organizeVelodynePointCloud]");
    this->header = unorganized_cloud.header;  // use sequence number, stamp and frame_id from the unorganized cloud
    this->is_dense = false;                   // Organized Point Cloud is not dense

    T aux;
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
        ROS_WARN("Point would erase previous point! (%f, %f, %f) vs (%f, %f, %f) and azimuth_index: %d. Point not "
                 "added!",
                 this->at(azimuth_index, unorganized_cloud.points[i].ring).x,
                 this->at(azimuth_index, unorganized_cloud.points[i].ring).y, unorganized_cloud.points[i].ring,
                 unorganized_cloud.points[i].x, unorganized_cloud.points[i].y, azimuth_index);
      }
    }
  }

  // template <typename T>
  OrganizedPointCloud<T> computeDistanceBetweenPointClouds(OrganizedPointCloud<T> point_cloud_1,
                                                           OrganizedPointCloud<T> point_cloud_2)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeDistanceBetweenPointClouds]");

    ROS_ASSERT_MSG((point_cloud_1.width == point_cloud_2.width) && (point_cloud_1.height == point_cloud_2.height),
                   "Point cloud dimensions disagree: (%i, %i) vs (%i, %i)", point_cloud_1.width, point_cloud_1.height,
                   point_cloud_2.width, point_cloud_2.height);

    OrganizedPointCloud<T> distance_map(point_cloud_1.width, point_cloud_1.height);

    for (int i = 0; i < point_cloud_1.width; ++i)
    {
      for (int j = 0; j < point_cloud_1.height; ++j)
      {
        distance_map.at(i, j) = computeEuclideanDistance(point_cloud_1.at(i, j), point_cloud_2.at(i, j));
      }
    }

    return distance_map;
  }

  // template <typename T>
  void computeDistanceBetweenPointClouds(OrganizedPointCloud<T> point_cloud, std::vector<double>& distance_vector)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeDistanceBetweenPointClouds]");
    ROS_INFO("(%i, %i) vs (%i, %i)", this->width, this->height, point_cloud.width, point_cloud.height);
    ROS_ASSERT_MSG((this->width == point_cloud.width) && (this->height == point_cloud.height),
                   "Point cloud dimensions disagree: (%i, %i) vs (%i, %i)", this->width, this->height,
                   point_cloud.width, point_cloud.height);

    for (int i = 0; i < this->width; ++i)
    {
      for (int j = 0; j < this->height; ++j)
      {
        double distance = (double)(computeEuclideanDistance(this->at(i, j), point_cloud.at(i, j)));
        ROS_INFO("Distance: %f", distance);
        distance_vector.push_back(distance);
      }
    }
  }
};
//}  // namespace organized_pointcloud

#endif  // ORGANIZED_POINT_CLOUD_H
