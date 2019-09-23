/**
 * \file   organized_pointcloud.cpp
 * \brief
 *
 */

#include <ros/ros.h>

#define _USE_MATH_DEFINES  // Define to use PI
#include <math.h>

#include "point_cloud_statistics/organized_pointcloud.hpp"

namespace organized_pointcloud
{
const float RADIAN_TO_DEGREE_F = 180.0F / (float)(M_PI);
const double RADIAN_TO_DEGREE_D = 180.0D / (double)(M_PI);
const long double RADIAN_TO_DEGREE_LD = 180.0L / (long double)(M_PI);

const float DEGREE_OFFSET_TO_POSITIVE_ANGLE_F = 180.0F;
const double DEGREE_OFFSET_TO_POSITIVE_ANGLE_D = 180.0D;
const long double DEGREE_OFFSET_TO_POSITIVE_ANGLE_L = 180.0L;

const float AZIMUTH_NORMALIZE_FACTOR_DEGREE_F = 360.0F;
const double AZIMUTH_NORMALIZE_FACTOR_DEGREE_D = 360.0D;
const long double AZIMUTH_NORMALIZE_FACTOR_DEGREE_L = 360.0L;

float getAzimuth(float y, float x)
{
  return atan2(y, x) * RADIAN_TO_DEGREE_F;
}

double getAzimuth(double y, double x)
{
  return atan2(y, x) * RADIAN_TO_DEGREE_D;
}

long double getAzimuth(long double y, long double x)
{
  return atan2(y, x) * RADIAN_TO_DEGREE_LD;
}

float computeEuclideanDistance(float x_1, float y_1, float z_1, float x_2 = 0.0F, float y_2 = 0.0F, float z_2 = 0.0F)
{
  float x_diff = x_2 - x_1;
  float y_diff = y_2 - y_1;
  float z_diff = z_2 - z_1;

  return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

double computeEuclideanDistance(double x_1, double y_1, double z_1, double x_2 = 0.0D, double y_2 = 0.0D,
                                double z_2 = 0.0D)
{
  double x_diff = x_2 - x_1;
  double y_diff = y_2 - y_1;
  double z_diff = z_2 - z_1;

  return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

long double computeEuclideanDistance(long double x_1, long double y_1, long double z_1, long double x_2 = 0.0L,
                                     long double y_2 = 0.0L, long double z_2 = 0.0L)
{
  long double x_diff = x_2 - x_1;
  long double y_diff = y_2 - y_1;
  long double z_diff = z_2 - z_1;

  return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

// organized_pointcloud
template <class T>
OrganizedPointCloud<T>::OrganizedPointCloud(unsigned int width, unsigned int height, const T& init_value)
  : pcl::PointCloud<T>(width, height, init_value)
{
  this->is_dense = false;  // Organized Point Cloud is not dense
}

template <typename T>
void OrganizedPointCloud<T>::clearPointsFromPointcloud()
{
  for (int i = 0; i < this->width; ++i)
  {
    for (int j = 0; j < this->height; ++j)
    {
      this->at(i, j) = std::numeric_limits<T>::quiet_NaN();
    }
  }
}

template <typename T>
float OrganizedPointCloud<T>::getAzimuth(const T point)
{
  return atan2(point.y, point.x) * RADIAN_TO_DEGREE_F;
}

template <typename T>
unsigned int OrganizedPointCloud<T>::getAzimuthIndex(const T point)
{
  float shifted_azimuth = OrganizedPointCloud<T>::getAzimuth(point) + DEGREE_OFFSET_TO_POSITIVE_ANGLE_F;
  float fixed_point_shifted_azimuth = floor(shifted_azimuth * 10.0f) / 10.0f;
  float index = fixed_point_shifted_azimuth * (float)(this->width) / AZIMUTH_NORMALIZE_FACTOR_DEGREE_F;

  return (unsigned int)(index);
}

template <typename T>
float OrganizedPointCloud<T>::computeEuclideanDistanceToOrigin(T point)
{
  return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
}

template <typename T>
float OrganizedPointCloud<T>::computeEuclideanDistance(T point1, T point2)
{
  float x_diff = point2.x - point1.x;
  float y_diff = point2.y - point1.y;
  float z_diff = point2.z - point1.z;

  return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
}

template <typename T>
void OrganizedPointCloud<T>::organizeVelodynePointCloud(pcl::PointCloud<T> unorganized_cloud)
{
  this->header = unorganized_cloud.header;  // use sequence number, stamp and frame_id from the unorganized cloud

  for (int i = 0; i < unorganized_cloud->size(); ++i)
  {
    unsigned int azimuth = getAzimuthIndex(unorganized_cloud.points[i]);
    // (width, height)
    if (this->at(azimuth, unorganized_cloud.points[i].ring) == std::numeric_limits<T>::quiet_NaN())
    {
      this->at(azimuth, unorganized_cloud.points[i].ring) = unorganized_cloud.points[i];
    }
    else
    {
      ROS_WARN("Point would erase previous point! (%f, %f, %f) vs (%f, %f, %f) and azimuth: %f. Point not added!",
               this->at(azimuth, unorganized_cloud.points[i].ring).x,
               this->at(azimuth, unorganized_cloud.points[i].ring).y, unorganized_cloud.points[i].ring,
               unorganized_cloud.points[i].x, unorganized_cloud.points[i].y, azimuth);
    }
  }
}

template <typename T>
OrganizedPointCloud<T> OrganizedPointCloud<T>::computeDistanceBetweenPointClouds(OrganizedPointCloud<T> point_cloud_1,
                                                                                 OrganizedPointCloud<T> point_cloud_2)
{
  ROS_ASSERT_MSG((point_cloud_1.width == point_cloud_2.width) && (point_cloud_1.height == point_cloud_2.height),
                 "Point cloud dimensions disagree: (%i, %i) vs (%i, %i)", point_cloud_1.width, point_cloud_1.height,
                 point_cloud_2.width, point_cloud_2.height);

  OrganizedPointCloud<T> distance_map(point_cloud_1.width, point_cloud_1.height, std::numeric_limits<T>::quiet_NaN());

  for (int i = 0; i < point_cloud_1.width; ++i)
  {
    for (int j = 0; j < point_cloud_1.height; ++j)
    {
      distance_map.at(i, j) = computeEuclideanDistance(point_cloud_1.at(i, j), point_cloud_2.at(i, j));
    }
  }

  return distance_map;
}
}  // namespace organized_pointcloud
