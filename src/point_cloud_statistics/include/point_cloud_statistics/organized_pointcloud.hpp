/**
 * \file   organized_pointcloud.hpp
 * \brief
 *
 */

#ifndef ORGANIZED_POINTCLOUD_H
#define ORGANIZED_POINTCLOUD_H

//#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <ros/ros.h>

#define _USE_MATH_DEFINES  // Define to use PI
#include <math.h>

// namespace organized_pointcloud
//{
const float RADIAN_TO_DEGREE_F = 180.0F / (float)(M_PI);
const double RADIAN_TO_DEGREE_D = 180.0D / (double)(M_PI);
const long double RADIAN_TO_DEGREE_LD = 180.0L / (long double)(M_PI);

const float DEGREE_OFFSET_TO_POSITIVE_ANGLE_F = 180.0F;
const double DEGREE_OFFSET_TO_POSITIVE_ANGLE_D = 180.0D;
const long double DEGREE_OFFSET_TO_POSITIVE_ANGLE_L = 180.0L;

const float AZIMUTH_NORMALIZE_FACTOR_DEGREE_F = 360.0F;
const double AZIMUTH_NORMALIZE_FACTOR_DEGREE_D = 360.0D;
const long double AZIMUTH_NORMALIZE_FACTOR_DEGREE_L = 360.0L;
/*
float getAzimuth(float y, float x);
double getAzimuth(double y, double x);
long double getAzimuth(long double y, long double x);

float computeEuclideanDistance(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2);
double computeEuclideanDistance(double x_1, double y_1, double z_1, double x_2, double y_2, double z_2);
long double computeEuclideanDistance(long double x_1, long double y_1, long double z_1, long double x_2,
                                     long double y_2, long double z_2);
*/
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
    this->is_dense = false;  // Organized Point Cloud is not dense
  }

  // template <typename T>
  void clearPointsFromPointcloud()
  {
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
    return atan2(point.y, point.x) * RADIAN_TO_DEGREE_F;
  }

  // template <typename T>
  unsigned int getAzimuthIndex(const T point)
  {
    float shifted_azimuth = OrganizedPointCloud<T>::getAzimuth(point) + DEGREE_OFFSET_TO_POSITIVE_ANGLE_F;
    float fixed_point_shifted_azimuth = floor(shifted_azimuth * 10.0f) / 10.0f;
    float index = fixed_point_shifted_azimuth * (float)(this->width) / AZIMUTH_NORMALIZE_FACTOR_DEGREE_F;

    return (unsigned int)(index);
  }

  // template <typename T>
  float computeEuclideanDistanceToOrigin(T point)
  {
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  }

  // template <typename T>
  float computeEuclideanDistance(T point1, T point2)
  {
    float x_diff = point2.x - point1.x;
    float y_diff = point2.y - point1.y;
    float z_diff = point2.z - point1.z;

    return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
  }

  // template <typename T>
  void organizeVelodynePointCloud(pcl::PointCloud<T> unorganized_cloud)
  {
    this->header = unorganized_cloud.header;  // use sequence number, stamp and frame_id from the unorganized cloud
    T aux;
    aux.x = std::numeric_limits<float>::quiet_NaN();
    aux.y = std::numeric_limits<float>::quiet_NaN();
    aux.z = std::numeric_limits<float>::quiet_NaN();
    aux.intensity = std::numeric_limits<float>::quiet_NaN();
    aux.ring = -1;

    for (int i = 0; i < unorganized_cloud.size(); ++i)
    {
      unsigned int azimuth = getAzimuthIndex(unorganized_cloud.points[i]);
      // (width, height)
      if (this->at(azimuth, unorganized_cloud.points[i].ring).intensity != std::numeric_limits<float>::quiet_NaN())
      {
        this->at(azimuth, unorganized_cloud.points[i].ring) = unorganized_cloud.points[i];
      }
      else
      {
        /*
      ROS_WARN("Point would erase previous point! (%f, %f, %f) vs (%f, %f, %f) and azimuth: %f. Point not added!",
               this->at(azimuth, unorganized_cloud.points[i].ring).x,
               this->at(azimuth, unorganized_cloud.points[i].ring).y, unorganized_cloud.points[i].ring,
               unorganized_cloud.points[i].x, unorganized_cloud.points[i].y, azimuth);
               */
      }
    }
  }

  // template <typename T>
  OrganizedPointCloud<T> computeDistanceBetweenPointClouds(OrganizedPointCloud<T> point_cloud_1,
                                                           OrganizedPointCloud<T> point_cloud_2)
  {
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
  void computeDistanceBetweenPointClouds(OrganizedPointCloud<T> point_cloud, std::vector<float> distance_vector)
  {
    ROS_ASSERT_MSG((this->width == point_cloud.width) && (this->height == point_cloud.height),
                   "Point cloud dimensions disagree: (%i, %i) vs (%i, %i)", this->width, this->height,
                   point_cloud.width, point_cloud.height);

    for (int i = 0; i < this->width; ++i)
    {
      for (int j = 0; j < this->height; ++j)
      {
        distance_vector.push_back(computeEuclideanDistance(this->at(i, j), point_cloud.at(i, j)));
      }
    }
  }
};
//}  // namespace organized_pointcloud

#endif  // ORGANIZED_POINTCLOUD_H
