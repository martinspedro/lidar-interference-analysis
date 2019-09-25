/**
 * \file   organized_pointcloud.cpp
 * \brief
 *
 */

#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"

#define _USE_MATH_DEFINES  // Define to use PI
#include <math.h>

namespace point_cloud
{
namespace organized
{
const float RADIAN_TO_DEGREE_F = 180.0F / (float)(M_PI);
const double RADIAN_TO_DEGREE_D = 180.0D / (double)(M_PI);
const long double RADIAN_TO_DEGREE_LD = 180.0L / (long double)(M_PI);

const float DEGREE_OFFSET_TO_POSITIVE_ANGLE_F = 180.0F;
const double DEGREE_OFFSET_TO_POSITIVE_ANGLE_D = 180.0D;
const long double DEGREE_OFFSET_TO_POSITIVE_ANGLE_L = 180.0L;

const float FULL_REVOLUTION_DEGREE_F = 360.0F;
const double FULL_REVOLUTION_DEGREE_D = 360.0D;
const long double FULL_REVOLUTION_DEGREE_L = 360.0L;

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

}  // namespace organized

}  // namespace point_cloud
