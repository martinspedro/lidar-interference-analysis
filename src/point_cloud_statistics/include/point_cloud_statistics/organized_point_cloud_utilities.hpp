/**
 * \file   organized_pointcloud.hpp
 * \brief
 *
 */

#ifndef ORGANIZED_POINT_CLOUD_UTILITIES_H
#define ORGANIZED_POINT_CLOUD_UTILITIES_H

#include <pcl/point_cloud.h>

namespace point_cloud
{
namespace organized
{
extern const float RADIAN_TO_DEGREE_F;
extern const double RADIAN_TO_DEGREE_D;
extern const long double RADIAN_TO_DEGREE_LD;

extern const float DEGREE_OFFSET_TO_POSITIVE_ANGLE_F;
extern const double DEGREE_OFFSET_TO_POSITIVE_ANGLE_D;
extern const long double DEGREE_OFFSET_TO_POSITIVE_ANGLE_L;

extern const float FULL_REVOLUTION_DEGREE_F;
extern const double FULL_REVOLUTION_DEGREE_D;
extern const long double FULL_REVOLUTION_DEGREE_L;

float getAzimuth(float y, float x);
double getAzimuth(double y, double x);
long double getAzimuth(long double y, long double x);

float computeEuclideanDistance(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2);
double computeEuclideanDistance(double x_1, double y_1, double z_1, double x_2, double y_2, double z_2);
long double computeEuclideanDistance(long double x_1, long double y_1, long double z_1, long double x_2,
                                     long double y_2, long double z_2);
}  // namespace organized
}  // namespace point_cloud

#endif  // ORGANIZED_POINT_CLOUD_UTILITIES_H
