/**
 * \file   vlp_16_utilities.hpp
 * \brief Constants used for VLP16
 *
 */

#ifndef VLP_16_UTILITIES_H
#define VLP_16_UTILITIES_H

#include <point_cloud_statistics/velodyne_point_type.h>
#include <pcl/point_cloud.h>

namespace velodyne
{
typedef pcl::PointCloud<velodyne::PointXYZIR> VelodynePointCloud;

namespace vlp16
{
extern const unsigned int VLP16_LASER_COUNT;
extern const float AZIMUTHAL_ANGULAR_RESOLUTION;
extern const unsigned int AZIMUTHAL_UNIQUE_ANGLES_COUNT;

}  // namespace vlp16

}  // namespace velodyne

#endif  // VLP_16_UTILITIES_H
