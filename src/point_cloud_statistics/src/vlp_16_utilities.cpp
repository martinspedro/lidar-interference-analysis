/**
 * \file   vlp_16_utilities.cpp
 * \brief Constants used for VLP16
 *
 */

#include "point_cloud_statistics/vlp_16_utilities.hpp"
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"

namespace velodyne
{
namespace vlp16
{
const unsigned int VLP16_LASER_COUNT = 16u;
const float AZIMUTHAL_ANGULAR_RESOLUTION = 0.2F;
const unsigned int AZIMUTHAL_UNIQUE_ANGLES_COUNT =
    (unsigned int)ceil(point_cloud::organized::FULL_REVOLUTION_DEGREE_F / AZIMUTHAL_ANGULAR_RESOLUTION);

}  // namespace vlp16

}  // namespace velodyne
