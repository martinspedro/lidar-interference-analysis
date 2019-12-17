/**
 * \file   vlp_16_utilities.hpp
 * \brief Constants used for VLP16
 *
 */

#ifndef VLP_16_UTILITIES_H
#define VLP_16_UTILITIES_H

#include <point_cloud_statistics/velodyne_point_type.h>

namespace velodyne
{
/*!
 * \namespace vlp16
 * \brief nested namespace that defines Velodyne VLP-16 related definitions
 */
namespace vlp16
{
extern const unsigned int VLP16_LASER_COUNT;              //!< Number of lasers in VLP-16
extern const float AZIMUTHAL_ANGULAR_RESOLUTION;          //!< Angular resolution defined for the VLP-16
extern const unsigned int AZIMUTHAL_UNIQUE_ANGLES_COUNT;  //!< Unique azimuth angles

/*
 * \brief gets the corresponding Laser ID from the point polar angle
 * \param[in] polar_angle Polar Angle of the point
 * \return the LASER ID of the VLP-16, from 0 to 15
 * \throws std::out_of_range if polar angle is invalid
 *
 * \see getPolarAngle
 */
const unsigned int getLaserIDfromPolarAngle(const int polar_angle);

}  // namespace vlp16

}  // namespace velodyne

#endif  // VLP_16_UTILITIES_H
