/**
 * \file   vlp_16_utilities.cpp
 * \brief Constants used for VLP16
 *
 */

#include "point_cloud_statistics/vlp_16_utilities.hpp"
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"

#include <map>

namespace velodyne
{
namespace vlp16
{
const unsigned int VLP16_LASER_COUNT = 16u;
const float AZIMUTHAL_ANGULAR_RESOLUTION = 0.2F;
const unsigned int AZIMUTHAL_UNIQUE_ANGLES_COUNT =
    (unsigned int)ceil(point_cloud::organized::FULL_REVOLUTION_DEGREE_F / AZIMUTHAL_ANGULAR_RESOLUTION);

// clang-format off
const std::map<const int, const unsigned int> polar_degree_to_laser_id_map = {
  { -15,  0 },
  { -13,  1 },
  { -11,  2 },
  {  -9,  3 },
  {  -7,  4 },
  {  -5,  5 },
  {  -3,  6 },
  {  -1,  7 },
  {   1,  8 },
  {   3,  9 },
  {   5, 10 },
  {   7, 11 },
  {   9, 12 },
  {  11, 13 },
  {  13, 14 },
  {  15, 15 },
};
// clang-format on

const unsigned int getLaserIDfromPolarAngle(const int polar_angle)
{
  std::map<int, const unsigned int>::const_iterator it = polar_degree_to_laser_id_map.find(polar_angle);
  if (it != polar_degree_to_laser_id_map.end())
  {
    return it->second;  // return the value of the key
  }
  else
  {
    throw std::out_of_range("Polar Angle is invalid");
  }
}

}  // namespace vlp16

}  // namespace velodyne
