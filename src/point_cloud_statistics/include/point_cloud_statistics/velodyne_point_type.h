/*! \file velodyne_point_type.h
 *  \brief Point Cloud Library Point Dataype Structure for Velodyne data.
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 * \remark This point is defined only to avoid pulling external dependencies
 */

#ifndef VELODYNE_POINT_TYPE_H
#define VELODYNE_POINT_TYPE_H

#define PCL_NO_PRECOMPILE

#include <climits>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/shared_ptr.hpp>
#include <ostream>

/*!
 * \namespace velodyne
 * \brief contaimns all the definitions related to velodyne datatype manipulation
 */
namespace velodyne
{
const float DEFAULT_FLOAT_VALUE = std::numeric_limits<float>::quiet_NaN();  //!< Default float value to init the point-
const uint16_t DEFAULT_RING_VALUE = USHRT_MAX;  //!< Default uint16_t value to init the point-

/*!
 * \struct PointXYZIR
 * \brief Velodyne Point: Euclidean Coordinates, including intensity and laser ring number.
 */
struct PointXYZIR
{
  PCL_ADD_POINT4D;  //!< quad-word (X, Y, Z, 0) Euclidean Coordinates
  float intensity;  //!< laser intensity reading
  uint16_t ring;    //!< laser ring number. uint datatype is forced by the but_velodyne_driver
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  //!< ensure proper alignment

/*!
 * \brief blank constructor that inits all points to their default values
 */
  PointXYZIR()
    : x(DEFAULT_FLOAT_VALUE)
    , y(DEFAULT_FLOAT_VALUE)
    , z(DEFAULT_FLOAT_VALUE)
    , intensity(DEFAULT_FLOAT_VALUE)
    , ring(DEFAULT_RING_VALUE)
  {
  }

  /*!
   * \brief Overload stream operator for constant PointXYZIR objects
   */
  friend std::ostream& operator<<(std::ostream& out, const PointXYZIR& object)
  {
    out << "(" << object.x << ", " << object.y << ", " << object.z << ", " << object.intensity << ", " << object.ring
        << ")";
    return out;
  }

  /*!
   * \brief Overload stream operator for PointXYZIR objects
   */
  friend std::ostream& operator<<(std::ostream& out, PointXYZIR& object)
  {
    out << "(" << object.x << ", " << object.y << ", " << object.z << ", " << object.intensity << ", " << object.ring
        << ")";
    return out;
  }

} EIGEN_ALIGN16;

// Usefull Point Declarations
typedef pcl::PointCloud<velodyne::PointXYZIR> VelodynePointCloud;     //!< PCL Point Cloud with Velodyne Points
typedef boost::shared_ptr<VelodynePointCloud> VelodynePointCloudPtr;  //!< Shared Boost Pointer to a Velodyne Point
                                                                      //!< Cloud
typedef boost::shared_ptr<VelodynePointCloud const> VelodynePointCloudConstPtr;  //!< Shared Boost Pointer to a constant
                                                                                 //!< Velodyne Point Cloud

using Ptr = boost::shared_ptr<PointXYZIR>;
using ConstPtr = boost::shared_ptr<const PointXYZIR>;

};  // namespace velodyne

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne::PointXYZIR,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (uint16_t, ring, ring)
                                 )
// clang-format on

#endif  // VELODYNE_POINT_TYPE_H
