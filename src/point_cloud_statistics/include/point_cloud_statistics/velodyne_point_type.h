/** \file velodyne_point_type.h
 *  \brief Point Cloud Library Point Dataype Structure for Velodyne data.
 *
 * \remark This point is defined only to avoid pulling external dependencies
 */

#ifndef VELODYNE_POINT_TYPE_H
#define VELODYNE_POINT_TYPE_H

#define PCL_NO_PRECOMPILE

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <boost/shared_ptr.hpp>

namespace velodyne
{
/**
 * \brief Velodyne Point: Euclidean Coordinates, including intensity and laser ring number.
 */
struct PointXYZIR
{
  PCL_ADD_POINT4D;                 //!< quad-word (X, Y, Z, 0) Euclidean Coordinates
  float intensity;                 //!< laser intensity reading
  uint16_t ring;                   //!< laser ring number. uint datatype is forced by the but_velodyne_driver
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  //!< ensure proper alignment

  PointXYZIR()
    : x(std::numeric_limits<float>::quiet_NaN())
    , y(std::numeric_limits<float>::quiet_NaN())
    , z(std::numeric_limits<float>::quiet_NaN())
    , intensity(std::numeric_limits<float>::quiet_NaN())
    , ring(-1)
  {
  }

} EIGEN_ALIGN16;

// Usefull Point Declarations
typedef pcl::PointCloud<velodyne::PointXYZIR> VelodynePointCloud;
typedef boost::shared_ptr<VelodynePointCloud> VelodynePointCloudPtr;
typedef boost::shared_ptr<VelodynePointCloud const> VelodynePointCloudConstPtr;
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
