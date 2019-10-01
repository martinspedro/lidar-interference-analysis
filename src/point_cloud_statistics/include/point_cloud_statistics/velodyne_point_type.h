/** \file velodyne_point_type.h
 *  \brief Point Cloud Library Point Dataype Structure for Velodyne data.
 *
 * \remark This point is defined only to avoid pulling external dependencies
 */

#ifndef VELODYNE_POINT_TYPE_H
#define VELODYNE_POINT_TYPE_H

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/instantiate.hpp>
#include <boost/shared_ptr.hpp>

namespace velodyne
{
/** Velodyne Point: Euclidean Coordinates, including intensity and laser ring number. */
struct PointXYZIR
{
  PCL_ADD_POINT4D;                 //!< quad-word XYZ
  float intensity;                 //!< laser intensity reading
  unsigned int ring;               //!< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  //!< ensure proper alignment
} EIGEN_ALIGN16;

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,
                                    (float, x, x)
                                    (float, y, y)
                                    (float, z, z)
                                    (float, intensity, intensity)
                                    (unsigned int, ring, ring)
                                 )
// clang-format on

};  // namespace velodyne

#endif  // VELODYNE_POINT_TYPE_H
