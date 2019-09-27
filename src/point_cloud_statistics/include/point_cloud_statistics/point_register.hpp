
#ifndef POINT_REGISTER_H
#define POINT_REGISTER_H

#define PCL_NO_PRECOMPILE

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

template <class T>
struct PointRegister
{
  float distance_mean;
  float distance_var;
  // float distance_sigma;
  float intensity_mean;
  float intensity_var;
  // float distance_sigma;
  int ring;

  union
  {
    float data_xyz[4];
    struct
    {
      float x;
      float y;
      float z;
    };
  };

  std::vector<T> data;
  std::vector<float> distance;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                   // enforce SSE padding for correct memory alignment

/*
// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(PointRegister,
                                  (float, distance_mean, distance_mean)
                                  (float, distance_var, distance_var)
                                  (float, intensity_mean, intensity_mean)
                                  (float, intensity_sigma, intensity_sigma)
                                  (std::vector<double>, distance, distance)
                                  (std::vector<double>, intensity, intensity))
// clang-format on
*/

#endif  // POINT_REGISTER_H
