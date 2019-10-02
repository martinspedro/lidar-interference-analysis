
#ifndef POINT_STATISTICS_CONTAINER_H
#define POINT_STATISTICS_CONTAINER_H

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

template <class PointT>
struct PointStatisticsContainer
{
  union
  {
    float data[4];
    struct
    {
      // All this Coordinates are mean coordinates but their name does not contain mean in order to allow PCL to use the
      // mean point as an actual point
      float x;  //!< Mean Euclidean X Coordinate
      float y;  //!< Mean Euclidean Y Coordinate
      float z;  //!< Mean Euclidean Z Coordinate
    };
  };

  unsigned int ring;  //!< laser ring number
  float intensity;
  union
  {
    float data_i[4];
    struct
    {
      float intensity_mean;  //!< Mean Intensity (does not have mean because PCL recognizes the field and can use it in
                             //!< its custom algorithm)
      float intensity_var;
      float intensity_min;
      float intensity_max;
    };
  };

  union
  {
    float data_d[4];
    struct
    {
      float distance_mean;
      float distance_var;
      float distance_min;
      float distance_max;
    };
  };

  std::vector<PointT> data_points;       /** Regist of all that points that have been stored on this point type */
  std::vector<float> euclidean_distance; /** Euclidean distance between all the points registered on data_points
                                           vector*/

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                   // enforce SSE padding for correct memory alignment

#endif  // POINT_STATISTICS_CONTAINER_H
