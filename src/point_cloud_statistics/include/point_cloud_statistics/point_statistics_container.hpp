/**
 * \file  point_statistics_container.hpp
 * \brief Implementation of a templated Point Cloud Container

 * \author Pedro Martins (martinspedro@ua.pt)
 *
 */
#ifndef POINT_STATISTICS_CONTAINER_H
#define POINT_STATISTICS_CONTAINER_H

#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>

/*!
 * \struct PointStatisticsContainer
 * \brief Container for point cloud data
 * \tparam PointT Generic Point Type, defined to be PCL compatible
 *
 * Contains:
 * - Average coordinate value of the point (X, Y, Z);
 * - Ring value;
 * - Intensity (average, min, max, var)
 * - Distance (average, min, max, var)
 * - vector to regist PointT elements
 * - vector to store the distance of each element to the origin
 */
template <class PointT>
struct PointStatisticsContainer
{
  /**
   * \brief unnamed union to store position data
   */
  union
  {
    float data[4];  //!< C-style array for memory access
    /*!
     * \brief unnamed struct to store XYZ position
     */
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

  /**
   * \brief unnamed union to store intensity data
   */
  union
  {
    float data_i[4];  //!< C-style array for memory access
                      /*!
                       * \brief unnamed struct to store intensity average, variance, minimum and maximum values
                       */
    struct
    {
      float intensity;      //!< Mean Intensity (does not have mean because PCL recognizes the field and can use it in
                            //!< its custom algorithm)
      float intensity_var;  //!< Intensity variance
      float intensity_min;  //!< Minimum Intensity value
      float intensity_max;  //!< Maximum Intensity value
    };
  };

  /**
   * \brief unnamed union to store distance data
   */
  union
  {
    float data_d[4];  //!< C-style array for memory access
                      /*!
                       * \brief unnamed struct to store distance average, variance, minimum and maximum values
                       */
    struct
    {
      float distance_mean;  //!< Average Distance value
      float distance_var;   //!< Distance variance
      float distance_min;   //!< Minimum Distance value
      float distance_max;   //!< Maximum Distance value
    };
  };

  std::vector<PointT> data_points;  //!< Vector to regist of all the points that have been stored on this point type on
                                    //!< this matrix index
  std::vector<float> euclidean_distance;  //!< Vector to regist the Euclidean distance between all the points registered
                                          //!< on data_points vector

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;                   // enforce SSE padding for correct memory alignment

#endif  // POINT_STATISTICS_CONTAINER_H
