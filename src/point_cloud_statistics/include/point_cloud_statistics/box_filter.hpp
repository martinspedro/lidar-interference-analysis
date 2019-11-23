/**
 * \file   box_filter.hpp
 * \brief Rectangular filter for XYZ Point Clouds
 *
 */

#ifndef BOX_FILTER_H
#define BOX_FILTER_H

#include <pcl/point_types.h>

/**
 * \brief Class that implements a box filter
 *
 * Rectangular Box filter for PointXYZ point clouds
 */
class BoxFilter
{
public:
  /**
   * \brief Filter object constructor
   *
   * \param[in] x_min X axis minimum coordinate value of the box
   * \param[in] x_max X axis maximum coordinate value of the box
   * \param[in] y_min Y axis minimum coordinate value of the box
   * \param[in] y_max Y axis maximum coordinate value of the box
   * \param[in] z_min Z axis minimum coordinate value of the box
   * \param[in] z_max Z axis maximum coordinate value of the box
   */
  BoxFilter(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);

  BoxFilter();

  /**
   * \brief Filter object constructor
   */
  ~BoxFilter();

  /**
   * \brief Checks if point is inside box dimensions
   *
   * \param[in] point a pcl::PointXYZ object with three cartesian coordinates
   * \return true if point is inside the box dimensions of the filter
   * \returns false if point is outside the box dimensions of the filter
   */
  bool pointInsideBox(pcl::PointXYZ point);

  void printBoxDimensions();

  float getXMin();
  float getYMin();
  float getZMin();
  float getXMax();
  float getYMax();
  float getZMax();
  void setXMin(float x_min);
  void setYMin(float y_min);
  void setZMin(float z_min);
  void setXMax(float x_max);
  void setYMax(float y_max);
  void setZMax(float z_max);

private:
  float x_min_;  //!< X axis minimum coordinate value of the box
  float x_max_;  //!< X axis maximum coordinate value of the box
  float y_min_;  //!< Y axis minimum coordinate value of the box
  float y_max_;  //!< Y axis maximum coordinate value of the box
  float z_min_;  //!< Z axis minimum coordinate value of the box
  float z_max_;  //!< Z axis maximum coordinate value of the box
};

#endif  // BOX_FILTER_H
