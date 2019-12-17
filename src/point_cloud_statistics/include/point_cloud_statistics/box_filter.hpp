/*!
 * \file   box_filter.hpp
 * \brief Rectangular filter for XYZ Point Clouds
 *
 */

#ifndef BOX_FILTER_H
#define BOX_FILTER_H

#include <pcl/point_types.h>

/*!
 * \brief Class that implements a box filter
 *
 * Rectangular Box filter for PointXYZ point clouds
 */
class BoxFilter
{
public:
  /*!
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

  /*!
   * \brief Filter object constructor
   */
  BoxFilter();

  /*!
   * \brief Filter object destructor
   */
  ~BoxFilter();

  /*!
   * \brief Checks if point is inside box dimensions
   *
   * \param[in] point a pcl::PointXYZ object with three cartesian coordinates
   * \retval True if point is inside the box dimensions of the filter
   * \retval False if point is outside the box dimensions of the filter
   */
  bool pointInsideBox(pcl::PointXYZ point);

  /*!
   * \brief Prints Box Dimensions
   */
  void printBoxDimensions();

  /*!
   * \brief Getter for X component minimum value
   * \return Box Filter X component minimum value
   */
  float getXMin();

  /*!
   * \brief Getter for Y component minimum value
   * \return Box Filter y component minimum value
   */
  float getYMin();

  /*!
   * \brief Getter for Z component minimum value
   * \return Box Filter Z component minimum value
   */
  float getZMin();

  /*!
   * \brief Getter for X component maximum value
   * \return Box Filter X component maximum value
   */
  float getXMax();

  /*!
   * \brief Getter for Y component maximum value
   * \return Box Filter Y component maximum value
   */
  float getYMax();

  /*!
   * \brief Getter for Z component maximum value
   * \return Box Filter Z component maximum value
   */
  float getZMax();

  /*!
   * \brief Setter for X component minimum value
   * \param[in] x_min Box Filter X component minimum value
   */
  void setXMin(float x_min);

  /*!
   * \brief Setter for Y component minimum value
   * \param[in] y_min Box Filter Y component minimum value
   */
  void setYMin(float y_min);

  /*!
   * \brief Setter for Z component minimum value
   * \param[in] z_min Box Filter Z component minimum value
   */
  void setZMin(float z_min);

  /*!
   * \brief Setter for X component maximum value
   * \param[in] x_max Box Filter X component maximum value
   */
  void setXMax(float x_max);

  /*!
   * \brief Setter for Y component maximum value
   * \param[in] y_max Box Filter Y component maximum value
   */
  void setYMax(float y_max);

  /*!
   * \brief Setter for Z component maximum value
   * \param[in] z_max Box Filter Z component maximum value
   */
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
