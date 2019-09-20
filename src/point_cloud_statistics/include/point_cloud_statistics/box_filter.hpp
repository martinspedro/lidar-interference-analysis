/**
 * \file   box_filter.hpp
 * \brief
 *
 */

#ifndef BOX_FILTER_H
#define BOX_FILTER_H

#include <pcl/point_types.h>

class BoxFilter
{
public:
  BoxFilter(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
  ~BoxFilter();
  bool pointInsideBox(pcl::PointXYZ point);

private:
  float x_min_;
  float x_max_;
  float y_min_;
  float y_max_;
  float z_min_;
  float z_max_;
};

#endif  // BOX_FILTER_H
