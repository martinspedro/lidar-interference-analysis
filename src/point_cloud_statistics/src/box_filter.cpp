/**
 * \file   box_filter.cpp
 * \brief Rectangular filter for XYZ Point Clouds
 *
 */

#include "point_cloud_statistics/box_filter.hpp"
#include <ros/assert.h>

BoxFilter::BoxFilter(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
  ROS_ASSERT_MSG(x_max >= x_min, "X axis minimum must be lower than its maximum (%f < %f)! Aborting!", x_max, x_min);
  ROS_ASSERT_MSG(y_max >= y_min, "Y axis minimum must be lower than its maximum (%f < %f)! Aborting!", y_max, y_min);
  ROS_ASSERT_MSG(z_max >= z_min, "Z axis minimum must be lower than its maximum (%f < %f)! Aborting!", z_max, z_min);

  this->x_min_ = x_min;
  this->x_max_ = x_max;

  this->y_min_ = y_min;
  this->y_max_ = y_max;

  this->z_min_ = z_min;
  this->z_max_ = z_max;
}

BoxFilter::BoxFilter()
{
}

BoxFilter::~BoxFilter()
{
}

bool BoxFilter::pointInsideBox(pcl::PointXYZ point)
{
  return ((point.x > this->x_min_) && (point.x < this->x_max_)) &&
         ((point.y > this->y_min_) && (point.y < this->y_max_)) &&
         ((point.z > this->z_min_) && (point.z < this->z_max_));
}

void BoxFilter::printBoxDimensions()
{
  std::cout << "(" << this->x_min_ << ", " << this->x_max_ << "; " << this->y_min_ << ", " << this->y_max_ << "; "
            << this->z_min_ << ", " << this->z_max_ << ")" << std::endl;
}

float BoxFilter::getXMin()
{
  return this->x_min_;
}

float BoxFilter::getYMin()
{
  return this->y_min_;
}
float BoxFilter::getZMin()
{
  return this->z_min_;
}

float BoxFilter::getXMax()
{
  return this->x_max_;
}

float BoxFilter::getYMax()
{
  return this->y_max_;
}

float BoxFilter::getZMax()
{
  return this->z_max_;
}

void BoxFilter::setXMin(float x_min)
{
  this->x_min_ = x_min;
}

void BoxFilter::setYMin(float y_min)
{
  this->y_min_ = y_min;
}

void BoxFilter::setZMin(float z_min)
{
  this->z_min_ = z_min;
}

void BoxFilter::setXMax(float x_max)
{
  this->x_max_ = x_max;
}

void BoxFilter::setYMax(float y_max)
{
  this->y_max_ = y_max;
}

void BoxFilter::setZMax(float z_max)
{
  this->z_max_ = z_max;
}
