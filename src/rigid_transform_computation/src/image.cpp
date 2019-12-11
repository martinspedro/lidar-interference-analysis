/**
 * \file   image.cpp
 * \brief  Implementation of a class to store image topics names from ROS
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

#include "rigid_transform_computation/image.hpp"

Image::Image()
{
  this->image_color_topic_ = "/image_color";
  this->image_info_topic_ = "/image_info";
}

Image::Image(std::string image_color_topic, std::string image_info_topic)
  : image_color_topic_(image_color_topic), image_info_topic_(image_info_topic)
{
}

std::string Image::getImageColorTopic() const
{
  return this->image_color_topic_;
}

std::string Image::getImageInfoTopic() const
{
  return this->image_info_topic_;
}

void Image::setImageColorTopic(std::string image_color_topic)
{
  this->image_color_topic_ = image_color_topic;
}

void Image::setImageInfoTopic(std::string image_info_topic)
{
  this->image_info_topic_ = image_info_topic;
}
