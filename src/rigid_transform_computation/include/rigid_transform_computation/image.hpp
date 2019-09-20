/**
 * \file   image.hpp
 * \brief  Implementation of a class to store image topics
 *
 */

#ifndef IMAGE_H
#define IMAGE_H

#include <string>

/**
 * \class Image
 * \brief Class to store ROS image ropics
 *
 */
class Image
{
public:
  /*
   * \brief Default Constructor
   *
   * Image Color topic default as image_color
   * Image info topic default is /image_info
   */
  Image();

  /*
   * \brief Constructor
   *
   * \param[in] image_color_topic  Image color topic name
   * \param[in] image_info_topic   Image info topic name
   */
  Image(std::string image_color_topic, std::string image_info_topic);

  /*
   * \brief  Getter for image_color_topic
   * \return image_color_topic name
   */
  std::string getImageColorTopic() const;

  /*
   * \brief  Getter for image_info_topic
   * \return const string with image_info_topic name
   */
  std::string getImageInfoTopic() const;

  /*
   * \brief     Setter for image_info_topic
   * \param[in] image_color_topic string with the image color topic name
   */
  void setImageColorTopic(std::string image_color_topic);

  /*
   * \brief     Setter for image_info_topic
   * \param[in] image_info_topic string with the image info topic name
   */
  void setImageInfoTopic(std::string image_info_topic);

private:
  std::string image_color_topic_;  //!< ROS topic name for the color topic of an image
  std::string image_info_topic_;   //!< ROS topic name for the camera information
};

#endif /* IMAGE_H */
