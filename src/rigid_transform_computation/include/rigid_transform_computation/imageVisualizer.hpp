/*!
 * \file   imageVisualizer.hpp
 * \brief  Header file for ImageVisualizer class
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

#ifndef IMAGE_VISUALIZER_H
#define IMAGE_VISUALIZER_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

#include <string>

#include "rigid_transform_computation/image.hpp"

/*!
 * \class ImageVisualizer
 * \brief Image Visualizer class
 *
 * Class that uses an OpenCV visualizer to show ROS Image message.
 * It is integrated with ROS but is detach from ROS simulated time for callback oprations, supporting freeze time
 */
class ImageVisualizer
{
public:
  /*!
   * \brief Construct that creates an OpenCV window to visualize the image feed to the topics given
   *
   * \param[in] camera_root_topic string containing the camera main topic, without the transport type
   * \param[in] clicked_pixel_topic name under which to publish the selected pixel data
   *
   * \remark The node handler is the default node handler
   */
  ImageVisualizer(std::string camera_root_topic, std::string clicked_pixel_topic);

  /*!
   * \brief Construct that creates an OpenCV window to visualize the image feed to the topics given
   *
   * \param[in] camera_root_topic string containing the camera main topic, without the transport type
   * \param[in] clicked_pixel_topic name under which to publish the selected pixel data
   * \param[in] node_handler_name the ROS node handler name to be created
   *
   */
  ImageVisualizer(std::string camera_root_topic, std::string clicked_pixel_topic, std::string node_handler_name);

  /*!
   * \brief Destructor
   *
   * Call OpenCV Image Visualizer termination method and deallocates dynamic memory
   */
  ~ImageVisualizer();

  /*!
   * \brief Allows the selection of pixels in the visualizer
   *
   * Enables the callback associated to mouse clicks on the OpenCV Window
   */
  void registerPixelPickingCallback();

  /*!
   * \brief Spins OpenCv visualizer and ROS node
   */
  void spin();

private:
  /*!
   * \brief Callback for every ROS Image message
   *
   * \param[in] msg
   *
   * Copies the image, redraws all the already selected points and updates the Visualizar
   */
  void viewerCallback(const sensor_msgs::ImageConstPtr& msg);

  /*!
   * \brief Callback for every ROS Image message
   *
   * \param[in] event type of event being received by the mouse
   * \param[in] x x coordinate of the selected pixel
   * \param[in] y y coordinate of the selected pixel
   * \param[in] flags
   * \param[in] param parameters passed by the using the MouseCallbackParams structure
   */
  static void onMouse(int event, int x, int y, int flags, void* param);

  const std::string OPENCV_WINDOW_NAME = "Video Viewer";  //! OpenCv window name
  const int DEFAULT_QUEUE_SIZE = 1;                       //! ROS queue size for topics

  Image* image;                   //!< Image object
  std::string camera_root_topic;  //!< Camera root topic name (without transform affixes)

  cv_bridge::CvImagePtr cv_ptr;  //!< CV bridge point ot the ROS Image

  ros::NodeHandle nh_;                  //!< ROS node handler object
  image_transport::ImageTransport it_;  //!< Image transport object

  image_transport::Subscriber image_sub_;  //!< ROS image topic subscriber
  ros::Publisher pixel_pub;                //!< ROS Pixel message publisher

  /*!
   * \struct MouseCallbackParams
   * \brief Structure for passing custom information to OpenCv mouse event callback
   */
  struct MouseCallbackParams
  {
    cv_bridge::CvImagePtr imgPtr;  //!< ROS to Opencv Bridge image pointer to the image where the selection happened
    ros::Publisher* pixelPublisherPtr;  //!< Pointer to ROS publisher of Pixel messages
  };

  struct MouseCallbackParams tempMouseCallback;  //!< MouseCallbackParams object to pass information for mouse eventsm
                                                 //!< callback
};

#endif  // IMAGE_VISUALIZER_H
