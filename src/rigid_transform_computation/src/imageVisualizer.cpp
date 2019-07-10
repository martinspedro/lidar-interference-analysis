/**
 * @file   image_visualizer.cpp
 * @brief  Image Stream and Visualizer Header File
 *
 * @author Pedro Martins
 * @date   @today
 */


#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include "rigid_transform_computation/imageVisualizer.hpp"


ImageVisualizer::ImageVisualizer() : it_(nh_) {
    this->camera_topic = "/camera";

    this->image = new Image();
    // Subscribe to input video feed and publish output video feed
    this->image_sub_ = this->it_.subscribe(camera_topic + this->image->getImageColorTopic(),
                               DEFAULT_QUEUE_SIZE,
                               &ImageVisualizer::imageCallback, this);
    this->image_pub_ = this->it_.advertise(camera_topic + this->image->getImageInfoTopic(),
                               DEFAULT_QUEUE_SIZE);

    cv::namedWindow(OPENCV_WINDOW, 0);
}

ImageVisualizer::ImageVisualizer(std::string camera_topic = "/camera") : it_(nh_) {
    this->camera_topic = camera_topic;

    this->image = new Image();
    // Subscribe to input video feed and publish output video feed
    this->image_sub_ = this->it_.subscribe(camera_topic + this->image->getImageColorTopic(),
                               DEFAULT_QUEUE_SIZE,
                               &ImageVisualizer::imageCallback, this);
    this->image_pub_ = this->it_.advertise(camera_topic + this->image->getImageInfoTopic(),
                               DEFAULT_QUEUE_SIZE);

    cv::namedWindow(OPENCV_WINDOW, 0);
}

ImageVisualizer::~ImageVisualizer() {
    cv::destroyWindow(OPENCV_WINDOW);
    delete this->image;
}

/** @brief Callback function for image visualization
*
*/
void ImageVisualizer::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;

    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
}
