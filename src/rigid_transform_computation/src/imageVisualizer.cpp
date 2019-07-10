/**
 * @file   image_visualizer.cpp
 * @brief  Image Stream and Visualizer Header File
 *
 * @author Pedro Martins
 * @date   @today
 */


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>

#include "rigid_transform_computation/imageVisualizer.hpp"

using namespace cv;


ImageVisualizer::ImageVisualizer() : it_(nh_) {
    this->camera_topic = "/camera";

    this->image = new Image();
    // Subscribe to input video feed and publish output video feed
    this->image_sub_ = this->it_.subscribe(camera_topic + this->image->getImageColorTopic(),
                               DEFAULT_QUEUE_SIZE,
                               &ImageVisualizer::pixelGrabberCallback, this);
    this->image_pub_ = this->it_.advertise(camera_topic + this->image->getImageInfoTopic(),
                               DEFAULT_QUEUE_SIZE);

    this->cv_ptr = nullptr;

    cv::namedWindow(OPENCV_WINDOW_NAME, WINDOW_NORMAL);
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

    cv::namedWindow(OPENCV_WINDOW_NAME, WINDOW_NORMAL);
}

ImageVisualizer::~ImageVisualizer() {
    cv::destroyWindow(OPENCV_WINDOW_NAME);
    delete this->image;
    delete this;
}

/** @brief Callback function for image visualization
*
*/
void ImageVisualizer::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW_NAME, this->cv_ptr->image);
    cv::waitKey(3);
}

void ImageVisualizer::pixelGrabberCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);;
        setMouseCallback(OPENCV_WINDOW_NAME, ImageVisualizer::onMouse, &(this->cv_ptr->image)); // pass the address
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW_NAME, this->cv_ptr->image);
    cv::waitKey(3);
}

void ImageVisualizer::onMouse(int event, int x, int y, int flags, void* param) // now it's in param
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        Mat &img_ptr = *((Mat*)param); //cast and deref the param
        Vec3b val = img_ptr.at<Vec3b>(y,x); // opencv is row-major !
        std::cout << img_ptr.type() << std::endl;
        std::cout << "x= " << x << " y= " << y << "val= "<<val<< std::endl;
    }

    return ;
}
