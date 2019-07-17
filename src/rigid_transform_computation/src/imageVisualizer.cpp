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

#include "rigid_transform_computation/Pixel.h"

using namespace cv;
const uint8_t DEFAULT_PIXEL_QUEUE_SIZE = 10;
const std::string DEFAULT_CAMERA_TOPIC = "/camera";



ImageVisualizer::ImageVisualizer(std::string camera_root_topic,
                                 std::string clicked_pixel_topic) : it_(nh_) {
    std::cout << "Init Success 1" << std::endl;

    this->camera_root_topic = camera_root_topic;

    this->image = new Image();
    //this->nh_ = ros::NodeHandlePtr(new ros::NodeHandle("image"));
    this->nh_ = ros::NodeHandle("image");
    //this->it_ = image_transport::ImageTransport(new image_transport::ImageTransport(*(this->nh_)));
    this->it_ = image_transport::ImageTransport(this->nh_);

    // Subscribe to input video feed and publish output video feed
    this->image_sub_ = this->it_.subscribe(this->camera_root_topic + this->image->getImageColorTopic(),
                               DEFAULT_QUEUE_SIZE,
                               &ImageVisualizer::viewerCallback, this);
    //this->image_pub_ = this->it_.advertise(this->camera_root_topic + this->image->getImageInfoTopic(),
    //                           DEFAULT_QUEUE_SIZE);

    this->cv_ptr = nullptr;

    //this->pixel_pub = new ros::Publisher(this->nh_.advertise<rigid_transform_computation::Pixel>(clicked_pixel_topic, DEFAULT_PIXEL_QUEUE_SIZE);

    cv::namedWindow(OPENCV_WINDOW_NAME, cv::WINDOW_NORMAL);
}

ImageVisualizer::ImageVisualizer(std::string camera_root_topic,
                                 std::string clicked_pixel_topic,
                                 std::string node_handler_name) : it_(nh_) {
    std::cout << "Init Started" << std::endl;
    this->camera_root_topic = camera_root_topic;

    this->image = new Image();
    // Subscribe to input video feed and publish output video feed
    this->image_sub_ = this->it_.subscribe(camera_root_topic + this->image->getImageColorTopic(),
                               DEFAULT_QUEUE_SIZE,
                               &ImageVisualizer::viewerCallback, this);

    //this->image_pub_ = this->it_.advertise(camera_root_topic + this->image->getImageInfoTopic(),
    //                           DEFAULT_QUEUE_SIZE);

    this->nh_ = ros::NodeHandle(node_handler_name);
    //this->nh_ = ros::NodeHandlePtr(new ros::NodeHandle(node_handler_name));

    this->cv_ptr = nullptr;
    std::cout << "Pointer Allocated" << std::endl;
    this->pixel_pub = this->nh_.advertise<rigid_transform_computation::Pixel>
                                             (clicked_pixel_topic, DEFAULT_PIXEL_QUEUE_SIZE);

    cv::namedWindow(OPENCV_WINDOW_NAME, WINDOW_NORMAL);
    std::cout << "Init Success" << std::endl;
}

ImageVisualizer::~ImageVisualizer() {
    cv::destroyWindow(OPENCV_WINDOW_NAME);
    //delete this->image;
    delete this;
}

/** @brief Callback function for image visualization
*
*/
void ImageVisualizer::viewerCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        //this->cv_ptr = cv_bridge::toCvShare(msg, "bgr8")->image);
        std::cout << "Reading Image MSg" << std::endl;
        this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        std::cout << "ReadImage MSg Success" << std::endl;
        this->tempMouseCallback.imgPtr = this->cv_ptr;
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Update GUI Window
    std::cout << "Im show" << std::endl;
    cv::imshow(OPENCV_WINDOW_NAME, this->cv_ptr->image);
    cv::waitKey(3);
}


void ImageVisualizer::pixelGrabberCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        //this->cv_ptr =  cv_bridge::toCvShare(msg, "bgr8")->image);
        this->cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);;
        //setMouseCallback(OPENCV_WINDOW_NAME, ImageVisualizer::onMouse, &(this->cv_ptr->image)); // pass the address

    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW_NAME, this->cv_ptr->image);
    cv::waitKey(3);
}

void ImageVisualizer::registerPixelPickingCallback(){
    //this->tempMouseCallback;
    //tempMouseCallback = (struct MouseCallbackParams*) malloc(sizeof(struct MouseCallbackParams));

    //this->tempMouseCallback.imgPtr = this->cv_ptr;
    this->tempMouseCallback.pixelPublisherPtr = &(this->pixel_pub);

    setMouseCallback(OPENCV_WINDOW_NAME, ImageVisualizer::onMouse, &(this->tempMouseCallback));
}

void ImageVisualizer::onMouse(int event, int x, int y, int flags, void* param) {
    std::cout << "Callback" << std::endl;
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        //Mat &img_ptr = *((Mat*)param); //cast and deference the param
        //ImageVisualizer aux = *((ImageVisualizer*)param );
        MouseCallbackParams &tempMouseCallback = *((MouseCallbackParams*) param);

        if(tempMouseCallback.imgPtr) {
            Mat &imgPtr = tempMouseCallback.imgPtr->image;

            Vec3b val = imgPtr.at<Vec3b>(y,x); // opencv is row-major !
            std::cout << imgPtr.type() << std::endl;


            rigid_transform_computation::Pixel tempPixel;
            tempPixel.x = x;
            tempPixel.y = y;
            tempPixel.b = val[0];    //BGR color coding
            tempPixel.g = val[1];
            tempPixel.r = val[2];



            std::cout << "Attempting to publish message" << std::endl;

            tempMouseCallback.pixelPublisherPtr->publish(tempPixel);
            std::cout << "x= " << x << " y= " << y << "val= "<< val << std::endl;

            // Should clean last image
            tempMouseCallback.imgPtr = nullptr;

        }
        else {
            ROS_WARN("ERROR!");
        }





    }

    return ;
}
