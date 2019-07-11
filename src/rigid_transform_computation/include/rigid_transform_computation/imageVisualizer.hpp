/**
 * @file   camera.cpp
 * @brief  Image Stream and Visualizer Header File
 *
 * @author Pedro Martins
 * @date   9 july 2019
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


class ImageVisualizer {
    private:
        const std::string OPENCV_WINDOW_NAME = "Video Viewer";
        const int DEFAULT_QUEUE_SIZE = 1;

        Image *image;
        std::string camera_topic;

        cv_bridge::CvImagePtr cv_ptr;


        ros::NodeHandlePtr nh_;
        image_transport::ImageTransport it_;
        image_transport::Subscriber     image_sub_;
        image_transport::Publisher      image_pub_;

    protected:
        static void onMouse(int event, int x, int y, int flags, void* param);

    public:
        ImageVisualizer();
        ImageVisualizer(std::string camera_topic);

        ~ImageVisualizer();

        /** @brief Callback function for image visualization
        *
        */
        void viewerCallback(const sensor_msgs::ImageConstPtr& msg);
        void pixelGrabberCallback(const sensor_msgs::ImageConstPtr& msg);

};
#endif // IMAGE_VISUALIZER_H
