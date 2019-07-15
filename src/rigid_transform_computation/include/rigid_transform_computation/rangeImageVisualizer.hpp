/**
 * @file   point_cloud_coloring.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 * @date   Created on May 18, 2019, 12:25
 */


#ifndef RANGE_IMAGE_VISUALIZER_H
#define RANGE_IMAGE_VISUALIZER_H

#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include <pcl_ros/point_cloud.h>


class RangeImageVisualizer {
    public:
        RangeImageVisualizer(std::string point_cloud_topic, std::string node_handler_name);
        ~RangeImageVisualizer();


    private:
        void rangeImageCallback(const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg);
        std::string node_handler_name;
        std::string point_cloud_topic;
        ros::NodeHandlePtr nh_;
        ros::Subscriber point_cloud_sub;

        point_cloud::PointCloud point_cloud;
        point_cloud::PointCloud::Ptr cloudPtr;

        pcl::visualization::RangeImageVisualizer::Ptr range_image_widget;
};

#endif // RANGE_IMAGE_VISUALIZER_H
