/**
 * @file   point_cloud_coloring.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 * @date   Created on May 18, 2019, 12:25
 */


#ifndef POINT_CLOUD_VISUALIZER_H
#define POINT_CLOUD_VISUALIZER_H

#include <string>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl_ros/point_cloud.h>




namespace point_cloud {
    typedef pcl::PointXYZI   PointI;
    typedef pcl::PointXYZRGB PointRGB;

    typedef pcl::PointCloud<pcl::PointXYZ>    PointCloud;
    typedef pcl::PointCloud<pcl::PointXYZI>   PointCloudI;
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;



    class PointCloudVisualizer{
        public:
            PointCloudVisualizer(std::string point_cloud_topic, std::string node_handler_name);
            ~PointCloudVisualizer();

            void initPointCloudVisualizer();
            void addNewPointCloudToVisualizer(const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg, std::string point_cloud_name);

            void registerPointPickingCallback(const uint8_t MODE);
            pcl::visualization::PCLVisualizer::Ptr pcl_viewer;

            static const uint8_t SINGLE_POINT_MODE = 0;
        private:


            std::string node_handler_name;
            std::string point_cloud_topic;
            ros::NodeHandlePtr nh_;
            ros::Subscriber point_cloud_sub;

            PointCloud point_cloud;
            PointCloud::Ptr cloudPtr;

            static void onPointPickingEvent (const pcl::visualization::PointPickingEvent& pickingEvent, void* viewerVoidPtr);
            void viewerCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);


    };
}


 #endif // POINT_CLOUD_VISUALIZER_H