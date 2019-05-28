/**
 * @file   point_cloud_coloring.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 * @date   Created on May 18, 2019, 12:25
 */


 #include <sensor_msgs/PointCloud2.h>

 #include <tf2_sensor_msgs/tf2_sensor_msgs.h>
 //

#include "automatic_calibration/automatic_calibration.hpp"
#include "automatic_calibration/image_visualizer.hpp"

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
//#include <transform_point_cloud/LookupTransformConfig.h>
//#include "tf2_bullet/tf2_bullet.h"
//#include <iostream>

// Create viewer object
pcl::visualization::CloudViewer viewer("Colored Cloud Viewer");
//const geometry_msgs::TransformStamped transform2;
// Create TF Buffer


ros::Publisher pub;

geometry_msgs::TransformStamped transformStamped;

void color_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].r = r;
        cloud->points[i].g = g;
        cloud->points[i].b = b;
    }
}


void callback_pcl(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
    // create Point Cloud with Intensity Field
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    PointCloudRGB point_cloud;
    PointCloudRGB point_cloud_camera;
    PointCloudRGB::Ptr cloudCameraPtr(new PointCloudRGB);
    // Convert ROS msg to Point Cloud
    fromROSMsg(*point_cloud_msg, point_cloud);
sensor_msgs::PointCloud2 cloud_out, cloud_colored;
    // pack r/g/b into rgb
    uint8_t r = 255, g = 123, b = 10;    // Example: Red color
    //uint32_t rgb = (uint32_t)(r << 16 | g << 8 | b);

    // Create dynamic pointer to point cloud data
    PointCloudRGB::Ptr cloudPtr(new PointCloudRGB);
    *cloudPtr = point_cloud;

//    ROS_INFO("%u", cloudPtr->points[0].x);
//    ROS_INFO("%f", cloudPtr->points[0].x);
//    ROS_INFO("%d", point_cloud.points[0].x);
//    ROS_INFO("%f", point_cloud.points[0].x);


    // Perform color manipulation on Point Cloud
    color_point_cloud(cloudPtr, r, g, b);
    pcl::toROSMsg(*cloudPtr,cloud_colored );
    try{
        transformStamped = tfBuffer.lookupTransform("velo_link", "imu_link", ros::Time::now(), ros::Duration(0.3)); //point_cloud_msg->header.stamp);
        tf2::doTransform(cloud_colored, cloud_out, transformStamped);
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      //ros::Duration(1.0).sleep();
    }

    //tf::TransformListener::transformPointCloud("camera_color_left", *transform2, cloudPtr);
    //tf2::doTransform(&cloudPtr, &cloudCameraPtr, &transform2);
    //tf2::Transform(&point_cloud_msg, "camera_color_left", ros::Time(0), "velo_link", 0);
    // Add new data to viewer
    viewer.showCloud(cloudPtr);

    // Convert to ROS data type
    //sensor_msgs::PointCloud2 output;
    //pcl_conversions::fromPCL(&cloudCameraPtr, output);

    // Publish the data.
    pub.publish(cloud_out);

    // Publish the data
    //pub.publish(cloudPtr);
}


int main(int argc, char** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "point_cloud_coloring");

  //cv::namedWindow(OPENCV_WINDOW, 0);
  ros::NodeHandle nh;



  ros::Rate rate(1.0);
  // Ros subscriber for ros msg for Point Cloud
  ros::Subscriber sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, callback_pcl);
  sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, callback_pcl);
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
  //ros::Subscriber sub_image = nh.subscribe<sensor_msgs::Image>("/camera/image_color/raw", 1, callback_image);
  //ImageVisualizer image_visualizer_object;
ros::spin();
/*
  while (nh.ok()) {

      //transform2 = new geometry_msgs::TransformStamped();



    //rate.sleep();

    }

*/
  return EXIT_SUCCESS;
}
