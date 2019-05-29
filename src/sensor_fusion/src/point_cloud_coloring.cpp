/**
 * @file   point_cloud_coloring.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 * @date   Created on May 18, 2019, 12:25
 */


#include "automatic_calibration/automatic_calibration.hpp"
#include "automatic_calibration/image_visualizer.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>


#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>

#include <stdint.h>

#define VIEWER_NAME "Colored Cloud Viewer" //!< Viewer Name

/** A structure to represent RGB Triplets
 */
typedef struct color {
    uint8_t r; //!< Red  component
    uint8_t g; //!< Green component
    uint8_t b; //!< Blue  component
} color_t;

pcl::visualization::CloudViewer viewer(VIEWER_NAME); //!< Create visualization object

geometry_msgs::TransformStamped transformStamped; //!< Create geometric transform object

ros::Publisher pub; //!< ROS Publisher



/** @brief Colors Point Cloud Voxels function
 *
 * Colors every point cloud voxel with the same color by iterating through each voxel RGB parameter
 *
 * @param[in] cloud Pointer to the XYZRGB PointCloud
 * @param[in] rgb   RGB triplet dataype
*/
void color_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, color_t rgb) {
    for (int i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].r = rgb.r;
        cloud->points[i].g = rgb.g;
        cloud->points[i].b = rgb.b;
    }
}


/** @brief Callback to color point cloud data
 *
 * @param[in] point_cloud_msg PointCloud2 messages from ROS nodes
*/
void callback_pcl(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {
    // create Point Cloud with Intensity Field

    PointCloudRGB point_cloud;
    PointCloudRGB point_cloud_camera;

    PointCloudRGB::Ptr cloudPtr(new PointCloudRGB);
    PointCloudRGB::Ptr cloudCameraPtr(new PointCloudRGB);

    sensor_msgs::PointCloud2 cloud_out, cloud_colored;

    // Select RGB color
    color_t point_cloud_RGB;
    point_cloud_RGB.r = 255;
    point_cloud_RGB.g = 123;
    point_cloud_RGB.b = 10;


    // Convert ROS msg to Point Cloud
    fromROSMsg(*point_cloud_msg, point_cloud);

    // Initialize pointer to point cloud data
    *cloudPtr = point_cloud;

    // Perform color manipulation on Point Cloud
    color_point_cloud(cloudPtr, point_cloud_RGB);

    pcl::toROSMsg(*cloudPtr,cloud_colored );

    try {
        tf2::doTransform(cloud_colored, cloud_out, transformStamped);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
    }


    // Add new data to viewer
    viewer.showCloud(cloudPtr);

    // Publish the data.
    pub.publish(cloud_out);
}


int main(int argc, char** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "point_cloud_coloring");

  //cv::namedWindow(OPENCV_WINDOW, 0);
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);


  // Ros subscriber for ros msg for Point Cloud
  ros::Subscriber sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, callback_pcl);
  sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, callback_pcl);
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  //ImageVisualizer image_visualizer_object;
  transformStamped = tfBuffer.lookupTransform("camera_color_left", "velo_link", ros::Time(0), ros::Duration(5.0));


  ros::spin();

  return EXIT_SUCCESS;
}
