/**
 * @file   point_cloud_coloring.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 * @date   Created on May 18, 2019, 12:25
 */

#include "automatic_calibration/automatic_calibration.hpp"
#include "automatic_calibration/image_visualizer.hpp"



// Create viewer object
pcl::visualization::CloudViewer viewer("Colored Cloud Viewer");

void color_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < cloud->points.size(); i++) {
        cloud->points[i].r = r;
        cloud->points[i].g = g;
        cloud->points[i].b = b;
    }
}


void callback_pcl(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
    // create Point Cloud with Intensity Field
    PointCloudRGB point_cloud;

    // Convert ROS msg to Point Cloud
    fromROSMsg(*point_cloud_msg, point_cloud);

    // pack r/g/b into rgb
    uint8_t r = 255, g = 123, b = 10;    // Example: Red color
    //uint32_t rgb = (uint32_t)(r << 16 | g << 8 | b);

    // Create dynamic pointer to point cloud data
    PointCloudRGB::Ptr cloudPtr(new PointCloudRGB);
    *cloudPtr = point_cloud;

    // Perform color manipulation on Point Cloud
    color_point_cloud(cloudPtr, r, g, b);



    // Add new data to viewer
    viewer.showCloud(cloudPtr);

    // Publish the data
    //pub.publish(cloudPtr);
}


int main(int argc, char** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "point_cloud_coloring");
  cv::namedWindow(OPENCV_WINDOW, 0);
  ros::NodeHandle nh;

  // Ros subscriber for ros msg for Point Cloud
  ros::Subscriber sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, callback_pcl);
  //ros::Subscriber sub_image = nh.subscribe<sensor_msgs::Image>("/camera/image_color/raw", 1, callback_image);
  //ImageVisualizer image_visualizer_object;
  ros::spin();

  return EXIT_SUCCESS;
}
