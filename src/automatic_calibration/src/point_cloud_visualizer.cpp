#include "automatic_calibration/automatic_calibration.hpp"

// Create viewer object
pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");


void callback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
    // create Point Cloud with Intensity Field
    pcl::PointCloud<pcl::PointXYZI> point_cloud;

    // Convert ROS msg to Point Cloud
    pcl::fromROSMsg(*point_cloud_msg, point_cloud);

    // Create dynamic pointer to point cloud data
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZI>);
    *cloudPtr = point_cloud;

    // Add new data to viewer
    viewer.showCloud(cloudPtr);
}


int main(int argc, char** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "point_cloud_visualizer");

  ros::NodeHandle nh;

  // Ros subscriber for input point cloud2
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, callback);

  ros::spin();

  return EXIT_SUCCESS;
}
