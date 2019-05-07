#include "automatic_calibration/automatic_calibration.hpp"

pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
void callback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {

//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
  //pcl::PointCloud<pcl::PointXYZ> cloud;
//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Convert ROS message to colored point cloud object
  //pcl::fromROSMsg(*point_cloud_msg, &cloud);
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptrCloud(&cloud);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*point_cloud_msg, cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
    *cloudPtr = cloud;
/*
      pcl::PCLPointCloud2 pcl_pc2;
      pcl_conversions::toPCL(*point_cloud_msg,pcl_pc2);
      pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);

*/
  //ROS_INFO("Got point cloud with %ld points", cloud->size());
  //pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(&cloud);

  viewer.showCloud(cloudPtr,"Udacity");
}


int main(int argc, char** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "point_cloud_visualizer");
  ros::NodeHandle nh;


  // Ros subscriber for input point cloud2
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, callback);

  // ROS publisher
  //pub = nh.advertise<sensor_msgs::PointCloud2>("output_point_cloud", 1);

  ros::spin();

  return EXIT_SUCCESS;
}
