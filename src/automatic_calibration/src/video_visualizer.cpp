#include "automatic_calibration/automatic_calibration.hpp"
#include "automatic_calibration/image_visualizer.hpp"


/*
ros::Publisher pub;

void callback(const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg) {
  sensor_msgs::PointCloud2 output_point_cloud;

  PointCloudC::Ptr cloud(new PointCloudC());
//pcl::fromROSMsg(point_cloud_msg, *cloud);

//  pcl_conversions::toPCL(*point_cloud_msg, *cloud);
  ROS_INFO("Got point cloud with %ld points", cloud->size());

  pub.publish(point_cloud_msg);
}

*/


int main(int argc, char** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "image_visualizer");


  /*
  ros::NodeHandle nh;

  // Ros subscriber for input point cloud2
  ros::Subscriber sub = nh.subscribe("velodyne_points", 1, callback);

  // ROS publisher
  pub = nh.advertise<sensor_msgs::PointCloud2>("output_point_cloud", 1);

  ros::NodeHandle nh;

  // Ros subscriber for input point cloud2
  ros::Subscriber sub = nh.subscribe("velodyne_points", 1, callback);

  // ROS publisher
  pub = nh.advertise<sensor_msgs::PointCloud2>("output_point_cloud", 1);
  */

  ImageVisualizer image_visualizer_object;
  ros::spin();

  return EXIT_SUCCESS;
}
