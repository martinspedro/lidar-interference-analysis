#include <ros/ros.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/PointCloud2.h>

#include <string>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <visualization_msgs/Marker.h>
#include "point_cloud_statistics/box_filter.hpp"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

long int msg_count = 0, point_count = 0, out_points_count = 0, in_points_count = 0;

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  std::cout << "Callback" << std::endl;
  static BoxFilter dark_room(-2.0, 7.0, -3.0, 4.5, -1.9, 3.5);  // added 1 meter in every direction

  PointCloud point_cloud;
  fromROSMsg(*msg, point_cloud);

  ++msg_count;

  for (int j = 0; j < point_cloud.points.size(); j++)
  {
    point_count++;
    if (dark_room.pointInsideBox(point_cloud.points[j]))
    {
      ++in_points_count;
    }
    else
    {
      ++out_points_count;
    }
  }
}

void initBoxMarker(visualization_msgs::Marker marker)
{
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "velodyne";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD and DELETE
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 3.0;  // -2.0;
  marker.pose.position.y = 0.5;  //-3.0;
  marker.pose.position.z = 0.9;  //-1.9;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 7.5;  // 7.0;
  marker.scale.y = 5.5;  // 4.5;
  marker.scale.z = 3.0;  // 3.5;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 0.5;

  marker.lifetime = ros::Duration();
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_histogram");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, callback);

  ros::Rate r(10);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visual_marker_pub", 1);
  visualization_msgs::Marker marker;
  initBoxMarker(marker);

  while (ros::ok())
  {
    marker_pub.publish(marker);  // Publish the marker
    ros::spinOnce();
    r.sleep();
  }

  float relative_percentage_out_points = (float)(out_points_count) / point_count * 100;
  float relative_percentage_in_points = (float)(in_points_count) / point_count * 100;

  std::cout << "Number of received Point Cloud Messages: " << msg_count << std::endl
            << "Number of received Point Cloud 3D Points: " << point_count << std::endl
            << "From which " << out_points_count << "(" << relative_percentage_out_points
            << "%) are outside the room limits and " << in_points_count << "(" << relative_percentage_in_points
            << "%) are inside room dimensions" << std::endl;

  return EXIT_SUCCESS;
}
