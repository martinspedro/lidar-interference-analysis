#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <thread>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/geometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <string>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <vector>
#include <iostream>
#include <utility>

#define VIEWER_NAME "Simple Cloud Visualizer"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

// const std::string bags_folder_path = "/media/martinspedro/Elements/mine/IT2 Dark Room/Scenario B/Calibration/";
// const std::string bag_file = "2019-07-31-17-03-29.bag";

// const std::string bags_folder_path = "/media/martinspedro/Elements/mine/IT2 Dark Room/Scenario B/Interference/low/";
// const std::string bag_file = "2019-07-31-17-48-10.bag";

const std::string bags_folder_path = "/media/martinspedro/Elements/mine/IT2 Dark Room/Scenario B/";
// const std::string bag_file = "2019-07-31-17-48-10.bag";

struct RoomDimensions
{
  float x_min_;
  float x_max_;
  float y_min_;
  float y_max_;
  float z_min_;
  float z_max_;

  RoomDimensions(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
    : x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max), z_min_(z_min), z_max_(z_max)
  {
  }
};

bool pointInsideRoom(pcl::PointXYZ point, RoomDimensions room_dimensions)
{
  return ((point.x > room_dimensions.x_min_) && (point.x < room_dimensions.x_max_)) &&
         ((point.y > room_dimensions.y_min_) && (point.y < room_dimensions.y_max_)) &&
         ((point.z > room_dimensions.z_min_) && (point.z < room_dimensions.z_max_));
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "point_cloud_histogram");

  PointCloud::Ptr cloudPtr(new PointCloud);

  std::cout << "Opening rosbag" << bags_folder_path + argv[1] << std::endl;
  rosbag::Bag bag;
  // bag.open(bags_folder_path + bag_file);
  bag.open(bags_folder_path + argv[1]);

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  pcl::PointXYZ* origin = new pcl::PointXYZ(0, 0, 0);

  long int msg_count = 0, point_count = 0, out_points_count = 0, in_points_count = 0;
  RoomDimensions dark_room(-2.0, 7.0, -3.0, 4.5, -1.9, 3.5);  // added 1 meter in every direction

  foreach (rosbag::MessageInstance const m, view)
  {
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    if (msg != NULL)
    {
      PointCloud point_cloud;
      fromROSMsg(*msg, point_cloud);
      ++msg_count;
      for (int j = 0; j < point_cloud.points.size(); j++)
      {
        point_count++;
        if (pointInsideRoom(point_cloud.points[j], dark_room))
        {
          ++in_points_count;
        }
        else
        {
          ++out_points_count;
        }
      }
    }
  }

  bag.close();

  float relative_percentage_out_points = (float)(out_points_count) / point_count * 100;
  float relative_percentage_in_points = (float)(in_points_count) / point_count * 100;

  std::cout << "Number of received Point Cloud Messages: " << msg_count << std::endl
            << "Number of received Point Cloud 3D Points: " << point_count << std::endl
            << "From which " << out_points_count << "(" << relative_percentage_out_points
            << "%) are outside the room limits and " << in_points_count << "(" << relative_percentage_in_points
            << "%) are inside room dimensions" << std::endl;

  return EXIT_SUCCESS;
}
