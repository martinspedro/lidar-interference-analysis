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

typedef pcl::PointCloud<pcl::PointXYZ>  PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;


const std::string bags_folder_path = "/media/martinspedro/DATA/UA/Thesis/multiple-lidar-interference-mitigation/datasets/mine/";
const std::string bag_file         = "2019-07-02-12-27-31.bag";

int main(int argc, char** argv) {
  //Initialize ROS
  ros::init(argc, argv, "point_cloud_histogram");

  bool flag = false;

  PointCloud::Ptr cloudPtr(new PointCloud);
  PointCloud point_cloud;
  std::cout << "Opening rosbag" << std::endl;
  rosbag::Bag bag;
  bag.open(bags_folder_path + bag_file);

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (VIEWER_NAME));
  //defining a plotter

  pcl::visualization::PCLPlotter *plotter = new pcl::visualization::PCLPlotter("Histogram");

  std::vector<std::string> topics;
  topics.push_back(std::string("/velodyne_points"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

 pcl::PointXYZ *origin = new pcl::PointXYZ(0, 0, 0);

  foreach(rosbag::MessageInstance const m, view) {
    //std::cout << "New Message Instance" << std::endl;
    sensor_msgs::PointCloud2::ConstPtr msg = m.instantiate<sensor_msgs::PointCloud2>();
    // Initialize pointer to point cloud data

    std::vector<double> distance;
    foreach (pcl::PointXYZ const point, msg)
    {
        distance.push_back(pcl::geometry::distance(origin, point));
    }

    if (msg != NULL) {
        fromROSMsg(*msg, point_cloud);

        if(!flag){
            *cloudPtr = point_cloud;
            viewer->addPointCloud<pcl::PointXYZ> (cloudPtr, "sample cloud");
            viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
            viewer->addCoordinateSystem (1.0);
            viewer->initCameraParameters ();
            flag = true;
        } else {
            *cloudPtr = point_cloud;
            viewer->updatePointCloud<pcl::PointXYZ> (cloudPtr, "sample cloud");
        }

        viewer->spinOnce(50);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


    bag.close();

  return EXIT_SUCCESS;
}
