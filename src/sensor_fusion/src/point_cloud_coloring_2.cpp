/**
 * @file   point_cloud_coloring.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 * @date   Created on May 18, 2019, 12:25
 */


//#include "automatic_calibration/automatic_calibration.hpp"
//#include "automatic_calibration/image_visualizer.hpp"
//#include "sensor_fusion/color.hpp"
//#include "automatic_calibration/automatic_calibration.hpp"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <thread>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <geometry_msgs/TransformStamped.h>

#include <image_geometry/pinhole_camera_model.h>


#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>

//#include <iostream>
#include <string>

#define VIEWER_NAME "Simple Cloud Visualizer"
/*******************************************************************************
 *                                 TYPEDEFS
 ******************************************************************************/
typedef pcl::PointCloud<pcl::PointXYZ>  PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;


const std::string kitti_bags_folder_path = "/media/martinspedro/DATA/UA/Thesis/multiple-lidar-interference-mitigation/datasets/Kitti/";
const std::string kitti_bag              = "kitti_2011_09_26_drive_0005_synced.bag";


int main(int argc, char** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "point_cloud");

  bool flag = false;
  geometry_msgs::TransformStamped transformStamped; //!< Create geometric transform object
  sensor_msgs::PointCloud2 world_point_cloud_msg;


  PointCloudRGB::Ptr cloudCameraPtr(new PointCloudRGB);
  PointCloudRGB::Ptr cloudSumPtr(new PointCloudRGB);
  PointCloudRGB point_cloud_world, point_cloud_sum;

  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer (VIEWER_NAME));

  std::cout << "Opening rosbag" << std::endl;

  rosbag::Bag bag;
  bag.open(kitti_bags_folder_path + kitti_bag);

  std::vector<std::string> topics;
  topics.push_back(std::string("/kitti/velo/pointcloud"));

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);


  //std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
  std::cout << "Finding Transform" << std::endl;
  //transformStamped = tfBuffer.lookupTransform("velo_link", "world", ros::Time(0), ros::Duration(20.0));
  std::cout << "Transform Found" << std::endl;

  foreach(rosbag::MessageInstance const m, view)
    {
        std::cout << "New Message Instance" << std::endl;

        sensor_msgs::PointCloud2::ConstPtr s = m.instantiate<sensor_msgs::PointCloud2>();


        // Initialize pointer to point cloud data

        if (s != NULL) {
            /*
            try {
                tf2::doTransform(*s, world_point_cloud_msg, transformStamped);
            }
            catch (tf2::TransformException &ex) {
                ROS_WARN("%s", ex.what());
            }
            */
            //fromROSMsg(world_point_cloud_msg, point_cloud_world);
            fromROSMsg(*s, point_cloud_world);

            if(!flag){
                *cloudCameraPtr = point_cloud_world;
                *cloudSumPtr = point_cloud_world;
                viewer->addPointCloud<pcl::PointXYZRGB> (cloudCameraPtr, "sample cloud");
                viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
                viewer->addCoordinateSystem (1.0);
                viewer->initCameraParameters ();
                flag = true;
            } else {
                *cloudSumPtr = *cloudSumPtr + point_cloud_world;
                *cloudCameraPtr =  point_cloud_world;
                viewer->updatePointCloud<pcl::PointXYZRGB> (cloudCameraPtr, "sample cloud");
            }

            viewer->spinOnce(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }


    bag.close();
    std::cout << "Infinite Cycle" << std::endl;
    /*
    while(1)
    {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    */




  return EXIT_SUCCESS;
}
