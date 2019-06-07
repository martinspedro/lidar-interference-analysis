/**
 * @file   point_cloud_coloring.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 * @date   Created on May 18, 2019, 12:25
 */


#include "automatic_calibration/automatic_calibration.hpp"
#include "automatic_calibration/image_visualizer.hpp"
#include "sensor_fusion/color.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>


#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stdint.h>
#include <image_geometry/pinhole_camera_model.h>
#include <typeinfo>


#define VIEWER_NAME "Colored Cloud Viewer" //!< Viewer Name

using namespace sensor_msgs;
using namespace message_filters;

//pcl::visualization::CloudViewer viewer(VIEWER_NAME); //!< Create visualization object

geometry_msgs::TransformStamped transformStamped; //!< Create geometric transform object

ros::Publisher pub; //!< ROS Publisher

image_geometry::PinholeCameraModel cam_model_;
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
    //viewer.showCloud(cloudPtr);

    // Publish the data.
    pub.publish(cloud_out);
}


/** @brief Callback to process imagem stream
 *
 * @param[in] msg Pointer to Image data messages from ROS nodes
*/
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

/** @brief Callback to process imagem stream
 *
 * @param[in] image Pointer to Image data messages from ROS nodes
 * @param[in] cam_info Information of camera calibration and frame characteristics
 * @param[in] point_cloud_msg PointCloud2 messages from ROS nodes
*/
void callback(const ImageConstPtr& image,
              const CameraInfoConstPtr& cam_info,
              const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg) {

  ROS_INFO("Data Sync successfull!");
  // create Point Cloud with Intensity Field
  PointCloudRGB point_cloud_camera;
  PointCloudRGB::Ptr cloudCameraPtr(new PointCloudRGB);
  PointCloudRGB::Ptr cloudColoredPtr(new PointCloudRGB);

  sensor_msgs::PointCloud2 point_cloud_camera_msg;

  try {
      tf2::doTransform(*point_cloud_msg, point_cloud_camera_msg, transformStamped);
  }
  catch (tf2::TransformException &ex) {
      ROS_WARN("%s", ex.what());
  }
  // Convert ROS msg to Point Cloud
  fromROSMsg(point_cloud_camera_msg, point_cloud_camera);

  // Initialize pointer to point cloud data
  *cloudCameraPtr = point_cloud_camera;
  cam_model_.fromCameraInfo(cam_info);
  for (int i = 0; i < cloudCameraPtr->points.size(); i++) {
      //ROS_INFO("%s", typeid(cloudCameraPtr->points[i]).name());
      //cv::Point3d* aux2 = new cv::Point3d(cloudCameraPtr->points[i].x, cloudCameraPtr->points[i].y, cloudCameraPtr->points[i].z);
      cv::Point2d uv = cam_model_.project3dToPixel(cv::Point3d(cloudCameraPtr->points[i].x, cloudCameraPtr->points[i].y, cloudCameraPtr->points[i].z));
      //cv::Point2d aux = image_geometry::PinholeCameraModel::project3dToPixel(cv::Point3d(cloudCameraPtr->points[i].x, cloudCameraPtr->points[i].y, cloudCameraPtr->points[i].z)); //const cv::Point3d &  	xyz	)
       ROS_INFO("(%f, %f, %f)", cloudCameraPtr->points[i].x, cloudCameraPtr->points[i].y, cloudCameraPtr->points[i].z);
       ROS_INFO("(%f, %f, %f)", cloudCameraPtr->points[i].x, cloudCameraPtr->points[i].y, cloudCameraPtr->points[i].z);
      //ROS_INFO("Coordenadas dos pontos: (%f, %f)", uv.x, uv.y);
  }


  // Perform color manipulation on Point Cloud
  //color_point_cloud(cloudPtr, point_cloud_RGB);

  //pcl::toROSMsg(*cloudColoredPtr,point_cloud_camera );




  // Add new data to viewer
  //viewer.showCloud(cloudPtr);

  // Publish the data.
  //pub.publish(cloudColoredPtr);
}

/** @brief Main code
 *
 * Node initialization and announcement of publishers and subscribers
 *
 * @param[in] argc
 * @param[in] argv
 * @return Node exit status
*/
int main(int argc, char** argv)
{
  //Initialize ROS
  ros::init(argc, argv, "point_cloud_coloring");

  //cv::namedWindow(OPENCV_WINDOW, 0);
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  transformStamped = tfBuffer.lookupTransform("camera_color_left", "velo_link", ros::Time(0), ros::Duration(20.0));

  // Ros subscriber for ros msg for Point Cloud
  //ros::Subscriber sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, callback_pcl);
  //sub_pcl = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, callback_pcl);
  //pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  //cv::namedWindow("view");
  //image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("/kitti/camera_color_left/image_raw", 1, imageCallback);



  message_filters::Subscriber<Image> image_sub(nh, "/kitti/camera_color_left/image_raw", 1);
  message_filters::Subscriber<CameraInfo> info_sub(nh, "/kitti/camera_color_left/camera_info", 1);
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub(nh, "velodyne_points", 1);

  typedef sync_policies::ApproximateTime<Image, CameraInfo, sensor_msgs::PointCloud2> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, info_sub,  point_cloud_sub);

  //TimeSynchronizer<Image, sensor_msgs::PointCloud2> sync(image_sub, point_cloud_sub,  100);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));


  ros::spin();

  return EXIT_SUCCESS;
}
