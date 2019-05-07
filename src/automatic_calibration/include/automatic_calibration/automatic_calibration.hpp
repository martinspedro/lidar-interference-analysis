/*******************************************************************************
 *                             INCLUDES
 ******************************************************************************/
#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>



#include <iostream>

// OPENCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>

/*******************************************************************************
 *                                 NAMESPACES
 ******************************************************************************/
using namespace std;
using namespace cv;
using namespace pcl;

/*******************************************************************************
 *                                 TYPEDEFS
 ******************************************************************************/
typedef pcl::PointCloud<pcl::PointXYZ>  PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
typedef pcl::PointXYZRGB PointRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;
