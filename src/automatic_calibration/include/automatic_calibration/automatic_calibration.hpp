/*******************************************************************************
 *                             INCLUDES
 ******************************************************************************/
#include <ros/ros.h>

#include <iostream>

// OPENCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


/*******************************************************************************
 *                                 NAMESPACES
 ******************************************************************************/
using namespace std;
using namespace cv;


/*******************************************************************************
 *                                 TYPEDEFS
 ******************************************************************************/
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;
