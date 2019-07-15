/**
 * @file   point_cloud_coloring.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 * @date   Created on May 18, 2019, 12:25
 */

#include <pcl/range_image/range_image.h>

#include "rigid_transform_computation/pointCloudVisualizer.hpp"
#include "rigid_transform_computation/rangeImageVisualizer.hpp"

#include "rigid_transform_computation/exceptions.hpp"

using namespace point_cloud;

const std::string POINT_CLOUD_VIEWER_NAME = "Point Cloud Viewer";
const int DEFAULT_QUEUE_SIZE = 1;

RangeImageVisualizer::RangeImageVisualizer(std::string point_cloud_topic,
                                           std::string node_handler_name) {
    // ROS initializations
    this->point_cloud_topic = point_cloud_topic;
    this->nh_ = ros::NodeHandlePtr(new ros::NodeHandle(node_handler_name));

    this->point_cloud_sub = this->nh_->subscribe<sensor_msgs::PointCloud2>
                                        (point_cloud_topic,
                                         DEFAULT_QUEUE_SIZE,
                                         &RangeImageVisualizer::rangeImageCallback, this);
    //this->pcl_viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(POINT_CLOUD_VIEWER_NAME)); // new pcl::visualization::PCLVisualizer(POINT_CLOUD_VIEWER_NAME); //!< Create visualization object
    this->cloudPtr = PointCloud::Ptr(new PointCloud);


    //PointCloudVisualizer::initPointCloudVisualizer();
    //pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
    //*(this->range_image_widget) = range_image_widget;
    //pcl::visualization::RangeImageVisualizer::Ptr ( new pcl::visualization::RangeImageVisualizer("Range image"));
}

RangeImageVisualizer::~RangeImageVisualizer() {
    //ROS handles the deletion of node handler
    delete &(this->cloudPtr);
    delete &(this->range_image_widget);
    delete this;
}


// http://pointclouds.org/documentation/tutorials/range_image_creation.php#range-image-creation
void RangeImageVisualizer::rangeImageCallback(const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg){
    fromROSMsg(*point_cloud_msg, *(this->cloudPtr));
    //this->cloudPtr->width = (uint32_t) this->cloudPtr->points.size();
    //this->cloudPtr->height = 1;

  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
  float angularResolutionX = (float) (  0.2f * (M_PI/180.0f));  //   1.0 degree in radians
  float angularResolutionY = (float) (  0.5f * (M_PI/180.0f));  //   1.0 degree in radians
  float maxAngleWidth     = (float) (180.0f * (M_PI/180.0f));  // 360.0 degree in radians
  float maxAngleHeight    = (float) (180.0f * (M_PI/180.0f));  // 180.0 degree in radians

  Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::LASER_FRAME;     // Set the coordinate_frame as a LASER: x (front) y (left) z (upwards)

  float noiseLevel=0.0f;
  float minRange = 0.0f;
  int borderSize = 1;
  Eigen::Affine3f sensor_pose (Eigen::Affine3f::Identity ());
     /*
       sensor_pose = Eigen::Affine3f (Eigen::Translation3f (*(this->cloudPtr).sensor_origin_[0],
                                                                   *(this->cloudPtr).sensor_origin_[1],
                                                                   *(this->cloudPtr).sensor_origin_[2])) *
                              Eigen::Affine3f (*(this->cloudPtr).sensor_orientation_);
    */


  pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;
  range_image.createFromPointCloud (*(this->cloudPtr), angularResolutionX, angularResolutionY,
                                    maxAngleWidth, maxAngleHeight,
                                    sensor_pose, coordinate_frame, noiseLevel, minRange, borderSize);


  //*(this->range_image_widget).showRangeImage (range_image);
  static pcl::visualization::RangeImageVisualizer widget("Range Image");
  widget.showRangeImage(range_image);
  // this->range_image_widget->showRangeImage (range_image);

  widget.spinOnce ();
}
