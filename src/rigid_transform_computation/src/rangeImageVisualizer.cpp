/*!
 * \file   rangeImageVisualizer.cpp
 * \brief  Implementation file for RangeImageVisualizer class
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

#include <pcl/range_image/range_image.h>

//#include "rigid_transform_computation/pointCloudVisualizer.hpp"
#include "rigid_transform_computation/rangeImageVisualizer.hpp"

#include "rigid_transform_computation/exceptions.hpp"

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>

using namespace point_cloud;

const std::string POINT_CLOUD_VIEWER_NAME = "Point Cloud Viewer";  //!< Point Cloud Viewer Name
const int DEFAULT_POINT_CLOUD_QUEUE_SIZE = 1;                      //!< ROS Point Cloud Subscriber Queue Size
const int DEFAULT_POSE_QUEUE_SIZE = 10;                            //!< ROS Pose Subscriber Queue Size

RangeImageVisualizer::RangeImageVisualizer(std::string point_cloud_topic, std::string node_handler_name)
{
  // ROS initializations
  this->point_cloud_topic = point_cloud_topic;
  this->nh_ = ros::NodeHandlePtr(new ros::NodeHandle(node_handler_name));

  this->point_cloud_sub = this->nh_->subscribe<sensor_msgs::PointCloud2>(
      point_cloud_topic, DEFAULT_POINT_CLOUD_QUEUE_SIZE, &RangeImageVisualizer::rangeImageCallback, this);
  // this->pcl_viewer = pcl::visualization::PCLVisualizer::Ptr(new
  // pcl::visualization::PCLVisualizer(POINT_CLOUD_VIEWER_NAME)); // new
  // pcl::visualization::PCLVisualizer(POINT_CLOUD_VIEWER_NAME); //!< Create visualization object
  this->cloudPtr = PointCloud::Ptr(new PointCloud);

  this->sensorPose = Eigen::Affine3f::Identity();

  // this->pcl_viewer = nullptr;

  // PointCloudVisualizer::initPointCloudVisualizer();
  // pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  //*(this->range_image_widget) = range_image_widget;
  // pcl::visualization::RangeImageVisualizer::Ptr ( new pcl::visualization::RangeImageVisualizer("Range image"));
}

RangeImageVisualizer::RangeImageVisualizer(std::string point_cloud_topic, std::string viewer_pose_topic,
                                           std::string node_handler_name)
{
  // ROS initializations
  this->point_cloud_topic = point_cloud_topic;
  this->viewer_pose_topic = viewer_pose_topic;

  this->nh_ = ros::NodeHandlePtr(new ros::NodeHandle(node_handler_name));

  this->point_cloud_sub = this->nh_->subscribe<sensor_msgs::PointCloud2>(
      point_cloud_topic, DEFAULT_POINT_CLOUD_QUEUE_SIZE, &RangeImageVisualizer::rangeImageCallback, this);

  this->pose_sub = this->nh_->subscribe<geometry_msgs::Pose>(viewer_pose_topic, DEFAULT_POSE_QUEUE_SIZE,
                                                             &RangeImageVisualizer::viewerPoseCallback, this);

  // this->pcl_viewer = pcl::visualization::PCLVisualizer::Ptr(new
  // pcl::visualization::PCLVisualizer(POINT_CLOUD_VIEWER_NAME)); // new
  // pcl::visualization::PCLVisualizer(POINT_CLOUD_VIEWER_NAME); //!< Create visualization object
  this->cloudPtr = PointCloud::Ptr(new PointCloud);

  this->sensorPose = Eigen::Affine3f::Identity();

  // this->pcl_viewer = nullptr;

  // PointCloudVisualizer::initPointCloudVisualizer();
  // pcl::visualization::RangeImageVisualizer range_image_widget ("Range image");
  //*(this->range_image_widget) = range_image_widget;
  // pcl::visualization::RangeImageVisualizer::Ptr ( new pcl::visualization::RangeImageVisualizer("Range image"));
}

RangeImageVisualizer::~RangeImageVisualizer()
{
  // ROS handles the deletion of node handler
  delete &(this->cloudPtr);
  // delete &(this->range_image_widget);
  delete this;
  ROS_WARN("Destructing Range Image Visualizer");
}

void RangeImageVisualizer::rangeImageCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
  fromROSMsg(*point_cloud_msg, *(this->cloudPtr));
  // this->cloudPtr->width = (uint32_t) this->cloudPtr->points.size();
  // this->cloudPtr->height = 1;

  // We now want to create a range image from the above point cloud, with a 1deg angular resolution
  float angularResolutionX = (float)(0.2f * (M_PI / 180.0f));  //   1.0 degree in radians
  float angularResolutionY = (float)(0.5f * (M_PI / 180.0f));  //   1.0 degree in radians
  float maxAngleWidth = (float)(180.0f * (M_PI / 180.0f));     // 360.0 degree in radians
  float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));    // 180.0 degree in radians

  // Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
  pcl::RangeImage::CoordinateFrame coordinate_frame =
      pcl::RangeImage::LASER_FRAME;  // Set the coordinate_frame as a LASER: x (front) y (left) z (upwards)

  float noiseLevel = 0.0f;
  float minRange = 0.0f;
  int borderSize = 1;
  // Eigen::Affine3f sensor_pose (Eigen::Affine3f::Identity ());
  /*
    sensor_pose = Eigen::Affine3f (Eigen::Translation3f (*(this->cloudPtr).sensor_origin_[0],
                                                                *(this->cloudPtr).sensor_origin_[1],
                                                                *(this->cloudPtr).sensor_origin_[2])) *
                           Eigen::Affine3f (*(this->cloudPtr).sensor_orientation_);
 */

  pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
  pcl::RangeImage& range_image = *range_image_ptr;
  range_image.createFromPointCloud(*(this->cloudPtr), angularResolutionX, angularResolutionY, maxAngleWidth,
                                   maxAngleHeight, this->sensorPose, coordinate_frame, noiseLevel, minRange,
                                   borderSize);

  //*(this->range_image_widget).showRangeImage (range_image);
  static pcl::visualization::RangeImageVisualizer widget("Range Image");

  widget.showRangeImage(range_image);
  // this->range_image_widget->showRangeImage (range_image);

  widget.spinOnce();
}

void RangeImageVisualizer::viewerPoseCallback(const geometry_msgs::Pose::ConstPtr& viewer_pose_msg)
{
  Eigen::Affine3d tempPose;
  tf::poseMsgToEigen(*viewer_pose_msg, tempPose);

  // Eigen::Affine3f viewerPose =

  this->sensorPose = tempPose.cast<float>();
}

/*
void RangeImageVisualizer::attachPoseTrigger(PointCloudVisualizer pcl_viewer) {
    std::cout << "Pose attached" << std::endl;
    //this->pcl_viewer = &pcl_viewer;
    //std::cout << (this->pcl_viewer->getViewerPose()).matrix() << std::endl;
    std::cout << "Pose attached Finished" << std::endl;
}
*/
