/**
 * @file   point_cloud_coloring.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 * @date   Created on May 18, 2019, 12:25
 */

#include "rigid_transform_computation/pointCloudVisualizer.hpp"

using namespace point_cloud;

const std::string POINT_CLOUD_VIEWER_NAME = "Point Cloud Viewer";
const int DEFAULT_QUEUE_SIZE = 1;


PointCloudVisualizer::PointCloudVisualizer(std::string point_cloud_topic,
                                           std::string node_handler_name) {
    this->point_cloud_topic = point_cloud_topic;
    this->nh_ = ros::NodeHandlePtr(new ros::NodeHandle(node_handler_name));

    this->point_cloud_sub = this->nh_->subscribe<sensor_msgs::PointCloud2>
                                        (point_cloud_topic,
                                         DEFAULT_QUEUE_SIZE,
                                         &PointCloudVisualizer::viewerCallback, this);
    this->pcl_viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(POINT_CLOUD_VIEWER_NAME)); // new pcl::visualization::PCLVisualizer(POINT_CLOUD_VIEWER_NAME); //!< Create visualization object
    this->cloudPtr = PointCloud::Ptr(new PointCloud);
    PointCloudVisualizer::initPointCloudVisualizer();
}

PointCloudVisualizer::~PointCloudVisualizer() {
    this->pcl_viewer->close();
    //ROS handles the deletion of node handler
    delete &(this->cloudPtr);
    delete this;
}


void PointCloudVisualizer::initPointCloudVisualizer() {
      this->pcl_viewer->setBackgroundColor (0, 0, 0);  // Rviz Background Colors
      this->pcl_viewer->addCoordinateSystem (1.0);
      this->pcl_viewer->initCameraParameters ();

      this->pcl_viewer->addPointCloud<pcl::PointXYZ> (this->cloudPtr, "point_cloud_name");
      this->pcl_viewer->setPointCloudRenderingProperties
                        (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "point_cloud_name");
}

void PointCloudVisualizer::addNewPointCloudToVisualizer(const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg, std::string point_cloud_name) {
    fromROSMsg(*point_cloud_msg, *(this->cloudPtr));

    this->pcl_viewer->addPointCloud<pcl::PointXYZ> (this->cloudPtr, point_cloud_name);
    this->pcl_viewer->setPointCloudRenderingProperties
                      (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, point_cloud_name);
}

void PointCloudVisualizer::viewerCallback(const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg) {
    fromROSMsg(*point_cloud_msg, *(this->cloudPtr));
    this->pcl_viewer->updatePointCloud<pcl::PointXYZ> (this->cloudPtr, "point_cloud_name");
    this->pcl_viewer->spinOnce(10);
}
