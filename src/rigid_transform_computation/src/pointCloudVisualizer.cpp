/**
 * @file   point_cloud_coloring.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 * @date   Created on May 18, 2019, 12:25
 */

#include "rigid_transform_computation/pointCloudVisualizer.hpp"
#include "rigid_transform_computation/exceptions.hpp"

#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Pose.h>

#include <rigid_transform_computation/PointXYZI.h>


using namespace point_cloud;

const std::string POINT_CLOUD_VIEWER_NAME = "Point Cloud Viewer";
const int DEFAULT_QUEUE_SIZE = 1;
const int DEFAULT_POSE_QUEUE_SIZE = 10;


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

PointCloudVisualizer::PointCloudVisualizer(std::string point_cloud_topic,
                                           std::string pose_topic,
                                           std::string node_handler_name) {

    this->point_cloud_topic = point_cloud_topic;
    this->nh_ = ros::NodeHandlePtr(new ros::NodeHandle(node_handler_name));

    this->point_cloud_sub = this->nh_->subscribe<sensor_msgs::PointCloud2>
                                        (point_cloud_topic,
                                         DEFAULT_QUEUE_SIZE,
                                         &PointCloudVisualizer::viewerWithPoseCallback, this);
    this->pose_pub = this->nh_->advertise<geometry_msgs::Pose>
                                         (pose_topic,
                                          DEFAULT_POSE_QUEUE_SIZE);

    this->point_pub = this->nh_->advertise<rigid_transform_computation::PointXYZI>
                                         ("picked_point",
                                         DEFAULT_QUEUE_SIZE);

    this->pcl_viewer = pcl::visualization::PCLVisualizer::Ptr(new pcl::visualization::PCLVisualizer(POINT_CLOUD_VIEWER_NAME)); // new pcl::visualization::PCLVisualizer(POINT_CLOUD_VIEWER_NAME); //!< Create visualization object
    this->cloudPtr = PointCloud::Ptr(new PointCloud);
    PointCloudVisualizer::initPointCloudVisualizer();
}

PointCloudVisualizer::~PointCloudVisualizer() {
    ROS_WARN("Destructing PCL Visualizer");
    this->pcl_viewer->close();
    //ROS handles the deletion of node handler
    delete &(this->cloudPtr);
    delete this;

}


void setViewerPose (pcl::visualization::PCLVisualizer& viewer, const Eigen::Affine3f& viewer_pose) {
  Eigen::Vector3f pos_vector = viewer_pose * Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f look_at_vector = viewer_pose.rotation () * Eigen::Vector3f(0, 0, 1) + pos_vector;
  Eigen::Vector3f up_vector = viewer_pose.rotation () * Eigen::Vector3f(0, -1, 0);
  viewer.setCameraPosition (pos_vector[0], pos_vector[1], pos_vector[2],
                            look_at_vector[0], look_at_vector[1], look_at_vector[2],
                            up_vector[0], up_vector[1], up_vector[2]);
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

    this->viewerPose = this->pcl_viewer->getViewerPose();

    this->pcl_viewer->spinOnce(10);
}

void PointCloudVisualizer::viewerWithPoseCallback(const sensor_msgs::PointCloud2::ConstPtr &point_cloud_msg) {
    fromROSMsg(*point_cloud_msg, *(this->cloudPtr));
    this->pcl_viewer->updatePointCloud<pcl::PointXYZ> (this->cloudPtr, "point_cloud_name");

    this->viewerPose = this->pcl_viewer->getViewerPose();
    geometry_msgs::Pose tempPoseMsg;
    tf::poseEigenToMsg (this->viewerPose.cast<double>(), tempPoseMsg);

    this->pose_pub.publish(tempPoseMsg);

    //std::cout << (this->viewerPose).matrix() << std::endl;


}


// http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_picking_event.html
void PointCloudVisualizer::onPointPickingEvent (const pcl::visualization::PointPickingEvent& pickingEvent, void* viewerVoidPtr) {
    //std::cout << "Point picking event occurred." << std::endl;

    //PointCloudVisualizer::PointCloudVisualizer* aux_pcl_handler = (PointCloudVisualizer::PointCloudVisualizer*) viewerVoidPtr;

    float x, y, z;
    int idx = pickingEvent.getPointIndex ();

    if (idx != -1) {
        pickingEvent.getPoint(x, y, z);

        //std::cout << "Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;

        rigid_transform_computation::PointXYZI tempPoint;
        tempPoint.x = x;
        tempPoint.y = y;
        tempPoint.z = z;
        tempPoint.intensity = -1;//aux_pcl_handler->cloudPtr[idx].intensity;

        ros::Publisher* aux_point_pub = (ros::Publisher*) viewerVoidPtr;
        //aux_pcl_handler->
        aux_point_pub->publish(tempPoint);
    }
}

void PointCloudVisualizer::registerPointPickingCallback(const uint8_t MODE) {
    if(MODE == PointCloudVisualizer::SINGLE_POINT_MODE) {
        this->pcl_viewer->registerPointPickingCallback (PointCloudVisualizer::onPointPickingEvent, (void*)&(this->point_pub));
    }
    else {
        throw NotImplemented();
    }

}


Eigen::Affine3f PointCloudVisualizer::getViewerPose() {
    //std::cout << (this->viewerPose).matrix() << std::endl;
    //this->viewerPose = this->pcl_viewer->getViewerPose();
    return this->viewerPose;
}

void PointCloudVisualizer::spin() {
    this->pcl_viewer->spinOnce(10); // Visualizer loop runs for 10 ms
    ros::spinOnce();
}
