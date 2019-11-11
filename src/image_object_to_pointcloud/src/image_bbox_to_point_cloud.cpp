/**
 * @file   correspondences.cpp
 * @brief  Point Cloud Coloring node CPP File
 *
 * @author Pedro Martins
 */

#define PCL_NO_PRECOMPILE        // must be included before any PCL include on this CPP file or HPP included before
#define EIGEN_RUNTIME_NO_MALLOC  // Define this symbol to enable runtime tests for allocations

#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>

#include "image_object_to_pointcloud/image_object_to_pointcloud.hpp"
#include "image_object_to_pointcloud/pinhole_camera_model_utilities.hpp"
#include "image_object_to_pointcloud/FOV.hpp"

//#include <Eigen/Core>  // Needs to be included before the opencv eigen interface!
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>  // Needs to be included before other opencv headers!
// OPENCV
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"

//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
//#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>

//#include "sensor_fusion/color.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>

#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_ros/transform_listener.h>
#include "tf2/transform_datatypes.h"
#include "tf2_eigen/tf2_eigen.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/CameraInfo.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/ObjectCount.h>
#include <std_msgs/Int8.h>

#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <stdint.h>
#include <image_geometry/pinhole_camera_model.h>
#include <typeinfo>

#include <exception>
#include <list>

#include <pcl/filters/frustum_culling.h>

#include <point_cloud_statistics/velodyne_point_type.h>
#include "point_cloud_statistics/point_cloud_statistics.hpp"
#include "image_object_to_pointcloud/FOV.hpp"

#include <geometry_msgs/TransformStamped.h>

#include <cmath>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/PoseArray.h>

#include "tf2/LinearMath/Matrix3x3.h"
#include <geometry_msgs/TransformStamped.h>
#include "tf2/LinearMath/Quaternion.h"

#include <string>

const float NEAR_PLANE_DISTANCE = 1.0f;
const float FAR_PLANE_DISTANCE = 100.0f;

const double PIXEL_SIZE = 3.45e-6;

const unsigned int SYNC_POLICY_QUEUE_SIZE = 20;  // queue size for syncPolicyForCallback;

const std::string LIDAR_TF2_REFERENCE_FRAME = "velo_link";
const std::string CAMERA_TF2_REFERENCE_FRAME = "camera_color_left";

ros::Publisher pub, pub_frustum_poses, pub_lidar_poses;  //!< ROS Publisher

Eigen::Matrix4f g_camera_to_lidar_6DOF;

void callback(const darknet_ros_msgs::ObjectCountConstPtr& object_count,
              const darknet_ros_msgs::BoundingBoxesConstPtr& b_boxes, const sensor_msgs::CameraInfoConstPtr& cam_info,
              const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
  // Create Datatype dependent if operating with KITTI datasets or Experimental Dataype
  PointCloudType point_cloud, target, final_point_cloud;
  PointCloudType::Ptr cloudPtr, point_cloud_ptr(new PointCloudType), final_point_cloud_ptr(new PointCloudType);

  // Convert ROS msg to PCL object
  fromROSMsg(*point_cloud_msg, point_cloud);
  *point_cloud_ptr = point_cloud;
  *final_point_cloud_ptr = final_point_cloud;

  // Get Camera Information from msg
  image_geometry::PinholeCameraModel cam_model_;
  cam_model_.fromCameraInfo(cam_info);

  // std::cout << "Image Size: " << cam_model_.fullResolution() << std::endl;
  // std::cout << "Instrinsic Calibration Matrix: \n" << cam_model_.fullIntrinsicMatrix() << std::endl;
  // FOV image_fov = pinhole_camera::getImageFOV(cam_model_);

  Eigen::Matrix4f camera_transform, frustrum_filter_pose, camera_rotation, lidar_pose;
  geometry_msgs::PoseArray frustum_poses, lidar_poses;

  for (int i = 0; i < object_count->count; ++i)
  {
    FOV bounding_box_fov;
    Eigen::Vector3f camera_rotation_vec;
    pinhole_camera::computeBoundingBoxFOV(cam_model_, b_boxes->bounding_boxes[i], &bounding_box_fov,
                                          camera_rotation_vec);
    camera_transform = angleAxisVecToTransform(camera_rotation_vec);

    // Print Stuff
    float fov_X =
        atan((b_boxes->bounding_boxes[i].xmax - b_boxes->bounding_boxes[i].xmin) / (cam_model_.fx())) * 180.0f / M_PI;
    float fov_Y =
        atan((b_boxes->bounding_boxes[i].ymax - b_boxes->bounding_boxes[i].ymin) / (cam_model_.fy())) * 180.0f / M_PI;

    std::cout << b_boxes->bounding_boxes[i].Class << ": " << b_boxes->bounding_boxes[i].probability << " (" << fov_X
              << " ," << fov_Y << ") - [ (" << b_boxes->bounding_boxes[i].xmin << ", "
              << b_boxes->bounding_boxes[i].xmax << "), (" << b_boxes->bounding_boxes[i].ymin << ", "
              << b_boxes->bounding_boxes[i].ymax << ") ]" << std::endl;
    std::cout << b_boxes->bounding_boxes[i].Class << ": " << b_boxes->bounding_boxes[i].probability << " ("
              << bounding_box_fov.x << " ," << bounding_box_fov.y << ") " << std::endl;

    // Get Initial LiDAR Pose & Rotation
    getCameraRotation(cam_model_, b_boxes->bounding_boxes[i], camera_rotation);

    /* Order is computed on the inverse order!
     * The first matrix/transform in the multiplication chain is the first you want to apply to the "object matrix"
     * The first matrix to be multiplied by the object is the last transform you wnat to perform
     *
      Order:
      1. Camera Rotation
      2. 6 DOF Camera to LiDAR
      3. LIDAR Pose
      4. Convert LiDAR Pose to Frustum Pose
     * First we need to apply the camera rotation to the camera pose. The camera pose, in this case is a Identity
  matrix
     * After the Camera Pose is Rotated to face the FOV, we need to translate and rotate the referential from the
  Camera
     * to the LiDAR Coordinates. Note that different coordinate reference frames are already covered in this transform
     and
     * therefore no extra matrices are needed
     * Despite not being a transformation, we must take into account the original LiDAR Pose, which is the last
     transform to be applied
     * Also, because Frustum Coordinate System is different from Velodyne's, a special transform must be carried out
     * So the order is 1. * 2. * 3. * 4. * Camera Pose
     *
     * Useful links:
     * - https://stackoverflow.com/questions/25058852/how-to-check-if-eigen-matrix-is-column-major-or-row-major
     * - https://eigen.tuxfamily.org/dox/group__TopicStorageOrders.html
     * -
     https://stackoverflow.com/questions/18785938/combining-multiple-transformations-in-eigen-into-one-transformation-matrix
     */

    lidar_pose = camera_rotation;  //* g_camera_to_lidar_6DOF;
    frustrum_filter_pose = LIDAR_POSE_TO_FRUSTRUM_POSE * lidar_pose;

    ROS_DEBUG_STREAM("LiDARPose:" << std::endl
                                  << lidar_pose << std::endl
                                  << "Frustum Pose:" << std::endl
                                  << frustrum_filter_pose << std::endl);

    // Apply filter to select points that correspond to a bounding box
    // pcl::transformPointCloud(*point_cloud_ptr, *point_cloud_ptr, LIDAR_POSE_TO_FRUSTRUM_POSE);
    pcl::FrustumCulling<PointType> fc;
    fc.setInputCloud(point_cloud_ptr);
    fc.setVerticalFOV(bounding_box_fov.y);
    fc.setHorizontalFOV(bounding_box_fov.x);
    // fc.setVerticalFOV(fov_Y);
    // fc.setHorizontalFOV(fov_X);
    // fc.setVerticalFOV(20);
    // fc.setHorizontalFOV(30);
    fc.setNearPlaneDistance(NEAR_PLANE_DISTANCE);
    fc.setFarPlaneDistance(FAR_PLANE_DISTANCE);
    fc.setCameraPose(frustrum_filter_pose);
    fc.filter(target);

    ROS_INFO_STREAM("Filtered Point Cloud size = " << target.size());
    *final_point_cloud_ptr += target;

    Eigen::Affine3d eigen_frustum_pose, eigen_lidar_pose;
    eigen_frustum_pose.matrix() = frustrum_filter_pose.cast<double>();
    geometry_msgs::Pose frustum_current_pose = Eigen::toMsg(eigen_frustum_pose);
    frustum_poses.poses.push_back(frustum_current_pose);

    eigen_lidar_pose.matrix() = lidar_pose.cast<double>();
    geometry_msgs::Pose lidar_current_pose = Eigen::toMsg(eigen_lidar_pose);
    lidar_poses.poses.push_back(lidar_current_pose);
  }

  ROS_INFO_STREAM("Filtered Final Point Cloud size = " << final_point_cloud_ptr->size());

  final_point_cloud_ptr->header.frame_id = LIDAR_TF2_REFERENCE_FRAME;  // Add coordinate frame id
  pub.publish(final_point_cloud_ptr);  // PointCloud Object is automatically serialized by ROS. No need for toROSMsg

  frustum_poses.header.stamp = ros::Time::now();
  frustum_poses.header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
  pub_frustum_poses.publish(frustum_poses);

  lidar_poses.header.stamp = ros::Time::now();
  lidar_poses.header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
  pub_lidar_poses.publish(lidar_poses);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "image_bbox_to_point_cloud_node");

  std::cout << "Node init successfull" << std::endl;
  ros::NodeHandle nh;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  /*
   * lookupTransform(target_frame, source_frame)
   * target_frame	The frame to which data should be transformed
   * source_frame	The frame where the data originated
   *
   * If one wants to transform the data from one referential to another, the target frame most be the final on which the
   * data will be processed.
   * The function construction might seem counter intuitive , but notice that the transform from the Referential Frame A
   * -> B allows the transformation of data from the frame B -> A
   *
   * On this case, since no data will be manipulated, onluy referentials, we want the transfrom from the Camera to the
   * LiDAR
   *
   * Another perspective is: we want to transform from <target_frame> from this <source_frame> frame
   */

  geometry_msgs::TransformStamped transformStamped = tfBuffer.lookupTransform(
      CAMERA_TF2_REFERENCE_FRAME, LIDAR_TF2_REFERENCE_FRAME, ros::Time(0), ros::Duration(20.0));

  // get Affine3d matrix that defines the 6DOF transformation between LiDAR and Camera
  Eigen::Affine3d transform_eigen = tf2::transformToEigen(transformStamped);
  g_camera_to_lidar_6DOF = (transform_eigen.matrix()).cast<float>();  // Convert Eigen Affine3d Trans

  ROS_DEBUG_STREAM("Geometry msgs transform: " << transformStamped.transform << std::endl
                                               << "Affine Transform (Eigen): " << g_camera_to_lidar_6DOF.matrix());

  pub = nh.advertise<sensor_msgs::PointCloud2>("frustum_filtered_point_cloud", 1);
  pub_frustum_poses = nh.advertise<geometry_msgs::PoseArray>("/poses/frustum", 1);
  pub_lidar_poses = nh.advertise<geometry_msgs::PoseArray>("/poses/lidar", 1);

  // Subscribers list to be synchronized
  message_filters::Subscriber<darknet_ros_msgs::ObjectCount> object_count_sub(nh, "/darknet_ros/found_object", 2);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub(nh, "/darknet_ros/bounding_boxes", 2);
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub(nh, "/camera/camera_info", 20);
  message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub(nh, "/velodyne_points", 20);

  typedef message_filters::sync_policies::ApproximateTime<
      darknet_ros_msgs::ObjectCount, darknet_ros_msgs::BoundingBoxes, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2>
      syncPolicyForCallback;

  // ApproximateTime takes a queue size as its constructor argument, hence SyncPolicy(20)
  message_filters::Synchronizer<syncPolicyForCallback> sync(syncPolicyForCallback(SYNC_POLICY_QUEUE_SIZE),
                                                            object_count_sub, bounding_boxes_sub, cam_info_sub,
                                                            point_cloud_sub);

  // Bind Callback to function to each of the subscribers
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::Rate r(100);  // 100 hz
  while (ros::ok())
  {
    ros::spinOnce();
    r.sleep();
  }

  return EXIT_SUCCESS;
}
