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

#include <geometry_msgs/TransformStamped.h>

#include <cmath>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>

#include "tf2/LinearMath/Matrix3x3.h"
#include <geometry_msgs/TransformStamped.h>
#include "tf2/LinearMath/Quaternion.h"

#include <string>

const float NEAR_PLANE_DISTANCE = 1.0f;
const float FAR_PLANE_DISTANCE = 20.0f;

const double PIXEL_SIZE = 3.45e-6;

const unsigned int SYNC_POLICY_QUEUE_SIZE = 20;  // queue size for syncPolicyForCallback;

const std::string LIDAR_TF2_REFERENCE_FRAME = "velo_link";
const std::string CAMERA_TF2_REFERENCE_FRAME = "camera_color_left";

geometry_msgs::TransformStamped transformStamped;

// Eigen::Quaterniond tf_quaternion;
// Eigen::Vector3d tf_translation;
Eigen::Affine3d transform_eigen;

ros::Publisher pub, pub_camera, pub_camera_lidar, pub_lidar, pub_final;  //!< ROS Publisher

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
  std::cout << "Camera Matrix:\n" << cam_model_.fullIntrinsicMatrix() << std::endl;

  cv::Size image_resolution = cam_model_.fullResolution();
  std::cout << "Image Resolution: " << image_resolution << " | " << image_resolution.width << ", "
            << image_resolution.height << std::endl;

  FOV image_fov = pinhole_camera::getImageFOV(cam_model_);
  std::cout << "FOV: " << image_fov << std::endl;

  // double sensor_active_area_width = 9.93e-3;
  // double sensor_active_area_height = 8.70e-3;

  // double aperture_width = image_resolution.width * PIXEL_SIZE;
  // double aperture_height = image_resolution.height * PIXEL_SIZE;
  double fov_x, fov_y, focal_length, aspect_ratio;
  double aperture_width = 16e-3 * image_resolution.width / cam_model_.fx();
  double aperture_height = 16e-3 * image_resolution.height / cam_model_.fy();
  std::cout << "Aperture Width: " << aperture_width << std::endl;
  std::cout << "Aperture Height: " << aperture_height << std::endl;

  cv::Point2d principal_point;
  cv::calibrationMatrixValues(cam_model_.fullIntrinsicMatrix(),
                              cv::Size(image_resolution.height, image_resolution.width), aperture_width,
                              aperture_height, fov_x, fov_y, focal_length, principal_point, aspect_ratio);
  std::cout << "FOV from calibrationMatrixValues: (" << fov_x << ", " << fov_y << ")" << std::endl;
  std::cout << "FOcal length: " << focal_length << std::endl;
  std::cout << "Principal Point: " << principal_point << std::endl;
  std::cout << "Aspect Ratio: " << aspect_ratio << std::endl;

  /*
    Eigen::Matrix4f camera_transform;
    Eigen::Matrix4f lidar_pose;
    Eigen::Matrix4f frustrum_filter_pose;
    Eigen::Matrix4f camera_to_lidar_6DOF;

    for (int i = 0; i < object_count->count; ++i)
    {
      FOV bounding_box_fov;
      Eigen::Vector3f camera_rotation;
      pinhole_camera::computeBoundingBoxFOV(cam_model_, b_boxes->bounding_boxes[i], &bounding_box_fov, camera_rotation);
      std::cout << "Compute BBOxFOV" << std::endl;

      // Pose Rotations

      // Compute Camera Affine Transform from Angle Axis vector
      cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1);
      cv::Vec3f camera_rotation_cv;
      Eigen::Matrix3f rotation_matrix_eigen;

      cv::eigen2cv<float, 3, 1>(camera_rotation, camera_rotation_cv);
      cv::Rodrigues(camera_rotation_cv, rotation_matrix);  // Rodrigues overloads matrix type from float to double!
      cv::cv2eigen<float, 3, 3>(rotation_matrix, rotation_matrix_eigen);

      std::cout << "Compute Camera Rotation" << std::endl;
      // Eigen::Quaternionf q_camera(rotation_matrix_eigen);
      // q_camera.normalize();
      camera_transform = Eigen::Matrix4f::Identity();
      camera_transform.block<3, 3>(0, 0) = rotation_matrix_eigen;  // q_camera.matrix();  // Quaternion to Rot Matrix
      std::cout << "Compute Rotation Matrix Done" << camera_transform << std::endl;
      std::cout << "Rotation Matrix: " << rotation_matrix_eigen << std::endl;
      // lidar_pose_.block<4, 1>(0, 4) = point_cloud_ptr->sensor_origin_;  // Vector 4f

      // Get Initial LiDAR Pose
      camera_to_lidar_6DOF = (transform_eigen.matrix()).cast<float>();  // Convert Eigen Affine3d Trans
      std::cout << "6DOF" << std::endl;
      lidar_pose = getLiDARPose(point_cloud_ptr);
      std::cout << "LiDAR Pose Done" << std::endl;

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
  /*
      Eigen::Matrix4f camera_pose = Eigen::Matrix4f::Identity();

      Eigen::Matrix4f lidar_rotation = getLiDARRotation(cam_model_, b_boxes->bounding_boxes[i]);

      // std::cout << "Camera Transform: " << camera_transform.IsRowMajor << std::endl;
      // std::cout << "Camera to LiDAR 6DOF Transform: " << camera_to_lidar_6DOF.IsRowMajor << std::endl;
      // std::cout << "Lidar Pose: " << lidar_pose.IsRowMajor << std::endl;
      // std::cout << "Camera Pose: " << camera_pose.IsRowMajor << std::endl;

      frustrum_filter_pose =
          // camera_transform * camera_to_lidar_6DOF * lidar_pose_to_frustum_pose;
          // camera_transform * camera_to_lidar_6DOF * lidar_pose * lidar_pose_to_frustum_pose * camera_pose;
          // lidar_pose_to_frustum_pose * lidar_rotation * camera_to_lidar_6DOF;
          // lidar_pose_to_frustum_pose *
          camera_to_lidar_6DOF * LIDAR_POSE_TO_FRUSTRUM_POSE * lidar_rotation;
      // lidar_pose_to_frustum_pose * camera_to_lidar_6DOF * camera_transform;
      // lidar_rotation * camera_to_lidar_6DOF * lidar_pose_to_frustum_pose;

      std::cout << "Fustrum Filter Pose: " << frustrum_filter_pose.IsRowMajor << std::endl;

      // Print Stuff
      float fov_X =
          atan((b_boxes->bounding_boxes[i].xmax - b_boxes->bounding_boxes[i].xmin) / (cam_model_.fx())) * 180.0f / M_PI;
      float fov_Y =
          atan((b_boxes->bounding_boxes[i].ymax - b_boxes->bounding_boxes[i].ymin) / (cam_model_.fy())) * 180.0f / M_PI;

      // std::cout << "(" << b_boxes->bounding_boxes[i].xmin << ", " << b_boxes->bounding_boxes[i].xmax << ") e ("
      //          << b_boxes->bounding_boxes[i].ymin << ", " << b_boxes->bounding_boxes[i].ymax << ") " << std::endl;
      // std::cout << b_boxes->bounding_boxes[i].Class << ": " << b_boxes->bounding_boxes[i].probability << " (" <<
     fov_X
      //          << " ," << fov_Y << ") with Principial Point: " << principal_point << std::endl;
      std::cout << b_boxes->bounding_boxes[i].Class << ": " << b_boxes->bounding_boxes[i].probability << " ("
                << bounding_box_fov.x << " ," << bounding_box_fov.y << ") with Principial Point: " << principal_point
                << std::endl;
      // std::cout << "Rotation: " << camera_rotation << std::endl;

      pcl::FrustumCulling<PointType> fc;
      fc.setInputCloud(point_cloud_ptr);
      fc.setVerticalFOV(bounding_box_fov.y);
      fc.setHorizontalFOV(bounding_box_fov.x);
      fc.setNearPlaneDistance(NEAR_PLANE_DISTANCE);
      fc.setFarPlaneDistance(FAR_PLANE_DISTANCE);
      std::cout << "FrustumCulling" << std::endl;

      /*
       * http://docs.pointclouds.org/trunk/frustum__culling_8hpp_source.html#l00047 cv::Size im_dimensions =
       * cam_model_.fullResolution(); This assumes a coordinate system where X is forward, Y is up, and Z is right. To
       * convert from the traditional camera coordinate system (X right, Y down, Z forward), one can use:
       */
  /*
      fc.setCameraPose(frustrum_filter_pose);
      fc.filter(target);
      std::cout << "Filtered. Target size = " << target.size() << std::endl;
      *final_point_cloud_ptr += target;
    }  // FOR ENDS HERE

    // sensor_msgs::PointCloud2 out_cloud;
    *point_cloud_ptr = target;
    std::cout << "Atributed" << std::endl;
    // pcl::toROSMsg(target, *out_cloud);
    std::cout << "toROSMsg Done" << std::endl;
    final_point_cloud_ptr->header.frame_id = LIDAR_TF2_REFERENCE_FRAME;
    pub.publish(final_point_cloud_ptr);  // PointCloud Object is automacally serialized by ROS and there is no need to
                                         // call toROSMsg
    std::cout << "Published" << std::endl;

    // POSE Publishing
    geometry_msgs::Pose pose_camera, pose_lidar_camera, pose_lidar, pose_final;
    geometry_msgs::PoseStamped pose_camera_s, pose_lidar_s, pose_lidar_camera_s, pose_final_s;
    Eigen::Affine3d camera_affine_transform, lidar_camera_affine_transform, lidar_affine_transform,
        final_affine_transform;

    camera_affine_transform.matrix() = camera_transform.cast<double>();
    lidar_affine_transform.matrix() = lidar_pose.cast<double>();
    lidar_camera_affine_transform.matrix() = camera_to_lidar_6DOF.cast<double>();
    final_affine_transform.matrix() = frustrum_filter_pose.cast<double>();

    pose_camera = Eigen::toMsg(camera_affine_transform);
    pose_lidar = Eigen::toMsg(lidar_affine_transform);
    pose_lidar_camera = Eigen::toMsg(lidar_camera_affine_transform);
    pose_final = Eigen::toMsg(final_affine_transform);

    pose_camera_s.header.stamp = ros::Time::now();
    pose_lidar_s.header.stamp = ros::Time::now();
    pose_lidar_camera_s.header.stamp = ros::Time::now();
    pose_final_s.header.stamp = ros::Time::now();

    pose_camera_s.header.frame_id = "camera_color_left";
    pose_lidar_s.header.frame_id = "velo_link";
    pose_lidar_camera_s.header.frame_id = "camera_color_left";
    pose_final_s.header.frame_id = "velo_link";

    pose_camera_s.pose = pose_camera;
    pose_lidar_s.pose = pose_lidar;
    pose_lidar_camera_s.pose = pose_lidar_camera;
    pose_final_s.pose = pose_final;
    // msg.header.frame_id = "detection";

    pub_camera.publish(pose_camera_s);
    pub_lidar.publish(pose_lidar_s);
    pub_camera_lidar.publish(pose_lidar_camera_s);
    pub_final.publish(pose_final_s);
    */
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "correspondences_finder");

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

  transformStamped = tfBuffer.lookupTransform(LIDAR_TF2_REFERENCE_FRAME, CAMERA_TF2_REFERENCE_FRAME, ros::Time(0),
                                              ros::Duration(20.0));

  // get Affine3d matrix that defines the 6DOF transformation between LiDAR and Camera
  transform_eigen = tf2::transformToEigen(transformStamped);
  std::cout << transform_eigen.matrix() << std::endl;

  pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud", 1);
  pub_camera = nh.advertise<geometry_msgs::PoseStamped>("/rotated_camera_pose", 1);
  pub_camera_lidar = nh.advertise<geometry_msgs::PoseStamped>("/camera_pose_in_lidar", 1);
  pub_lidar = nh.advertise<geometry_msgs::PoseStamped>("/lidar_pose", 1);
  pub_final = nh.advertise<geometry_msgs::PoseStamped>("/final_pose", 1);

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
