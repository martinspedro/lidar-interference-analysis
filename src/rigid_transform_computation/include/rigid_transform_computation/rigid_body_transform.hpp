/**
 * @file   rigid_body_transform.cpp
 * @brief
 *
 * @author Pedro Martins
 */


#ifndef RIGID_BODY_TRANSFORM_H
#define RIGID_BODY_TRANSFORM_H


#include <string>

#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"

#include "rigid_transform_computation/computeRigidBodyTransform.h" //!< Srv message for rigid body transform computation
#include "rigid_transform_computation/save_correspondences.h"   //!< Service to save correspondences

#include "rigid_transform_computation/pointCloudVisualizer.hpp"

#include "rigid_transform_computation/Pixel.h"
#include "rigid_transform_computation/PointXYZI.h"


namespace rigid_body_transform {
    static const std::string THESIS_FILE_PATH = "/media/martinspedro/DATA/UA/Thesis/multiple-lidar-interference-mitigation/";

    cv::Point3f transformLiDARToCameraReferential(cv::Point3f lidar_point);

    typedef struct Pixel_struct {
        uint32_t x;
        uint32_t y;
        uint8_t r; //!< Red  component
        uint8_t g; //!< Green component
        uint8_t b; //!< Blue  component
    } Pixel_t;

    class RigidTransformData {
        point_cloud::PointI pointCloudPoint;
        Pixel_t     imagePixel;
        sensor_msgs::CameraInfo camera_info;
    };


    class LiDARCameraCalibrationData {
        public:
            LiDARCameraCalibrationData(std::string frame_id, std::string child_frame_id);
            LiDARCameraCalibrationData(std::string frame_id,
                                       std::string child_frame_id,
                                       std::vector<cv::Point3f> point_cloud_points,
                                       std::vector<cv::Point2f> image_pixels);

            ~LiDARCameraCalibrationData();


            void publishStaticTransformMsg();
            void computeRigidBodyTransform(int pnp_mode);
            void readCameraInfoFromYAML(std::string filename);

            bool saveCorrespondencesSrvCallback(rigid_transform_computation::save_correspondences::Request  &req,
                                                rigid_transform_computation::save_correspondences::Response &res);

            bool computeRigidTransformSrvCallback(rigid_transform_computation::computeRigidBodyTransform::Request  &req,
                                                  rigid_transform_computation::computeRigidBodyTransform::Response &res);

            void pixelCallback(const rigid_transform_computation::Pixel::ConstPtr& msg);
            void pointCloudCallback(const rigid_transform_computation::PointXYZI::ConstPtr& msg);
            void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);
        private:
            geometry_msgs::TransformStamped createStaticTransformMsg();
            void solvePnP(int solvePnP_mode);

            std::string frame_id_;
            std::string child_frame_id_;

            std::vector<cv::Point3f>           point_cloud_points_;
            std::vector<cv::Point2f>           image_pixels_;
            image_geometry::PinholeCameraModel camera_info_;

            cv::Mat rotation_vector_;
            cv::Mat translation_vector_;
            tf2::Quaternion rotation_quaternion_;

            tf2_ros::StaticTransformBroadcaster* static_broadcaster_;
    };


}

#endif // RIGID_BODY_TRANSFORM_H
