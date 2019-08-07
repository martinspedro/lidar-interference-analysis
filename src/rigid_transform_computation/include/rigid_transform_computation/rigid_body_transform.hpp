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



namespace rigid_body_transform {
    static const std::string THESIS_FILE_PATH = "/media/martinspedro/DATA/UA/Thesis/multiple-lidar-interference-mitigation/";

    cv::Point3f transformLiDARToCameraReferential(cv::Point3f lidar_point);

    class LiDARCameraCalibrationData {
        public:
            LiDARCameraCalibrationData(std::string frame_id, std::string child_frame_id);
            LiDARCameraCalibrationData(std::string frame_id,
                                       std::string child_frame_id,
                                       std::vector<cv::Point3f> point_cloud_points,
                                       std::vector<cv::Point2f> image_pixels);

            ~LiDARCameraCalibrationData();


            bool computeRigidBodyTransform(int pnp_mode);
            void readCameraInfoFromYAML(std::string filename);

        private:
            void createStaticTransformMsg();

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
