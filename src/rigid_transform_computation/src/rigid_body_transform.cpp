

#include <yaml-cpp/yaml.h>  //!< yaml-cpp is installed as a shared library in usr/local/

#include "tf2/LinearMath/Matrix3x3.h"
#include <geometry_msgs/TransformStamped.h>
#include "rigid_transform_computation/CSV_manager.hpp"

#include "rigid_transform_computation/rigid_body_transform.hpp"


namespace rigid_body_transform {

    cv::Point3f transformLiDARToCameraReferential(cv::Point3f lidar_point) {
        cv::Point3f temp_3D_point;

        temp_3D_point.x = lidar_point.y;
        temp_3D_point.y = -lidar_point.z;
        temp_3D_point.z = lidar_point.x;

        return temp_3D_point;
    }

    LiDARCameraCalibrationData::LiDARCameraCalibrationData(std::string frame_id, std::string child_frame_id) :
        frame_id_(frame_id), child_frame_id_(child_frame_id)
    {
        this->rotation_vector_     = cv::Mat(3, 1, CV_32FC1);
        this->translation_vector_  = cv::Mat(3, 1, CV_32FC1);
        this->rotation_quaternion_ = tf2::Quaternion(0, 0, 0, 1);

        this->static_broadcaster_  = new tf2_ros::StaticTransformBroadcaster();
    }

    LiDARCameraCalibrationData::LiDARCameraCalibrationData(std::string frame_id,
                                     std::string child_frame_id,
                                     std::vector<cv::Point3f> point_cloud_points,
                                     std::vector<cv::Point2f> image_pixels) :
                                     // initialization
                                     frame_id_(frame_id),
                                     child_frame_id_(child_frame_id),
                                     point_cloud_points_(point_cloud_points),
                                     image_pixels_(image_pixels)
    {
        this->rotation_vector_     = cv::Mat(3, 1, CV_32FC1);
        this->translation_vector_  = cv::Mat(3, 1, CV_32FC1);
        this->rotation_quaternion_ = tf2::Quaternion(0, 0, 0, 1);

        this->static_broadcaster_  = new tf2_ros::StaticTransformBroadcaster();
    }

    LiDARCameraCalibrationData::~LiDARCameraCalibrationData()
    {
        if(this->static_broadcaster_)
        {
            delete this->static_broadcaster_;
        }
    }


    geometry_msgs::TransformStamped LiDARCameraCalibrationData::createStaticTransformMsg()
    {
        geometry_msgs::TransformStamped static_transform_stamped;

        static_transform_stamped.header.stamp    = ros::Time::now();
        static_transform_stamped.header.frame_id = this->frame_id_;       //"velo_link";
        static_transform_stamped.child_frame_id  = this->child_frame_id_; //"camera_link";

        static_transform_stamped.transform.translation.x = this->translation_vector_.at<double>(0, 0);
        static_transform_stamped.transform.translation.y = this->translation_vector_.at<double>(1, 0);
        static_transform_stamped.transform.translation.z = this->translation_vector_.at<double>(2, 0);

        static_transform_stamped.transform.rotation.x = this->rotation_quaternion_.x();
        static_transform_stamped.transform.rotation.y = this->rotation_quaternion_.y();
        static_transform_stamped.transform.rotation.z = this->rotation_quaternion_.z();

        static_transform_stamped.transform.rotation.w = - this->rotation_quaternion_.w(); //invert the transform

        return static_transform_stamped;
    }

    void LiDARCameraCalibrationData::publishStaticTransformMsg()
    {
        geometry_msgs::TransformStamped temp_static_transform_stamped = this->createStaticTransformMsg();
        this->static_broadcaster_->sendTransform(temp_static_transform_stamped);
    }

    void LiDARCameraCalibrationData::solvePnP(int solvePnP_mode)
    {
        switch (solvePnP_mode)
        {
        case 0:
            cv::solvePnP(this->point_cloud_points_, this->image_pixels_,
                         this->camera_info_.fullIntrinsicMatrix(), this->camera_info_.distortionCoeffs(),
                         this->rotation_vector_, this->translation_vector_, false, cv::SOLVEPNP_ITERATIVE);
            std::cout << "Solving using SOLVEPNP_ITERATIVE" << std::endl;
            break;

        case 1:
            cv::solvePnP(this->point_cloud_points_, this->image_pixels_,
                         this->camera_info_.fullIntrinsicMatrix(), this->camera_info_.distortionCoeffs(),
                         this->rotation_vector_, this->translation_vector_, false, cv::SOLVEPNP_P3P);
            std::cout << "Solving using SOLVEPNP_P3P" << std::endl;
            break;

        case 2:
            cv::solvePnP(this->point_cloud_points_, this->image_pixels_,
                         this->camera_info_.fullIntrinsicMatrix(), this->camera_info_.distortionCoeffs(),
                         this->rotation_vector_, this->translation_vector_, false, cv::SOLVEPNP_EPNP);
            std::cout << "Solving using SOLVEPNP_EPNP" << std::endl;
            break;

        case 3:
            cv::solvePnP(this->point_cloud_points_, this->image_pixels_,
                       this->camera_info_.fullIntrinsicMatrix(), this->camera_info_.distortionCoeffs(),
                       this->rotation_vector_, this->translation_vector_, false, cv::SOLVEPNP_DLS);
            std::cout << "Solving using SOLVEPNP_DLS" << std::endl;
            break;

        case 4:
            cv::solvePnP(this->point_cloud_points_, this->image_pixels_,
                         this->camera_info_.fullIntrinsicMatrix(), this->camera_info_.distortionCoeffs(),
                         this->rotation_vector_, this->translation_vector_, false, cv::SOLVEPNP_UPNP);
            std::cout << "Solving using SOLVEPNP_UPNP" << std::endl;
        default:
            ROS_ERROR("Invalid solvePnP Method!");
            break;
        }
    }


    void LiDARCameraCalibrationData::computeRigidBodyTransform(int pnp_mode)
    {
        solvePnP(pnp_mode);

        cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1);
        cv::Rodrigues(this->rotation_vector_,rotation_matrix); // Rodrigues overloads matrix type from float to double!

        tf2::Matrix3x3 tf2_rotation_matrix(
                        rotation_matrix.at<double>(0,0), rotation_matrix.at<double>(0,1), rotation_matrix.at<double>(0,2),
                        rotation_matrix.at<double>(1,0), rotation_matrix.at<double>(1,1), rotation_matrix.at<double>(1,2),
                        rotation_matrix.at<double>(2,0), rotation_matrix.at<double>(2,1), rotation_matrix.at<double>(2,2));

        tf2_rotation_matrix.getRotation(this->rotation_quaternion_);
        this->rotation_quaternion_.normalize();     // always normalize the quaternion to avoid numerical errors

        std::cout << "Normalized Camera to LiDAR Quaternion (x, y, z, w): (" <<
                     this->rotation_quaternion_.x() << ", " <<
                     this->rotation_quaternion_.y() << ", " <<
                     this->rotation_quaternion_.z() << ", " <<
                     this->rotation_quaternion_.w() << ")"  <<
                     std::endl;
        std::cout << "Normalized LiDAR to Camera Quaternion (x, y, z, w): (" <<
                     this->rotation_quaternion_.x()   << ", " <<
                     this->rotation_quaternion_.y()   << ", " <<
                     this->rotation_quaternion_.z()   << ", " <<
                     - this->rotation_quaternion_.w() << ")"  <<
                     std::endl;
    }


    /* http://library.isr.ist.utl.pt/docs/roswiki/yaml_cpp.html
     * https://ossyaritoori.hatenablog.com/entry/2017/08/16/Read_yaml_file_with_yaml-cpp_in_ROS_C%2B%2B_/_
     * https://github.com/jbeder/yaml-cpp/wiki/Tutorial
     * https://stackoverflow.com/questions/50150387/parsing-yaml-file-using-yaml-cpp-error
     */
    void LiDARCameraCalibrationData::readCameraInfoFromYAML(std::string filename)
    {
        YAML::Node camera_info_yaml = YAML::LoadFile(filename);

        sensor_msgs::CameraInfo camera_info_msg;
        std::array<double, 9ul> K, R;
        std::array<double, 12ul> P;

        camera_info_msg.width = camera_info_yaml["image_width"].as<unsigned int>();
        camera_info_msg.height = camera_info_yaml["image_height"].as<unsigned int>();

        camera_info_msg.distortion_model = camera_info_yaml["distortion_model"].as<std::string>();

        // Instead of creating YAML nodes to deal with Map events, acess directly the data field, since the others are not needed
        camera_info_msg.D = camera_info_yaml["distortion_coefficients"]["data"].as<std::vector<double>>();

        K = camera_info_yaml["camera_matrix"]["data"].as< std::array<double, 9ul> >();
        R = camera_info_yaml["rectification_matrix"]["data"].as< std::array<double, 9ul> >();
        P = camera_info_yaml["projection_matrix"]["data"].as< std::array<double, 12ul> >();

        std::memcpy(&camera_info_msg.K[0], &K[0], sizeof(double)*9);
        std::memcpy(&camera_info_msg.R[0], &R[0], sizeof(double)*9);
        std::memcpy(&camera_info_msg.P[0], &P[0], sizeof(double)*12);

        this->camera_info_.fromCameraInfo(camera_info_msg);
    }


    bool LiDARCameraCalibrationData::saveCorrespondencesSrvCallback(rigid_transform_computation::save_correspondences::Request  &req,
                                                                    rigid_transform_computation::save_correspondences::Response &res)
    {
        std::ostringstream oss;

        if (!( (this->image_pixels_.size() == 0) | (this->point_cloud_points_.size() == 0) )) {
            if (this->image_pixels_.size() == this->point_cloud_points_.size()) {
                create(THESIS_FILE_PATH + req.filename, this->image_pixels_, this->point_cloud_points_);

                oss << "File saved in " << req.filename;
                res.message = oss.str();
                res.success = true;
            } else {
                oss << "Number of selected pixels differs from the number of selected 3D Points: "<< this->image_pixels_.size()
                    << " vs " << this->point_cloud_points_.size() << ". File not saved!" ;
                res.message = oss.str();
                res.success = false;
            }
        } else {
            oss << "No pair of data points have yet been selecting. Aborting!";
            res.message = oss.str();
            res.success = false;
        }

        return true;    // Return true means the service is called
    }

    bool LiDARCameraCalibrationData::computeRigidTransformSrvCallback(rigid_transform_computation::computeRigidBodyTransform::Request  &req,
                                                                      rigid_transform_computation::computeRigidBodyTransform::Response &res)
    {
        this->computeRigidBodyTransform(req.solvePnpType);
        res.rigidBodyTransform = this->createStaticTransformMsg();

        return true;
    }


    void LiDARCameraCalibrationData::pixelCallback(const rigid_transform_computation::Pixel::ConstPtr& msg)
    {
        cv::Point2f temp_pixel_point;

        temp_pixel_point.x = msg->x;
        temp_pixel_point.y = msg->y;

        this->image_pixels_.push_back(temp_pixel_point);
    }

    void LiDARCameraCalibrationData::pointCloudCallback(const rigid_transform_computation::PointXYZI::ConstPtr& msg)
    {
        cv::Point3f temp_3D_point;

        temp_3D_point.x = msg->x;
        temp_3D_point.y = msg->y;
        temp_3D_point.z = msg->z;

        this->point_cloud_points_.push_back(temp_3D_point);
    }

    void LiDARCameraCalibrationData::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info)
    {
        this->camera_info_.fromCameraInfo(cam_info);
    }

}
