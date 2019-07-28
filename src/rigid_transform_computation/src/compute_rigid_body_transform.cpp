#include <iostream>
#include <sstream>

#include "rigid_transform_computation/imageVisualizer.hpp"
#include "rigid_transform_computation/pointCloudVisualizer.hpp"


#include <image_geometry/pinhole_camera_model.h>

#include "rigid_transform_computation/Pixel.h"
#include "rigid_transform_computation/PointXYZI.h"
#include <yaml-cpp/yaml.h>  //!< yaml-cpp is installed as a shared library in usr/local/

#include <string>

#include "rigid_transform_computation/computeRigidBodyTransform.h" //!< Srv message for rigid body transform computation
#include "rigid_transform_computation/save_correspondences.h"      //!< Service to save correspondences
#include "rigid_transform_computation/CSV_manager.hpp"


#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>


using namespace point_cloud;

const std::string THESIS_FILE_PATH = "/media/martinspedro/DATA/UA/Thesis/multiple-lidar-interference-mitigation/";

std::vector<cv::Point3f> pointCloudPoints;
std::vector<cv::Point2f> imagePixels;
image_geometry::PinholeCameraModel camera_info;

cv::Mat rVec = cv::Mat(3, 1, CV_32FC1);
cv::Mat tVec = cv::Mat(3, 1, CV_32FC1);



void createStaticTransformMsg(cv::Mat translation_vector, tf2::Quaternion rotation_quaternion) {
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = "velo_link";
    static_transformStamped.child_frame_id = "camera_color_left";

    static_transformStamped.transform.translation.x = translation_vector.at<double>(0, 0);
    static_transformStamped.transform.translation.y = translation_vector.at<double>(1, 0);
    static_transformStamped.transform.translation.z = translation_vector.at<double>(2, 0);

    static_transformStamped.transform.rotation.x = rotation_quaternion.x();
    static_transformStamped.transform.rotation.y = rotation_quaternion.y();
    static_transformStamped.transform.rotation.z = rotation_quaternion.z();

    static_transformStamped.transform.rotation.w = - rotation_quaternion.w(); //invert the transform

    static_broadcaster.sendTransform(static_transformStamped);
}

bool computeRigidTransform(int pnp_mode) {
    switch (pnp_mode)
    {
    case 0:
        cv::solvePnP(pointCloudPoints, imagePixels,
                     camera_info.fullIntrinsicMatrix(), camera_info.distortionCoeffs(),
                     rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);
        std::cout << "Solving using SOLVEPNP_ITERATIVE" << std::endl;
        break;
    case 1:
        cv::solvePnP(pointCloudPoints, imagePixels,
                     camera_info.fullIntrinsicMatrix(), camera_info.distortionCoeffs(),
                     rVec, tVec, false, cv::SOLVEPNP_P3P);
        std::cout << "Solving using SOLVEPNP_P3P" << std::endl;
        break;
     case 2:
         cv::solvePnP(pointCloudPoints, imagePixels,
                      camera_info.fullIntrinsicMatrix(), camera_info.distortionCoeffs(),
                      rVec, tVec, false, cv::SOLVEPNP_EPNP);
        std::cout << "Solving using SOLVEPNP_EPNP" << std::endl;
        break;
  case 3:
      cv::solvePnP(pointCloudPoints, imagePixels,
                   camera_info.fullIntrinsicMatrix(), camera_info.distortionCoeffs(),
                   rVec, tVec, false, cv::SOLVEPNP_DLS);
    std::cout << "Solving using SOLVEPNP_DLS" << std::endl;
    break;
case 4:
        cv::solvePnP(pointCloudPoints, imagePixels,
                     camera_info.fullIntrinsicMatrix(), camera_info.distortionCoeffs(),
                     rVec, tVec, false, cv::SOLVEPNP_UPNP);
      std::cout << "Solving using SOLVEPNP_UPNP" << std::endl;
    default:

        break;
    }


  std::cout << "Rotation Vector: \n" << rVec << std::endl;
  std::cout << "Translation Vector: \n" << tVec << std::endl;

  cv::Mat rMat = cv::Mat(3, 3, CV_32FC1);
  cv::Vec3f eulerVec;
  cv::Rodrigues(rVec,rMat); // Rodrigues overloads matrix type from float to double!
  std::cout << "Rotation Matrix: \n" << rMat << std::endl;

  tf2::Matrix3x3 aux(rMat.at<double>(0,0), rMat.at<double>(0,1), rMat.at<double>(0,2),
                    rMat.at<double>(1,0), rMat.at<double>(1,1), rMat.at<double>(1,2),
                    rMat.at<double>(2,0), rMat.at<double>(2,1), rMat.at<double>(2,2));



  tf2::Quaternion rotQ;
  aux.getRotation(rotQ);
  rotQ.normalize();     // always normalize the quaternion to avoid numerical errors

  std::cout << "Normalized Camera to LiDAR Quaternion (x, y, z, w): (" <<
             rotQ.x() << ", " <<
             rotQ.y() << ", " <<
             rotQ.z() << ", " <<
             rotQ.w() << ")"  <<
             std::endl;
  std::cout << "Normalized LiDAR to Camera Quaternion (x, y, z, w): (" <<
            rotQ.x() << ", " <<
            rotQ.y() << ", " <<
            rotQ.z() << ", " <<
            -rotQ.w() << ")"  <<
            std::endl;

  createStaticTransformMsg(tVec, rotQ);

  return true;
}


// http://library.isr.ist.utl.pt/docs/roswiki/yaml_cpp.html
// https://ossyaritoori.hatenablog.com/entry/2017/08/16/Read_yaml_file_with_yaml-cpp_in_ROS_C%2B%2B_/_
// https://github.com/jbeder/yaml-cpp/wiki/Tutorial
// https://stackoverflow.com/questions/50150387/parsing-yaml-file-using-yaml-cpp-error
void readCameraInfoYAML(std::string filename) {

    YAML::Node camera_info_yaml = YAML::LoadFile(filename);

    sensor_msgs::CameraInfo camera_info_msg;
    std::array<double, 9ul> K, R;
    std::array<double, 12ul> P;

    camera_info_msg.width = camera_info_yaml["image_width"].as<uint32_t>();
    camera_info_msg.height = camera_info_yaml["image_height"].as<uint32_t>();

    camera_info_msg.distortion_model = camera_info_yaml["distortion_model"].as<std::string>();

    // Instead of creating YAML nodes to deal with Map events, acess directly the data field, since the others are not needed
    camera_info_msg.D = camera_info_yaml["distortion_coefficients"]["data"].as<std::vector<double>>();

    K = camera_info_yaml["camera_matrix"]["data"].as< std::array<double, 9ul> >();
    R = camera_info_yaml["rectification_matrix"]["data"].as< std::array<double, 9ul> >();
    P = camera_info_yaml["projection_matrix"]["data"].as< std::array<double, 12ul> >();

    std::memcpy(&camera_info_msg.K[0], &K[0], sizeof(double)*9);
    std::memcpy(&camera_info_msg.R[0], &R[0], sizeof(double)*9);
    std::memcpy(&camera_info_msg.P[0], &P[0], sizeof(double)*12);

    camera_info.fromCameraInfo(camera_info_msg);
}

int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "rigid_transform_computation");
    std::cout << "ROS Node init succesfully" << std::endl;

    if (argc != 4) {
      ROS_INFO("Usage: rosrun rigid_transform_computation compute_rigid_body_transform csv_filename camera_info_yaml_filename solvePnP_mode");
      return EXIT_FAILURE;
    }


    std::cout << "Loading 2D <-> 3D correspondences for calibration... ";
    load(THESIS_FILE_PATH + argv[1], &imagePixels, &pointCloudPoints);
    std::cout << "Done!" << std::endl << "Loading Camera Info YAML file... ";
    readCameraInfoYAML(argv[2]);
    std::cout << "Done!" << std::endl << "Computing Rigid Body Transform... ";
    computeRigidTransform(atoll(argv[3]));
    std::cout << "Done! " << std::endl << "Static Transform is going to be published under /tf_static. " << std::endl;


    ros::Rate r(1);
    while(ros::ok()){
        ros::spinOnce();    // Spin even if no callbacks or services exist
        r.sleep();
    }

    return EXIT_SUCCESS;
}
