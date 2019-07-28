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


/*
#include <Matrix3x3.h>


#include <Transform.h>

#include "tf_conversions/tf_eigen.h"
#include "tf/transform_datatypes.h"
#include "Eigen/Core"
#include <Eigen/Geometry>
*/

using namespace point_cloud;

const std::string THESIS_FILE_PATH = "/media/martinspedro/DATA/UA/Thesis/multiple-lidar-interference-mitigation/";

int flag = 0;   // Temporary solution. Yaml-cpp should be used to read calibration file

std::vector<cv::Point3f> pointCloudPoints;
std::vector<cv::Point2f> imagePixels;
std::vector<image_geometry::PinholeCameraModel> camera_info;

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
                     camera_info[0].fullIntrinsicMatrix(), camera_info[0].distortionCoeffs(),
                     rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);
        std::cout << "Solving using SOLVEPNP_ITERATIVE" << std::endl;
        break;
    case 1:
        cv::solvePnP(pointCloudPoints, imagePixels,
                     camera_info[0].fullIntrinsicMatrix(), camera_info[0].distortionCoeffs(),
                     rVec, tVec, false, cv::SOLVEPNP_P3P);
        std::cout << "Solving using SOLVEPNP_P3P" << std::endl;
        break;
     case 2:
         cv::solvePnP(pointCloudPoints, imagePixels,
                      camera_info[0].fullIntrinsicMatrix(), camera_info[0].distortionCoeffs(),
                      rVec, tVec, false, cv::SOLVEPNP_EPNP);
        std::cout << "Solving using SOLVEPNP_EPNP" << std::endl;
        break;
  case 3:
      cv::solvePnP(pointCloudPoints, imagePixels,
                   camera_info[0].fullIntrinsicMatrix(), camera_info[0].distortionCoeffs(),
                   rVec, tVec, false, cv::SOLVEPNP_DLS);
    std::cout << "Solving using SOLVEPNP_DLS" << std::endl;
    break;
case 4:
        cv::solvePnP(pointCloudPoints, imagePixels,
                     camera_info[0].fullIntrinsicMatrix(), camera_info[0].distortionCoeffs(),
                     rVec, tVec, false, cv::SOLVEPNP_UPNP);
      std::cout << "Solving using SOLVEPNP_UPNP" << std::endl;
    default:

        break;
    }


  std::cout << "Rotation Vector: " << rVec << std::endl;
  std::cout << "Translation Vector: " << tVec << std::endl;

  cv::Mat rMat = cv::Mat(3, 3, CV_32FC1);
  cv::Vec3f eulerVec;
  cv::Rodrigues(rVec,rMat); // Rodrigues overloads matrix type from float to double!
  std::cout << "Rotation Matrix: " << rMat << std::endl;

 /*
  cv::Mat tVec4 = cv::Mat(4, 1, CV_32FC1);
  cv::Mat univector = cv::Mat(1, 1, CV_64FC1);
  std::cout << "New Translation Vector: " << tVec4 << std::endl;
  std::cout << "x " << tVec.dims << std::endl;
  std::cout << "x " << tVec.cols << std::endl;
  std::cout << "x " << tVec.type()     << std::endl;

  std::cout << "x " << univector.dims << std::endl;
  std::cout << "x " << univector.cols << std::endl;
  std::cout << "x " << univector.type()     << std::endl;

  //cv:vconcat(tVec, univector, tVec4); // rVec is a column vector already
  //tVec.push_back( cv::Mat(1, 1, CV_64FC1));
  std::cout << "New Translation Vector: " << tVec4 << std::endl;

  cv::Mat projectionMatrix = cv::Mat(3, 4, CV_32FC1);
   std::cout << "Projection Matrix: " << projectionMatrix << std::endl;
  cv::hconcat(rMat, rVec, projectionMatrix); // rVec is a column vector already
  std::cout << "Projection Matrix: " << projectionMatrix << std::endl;

  cv::Mat rotMat = cv::Mat(3, 3, CV_32FC1);
  cv::Mat camera_matrix = cv::Mat(3, 3, CV_32FC1);
  cv::Mat rotMatrixX = cv::Mat(3, 3, CV_32FC1);
  cv::Mat rotMatrixY = cv::Mat(3, 3, CV_32FC1);
  cv::Mat rotMatrixZ = cv::Mat(3, 3, CV_32FC1);

  cv::Vec3f eulerAngles;

  std::cout << "Decomposing Matrix" << std::endl;
  std::cout << projectionMatrix.type()     << std::endl;
   std::cout << camera_matrix.type()     << std::endl;
   //std::cout << camera_info[0].fullIntrinsicMatrix().type()     << std::endl;
    std::cout << rMat.type()     << std::endl;
     std::cout << tVec4.type()     << std::endl;

  cv::decomposeProjectionMatrix( projectionMatrix,
                                 camera_matrix,
                                 rotMat,
                                 tVec4,
                                 rotMatrixX,
                                 rotMatrixY,
                                 rotMatrixZ,
                                 eulerAngles);
  std::cout << "Euler Angles: " << eulerAngles << std::endl;
*/

  //geometry_msgs::Pose rigidBodyPose;
  //tf::Transform rigidBodyTransform;

  // row major storing. Opencv uses row-major access so it is create using a commonly matrix convention
  /*
  std::cout << "xx (0,0): " << rMat.at<double>(0,0) << std::endl;
  std::cout << "xy (0,1): " << rMat.at<double>(0,1) << std::endl;
  std::cout << "xz (0,2): " << rMat.at<double>(0,2) << std::endl;
  std::cout << "yx (1,0): " << rMat.at<double>(1,0) << std::endl;
  std::cout << "yy (1,1): " << rMat.at<double>(1,1) << std::endl;
  std::cout << "yz (1,2): " << rMat.at<double>(1,2) << std::endl;
  std::cout << "zx (2,0): " << rMat.at<double>(2,0) << std::endl;
  std::cout << "zy (2,1): " << rMat.at<double>(2,1) << std::endl;
  std::cout << "zz (2,2): " << rMat.at<double>(2,2) << std::endl;
  */

  tf2::Matrix3x3 aux(rMat.at<double>(0,0), rMat.at<double>(0,1), rMat.at<double>(0,2),
                    rMat.at<double>(1,0), rMat.at<double>(1,1), rMat.at<double>(1,2),
                    rMat.at<double>(2,0), rMat.at<double>(2,1), rMat.at<double>(2,2));

  //std::cout << aux << std::endl;

  tf2::Quaternion rotQ;
  aux.getRotation(rotQ);
  rotQ.normalize();     // always normalize the quaternion to avoid numerical errors

  //std::cout << rotQ << std::endl;
  ///std::cout << "q.w() = " << rotQ.w() << std::endl; //Print out the scalar
  //std::cout << "q.vec() = " << rotQ.getAxis() << std::endl; //Print out the orientation vector
  //Eigen::Quaterniond q(rMat);

  //tf::Quaternion()

  createStaticTransformMsg(tVec, rotQ);


  //rigidBodyTransform.position = tVec;
  //rigidBodyTransform.orientation = q;
  flag = 2;
  return true;
}

void getEulerAngles(cv::Mat &rotCamerMatrix,cv::Vec3f &eulerAngles){
    cv::Mat cameraMatrix,rotMatrix,transVect,rotMatrixX,rotMatrixY,rotMatrixZ;
    float* _r = rotCamerMatrix.ptr<float>();
    float projMatrix[12] = {_r[0],_r[1],_r[2], 0,
                            _r[3],_r[4],_r[5], 0,
                            _r[6],_r[7],_r[8], 0};

    cv::decomposeProjectionMatrix( cv::Mat(3,4,CV_32FC1,projMatrix),
                               cameraMatrix,
                               rotMatrix,
                               transVect,
                               rotMatrixX,
                               rotMatrixY,
                               rotMatrixZ,
                               eulerAngles);
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    image_geometry::PinholeCameraModel tempCameraModel;

    tempCameraModel.fromCameraInfo(cam_info);
    camera_info.push_back(tempCameraModel);
    std::cout << "Callback" << std::endl;

    if (flag == 0) {
        flag = 1;
    }

}

int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "rigid_transform_computation");
    std::cout << "Node init succesfully" << std::endl;

    if (argc != 3) {
      ROS_INFO("Usage: rosrun rigid_transform_computation compute_rigid_body_transform csv_filename solvePnP_mode");
      return EXIT_FAILURE;
    }


    for (int i = 0; i < argc; ++i) {
        std::cout << argv[i] << std::endl;
    }

    ros::NodeHandle nh;
    ros::Subscriber camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>("/camera/camera_info", 1, cameraInfoCallback);

    load(THESIS_FILE_PATH + argv[1], &imagePixels, &pointCloudPoints);

    std::cout << "Data Load succesfull" << std::endl;
    //std::cout << imagePixels << std::endl;
    //std::cout << pointCloudPoints << std::endl;

    //ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/camera/camera_info",ros::Duration(10));

    ros::Rate r(1);
    while(ros::ok()){
        if(flag == 1) {
            //std::cout << "Camera Info" << camera_info[0].fullIntrinsicMatrix() << std::endl;
            computeRigidTransform(atoll(argv[2]));
        }
        ros::spinOnce();
        r.sleep();
    }

    ros::spin();
    return EXIT_SUCCESS;
}
