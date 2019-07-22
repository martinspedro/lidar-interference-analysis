#include <iostream>
#include "rigid_transform_computation/imageVisualizer.hpp"
#include "rigid_transform_computation/pointCloudVisualizer.hpp"


#include <image_geometry/pinhole_camera_model.h>

#include "rigid_transform_computation/Pixel.h"
#include "rigid_transform_computation/PointXYZI.h"
#include <yaml-cpp/yaml.h>  // yaml-cpp is installed as a shared library in usr/local/

#include <string>

using namespace point_cloud;

typedef struct Pixel_struct {
    uint32_t x;
    uint32_t y;
    uint8_t r; //!< Red  component
    uint8_t g; //!< Green component
    uint8_t b; //!< Blue  component
} Pixel_t;

class RigidTransformData {
    PointI pointCloudPoint;
    Pixel_t     imagePixel;
    sensor_msgs::CameraInfo camera_info;
};

std::vector<cv::Point3f> pointCloudPoints;
std::vector<cv::Point2f> imagePixels;
std::vector<image_geometry::PinholeCameraModel> camera_info;

cv::Mat rVec = cv::Mat_<float>(3, 1);
cv::Mat tVec = cv::Mat_<float>(3, 1);


cv::Point3f transformLiDARToCameraReferential(cv::Point3f lidar_point) {
    cv::Point3f temp3DPoint;

    temp3DPoint.x = lidar_point.y;
    temp3DPoint.y = - lidar_point.z;
    temp3DPoint.z = lidar_point.x;
}

void computeRigidTransform(){
    //solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=ITERATIVE )¶
    cv::solvePnP(pointCloudPoints, imagePixels, camera_info[10].fullIntrinsicMatrix(), camera_info[10].distortionCoeffs(), rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);
    std::cout << rVec << std::endl;
    std::cout << tVec << std::endl;

    ros::shutdown(); // send shutdown command to node because its functionality its over

}

void pixelCallback(const rigid_transform_computation::Pixel::ConstPtr& msg) {
    cv::Point2f tempPixelPoint;

    tempPixelPoint.x = msg->x;
    tempPixelPoint.y = msg->y;


    imagePixels.push_back(tempPixelPoint);

    std::cout << "Added Pixel: " << tempPixelPoint << std::endl;
}

void pointCloudCallback(const rigid_transform_computation::PointXYZI::ConstPtr& msg) {
    cv::Point3f temp3DPoint;

    temp3DPoint.x = msg->x;
    temp3DPoint.y = msg->y;
    temp3DPoint.z = msg->z;

    pointCloudPoints.push_back(transformLiDARToCameraReferential(temp3DPoint));

    std::cout << "Added Point3D: " << temp3DPoint << std::endl;
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    image_geometry::PinholeCameraModel tempCameraModel;

    tempCameraModel.fromCameraInfo(cam_info);
    camera_info.push_back(tempCameraModel);

    //std::cout << tempCameraModel.fullIntrinsicMatrix() << std::endl;
    //std::cout << "Added Camera Info " << std::endl;
    //std::cout << tempCameraModel.distortionCoeffs() << std::endl;

    if ( (pointCloudPoints.size() >= 5) && (imagePixels.size() >= 5) )
    {
        std::cout << "Solving PnP" << std::endl;
        computeRigidTransform();
    }
}


int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "Image_Visualizer_Node");
    std::cout << "Node init succesfully" << std::endl;


    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe<rigid_transform_computation::Pixel>            ("camera/clicked_pixel", 1, pixelCallback);
    ros::Subscriber point_cloud_sub = nh.subscribe<rigid_transform_computation::PointXYZI>  ("velo/picked_point", 1, pointCloudCallback);
    ros::Subscriber camera_info_sub = nh.subscribe<sensor_msgs::CameraInfo>                 ("camera/camera_info", 1, cameraInfoCallback);
    /*
    if (argc == 2) { //argv is the first argumwent, file parh for camera calbiration is the second
    /* http://library.isr.ist.utl.pt/docs/roswiki/yaml_cpp.html
    https://ossyaritoori.hatenablog.com/entry/2017/08/16/Read_yaml_file_with_yaml-cpp_in_ROS_C%2B%2B_/_
    https://github.com/jbeder/yaml-cpp/wiki/Tutorial
    https://stackoverflow.com/questions/50150387/parsing-yaml-file-using-yaml-cpp-error

    // /
        std::cout << "Loading Calibration file" << argv[1] << std::endl;

        YAML::Node camera_info_yaml = YAML::LoadFile(argv[1]);
        std::cout << "Open" << std::endl;
        sensor_msgs::CameraInfo camera_info_msg;

        std::vector<double> K, R, P;
        camera_info_msg.width = camera_info_yaml["image_width"].as<uint32_t>();
        std::cout << "width" << std::endl;
        camera_info_msg.height = camera_info_yaml["image_height"].as<uint32_t>();
        std::cout << "hiegth" << std::endl;

        camera_info_msg.distortion_model = camera_info_yaml["distortion_model"].as<std::string>();
        std::cout << "D model" << std::endl;
        //camera_info_msg.D = camera_info_yaml["distortion_coefficients"].as<std::vector<double>>();
        std::cout << "Model" << std::endl;

        K = camera_info_yaml["camera_matrix"].as<std::vector<float>>();
        std::cout << "K" << std::endl;
        R = camera_info_yaml["rectification_matrix"].as<std::vector<double>>();
        std::cout << "R" << std::endl;
        P = camera_info_yaml["projection_matrix"].as<std::vector<double>>();
        std::cout << "P" << std::endl;

        // conversion
        // Coversion from std::vector<double> to boost::array<double> is not implicit supported on K, R and P matrices
        // because they have a fixed number of elements
        boost::array<double, 9ul> kl;
        std::memcpy(&kl[0], &K[0], sizeof(double)*9);
        boost::array<double, 9ul> rl;
        std::memcpy(&rl[0], &R[0], sizeof(double)*9);
        boost::array<double, 12ul> pl;
        std::memcpy(&pl[0], &P[0], sizeof(double)*12);
        std::cout << "conversion" << std::endl;
        // put value
        camera_info_msg.K =  kl;
        camera_info_msg.R =  rl;
        camera_info_msg.P =  pl;
        std::cout << "passed to vector" << std::endl;
    }
*/








    ros::spin();
    return EXIT_SUCCESS;
}
