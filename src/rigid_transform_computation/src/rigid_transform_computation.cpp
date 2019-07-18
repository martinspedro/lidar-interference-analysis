#include <iostream>
#include "rigid_transform_computation/imageVisualizer.hpp"
#include "rigid_transform_computation/pointCloudVisualizer.hpp"


#include <image_geometry/pinhole_camera_model.h>

#include "rigid_transform_computation/Pixel.h"
#include "rigid_transform_computation/PointXYZI.h"

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

void computeRigidTransform(){
    //solvePnP(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess=false, int flags=ITERATIVE )¶
    cv::solvePnP(pointCloudPoints, imagePixels, camera_info[10].fullIntrinsicMatrix(), camera_info[10].distortionCoeffs(), rVec, tVec, false, cv::SOLVEPNP_ITERATIVE);
    std::cout << rVec << std::endl;
    std::cout << tVec << std::endl;

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

    pointCloudPoints.push_back(temp3DPoint);

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



    ros::spin();
    return EXIT_SUCCESS;
}
