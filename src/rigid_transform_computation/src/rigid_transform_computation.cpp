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

std::vector<PointI> pointCloudPoints;
std::vector<Pixel_t> imagePixels;
std::vector<image_geometry::PinholeCameraModel> camera_info;

void pixelCallback(const rigid_transform_computation::Pixel::ConstPtr& msg) {
    Pixel_t tempPixel;

    tempPixel.x = msg->x;
    tempPixel.y = msg->y;
    tempPixel.r = msg->r;
    tempPixel.g = msg->g;
    tempPixel.b = msg->b;

    imagePixels.push_back(tempPixel);

    std::cout << "Added Pixel: " << msg << std::endl;
}

void pointCloudCallback(const rigid_transform_computation::PointXYZI::ConstPtr& msg) {
    PointI tempPoint;

    tempPoint.x = msg->x;
    tempPoint.y = msg->y;
    tempPoint.z = msg->z;
    tempPoint.intensity = msg->intensity;

    pointCloudPoints.push_back(tempPoint);

    std::cout << "Added Point3D: " << msg << std::endl;
}

void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info) {
    image_geometry::PinholeCameraModel tempCameraModel;

    tempCameraModel.fromCameraInfo(cam_info);
    camera_info.push_back(tempCameraModel);

    std::cout << "Added Camera Info: " << cam_info << std::endl;
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
