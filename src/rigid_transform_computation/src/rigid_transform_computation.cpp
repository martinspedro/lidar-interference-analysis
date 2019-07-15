#include <iostream>
#include "rigid_transform_computation/imageVisualizer.hpp"
#include "rigid_transform_computation/pointCloudVisualizer.hpp"
#include "rigid_transform_computation/rangeImageVisualizer.hpp"

int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "Image_Visualizer_Node");
    std::cout << "Node init succesfully" << std::endl;


    //ImageVisualizer image_visualizer_object;
    RangeImageVisualizer range_image_visualizer_object("/velodyne_points", "velo");
    //point_cloud_visualizer_object.registerPointPickingCallback(point_cloud::PointCloudVisualizer::SINGLE_POINT_MODE);

    ros::spin();
    return EXIT_SUCCESS;
}
