#include <iostream>
#include "rigid_transform_computation/imageVisualizer.hpp"
#include "rigid_transform_computation/pointCloudVisualizer.hpp"


int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "Image_Visualizer_Node");
    std::cout << "Node init succesfully" << std::endl;


    //ImageVisualizer image_visualizer_object;
    point_cloud::PointCloudVisualizer point_cloud_visualizer_object("/velodyne_points", "velo");

    ros::spin();
    return EXIT_SUCCESS;
}
