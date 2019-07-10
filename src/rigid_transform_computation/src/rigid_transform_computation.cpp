#include <iostream>
#include "rigid_transform_computation/imageVisualizer.hpp"


int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "Image_Visualizer_Node");

    ImageVisualizer image_visualizer_object;

    ros::spin();
    return EXIT_SUCCESS;
}
