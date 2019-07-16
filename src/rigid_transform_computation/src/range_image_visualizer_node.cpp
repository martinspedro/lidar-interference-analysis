#include <iostream>
#include "rigid_transform_computation/rangeImageVisualizer.hpp"

int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "Point_Cloud_Visualizer_Node");
    std::cout << "Node init succesfully" << std::endl;


    RangeImageVisualizer range_image_visualizer_object("/velodyne_points", "/pose", "velo");


    ros::spin();

    return EXIT_SUCCESS;
}
