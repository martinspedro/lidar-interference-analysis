#include <iostream>
#include "rigid_transform_computation/imageVisualizer.hpp"

int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "Image_Visualizer_Node");
    std::cout << "Node init succesfully" << std::endl;


    ImageVisualizer image_visualizer_object("/camera", "clicked_point", "img_pixel_picker");
    image_visualizer_object.registerPixelPickingCallback();


    ros::spin();

    return EXIT_SUCCESS;
}
