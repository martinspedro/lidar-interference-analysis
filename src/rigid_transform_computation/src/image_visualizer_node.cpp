#include <iostream>
#include "rigid_transform_computation/imageVisualizer.hpp"

int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "Image_Visualizer_Node");
    std::cout << "Node init succesfully" << std::endl;


    ImageVisualizer image_visualizer_object("/camera", "clicked_point", "img_pixel_picker");
    image_visualizer_object.registerPixelPickingCallback();

    //ros::Time timer;

    // Use Wall CLock as time source instead of /clock (even using use_simulated_time = True)
    ros::WallRate wallTimer(100); // Wake-up at every 100 Hz.

    //ros::spin();

    //ros::Rate r(10); // Spin at 100 Hz
    while (ros::ok()) {
        //std::cout << "Spinning..." << std::endl;
        image_visualizer_object.spin();
        //ros::spinOnce();
        //std::cout << "Going to sleep..." << std::endl;
        //r.sleep();
        wallTimer.sleep();
        //std::cout << "Stopped Sleeping..." << std::endl;
    }

    //std::cout << "Node shutdown..." << std::endl;
    return EXIT_SUCCESS;
}
