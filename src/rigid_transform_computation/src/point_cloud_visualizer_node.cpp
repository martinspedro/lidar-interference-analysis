#include <iostream>
#include "rigid_transform_computation/pointCloudVisualizer.hpp"

int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "Point_Cloud_Visualizer_Node");
    std::cout << "Node init succesfully" << std::endl;


    point_cloud::PointCloudVisualizer point_cloud_visualizer_object("/velodyne_points", "pcl_viewer_pose", "velo");
    point_cloud_visualizer_object.registerPointPickingCallback(point_cloud::PointCloudVisualizer::SINGLE_POINT_MODE);



    // Use Wall CLock as time source instead of /clock (even using use_simulated_time = True)
    ros::WallRate wallTimer(100); // Wake-up at every 100 Hz.

    //ros::spin();

    while (ros::ok()) {
        //std::cout << "Spinning..." << std::endl;
        point_cloud_visualizer_object.spin();
        //std::cout << "Going to sleep..." << std::endl;
        wallTimer.sleep();
        //std::cout << "Stopped Sleeping..." << std::endl;
    }

    std::cout << "Node shutdown..." << std::endl;
    return EXIT_SUCCESS;
}
