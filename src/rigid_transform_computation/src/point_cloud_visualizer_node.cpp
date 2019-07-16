#include <iostream>
#include "rigid_transform_computation/pointCloudVisualizer.hpp"

int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "Point_Cloud_Visualizer_Node");
    std::cout << "Node init succesfully" << std::endl;


    point_cloud::PointCloudVisualizer point_cloud_visualizer_object("/velodyne_points", "pcl_viewer_pose", "velo");
    point_cloud_visualizer_object.registerPointPickingCallback(point_cloud::PointCloudVisualizer::SINGLE_POINT_MODE);


    ros::spin();

    return EXIT_SUCCESS;
}
