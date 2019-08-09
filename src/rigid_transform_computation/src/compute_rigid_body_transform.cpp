#include <iostream>
#include <sstream>

#include "rigid_transform_computation/rigid_body_transform.hpp"
#include "rigid_transform_computation/CSV_manager.hpp"



using namespace rigid_body_transform;



int main(int argc, char* argv[])
{
    //Initialize ROS
    ros::init(argc, argv, "rigid_transform_computation");
    std::cout << "ROS Node init succesfully" << std::endl;

    if (argc != 4) {
      ROS_INFO("Usage: rosrun rigid_transform_computation compute_rigid_body_transform csv_filename camera_info_yaml_filename solvePnP_mode");
      return EXIT_FAILURE;
    }


    std::vector<cv::Point3f> point_cloud_points;
    std::vector<cv::Point2f> image_pixels;

    std::cout << "Loading 2D <-> 3D correspondences for calibration... ";
    csv_file_manager::read(THESIS_FILE_PATH + argv[1], &image_pixels, &point_cloud_points);
    std::cout << "Done!" << std::endl << "Creating LiDAR & Camera Calibration Object... ";
    LiDARCameraCalibrationData calibration_data_object("velo_link", "camera_link", point_cloud_points, image_pixels);
    std::cout << "Done!" << std::endl << "Loading Camera Info YAML file... ";
    calibration_data_object.readCameraInfoFromYAML(argv[2]);
    std::cout << "Done!" << std::endl << "Computing Rigid Body Transform... ";
    calibration_data_object.computeRigidBodyTransform(atoll(argv[3]));
    std::cout << "Done! " << std::endl << "Create Static Transform Message to be published... ";
    calibration_data_object.publishStaticTransformMsg();
    std::cout << "Done! " << std::endl << "Static Transform is going to be published under /tf_static. " << std::endl;

    ros::Rate r(1);
    while(ros::ok())
    {
        ros::spinOnce();    // Spin even if no callbacks or services exist
        r.sleep();          // loop at 1 Hz
    }

    return EXIT_SUCCESS;
}
