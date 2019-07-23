#include "ros/ros.h"
#include "rigid_transform_computation/save_correspondences.h"
#include <cstdlib>
#include <string>

int main(int argc, char **argv) {
  ros::init(argc, argv, "save_correspondences_client");
  if (argc != 2) {
    ROS_INFO("Usage: save_corrrespondences filename");
    return EXIT_FAILURE;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<rigid_transform_computation::save_correspondences>("save_correspondences");
  rigid_transform_computation::save_correspondences srv;
  srv.request.filename = argv[1];

  if (client.call(srv)) {
      if(srv.response.success) {
          std::cout << srv.response.message << std::endl;
      }
      else {
          ROS_ERROR("%s", srv.response.message.c_str());
      }

  }
  else {
      ROS_ERROR("Failed to call service save_correspondences");
      return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
