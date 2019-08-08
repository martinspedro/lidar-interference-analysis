#include "ros/ros.h"
#include "rigid_transform_computation/computeRigidBodyTransform.h"
#include <cstdlib>

int main(int argc, char **argv) {
  ros::init(argc, argv, "computeRigidBodyTransform_client");
  if (argc != 2)
  {
    ROS_INFO("Usage: rigidBodyTransform_client solvePnpType");
    return EXIT_FAILURE;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<rigid_transform_computation::computeRigidBodyTransform>("computeRigidBodyTransform");
  rigid_transform_computation::computeRigidBodyTransform srv;

  srv.request.solvePnpType = atoll(argv[1]);

  if (client.call(srv)) {
    std::cout << srv.response.rigidBodyTransform << std::endl;
  }
  else {
    ROS_ERROR("Failed to call service computeRigidBodyTransform");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
