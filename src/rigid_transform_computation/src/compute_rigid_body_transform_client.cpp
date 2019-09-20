/*!
 *  \file   compute_rigid_body_transform_client.cpp
 *  \brief  Implementation of a service client node to request the computation of the rigid body transform between the
 LiDAR and the monocular camera
 *  \author Pedro Martins (martinspedro@av.it.pt)
 *
 *  \remark Requires that the solvePnP algorithm to given as argument
 *  \parblock
 *   Available methods:
 *  - 0: SOLVEPNP_ITERATIVE
 *  - 1: SOLVEPNP_P3P
 *  - 2: SOLVEPNP_EPNP
 *  - 3: SOLVEPNP_DLS
 *  - 4: SOLVEPNP_UPNP
 *  \endparblock
 */

#include "ros/ros.h"
#include "rigid_transform_computation/compute_rigid_body_transform.h"
#include <cstdlib>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "compute_rigid_body_transform_client");

  // Client must be called by giving the solvePnP method
  if (argc != 2)
  {
    ROS_INFO("Usage: rigidBodyTransform_client solvePnpType");
    return EXIT_FAILURE;
  }

  ros::NodeHandle nh;
  // clang-format off
  ros::ServiceClient client
        = nh.serviceClient<rigid_transform_computation::compute_rigid_body_transform>("compute_rigid_body_transform");
  // clang-format on

  rigid_transform_computation::compute_rigid_body_transform srv;
  srv.request.solve_PnP_algorithm = atoll(argv[1]);

  if (client.call(srv))
  {
    std::cout << srv.response.rigid_body_transform << std::endl;
  }
  else
  {
    ROS_ERROR("Failed to call service computeRigidBodyTransform");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
