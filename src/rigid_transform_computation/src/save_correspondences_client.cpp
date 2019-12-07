/*!
 *  \file   save_correspondences_client.cpp
 *  \brief  Implementation of a service client node to request that the registed 2D <-> 3D correspondences be saved on a
 * CSV file
 *  \author Pedro Martins (martinspedro@av.it.pt)
 *
 */

#include "ros/ros.h"
#include "rigid_transform_computation/save_correspondences.h"

#include <cstdlib>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "save_correspondences_client");

  // Client must be called by giving the output file name
  if (argc != 2)
  {
    ROS_INFO("Usage: save_corrrespondences file name");
    return EXIT_FAILURE;
  }

  ros::NodeHandle nh;
  // clang-format off
  ros::ServiceClient client =
        nh.serviceClient<rigid_transform_computation::save_correspondences>("save_correspondences");
  // clang-format off

  rigid_transform_computation::save_correspondences srv;
  srv.request.filename = argv[1];

  if (client.call(srv))
  {
    // service can be succesfully called but may not be able to save the file
    if (srv.response.success)
    {
      ROS_INFO("%s", srv.response.message.c_str());
    }
    else
    {
      ROS_ERROR("%s", srv.response.message.c_str());
    }
  }
  else
  {
    ROS_ERROR("Failed to call service save_correspondences");
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
