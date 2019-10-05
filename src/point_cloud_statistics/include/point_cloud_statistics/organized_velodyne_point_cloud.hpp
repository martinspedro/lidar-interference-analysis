/**
 * \file   organized_pointcloud.hpp
 * \brief
 *
 */

#ifndef ORGANIZED_VELODYNE_POINT_CLOUD_H
#define ORGANIZED_VELODYNE_POINT_CLOUD_H

#include <pcl/point_cloud.h>
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"
#include "point_cloud_statistics/organized_point_cloud.hpp"
#include "point_cloud_statistics/velodyne_point_type.h"

#include <ros/ros.h>
#include <ros/console.h>

namespace point_cloud
{
namespace organized
{
class OrganizedVelodynePointCloud : public OrganizedPointCloud<velodyne::PointXYZIR>
{
public:
  OrganizedVelodynePointCloud(unsigned int width, unsigned int height)
    : point_cloud::organized::OrganizedPointCloud<velodyne::PointXYZIR>(width, height)
  {
    ROS_DEBUG_NAMED("call_stack", "[OrganizedVelodynePointCloud] with (%d, %d)", width, height);
  }

  void organizePointCloud(const velodyne::VelodynePointCloud unorganized_cloud)
  {
    ROS_DEBUG_NAMED("call_stack", "[organizeVelodynePointCloud]");
    this->header = unorganized_cloud.header;  // use sequence number, stamp and frame_id from the unorganized cloud
    this->is_dense = false;                   // Organized Point Cloud is not dense

    for (int i = 0; i < unorganized_cloud.size(); ++i)
    {
      unsigned int azimuth_index = getAzimuthIndex(unorganized_cloud.points[i]);

      ROS_ASSERT_MSG(azimuth_index < this->width, "Azimuth index %d vs size %d", azimuth_index, this->width);

      if (this->at(azimuth_index, unorganized_cloud.points[i].ring).ring != velodyne::DEFAULT_RING_VALUE)
      {
        this->at(azimuth_index, unorganized_cloud.points[i].ring) = unorganized_cloud.points[i];
      }
      else
      {
        ROS_WARN("Point would erase previous point! (%f, %f, %d) vs (%f, %f, %d) and azimuth_index: %d. Point not "
                 "added!",
                 this->at(azimuth_index, unorganized_cloud.points[i].ring).x,
                 this->at(azimuth_index, unorganized_cloud.points[i].ring).y,
                 this->at(azimuth_index, unorganized_cloud.points[i].ring).ring, unorganized_cloud.points[i].x,
                 unorganized_cloud.points[i].y, unorganized_cloud.points[i].ring, azimuth_index);
      }
    }
  }
};

}  // namespace organized

}  // namespace point_cloud

#endif  // ORGANIZED_VELODYNE_POINT_CLOUD_H
