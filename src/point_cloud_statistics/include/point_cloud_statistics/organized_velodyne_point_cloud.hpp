/**
 * \file   organized_velodyne_point_cloud.hpp
 * \brief Organized Point cloud Class to store Velodyne data
 *
 * \author Pedro Martins (martinspedro@ua.pt)
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
/*!
 * \class OrganizedVelodynePointCloud
 * \brief Organized Point Cloud that inherits from OrganizedPointCloud and narrows it down to Velodyne Point Clouds
 * cloud
 *
 */
class OrganizedVelodynePointCloud : public OrganizedPointCloud<velodyne::PointXYZIR>
{
public:
  OrganizedVelodynePointCloud(unsigned int width, unsigned int height)
    : point_cloud::organized::OrganizedPointCloud<velodyne::PointXYZIR>(width, height)
  {
    ROS_DEBUG_NAMED("call_stack", "[OrganizedVelodynePointCloud] with (%d, %d)", width, height);
  }

  void organizePointCloud(velodyne::VelodynePointCloud unorganized_cloud)
  {
    ROS_DEBUG_NAMED("call_stack", "[organizeVelodynePointCloud]");
    this->header = unorganized_cloud.header;  // use sequence number, stamp and frame_id from the unorganized cloud
    this->is_dense = false;                   // Organized Point Cloud is not dense

    for (int i = 0; i < unorganized_cloud.size(); ++i)
    {
      unsigned int azimuth_index = getAzimuthIndex(unorganized_cloud.points[i]);

      ROS_ASSERT_MSG(azimuth_index < this->width, "Azimuth index %d vs size %d", azimuth_index, this->width);

      // If the ring value of the point is invalid (normally caused by PCL algorithms), compute ring value from Polar
      // Angle
      if (unorganized_cloud.points[i].ring == velodyne::DEFAULT_RING_VALUE)
      {
        unorganized_cloud.points[i].ring = computeLaserID(unorganized_cloud.points[i]);
      }

      if (this->at(azimuth_index, unorganized_cloud.points[i].ring).ring == velodyne::DEFAULT_RING_VALUE)
      {
        this->at(azimuth_index, unorganized_cloud.points[i].ring) = unorganized_cloud.points[i];
      }
      else
      {
        /* \TODO fix the point overload otherwise it replaces a lot of points
      ROS_WARN("Point would erase previous point! (%f, %f, %f, %f, %d) vs (%f, %f, %f, %f, %d) and azimuth_index: "
               "%d. Point not added!",
               this->at(azimuth_index, unorganized_cloud.points[i].ring).x,
               this->at(azimuth_index, unorganized_cloud.points[i].ring).y,
               this->at(azimuth_index, unorganized_cloud.points[i].ring).z,
               this->at(azimuth_index, unorganized_cloud.points[i].ring).intensity,
               this->at(azimuth_index, unorganized_cloud.points[i].ring).ring, unorganized_cloud.points[i].x,
               unorganized_cloud.points[i].y, unorganized_cloud.points[i].z, unorganized_cloud.points[i].intensity,
               unorganized_cloud.points[i].ring, azimuth_index);
               */
      }
    }
  }

  // \todo This two functions are repeated from organized_point_cloud_with_container code and should be fixed
protected:
  /*!
   * \brief Compute polar angle
   * \param[in] point velodyne point cloud point for computing the polar angle
   * \return the polar angle, in degrees
   */
  float getPolar(const velodyne::PointXYZIR point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getPolar]");
    float r = computeEuclideanDistanceToOrigin(point);
    return asin(point.z / r) * point_cloud::organized::RADIAN_TO_DEGREE_F;
  }

  /*!
   * \brief Computes Laser ID of the organized Point Cloud Matrix
   * \param[in] point velodyne point cloud point for computing the polar angle
   * \return the corresponding laser ID
   *
   * Computes the the polar angle for a given point, shift it to be defined between [0, max(Laser ID)[ and then
   * computes the corresponding Laser ID
   */
  unsigned int computeLaserID(const velodyne::PointXYZIR point)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeLaserID]");
    float polar_angle = getPolar(point);
    float rounded_polar_angle = roundf(polar_angle);
    unsigned int index = velodyne::vlp16::getLaserIDfromPolarAngle((unsigned int)(rounded_polar_angle));

    return index;
  }
};

}  // namespace organized

}  // namespace point_cloud

#endif  // ORGANIZED_VELODYNE_POINT_CLOUD_H
