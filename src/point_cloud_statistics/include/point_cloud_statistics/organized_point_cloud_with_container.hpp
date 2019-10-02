/**
 * \file   organized_pointcloud.hpp
 * \brief
 *
 */

#ifndef ORGANIZED_POINT_CLOUD_WITH_CONTAINER_H
#define ORGANIZED_POINT_CLOUD_WITH_CONTAINER_H

#include <pcl/point_cloud.h>
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"
#include "point_cloud_statistics/organized_pointcloud.hpp"
#include "point_cloud_statistics/organized_velodyne_point_cloud.hpp"

#include "point_cloud_statistics/velodyne_point_type.h"
#include <ros/ros.h>
#include <ros/console.h>

namespace point_cloud
{
namespace organized
{
template <class ContainerT, class DataT>
class OrganizedPointCloudWithContainer : public OrganizedPointCloud<ContainerT>
{
public:
  OrganizedPointCloudWithContainer(unsigned int width, unsigned int height)
    : point_cloud::organized::OrganizedPointCloud<ContainerT>(width, height)
  {
    ROS_DEBUG_NAMED("call_stack", "[OrganizedPointCloudWithContainer] with (%d, %d)", width, height);
  }

  void registerVelodynePointCloud(pcl::PointCloud<DataT> unorganized_cloud)
  {
    ROS_DEBUG_NAMED("call_stack", "[registerVelodynePointCloud]");

    for (int i = 0; i < unorganized_cloud.size(); ++i)
    {
      unsigned int azimuth_index = getAzimuthIndex(unorganized_cloud.points[i]);

      ROS_ASSERT_MSG(azimuth_index < this->width, "Azimuth index %d vs size %d", azimuth_index, this->width);

      // push_back point to data_points vector
      this->at(azimuth_index, unorganized_cloud.points[i].ring).data_points.push_back(unorganized_cloud.points[i]);
    }
  }

  template <typename IntensityT>
  void computeStats(std::vector<IntensityT>& azimuth, std::vector<IntensityT>& laser)
  {
    ROS_DEBUG_NAMED("call_stack", "[registerVelodynePointCloud]");

    for (int i = 0; i < this->height; ++i)
    {
      for (int j = 0; j < this->width; ++j)
      {
        for (int k = 0; k < this->at(j, i).data_points.size(); ++k)
        {
          this->at(j, i).x += this->at(j, i).data_points[k].x;
          this->at(j, i).y += this->at(j, i).data_points[k].y;
          this->at(j, i).z += this->at(j, i).data_points[k].z;

          this->at(j, i).euclidean_distance.push_back(computeEuclideanDistanceToOrigin(this->at(j, i).data_points[k]));
          this->at(j, i).distance_mean += this->at(j, i).euclidean_distance[k];
          this->at(j, i).distance_var += this->at(j, i).euclidean_distance[k] * this->at(j, i).euclidean_distance[k];
          this->at(j, i).intensity_var +=
              this->at(j, i).data_points[k].intensity * this->at(j, i).data_points[k].intensity;
          this->at(j, i).intensity_mean += this->at(j, i).data_points[k].intensity;
        }
        if (this->at(j, i).data_points.size() > 0)
        {
          this->at(j, i).x /= (float)(this->at(j, i).data_points.size());
          this->at(j, i).y /= (float)(this->at(j, i).data_points.size());
          this->at(j, i).z /= (float)(this->at(j, i).data_points.size());

          this->at(j, i).ring = this->at(j, i).data_points[0].ring;

          this->at(j, i).distance_var += -(this->at(j, i).distance_mean * this->at(j, i).distance_mean) /
                                         (float)(this->at(j, i).data_points.size());
          this->at(j, i).intensity_var += -(this->at(j, i).intensity_mean * this->at(j, i).intensity_mean) /
                                          (float)(this->at(j, i).data_points.size());

          this->at(j, i).distance_mean /= (float)(this->at(j, i).data_points.size());
          this->at(j, i).intensity_mean /= (float)(this->at(j, i).data_points.size());
        }

        laser[i].mean += this->at(j, i).intensity_mean;

        azimuth[j].mean += (this->at(j, i).intensity_mean / (float)(this->height));
      }
      laser[i].mean /= (float)(this->width);
    }
  }

  void generateModel(OrganizedPointCloud<DataT> point_cloud)
  {
    ROS_DEBUG_NAMED("call_stack", "[generateModel]");

    for (int i = 0; i < this->height; ++i)
    {
      for (int j = 0; j < this->width; ++j)
      {
        point_cloud.at(j, i).x = this->at(j, i).x;
        point_cloud.at(j, i).y = this->at(j, i).y;
        point_cloud.at(j, i).z = this->at(j, i).z;
        point_cloud.at(j, i).ring = this->at(j, i).ring;
        point_cloud.at(j, i).intensity = this->at(j, i).intensity_mean;
      }
    }
  }

  void organizePointCloud(const pcl::PointCloud<DataT> unorganized_cloud)
  {
    // This function is not implemented and is only here to provide an overload to the class this subclass inherits
  }

protected:
  float getAzimuth(const DataT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getAzimuth]");
    return atan2(point.y, point.x) * point_cloud::organized::RADIAN_TO_DEGREE_F;
  }

  unsigned int getAzimuthIndex(const DataT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getAzimuthIndex]");
    float shifted_azimuth = getAzimuth(point) + point_cloud::organized::DEGREE_OFFSET_TO_POSITIVE_ANGLE_F;
    float fixed_point_shifted_azimuth = floor(shifted_azimuth * 10.0f) / 10.0f;
    float index =
        fixed_point_shifted_azimuth * (float)(this->width - 1) / point_cloud::organized::FULL_REVOLUTION_DEGREE_F;

    return (unsigned int)(index);
  }

  float computeEuclideanDistanceToOrigin(const DataT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeEuclideanDistanceToOrigin]");
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  }
};

}  // namespace organized

}  // namespace point_cloud

#endif  // ORGANIZED_POINT_CLOUD_WITH_CONTAINER_H
