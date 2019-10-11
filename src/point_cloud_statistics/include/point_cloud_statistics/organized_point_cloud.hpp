/**
 * \file   organized_pointcloud.hpp
 * \brief
 *
 */

#ifndef ORGANIZED_POINT_CLOUD_H
#define ORGANIZED_POINT_CLOUD_H

#include <pcl/point_cloud.h>
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"
#include <ros/ros.h>
#include <ros/console.h>

#include <boost/shared_ptr.hpp>

namespace point_cloud
{
namespace organized
{
template <class PointT>
class OrganizedPointCloud : public pcl::PointCloud<PointT>
{
public:
  using Ptr = boost::shared_ptr<OrganizedPointCloud>;
  using ConstPtr = boost::shared_ptr<const OrganizedPointCloud>;

  OrganizedPointCloud(unsigned int width, unsigned int height) : pcl::PointCloud<PointT>(width, height)
  {
    ROS_DEBUG_NAMED("call_stack", "[OrganizedPointCloud] with (%d, %d)", width, height);
  }

  void clearPointsFromPointcloud()
  {
    ROS_DEBUG_NAMED("call_stack", "[clearPointsFromPointcloud]");

    for (int i = 0; i < this->width; ++i)
    {
      for (int j = 0; j < this->height; ++j)
      {
        this->at(i, j) = PointT();
      }
    }
  }

  virtual float getAzimuth(const PointT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getAzimuth]");
    return atan2(point.y, point.x) * point_cloud::organized::RADIAN_TO_DEGREE_F;
  }

  virtual unsigned int getAzimuthIndex(const PointT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getAzimuthIndex]");
    float shifted_azimuth =
        OrganizedPointCloud<PointT>::getAzimuth(point) + point_cloud::organized::DEGREE_OFFSET_TO_POSITIVE_ANGLE_F;
    float fixed_point_shifted_azimuth = floor(shifted_azimuth * 10.0f) / 10.0f;

    // Index calculcation uses subtracts one to vector width because azimuth interval is closed  [-180, 180]
    float index =
        fixed_point_shifted_azimuth * (float)(this->width - 1) / point_cloud::organized::FULL_REVOLUTION_DEGREE_F;

    return (unsigned int)(index);
  }

  virtual float computeEuclideanDistanceToOrigin(const PointT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeEuclideanDistanceToOrigin]");
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  }

  virtual float computeEuclideanDistance(const PointT point1, const PointT point2)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeEuclideanDistance]");
    ROS_DEBUG("P1->(%f, %f, %f), P2->(%f, %f, %f)", point1.x, point1.y, point1.z, point2.x, point2.y, point2.z);
    float x_diff = point2.x - point1.x;
    float y_diff = point2.y - point1.y;
    float z_diff = point2.z - point1.z;

    return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
  }

  virtual void organizePointCloud(const pcl::PointCloud<PointT> unorganized_cloud)
  {
    ROS_DEBUG_NAMED("call_stack", "[organizePointCloud]");
    this->header = unorganized_cloud.header;  // use sequence number, stamp and frame_id from the unorganized cloud
    this->is_dense = false;                   // Organized Point Cloud is not dense

    for (int i = 0; i < unorganized_cloud.size(); ++i)
    {
      unsigned int azimuth_index = getAzimuthIndex(unorganized_cloud.points[i]);

      ROS_ASSERT_MSG(azimuth_index < this->width, "Azimuth index %d vs size %d", azimuth_index, this->width);

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

  OrganizedPointCloud<PointT> computeDistanceBetweenPointClouds(OrganizedPointCloud<PointT> organized_point_cloud)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeDistanceBetweenPointClouds]");

    ROS_ASSERT_MSG((this->width == organized_point_cloud.width) && (this->height == organized_point_cloud.height),
                   "Point cloud dimensions disagree: (%i, %i) vs (%i, %i)", this->width, this->height,
                   organized_point_cloud.width, organized_point_cloud.height);

    OrganizedPointCloud<PointT> distance_map(this->width, this->height);

    for (int i = 0; i < this->width; ++i)
    {
      for (int j = 0; j < this->height; ++j)
      {
        distance_map.at(i, j) = computeEuclideanDistance(this->at(i, j), organized_point_cloud.at(i, j));
      }
    }

    return distance_map;
  }

  /**
   * \remark Despite the output being a vector, the distance and intensity calculations are pushed first by laser/ring
   * and then by azimuth. The output vector can be resized to a 1800 x 16 matrix, where eacha column represents a
   * laser/ring and each row a azimuth angle. If the vector accumulates results for consecutive frames, the number of
   * rows is N * 1800, where N is the number of frames. Therefore, every 1800th row a new frame starts
   */
  void computeDistanceBetweenPointClouds(OrganizedPointCloud<PointT> organized_point_cloud,
                                         std::vector<double>& distance_vector, std::vector<double>& intensity_vector)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeDistanceBetweenPointClouds] (%i, %i) vs (%i, %i)", this->width, this->height,
                    organized_point_cloud.width, organized_point_cloud.height);
    ROS_ASSERT_MSG((this->width == organized_point_cloud.width) && (this->height == organized_point_cloud.height),
                   "Point cloud dimensions disagree: (%i, %i) vs (%i, %i)", this->width, this->height,
                   organized_point_cloud.width, organized_point_cloud.height);

    for (int i = 0; i < this->width; ++i)
    {
      for (int j = 0; j < this->height; ++j)
      {
        double euclidean_distance = (double)(computeEuclideanDistance(this->at(i, j), organized_point_cloud.at(i, j)));
        ROS_DEBUG("euclidean_distance: %f", euclidean_distance);
        distance_vector.push_back(euclidean_distance);
        intensity_vector.push_back((double)(this->at(i, j).intensity - organized_point_cloud.at(i, j).intensity));
      }
    }
  }
};

}  // namespace organized

}  // namespace point_cloud

#endif  // ORGANIZED_POINT_CLOUD_H
