/*!
 * \file  organized_point_cloud_with_container.hpp
 * \brief Organized Point cloud where each of its elements is a Container
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

#ifndef ORGANIZED_POINT_CLOUD_WITH_CONTAINER_H
#define ORGANIZED_POINT_CLOUD_WITH_CONTAINER_H

#include <pcl/point_cloud.h>
#include "point_cloud_statistics/organized_point_cloud_utilities.hpp"
#include "point_cloud_statistics/organized_point_cloud.hpp"
#include "point_cloud_statistics/organized_velodyne_point_cloud.hpp"

#include "point_cloud_statistics/velodyne_point_type.h"
#include <ros/ros.h>
#include <ros/console.h>
#include <boost/shared_ptr.hpp>

#include <cmath>

namespace point_cloud
{
namespace organized
{
/*!
 * \class OrganizedPointCloudWithContainer
 * \brief Organized Point Cloud with a Data Container for every index of the organized Data
 * \tparam ContainerT Template class for the Data Container
 * \tparam DataT Template class for each element of the Data Container element vector
 *
 * This class inherits from OrganizedPointCloud and makes each of the elements of the point cloud as a Templated
 * Container to store the data
 *
 * \see OrganizedPointCloud
 */
template <class ContainerT, class DataT>
class OrganizedPointCloudWithContainer : public OrganizedPointCloud<ContainerT>
{
public:
  using Ptr = boost::shared_ptr<OrganizedPointCloudWithContainer<ContainerT, DataT> >;
  using ConstPtr = boost::shared_ptr<const OrganizedPointCloudWithContainer<ContainerT, DataT> >;

  /*!
   * \brief Constructor
   * \param[in] width organized point cloud matrix number of columns
   * \param[in] height organized point cloud matrix number of rows
   *
   * Call the OrganizedPointCloud constructor
   */
  OrganizedPointCloudWithContainer(unsigned int width, unsigned int height)
    : point_cloud::organized::OrganizedPointCloud<ContainerT>(width, height)
  {
    ROS_DEBUG_NAMED("call_stack", "[OrganizedPointCloudWithContainer] with (%d, %d)", width, height);
  }

  /*!
   * \brief Appends the point cloud to the organized point cloud, in an organized manner
   * \param[in] unorganized_cloud unorganized point cloud to be stored
   * \tparam DataT Template class for each element of the Data Container element vector
   *
   * Iterates over the input point cloud, unorganized, and appends the current point to the Container vector on its
   * correspondent row and column
   *
   * \todo remove dependency from velodyne::DEFAULT_RING_VALUE
   */
  void registerPointCloud(pcl::PointCloud<DataT> unorganized_cloud)
  {
    ROS_DEBUG_NAMED("call_stack", "[registerPointCloud]");

    for (int i = 0; i < unorganized_cloud.size(); ++i)
    {
      unsigned int azimuth_index = getAzimuthIndex(unorganized_cloud.points[i]);
      ROS_ASSERT_MSG(azimuth_index < this->width, "Azimuth index %d vs size %d", azimuth_index, this->width);

      // push_back point to data_points vector
      if (unorganized_cloud.points[i].ring == velodyne::DEFAULT_RING_VALUE)
      {
        unorganized_cloud.points[i].ring = computeLaserID(unorganized_cloud.points[i]);
      }

      this->at(azimuth_index, unorganized_cloud.points[i].ring).data_points.push_back(unorganized_cloud.points[i]);
    }
  }

  /*!
   * \brief Process the organized point cloud containers and outputs Intensity Statistics
   * \param[out] azimuth vector containing the statistisc for each azimuthal angle
   * \param[out] laser vector containing the statistisc for each laser ID
   * \tparam IntensityT Template class for the intensity statistics to be computed
   *
   * Iterates over the organized point cloud matrix to generate a average representation of the data.
   * In each index, iterates on the data vector and computes the average (X, Y, Z) coordinates of the point. Computes
   * also the ring.
   *
   * Computes:
   * - the average (X, Y, Z) coordinates of the point
   * - the ring value of the point
   * - the average value and variance of intensity for every point
   * - The distance to the origin for every point
   * - the average value and variance of distance for every point
   * - the intensity average and variance for each laser ID
   * - the intensity average and variance for each azimuthal angle
   *
   * \todo remove dependency from velodyne::DEFAULT_RING_VALUE
   */
  template <typename IntensityT>
  void computeStats(std::vector<IntensityT>& azimuth, std::vector<IntensityT>& laser)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeStats]");

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
          this->at(j, i).intensity += this->at(j, i).data_points[k].intensity;
        }
        if (this->at(j, i).data_points.size() > 0)
        {
          this->at(j, i).x /= (float)(this->at(j, i).data_points.size());
          this->at(j, i).y /= (float)(this->at(j, i).data_points.size());
          this->at(j, i).z /= (float)(this->at(j, i).data_points.size());

          this->at(j, i).ring = this->at(j, i).data_points[0].ring;

          this->at(j, i).distance_var += -(this->at(j, i).distance_mean * this->at(j, i).distance_mean) /
                                         (float)(this->at(j, i).data_points.size());
          this->at(j, i).intensity_var +=
              -(this->at(j, i).intensity * this->at(j, i).intensity) / (float)(this->at(j, i).data_points.size());

          this->at(j, i).distance_mean /= (float)(this->at(j, i).data_points.size());
          this->at(j, i).intensity /= (float)(this->at(j, i).data_points.size());
        }

        laser[i].mean += this->at(j, i).intensity;

        azimuth[j].mean += (this->at(j, i).intensity / (float)(this->height));
      }
      laser[i].mean /= (float)(this->width);
    }
    ROS_DEBUG_NAMED("call_stack", "[registerVelodynePointCloud] End");
  }

  /*!
   * \brief Copy the data model to an OrganizedPointCloud object
   * \param[out] point_cloud organized point cloud containing only the
   * \tparam DataT Template class for each element of the Data Container element vector
   *
   * Iterates over each of the data containers on the organized point cloud matrix and copies the data to a new
   * Organized Point Cloud with no Container
   *
   */
  void generateModel(OrganizedPointCloud<DataT>* point_cloud)
  {
    ROS_DEBUG_NAMED("call_stack", "[generateModel]");

    for (int i = 0; i < this->height; ++i)
    {
      for (int j = 0; j < this->width; ++j)
      {
        point_cloud->at(j, i).x = this->at(j, i).x;
        point_cloud->at(j, i).y = this->at(j, i).y;
        point_cloud->at(j, i).z = this->at(j, i).z;
        point_cloud->at(j, i).ring = this->at(j, i).ring;
        point_cloud->at(j, i).intensity = this->at(j, i).intensity;
      }
    }
  }

  /*!
   * \brief Organizes the point cloud
   * \warning This function is empty
   */
  void organizePointCloud(const pcl::PointCloud<DataT> unorganized_cloud)
  {
    // This function is not implemented and is only here to provide an overload to the class this subclass inherits
  }

protected:
  /*!
   * \brief Compute azimuth angle
   * \param[in] point the point cloud point for computing the azimuth
   * \tparam DataT Template class for each element of the Data Container element vector
   * \return the azimuthal angle, in degrees
   * \remark Computation is done using atan2
   */
  float getAzimuth(const DataT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getAzimuth]");
    return atan2(point.y, point.x) * point_cloud::organized::RADIAN_TO_DEGREE_F;
  }

  /*!
   * \brief Compute azimuth index of the organized Point Cloud Matrix like Structure
   * \param[in] point the point cloud point for computing the azimuth
   * \tparam DataT Template class for each element of the Data Container element vector
   * \return the azimuthal angle index
   *
   * Computes the the azimuth angle for a given point, shift it to be defined between [0, 360[ and then computes the
   * corresponding index
   */
  unsigned int getAzimuthIndex(const DataT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getAzimuthIndex]");
    float shifted_azimuth = getAzimuth(point) + point_cloud::organized::DEGREE_OFFSET_TO_POSITIVE_ANGLE_F;
    float fixed_point_shifted_azimuth = floor(shifted_azimuth * 10.0f) / 10.0f;
    float index =
        fixed_point_shifted_azimuth * (float)(this->width - 1) / point_cloud::organized::FULL_REVOLUTION_DEGREE_F;
    return (unsigned int)(index);
  }

  /*!
   * \brief Compute polar angle
   * \param[in] point the point cloud point for computing the polar angle
   * \tparam DataT Template class for each element of the Data Container element vector
   * \return the polar angle, in degrees
   */
  float getPolar(const DataT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getPolar]");
    float r = computeEuclideanDistanceToOrigin(point);
    return asin(point.z / r) * point_cloud::organized::RADIAN_TO_DEGREE_F;
  }

  /*!
   * \brief Computes laser ID of the organized Point Cloud Matrix like Structure
   * \param[in] point the point cloud point for computing the polar angle
   * \tparam DataT Template class for each element of the Data Container element vector
   * \return the polar angle index
   *
   * Computes the the polar angle for a given point, shift it to be defined between [0, max(Laser ID)[ and then
   * computes the corresponding Laser ID
   */
  unsigned int computeLaserID(const DataT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeLaserID]");
    float polar_angle = getPolar(point);
    float rounded_polar_angle = roundf(polar_angle);
    unsigned int index = velodyne::vlp16::getLaserIDfromPolarAngle((unsigned int)(rounded_polar_angle));
    return index;
  }

  /*!
   * \brief Compute Euclidean distance between a point and the Origin
   * \param[in] point the point cloud point for computing the azimuth
   * \tparam DataT Template class for each element of the Data Container element vector
   * \return Euclidean Distance between the point and the Origin, in the units of the point
   *
   * The Euclidean distance between the points \f$(x, y, z)\f$ and the origin is
    \f$\sqrt{x^2+y^2+z^2}\f$.
   *
   */
  float computeEuclideanDistanceToOrigin(const DataT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeEuclideanDistanceToOrigin]");
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  }
};

}  // namespace organized

}  // namespace point_cloud

#endif  // ORGANIZED_POINT_CLOUD_WITH_CONTAINER_H
