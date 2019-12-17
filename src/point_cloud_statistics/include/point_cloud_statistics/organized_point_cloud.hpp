/**
 * \file  organized_point_cloud.hpp
 * \brief Organized Point cloud Class to store generic data
 *
 * \author Pedro Martins (martinspedro@ua.pt)
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
/*!
 * \class OrganizedPointCloud
 * \brief Organized Point Cloud that inherits from pcl::PointCloud and extends its capabilities for organized point
 * cloud
 * \tparam PointT Generic Point Type, defined to be PCL compatible
 *
 * This class inherits from  pcl::PointCloud
 *
 */
template <class PointT>
class OrganizedPointCloud : public pcl::PointCloud<PointT>
{
public:
  using Ptr = boost::shared_ptr<OrganizedPointCloud>;
  using ConstPtr = boost::shared_ptr<const OrganizedPointCloud>;

  /*!
   * \brief Constructor
   * \param[in] width organized point cloud matrix number of columns
   * \param[in] height organized point cloud matrix number of rows
   *
   * Call the pcl::PointCloud constructor
   */
  OrganizedPointCloud(unsigned int width, unsigned int height) : pcl::PointCloud<PointT>(width, height)
  {
    ROS_DEBUG_NAMED("call_stack", "[OrganizedPointCloud] with (%d, %d)", width, height);
  }

  /*!
   * \brief Replaces all points on the point cloud matrix using the blank object constructor
   */
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

  /*!
   * \brief Compute azimuth angle
   * \param[in] point the point cloud point for computing the azimuth
   * \tparam PointT Generic Point Type, defined to be PCL compatible
   * \return the azimuthal angle, in degrees
   * \remark Computation is done using atan2
   */
  virtual float getAzimuth(const PointT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[getAzimuth]");
    return atan2(point.y, point.x) * point_cloud::organized::RADIAN_TO_DEGREE_F;
  }

  /*!
   * \brief Compute azimuth index of the organized Point Cloud Matrix like Structure
   * \param[in] point the point cloud point for computing the azimuth
   * \tparam PointT Generic Point Type, defined to be PCL compatible
   * \return the azimuthal angle index
   *
   * Computes the the azimuth angle for a given point, shift it to be defined between [0, 360[ and then computes the
   * corresponding index
   */
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

  /*!
   * \brief Compute Euclidean distance between a point and the Origin
   * \param[in] point the point cloud point for computing the azimuth
   * \tparam PointT Generic Point Type, defined to be PCL compatible
   * \return Euclidean Distance between the point and the Origin, in the units of the point
   *
   * The Euclidean distance between the points \f$(x, y, z)\f$ and the origin is
    \f$\sqrt{x^2+y^2+z^2}\f$.
   *
   */
  virtual float computeEuclideanDistanceToOrigin(const PointT point)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeEuclideanDistanceToOrigin]");
    return std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
  }

  /*!
   * \brief Compute Euclidean distance between a point and the Origin
   * \param[in] point1 the source point cloud point for computing the distance
   * \param[in] point2 the target point cloud point for computing the distance
   * \tparam PointT Generic Point Type, defined to be PCL compatible
   * \return Euclidean Distance between the point1 and the point2, in the units of the point
   *
   * The Euclidean distance between the two points \f$Point_1 = (x, y, z)\f$ and \f$Point_2 = (x, y, z)\f$ is
   * \f$\sqrt{(point2.x-point2.x)^2+(point2.y-point1.y)^2+(point2.z-point1.z)^2}\f$.
   *
   */
  virtual float computeEuclideanDistance(const PointT point1, const PointT point2)
  {
    ROS_DEBUG_NAMED("call_stack", "[computeEuclideanDistance]");
    ROS_DEBUG("P1->(%f, %f, %f), P2->(%f, %f, %f)", point1.x, point1.y, point1.z, point2.x, point2.y, point2.z);
    float x_diff = point2.x - point1.x;
    float y_diff = point2.y - point1.y;
    float z_diff = point2.z - point1.z;

    return std::sqrt(x_diff * x_diff + y_diff * y_diff + z_diff * z_diff);
  }

  /*!
   * \brief  Appends the point cloud to the organized point cloud, in an organized manner
   * \param[in] unorganized_cloud unorganized point cloud to be stored
   * \tparam PointT Generic Point Type, defined to be PCL compatible
   *
   * Iterates over the input point cloud, unorganized, and appends the current point to the Container vector on its
   * correspondent row and column.
   */
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

  /*!
   * \brief Computes the point to point distance between the method invocator and other cloud passed by argument
   * \param[in] organized_point_cloud Organized Point Cloud object, to be compared with this
   * \tparam PointT Generic Point Type, defined to be PCL compatible
   * \return the OrganizedPointCloud object containing the distance in each point
   *
   */
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

  /*!
   * \brief Computes the point to point distance between the method invocator and other cloud passed by argument
   * \param[in] organized_point_cloud Organized Point Cloud object, to be compared with this
   * \param[in] distance_vector vector containing the distances between the corresponding points from one cloud and the
   * other
   * \param[in] intensity_vector vector containing the intensity differences between the corresponding points from one
   * cloud and the other
   * \tparam PointT Generic Point Type, defined to be PCL compatible
   *
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
