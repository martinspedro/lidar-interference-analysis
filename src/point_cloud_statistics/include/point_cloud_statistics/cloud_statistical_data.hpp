/**
 * \file  cloud_statistical_data.hpp
 * \brief Header file for Statistical Cloud data class
 *
 */

#ifndef CLOUD_STATISTICAL_DATA_H
#define CLOUD_STATISTICAL_DATA_H

#include <string>
#include <sstream>

namespace point_cloud
{
/*!
 * \brief Point Cloud nested namespace for statistics
 */
namespace statistics
{
/*!
 * \struct CloudStatisticalData
 * \brief Structure to hold statistics about the point cloud data
 */
struct CloudStatisticalData
{
  long int point_cloud_msg_count;  //!< Number of point cloud messages received
  long int point_count;            //!< Number of point cloud points received
  long int outliers_points_count;  //!< Number of point cloud points that are outliers
  long int inliers_points_count;   //!< Number of point cloud points that are outliers

  double points_average_per_msg;   //!< Average number of points per point cloud message
  double points_variance_per_msg;  //!< Number of point per message variance

  double relative_out_points;  //!< Relative number of outliers to the total number of points
  double relative_in_points;   //!< Relative number of outliers to the total number of points

  /*!
   * \brief Structure Constructor
   * Initializes all fields to zero
   */
  CloudStatisticalData();

  /*!
   * \brief prints to std::cout the general message statistics
   *
   * Prints number of messages, points (absolute and average per message), outliers (absolute and relative)
   */
  void printStatistics();

  /*!
   * \brief prints to std::cout the general point statistics
   *
   * Prints number of messages, points (absolute and average per message),
   */
  void printPointStatistics();

  /*!
   * \brief creates a std::stringstream with the general message statistics
   * \return A formatted std::stringstream object with number of messages, points (absolute and average per message),
   * outliers (absolute and relative)
   */
  std::stringstream outputStringFormattedPointStatistics();

  /*!
   * \brief creates a std::stringstream with the general point statistics
   * \return A formatted std::stringstream object with number of messages, points (absolute and average per message),
   */
  std::stringstream outputStringFormattedStatistics();

  /*!
   * \brief Computes relative number of outliers, inliers and average point statistics
   * \todo Force invariant to the other functions using this one
   */
  void computeStats();

  /*!
   * \brief Computes relative number of outliers
   */
  void computeOutliersRelativeValue();

  /*!
   * \brief Computes relative number of inliers
   */
  void computeInliersRelativeValue();

  /*!
   * \brief Computes the percentage of outliers
   * \return the relative number of outliers, in percentage
   */
  inline double getOutliersPercentage()
  {
    return this->relative_out_points * 100.0d;
  }

  /*!
   * \brief Computes the percentage of inliers
   * \return the relative number of inliers, in percentage
   */
  inline double getInliersPercentage()
  {
    return this->relative_in_points * 100.0d;
  }
};

}  // namespace statistics

}  // namespace point_cloud

#endif  // CLOUD_STATISTICAL_DATA_H
