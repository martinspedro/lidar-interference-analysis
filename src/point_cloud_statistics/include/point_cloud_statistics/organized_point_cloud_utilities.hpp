/*!
 * \file  organized_point_cloud_utilities.hpp
 * \brief Header file of utilities and constants to organize velodyne point clouds
 *
 * \author Pedro Martins (martinspedro@ua.pt)
 */

#ifndef ORGANIZED_POINT_CLOUD_UTILITIES_H
#define ORGANIZED_POINT_CLOUD_UTILITIES_H

#include <pcl/point_cloud.h>

namespace point_cloud
{
/*!
 * \namespace organized
 * \brief nested point_cloud namespace for organized point clouds related Classes, methods and definitions
 */
namespace organized
{
extern const float RADIAN_TO_DEGREE_F;   //!< Float-Precision multiplication factor to convert an angle in radians to
                                         //!< degree
extern const double RADIAN_TO_DEGREE_D;  //!< Double-Precision multiplication factor to convert an angle in radians to
                                         //!< degree
extern const long double RADIAN_TO_DEGREE_L;  //!< Long Double-Precision multiplication factor to convert an angle in
                                              //!< radians to degree

extern const float DEGREE_OFFSET_TO_POSITIVE_ANGLE_F;        //!< Float-Precision angle shift a rotation by 180
extern const double DEGREE_OFFSET_TO_POSITIVE_ANGLE_D;       //!< Double-Precision angle shift a rotation by 180
extern const long double DEGREE_OFFSET_TO_POSITIVE_ANGLE_L;  //!< Long Double-Precision angle shift a rotation by
                                                             //!< 180

extern const float FULL_REVOLUTION_DEGREE_F;        //!< Float-Precision full revolution angle (360º)
extern const double FULL_REVOLUTION_DEGREE_D;       //!< Double-Precision full revolution angle (360º)
extern const long double FULL_REVOLUTION_DEGREE_L;  //!< Long Double-Precision full revolution angle (360º)

/*!
 * \brief Compute azimuth angle
 * \param[in] y oposite side length
 * \param[in] x adjacent side length
 * \return the azimuthal angle, in degrees
 * \remark Computation is done using atan2
 */
float getAzimuth(float y, float x);

/*!
 * \brief Compute azimuth angle
 * \param[in] y oposite side length
 * \param[in] x adjacent side length
 * \return the azimuthal angle, in degrees
 * \remark Computation is done using atan2
 */
double getAzimuth(double y, double x);

/*!
 * \brief Compute azimuth angle
 * \param[in] y oposite side length
 * \param[in] x adjacent side length
 * \return the azimuthal angle, in degrees
 * \remark Computation is done using atan2
 */
long double getAzimuth(long double y, long double x);

/*!
 * \brief Compute Euclidean distance between two points
 * \param[in] x_1  x coordinate of Point 1
 * \param[in] y_1  y coordinate of Point 1
 * \param[in] z_1  z coordinate of Point 1
 * \param[in] x_2  x coordinate of Point 2
 * \param[in] y_2  y coordinate of Point 2
 * \param[in] z_2  z coordinate of Point 2
 * \return Euclidean Distance between the Point 1 and Point 2
 *
 * The Euclidean distance between the two points \f$(x_1, y_1, z_1)\f$ and \f$(x_2, y_2, z_2)\f$ is
  \f$\sqrt{(x_2-x_1)^2+(y_2-y_1)^2+(z_2-z_1)^2}\f$.
 *
 */
float computeEuclideanDistance(float x_1, float y_1, float z_1, float x_2, float y_2, float z_2);

/*!
 * \brief Compute Euclidean distance between two points
 * \param[in] x_1  x coordinate of Point 1
 * \param[in] y_1  y coordinate of Point 1
 * \param[in] z_1  z coordinate of Point 1
 * \param[in] x_2  x coordinate of Point 2
 * \param[in] y_2  y coordinate of Point 2
 * \param[in] z_2  z coordinate of Point 2
 * \return Euclidean Distance between the Point 1 and Point 2
 *
 * The Euclidean distance between the two points \f$(x_1, y_1, z_1)\f$ and \f$(x_2, y_2, z_2)\f$ is
  \f$\sqrt{(x_2-x_1)^2+(y_2-y_1)^2+(z_2-z_1)^2}\f$.
 *
 */
double computeEuclideanDistance(double x_1, double y_1, double z_1, double x_2, double y_2, double z_2);

/*!
 * \brief Compute Euclidean distance between two points
 * \param[in] x_1  x coordinate of Point 1
 * \param[in] y_1  y coordinate of Point 1
 * \param[in] z_1  z coordinate of Point 1
 * \param[in] x_2  x coordinate of Point 2
 * \param[in] y_2  y coordinate of Point 2
 * \param[in] z_2  z coordinate of Point 2
 * \return Euclidean Distance between the Point 1 and Point 2
 *
 * The Euclidean distance between the two points \f$(x_1, y_1, z_1)\f$ and \f$(x_2, y_2, z_2)\f$ is
  \f$\sqrt{(x_2-x_1)^2+(y_2-y_1)^2+(z_2-z_1)^2}\f$.
 *
 */
long double computeEuclideanDistance(long double x_1, long double y_1, long double z_1, long double x_2,
                                     long double y_2, long double z_2);

}  // namespace organized

}  // namespace point_cloud

#endif  // ORGANIZED_POINT_CLOUD_UTILITIES_H
