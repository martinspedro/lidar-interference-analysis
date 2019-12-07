/*!
 * \file   rigid_body_transform.hpp
 * \brief  Header file for the Rigid Body Trasform Namespace and Class
 *
 *
 */

#ifndef RIGID_BODY_TRANSFORM_H
#define RIGID_BODY_TRANSFORM_H

#include <string>

#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include "tf2/LinearMath/Quaternion.h"

#include "rigid_transform_computation/compute_rigid_body_transform.h"  // Service for rigid body transform computation
#include "rigid_transform_computation/save_correspondences.h"          // Service for saving 2D <-> 3D correspondences

#include "rigid_transform_computation/pointCloudVisualizer.hpp"  // necessary to include due to the typedefs

#include "rigid_transform_computation/Pixel.h"      // Message for a single Pixel
#include "rigid_transform_computation/PointXYZI.h"  // Message for a single 3D point cloud point

/*!
 * \namespace rigid_body_transform
 * \brief Contains all the structures, classes and standalone functions related to a Rigid Body Transform
 *
 * Contains the LiDARCameraCalibrationData class
 */
namespace rigid_body_transform
{
/// String containing the default file path for the thesis path
const std::string THESIS_FILE_PATH = "/media/martinspedro/DATA/UA/Thesis/multiple-lidar-interference-mitigation/";

/*!
 * \brief Transforms a point on the LiDAR referential to the camera referential
 * \param[in] lidar_point 3 dimensional point (x, y, z) using OpenCV datatype on the LiDAR referential
 * \return 3 dimensional point (x, y, z) using Opencv datatype on the camera referential
 *
 * \note
 * \parblock
 * \par LiDAR referential:
 * - x forwards
 * - y right
 * - z upwards
 * \par Camera referential:
 * - z forwards
 * - x right
 * - y downwards
 * \endparblock
 */
cv::Point3f transformLiDARToCameraReferential(cv::Point3f lidar_point);

/*!
 * \struct Pixel_struct
 * \brief Data structure containing position (x, y) and color (R, G, B) of a Pixel in a given image
 */
typedef struct Pixel_struct
{
  uint32_t x;  //!< Column-wise position value
  uint32_t y;  //!< Row-wise    position value
  uint8_t r;   //!< Red   color component
  uint8_t g;   //!< Green color component
  uint8_t b;   //!< Blue  color component
} Pixel_t;

/*!
 * \class RigidTransformData
 * \brief Data class to store the LiDAR points, Pixel and ROS camera info messages
 */
class RigidTransformData
{
  point_cloud::PointI point_cloud_point;  //!< point cloud point with position coordinates and an intensity field
  Pixel_t image_pixel;                    //!< Pixel defined by a position in a image and a color
  sensor_msgs::CameraInfo camera_info;    //!< Camera parameters in a ROS sensor messages format
};

/*!
 * \class LiDARCameraCalibrationData
 * \brief Handles the computation of the 6DoF transform between a LiDAR and a Camera
 *
 * Class to store the LiDAR and Camera correspondences and computing the rigod body transform
 *
 * Provides the callbacks for Pixel, PointXYZI and Camera Info messages and the ServiceServer implementation for
 * saving correspondences and computing the rigid body transform on demand.
 *
 * Also contains utilities methods to read the camera configuration for YAML files.
 *
 * Can publish the 6DoF transform under a TF static transform
 */
class LiDARCameraCalibrationData
{
public:
  /*!
   * \brief Construct that receives the origin and the destination frame name for the 6DoF transform, respectively
   *
   * Creates a LiDARCameraCalibrationData object that uses the given names as frames.
   *
   * \warning 2D <-> 3D correspondences datapoints and camera parameters are **not** initialized using this constructor.
   *
   * Caller must initialize them using other methods such as LiDARCameraCalibrationData::readCameraInfoFromYAML,
   * csv_file_manager::read or instantiate a ros::Subscriber object for each topic using the available callbacks. See
   * LiDARCameraCalibrationData::pixelCallback, LiDARCameraCalibrationData::pointCloudCallback and/or
   * LiDARCameraCalibrationData::cameraInfoCallback.
   */
  LiDARCameraCalibrationData(std::string frame_id, std::string child_frame_id);

  /*!
   * \brief Construct that receives the origin and the destination frame names for the 6DoF transform, the LiDAR and
   * Camera Data
   *
   * Creates a LiDARCameraCalibrationData object that uses the given names as frames.
   *
   * 2D <-> 3D correspondences are given through the vectors point_cloud_points and image_pixels, passed by argument
   *
   * \warning camera parameters are **not** initialized using this constructor
   *
   * Caller must initialize them camera parameters, either by reading the camera parameters using
   * LiDARCameraCalibrationData::readCameraInfoFromYAML or instantiating a ros::Subscriber object to the
   * LiDARCameraCalibrationData::cameraInfoCallback
   */
  LiDARCameraCalibrationData(std::string frame_id, std::string child_frame_id,
                             std::vector<cv::Point3f> point_cloud_points, std::vector<cv::Point2f> image_pixels);

  /*!
   * \brief Destructor
   *
   * Deletes static broadcaster dynamic memory
   */
  ~LiDARCameraCalibrationData();

  /*!
   * \brief Publishs the 6 DoF transform between the destination and origin frames
   *
   * Converts the rigid body transform data into a ROS message (geometry_msgs/TransformStamped), containing the 6DoF
   * transform between the destination and origin frames and then publishes it under /tf_static
   */
  void publishStaticTransformMsg();

  /*!
   * \brief Calculates the 6DoF rigid body transform between the origin and destination frames, given the solvePnP mode
   * \param[in] pnp_mode integer that selects the algorithm used to solve the Perspective-n-Point problem. See below for
   * the convetion.
   *
   * Converts the rigid body transform data into a ROS message (geometry_msgs/TransformStamped), containing the 6DoF
   * transform between the destination and origin frames and then publishes it under /tf_static
   *
   * \remark Requires that the solvePnP algorithm to given as argument
   *  \parblock
   *  Available methods:
   *  - 0: SOLVEPNP_ITERATIVE
   *  - 1: SOLVEPNP_P3P
   *  - 2: SOLVEPNP_EPNP
   *  - 3: SOLVEPNP_DLS
   *  - 4: SOLVEPNP_UPNP
   *  \endparblock
   */
  void computeRigidBodyTransform(int pnp_mode);

  /*!
   * \brief Reads camera parameters from a YAML file
   * \param[in] filename full path of the YAML file
   *
   * Reads and parses the file corresponding to the input parameter, allocating all the necessary resources to
   * accomodate the memory and initializing camera parameters member camera_info_
   */
  void readCameraInfoFromYAML(std::string filename);

  /*!
   * \brief Saves image and point cloud correspondences in a CSV file
   * \param[in] req Request object containing the fields of a save_correspondences request
   * \param[out] res Response object containg the fields of a save_correspondences response
   *
   * Saves a CSV file containg the correspondences between 2D and 3D point. For more informations on the file format see
   * csv_file_manager. Also see save_correspondences.srv for more informations on the service data specification
   *
   * \pre The same number of 2D pixels and 3D point cloud points must be the same and different from zero (0)
   */
  bool saveCorrespondencesSrvCallback(rigid_transform_computation::save_correspondences::Request& req,
                                      rigid_transform_computation::save_correspondences::Response& res);
  /*!
   * \brief  Asynchronous calculates the 6DoF rigid body transform between the origin and destination frames
   * \param[in] req Request object containing the fields of a compute_rigid_body_transform request
   * \param[out] res Response object containg the fields of a compute_rigid_body_transform response
   *
   * Converts the rigid body transform data into a ROS message (geometry_msgs/TransformStamped), containing the 6DoF
   * transform between the destination and origin frames and then sends it as the service response.
   *
   * See compute_rigid_body_transform.srv for more informations on the service data specification
   * \pre The same number of 2D pixels and 3D point cloud points are required
   * \pre The pixel and/or point cloud points selected must be different from zero (0)
   */
  bool computeRigidTransformSrvCallback(rigid_transform_computation::compute_rigid_body_transform::Request& req,
                                        rigid_transform_computation::compute_rigid_body_transform::Response& res);
  /*!
   * \brief Calculates the 6DoF rigid body transform between the origin and destination frames, given the solvePnP mode
   * \param[in] msg Pixel custom message smart pointer. See Pixel.msg for a depper understanding of the message contents
   *
   * Pushes the newer pixel positional data on the LiDARCameraCalibrationData object
   */
  void pixelCallback(const rigid_transform_computation::Pixel::ConstPtr& msg);

  /*!
   * \brief Calculates the 6DoF rigid body transform between the origin and destination frames, given the solvePnP mode
   * \param[in] msg PointXYZI custom message smart pointer. See PointXYZI.msg for a depper understanding of the message
   * contents
   *
   * Pushes the newer the point_cloud positional data on the LiDARCameraCalibrationData object
   */
  void pointCloudCallback(const rigid_transform_computation::PointXYZI::ConstPtr& msg);

  /*!
   * \brief Calculates the 6DoF rigid body transform between the origin and destination frames, given the solvePnP mode
   * \param[in] cam_info Camera Parameters message smart pointer.
   *
   * Stores the camera parameters on the LiDARCameraCalibrationData object
   */
  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& cam_info);

private:
  /*!
   * \brief Initializes a geometry_msgs::TransformStamped object, using the already computed 6DoF Rigid Body Transform
   * \return The 6DoF Rigid Body Transform through a geometry_msgs::TransformStamped object
   *
   * \pre The Rigid Body Transform must be already calculated, either by calling explicitely
   * LiDARCameraCalibrationData::computeRigidBodyTransform or by requesting the compute_rigid_body_transform service
   */
  geometry_msgs::TransformStamped createStaticTransformMsg();

  /*!
   * \brief Determines the rotation and translation vector between two reference frames
   * \param[in] solvePnP_mode Algorithm selected for the OpenCV solvePnP function
   *
   * The OpenCV solvesPnP algorithm is used to solve the Perspective-n_Problem between the monocular camera and LiDAR 2D
   * to 3D correspondences. This method only implements a wrapper that allows a easier use and selecting the desired
   * algorithm to be run
   *
   * \note
   * Rotation vector format is such that the corresponds to the rotation axis with angle magnitude (radians) [x, y, z]
   * \pre The LiDARCameraCalibrationData object must be fully initialized
   */
  void solvePnP(int solvePnP_mode);

  std::string frame_id_;        //!< Origin frame name for the 6DoF transform
  std::string child_frame_id_;  //!< Destination frame name for the  6DoF transform

  std::vector<cv::Point3f> point_cloud_points_;     //!< 3D points vector containing the selected point cloud points
  std::vector<cv::Point2f> image_pixels_;           //!< 2D points vector containing the selected image pixels
  image_geometry::PinholeCameraModel camera_info_;  //!< ROS Pinhole Camera Model object containg the camera parameters

  cv::Mat rotation_vector_;              //!<  Rotation vector axis with angle magnitude (radians) [x, y, z]
  cv::Mat translation_vector_;           //!<  Translation Vector [x, y, z]
  tf2::Quaternion rotation_quaternion_;  //!<  Quaternion encoding the rotation_vector_ [x, y, z, w]

  tf2_ros::StaticTransformBroadcaster* static_broadcaster_;  //!< static TF broadcaster pointer for the 6DoF transform
};

} /* namespace rigid_body_transform */

#endif /* RIGID_BODY_TRANSFORM_H */
