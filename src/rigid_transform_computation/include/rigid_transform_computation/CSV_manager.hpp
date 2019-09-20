/*!
 *  \file CSV_manager.hpp
 *  \brief Basic implementation of a CSV file manager tailored for the rigid_transform_computation package
 *  \author Pedro Martins (martinspedro@av.it.pt)
 *
 *  Header file of a basic file manager for CSV (Comma Separated Values) file-types, with read and write functionatilies

 *  Provides read and write functionalities for CSV files that store the LiDAR and Camera points correspondence
 */

#ifndef CSV_MANAGER_H
#define CSV_MANAGER_H

#include <opencv2/opencv.hpp>

/*!
 *  \brief Basic CSV file manager namespace
 */
namespace csv_file_manager
{
/*!
 *  \brief Writes pixel and point cloud points correspondences to a CSV file
 *  \param[in] filename String containg the full path of the file to be saved
 *  \param[in] image_pixels vector of 2D OpenCV points, corresponding to selected pixels from an image
 *  \param[in] point_cloud_points vector of 3D OpenCV points, corresponding to selected points from a point cloud
 *
 *  \todo Take care of exceptions and add robustness to errors and invalid input/files
 *
 *  \par CSV file style:
 *  Data is saved using row-major format (each 2D <-> 3D correspondence in each line)
 *  \par
 *  image X, image Y, point cloud X, point cloud y, point cloud z
 */
void write(std::string filename, std::vector<cv::Point2f> image_pixels, std::vector<cv::Point3f> point_cloud_points);

/*!
 *  \brief Reads pixel and point cloud points correspondences to a CSV file
 *  \param[in] filename String containg the full path of the file to be saved
 *  \param[out] image_pixels vector pointer of 2D OpenCV points, corresponding to selected pixels from an image
 *  \param[out] point_cloud_points vector pointer of 3D OpenCV points, corresponding to selected points from a point
 * cloud
 *
 *  \todo Take care of exceptions and add robustness to errors and invalid input/files
 *
 *  \par CSV file style:
 *  Assumes data is saved using row-major format (each 2D <-> 3D correspondence in each line)
 *  \par
 *  image X, image Y, point cloud X, point cloud y, point cloud z
 */
void read(std::string filename, std::vector<cv::Point2f>* image_pixels, std::vector<cv::Point3f>* point_cloud_points);
} /* namespace csv_file_manager */

#endif /* CSV_MANAGER_H */
