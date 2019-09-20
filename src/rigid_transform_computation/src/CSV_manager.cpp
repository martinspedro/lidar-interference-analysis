/*!
 *  \file CSV_manager.cpp
 *  \brief Basic implementation of a CSV file manager tailored for the rigid_transform_computation package
 *  \author Pedro Martins (martinspedro@av.it.pt)
 *
 *  C++ implementation file of a basic file manager for CSV (Comma Separated Values) file-types, with read and write
 * functionatilies.
 *
 *  Provides read and write functionalities for CSV files that store the LiDAR and Camera points correspondence
 */

#include <iostream>
#include <fstream>
#include <string>

#include "rigid_transform_computation/CSV_manager.hpp"

namespace csv_file_manager
{
void write(std::string filename, std::vector<cv::Point2f> image_pixels, std::vector<cv::Point3f> point_cloud_points)
{
  std::fstream fout;                   // file pointer
  fout.open(filename, std::ios::out);  // creates a new csv file with writing permission

  /* Formats the output in CSV style and streams it to file pointer
   * image X, image Y, point cloud X, point cloud y, point cloud z
   */
  for (int i = 0; i < image_pixels.size(); i++)
  {
    fout << image_pixels.at(i).x << ", " << image_pixels.at(i).y << ", " << point_cloud_points.at(i).x << ", "
         << point_cloud_points.at(i).y << ", " << point_cloud_points.at(i).z << "\n";
  }

  fout.close();
}

void read(std::string filename, std::vector<cv::Point2f>* image_pixels, std::vector<cv::Point3f>* point_cloud_points)
{
  std::fstream fin;                  // File pointer
  fin.open(filename, std::ios::in);  // Open an existing file

  // temporary variables
  std::string line;
  std::vector<cv::Point2f> temp_image_pixels;
  std::vector<cv::Point3f> temp_point_cloud_points;

  // while still has lines to be read
  while (getline(fin, line))
  {
    std::stringstream ss(line);

    std::vector<std::string> words;
    cv::Point2f temp_pixel;
    cv::Point3f temp_point;

    // split a correspondence in substrings contain the 5 data fields
    while (ss.good())
    {
      std::string substr;
      getline(ss, substr, ',');
      words.push_back(substr);
    }

    try
    {
      // convert pixel coordinates to integer and save the 2D point in the vector of selected pixels
      temp_pixel.x = std::stoi(words.at(0));
      temp_pixel.y = std::stoi(words.at(1));
      temp_image_pixels.push_back(temp_pixel);

      // convert point cloud coordinates to float and save the 3D point in the vector of selected points
      temp_point.x = std::atof(words.at(2).c_str());
      temp_point.y = std::atof(words.at(3).c_str());
      temp_point.z = std::atof(words.at(4).c_str());
      temp_point_cloud_points.push_back(temp_point);
    }
    catch (const char* exception)
    {
      std::cout << exception << std::endl;
    }
  }

  fin.close();

  // Fill the pointers with the read data
  *image_pixels = temp_image_pixels;
  *point_cloud_points = temp_point_cloud_points;
}
} /* namespace csv_file_manager */
