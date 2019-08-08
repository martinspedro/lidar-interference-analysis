#ifndef CSV_MANAGER_H
#define CSV_MANAGER_H

#include <opencv2/opencv.hpp>

bool create(std::string filename, std::vector<cv::Point2f> imagePixels, std::vector<cv::Point3f> pointCloudPoints);
void load(std::string filename, std::vector<cv::Point2f>* imagePixels, std::vector<cv::Point3f>* pointCloudPoints);

#endif // CSV_MANAGER_H
