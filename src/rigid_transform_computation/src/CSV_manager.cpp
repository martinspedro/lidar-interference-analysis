#include <iostream>
#include <fstream>
#include <string>

#include <opencv2/opencv.hpp>

#include "ros/ros.h"

bool create(std::string filename, std::vector<cv::Point2f> imagePixels, std::vector<cv::Point3f> pointCloudPoints) {


    std::fstream fout; // file pointer

    std::cout << filename << std::endl;
    fout.open(filename, std::ios::out); // opens an existing csv file


    // Read the input
    for (int i = 0; i < imagePixels.size(); i++) {
        fout << imagePixels.at(i).x << ", "
             << imagePixels.at(i).y << ", "
             << pointCloudPoints.at(i).x << ", "
             << pointCloudPoints.at(i).y << ", "
             << pointCloudPoints.at(i).z
             << "\n";
    }

    fout.close();
    return true;
}


void load(std::string filename, std::vector<cv::Point2f>* imagePixels, std::vector<cv::Point3f>* pointCloudPoints) {
    std::fstream fin; // File pointer

    // Open an existing file
    fin.open(filename, std::ios::in);

    std::string line, word;
    std::vector<cv::Point2f> tempImagePixels;
    std::vector<cv::Point3f> tempPointCloudPoints;


    while (getline(fin, line)) {
        std::stringstream ss(line);

        std::vector<std::string> results;
        cv::Point2f tempPixel;
        cv::Point3f tempPoint;


        while( ss.good() ) {
            std::string substr;
            getline( ss, substr, ',' );
            results.push_back( substr );
        }
        try {
            tempPixel.x = std::stoi(results.at(0));
            tempPixel.y = std::stoi(results.at(1));
            tempImagePixels.push_back(tempPixel);


            tempPoint.x = std::atof(results.at(2).c_str());
            tempPoint.y = std::atof(results.at(3).c_str());
            tempPoint.z = std::atof(results.at(4).c_str());
            tempPointCloudPoints.push_back(tempPoint);
        }
        catch (const char *exception) {
            std::cout << exception << std::endl;
        }


    }

    fin.close();
    *imagePixels = tempImagePixels;
    *pointCloudPoints = tempPointCloudPoints;
}
