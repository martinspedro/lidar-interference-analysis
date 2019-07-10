/**
 * @file   camera.cpp
 * @brief  Image Stream and Visualizer Header File
 *
 * @author Pedro Martins
 * @date   9 july 2019
 */

#ifndef IMAGE_H
#define IMAGE_H

#include <string>

class Image {
    private:
        std::string image_color_topic;
        std::string image_info_topic;

    public:
        Image();
        Image(std::string _color_topic, std::string _info_topic);

        std::string getImageColorTopic() const;
        std::string getImageInfoTopic() const;

        int getImageColorTopic(std::string color_topic);
        int getImageInfoTopic(std::string info_topic);
};

#endif // IMAGE_H
