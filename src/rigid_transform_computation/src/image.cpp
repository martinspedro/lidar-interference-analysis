/**
 * @file   camera.cpp
 * @brief  Image Stream and Visualizer Header File
 *
 * @author Pedro Martins
 * @date   9 july 2019
 */
#include "rigid_transform_computation/image.hpp"


Image::Image(){
    this->image_color_topic = "/image_color";
    this->image_info_topic = "/image_info";
}

Image::Image(std::string color_topic, std::string info_topic) {
    this->image_color_topic = color_topic;
    this->image_info_topic = info_topic;
}

std::string Image::getImageColorTopic() const {
    return this->image_color_topic;
}

std::string Image::getImageInfoTopic() const {
    return this->image_info_topic;
}

int Image::getImageColorTopic(std::string color_topic) {
    this->image_color_topic = color_topic;
    return EXIT_SUCCESS;
}

int Image::getImageInfoTopic(std::string info_topic) {
    this->image_info_topic = info_topic;
    return EXIT_SUCCESS;
}
