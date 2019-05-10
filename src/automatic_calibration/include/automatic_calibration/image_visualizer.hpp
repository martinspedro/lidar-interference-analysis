/**
 * @file   image_visualizer.hpp
 * @brief  Image Stream and Visualizer Header File
 *
 * @author Pedro Martins
 * @date   @today
 */


static const std::string OPENCV_WINDOW = "Simple Video Stream Viewer";


class ImageVisualizer {
  ros::NodeHandle nh_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber     image_sub_;
  image_transport::Publisher      image_pub_;

public:
  ImageVisualizer() : it_(nh_) {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/image_color/raw", 1, &ImageVisualizer::imageCb, this);
    image_pub_ = it_.advertise("/camera/image_info", 1);

    cv::namedWindow(OPENCV_WINDOW, 0);
  }

  ~ImageVisualizer() {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  /** @brief Callback function for image visualization
   *
   */
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    //image_pub_.publish(cv_ptr->toImageMsg());
  }
};
