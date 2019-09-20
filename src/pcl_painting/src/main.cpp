#include <ros/ros.h>

// PCL specific includes
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

//using namespace pcl;

void color_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> cloud, uint8_t r, uint8_t g, uint8_t b)
{
    for (int i = 0; i < cloud.points.size(); i++) {
        cloud.points[i].r = r;
        cloud.points[i].g = g;
        cloud.points[i].b = b;
    }
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // create XYZRGB point cloud
    PointCloudC cloud_colored;
    
    // Convert ROS message to colored point cloud object
    pcl::fromROSMsg(*cloud_msg, cloud_colored);
   
    // pack r/g/b into rgb
    uint8_t r = 255, g = 0, b = 0;    // Example: Red color
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
     
    // Perform color manipulation on Point Cloud
    color_point_cloud(cloud_colored, r, g, b);
    
    // Publish the data
    pub.publish(cloud_colored);
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "pcl_painting");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("colored_point_cloud", 1);

    // Spin
    ros::spin();
}
