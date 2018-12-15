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

    pcl::PointCloud<pcl::PointXYZRGB> cloud_colored;

    //PointCloudC cloud_colored;
    //pcl::PCLPointCloud2* cloud; // = new pcl::PCLPointCloud2;
    //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    
    pcl::fromROSMsg(*cloud_msg, cloud_colored);
   
    // pack r/g/b into rgb
    uint8_t r = 255, g = 0, b = 0;    // Example: Red color
    //uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
     
    //pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    //pcl::PointCloud<pcl::PointXYZRGB> cloud_xyzrgb;
    
    //pcl::copyPointCloud(cloud, cloud_colored);
    
    color_point_cloud(cloud_colored, r, g, b);
/*
    for (size_t i = 0; i < cloud_colored.points.size(); i++) {
        cloud_colored.points[i].r = r;
        cloud_colored.points[i].g = g;
        cloud_colored.points[i].b = b;
    }
    //ROS_INFO("Got point cloud with %ld points", cloud->size());
    //cloud->size;
    // c//loud_colored[0].rgb;
    //PointCloudC::Ptr colored_cloud(new PointCloudC());

    // Create a container for the data.
    //sensor_msgs::PointCloud2 output;

    // Container for original & filtered data
    //pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    //pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    //pcl::PCLPointCloud2 cloud_filtered;

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    //pcl::fromROSMsg(*input, cloud);

    //pcl::PointCloud<pcl::PointXYZRGB> cloud_colored;
    //pcl::copyPointCloud(cloud, cloud_colored);
    // PointCloud<PointXYZ> cloud_xyz;

    //PointCloud<PointXYZRGB> cloud_xyzrgb;
    //copyPointCloud(&cloud, &cloud_colored);

    //cloud_colored.width = cloud->width;
    //cloud_colored.height = cloud-> height;

    //cloud_colored.data = cloud.points.data
    /*
    for (size_t i = 0; i < 1; i++) {
        cloud_colored[i].x = cloud[i].data[0];
        cloud_colored[i].y = cloud[i].data[1];
        cloud_colored[i].z = cloud[i].data[2];
        //cloud_colored[i].r = r;
        //cloud_colored[i].g = g;
        //cloud_colored[i].b = b;
    }
    //cloud.rgb = *reinterpret_cast<float*>(&rgb);
    //  */
    // Convert to ROS data type
    //pcl_conversions::fromPCL(input, output);
    //pcl_conversions::fromPCL(cloud_colored, output);

    // Publish the data.
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
    //pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("colored_point_cloud", 1);

    // Spin
    ros::spin();
}
