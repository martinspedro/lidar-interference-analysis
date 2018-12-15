#include <ros/ros.h>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    /**
     * Create a colored point cloud object
     */
    PointCloudC cloud_colored;
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    pcl_conversions::toPCL(*cloud_msg, *cloud);
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
    //copyPointCloud(cloud, cloud_colored);
    //cloud_colored.points.resize(cloud->points.size());
    uint8_t r = 255, g = 0, b = 0;    //  Red color
    uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
    cloud_colored.width = cloud->width;
    cloud_colored.height = cloud-> height;

    cloud_colored.data = *cloud.points.data
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
    pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("colored_point_cloud", 1);
    
    // Spin
    ros::spin();
}
