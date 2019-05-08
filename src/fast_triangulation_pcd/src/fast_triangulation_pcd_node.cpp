#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>

#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/search/kdtree.h>


using namespace pcl;

// Add these typedefs after your #includes
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

ros::Publisher pub;

void cloud_cb (sensor_msgs::PointCloud2ConstPtr& point_cloud_msg)
{
	// Create a container for the data
	sensor_msgs::PointCloud2 output;
	PointCloudC cloud;

	pcl::fromROSMsg(*point_cloud_msg, cloud);

  // Load input file into a PointCloud<T> with an appropriate type
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl_conversions::toPCL(*input, *cloud)
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	//pcl::PointCloud<pcl::PointXYZ> cloud;
	//pcl::fromROSMsg(*input, cloud);



	PointCloudC::Ptr cloudPtr(new PointCloudC());
	*cloudPtr = cloud;
	//ROS_INFO("Got point cloud with %ld points", cloud->size());
	/*
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2 (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2 (*cloud, *cloud2)

	// Normal estimation
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (&cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	/* normals should not contain the point normals + surface curvatures


	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (*cloud);
	n.setInputCloud (cloud);
	n.setSearchMethod (tree);
	n.setKSearch (20);
	n.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	pcl::PolygonMesh triangles;

	// Set the maximum distance between connected points (maximum edge length)
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(false);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();


	pcl::io::saveVTKFile ("mesh.vtk", triangles);
*/

	//output = *input;

	// Publish the data
	//pub.publish(output);
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init(argc, argv, "fast_triangulation_pcd");
	ros::NodeHandle nh;

	// Create a ROS subscriber for the input point cloud
	 ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("velodyne_points", 1, cloud_cb);


	// Create a ROS publisher for the output point cloud
	//pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

	// Spin
	ros::spin();

	return EXIT_SUCCESS;
}
