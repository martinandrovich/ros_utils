#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros_utils/pcl.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_pcl");
	ros::NodeHandle nh;
	
	// find rovi_models pkg
	std::cout << ros::package::find("package://rovi_models") << std::endl;
	
	auto cloud_pcd = pcl::load_cloud<pcl::PointXYZ>("package://rovi_models/models/milk/milk.pcd");
	auto cloud_ply = pcl::load_cloud<pcl::PointXYZ>("package://rovi_models/models/milk/milk.ply"); // will and should fail

	return 0;
}
