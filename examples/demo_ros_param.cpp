#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros_utils/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_ros_param");
	ros::NodeHandle nh;
	
	ros::param::set("my_param", "hello");
	std::cout << "ros::param::read<std::string>(\"my_param\", \"default\"): "
	          << ros::param::read<std::string>("my_param", "default")
	          << std::endl;

	return 0;
}
