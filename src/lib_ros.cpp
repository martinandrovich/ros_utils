#include <ros_utils/ros.h>

bool
ros::init(const std::string& name)
{
	// init ros with empty argc and argg
	// usage:
	// static auto init = ros::init("ros_utils_read");

	int argc = 0; char** argv = nullptr;
	if (not ros::isInitialized())
		ros::init(argc, argv, name);

	return true;
}

ros::NodeHandle*
ros::make_node(const std::string& name)
{
	// init ros with empty argc and argv and make a new node
	ros::init(name);
	return new ros::NodeHandle(name);
}