#include <ros_utils/ros.h>

bool
ros::init(const std::string& name)
{
	// init ros with empty argc and argg
	int argc = 0; char** argv = nullptr;
	if (not ros::isInitialized())
		ros::init(argc, argv, name);

	return true;
}

void
ros::wait_for_init()
{
	while (not ros::isInitialized());
}

std::shared_ptr<ros::NodeHandle>
ros::make_node(const std::string& name)
{
	return std::make_shared<ros::NodeHandle>(name);
}