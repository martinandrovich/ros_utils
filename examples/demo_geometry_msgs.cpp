#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros_utils/geometry_msgs.h>

int main(int argc, char** argv)
{
	using namespace geometry_msgs;
	constexpr auto PI = M_PI;

	ros::init(argc, argv, "demo_geometry_msgs");
	ros::NodeHandle nh;

	auto pose = make_pose({5, 10, 0}, {0, PI/2, 0});
	     pose = make_pose({5, 10, 0, 0, PI/2, 0 });
	     pose = make_pose({5, 10, 0}, {0.707107, 0, 0.707107, 0});

	std::cout << "geometry_msgs::make_pose({5, 10, 0}, {0, PI/2, 0}):\n\n"
	          << pose << std::endl;

	auto pos = read_pose(pose).pos;
	auto rpy = read_pose(pose).rpy;
	auto ori = read_pose(pose).ori;

	std::cout << "geometry_msgs::read_pose(pose).rpy[..]:\n\n"
	          << "rpy[0]: " << rpy[0] << "\n"
	          << "rpy[1]: " << rpy[1] << "\n"
	          << "rpy[2]: " << rpy[2] << "\n";

	return 0;
}
