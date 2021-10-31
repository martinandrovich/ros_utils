#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros_utils/std.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_std");
	ros::NodeHandle nh;
	
	std::cout << "is_in(\"a\", {\"a\", \"b\", \"c\"}): " << std::boolalpha << is_in("a", {"a", "b", "c"}) << std::endl;
	std::cout << "is_in(\"d\", {\"a\", \"b\", \"c\"}): " << std::boolalpha << is_in("d", {"a", "b", "c"}) << std::endl;
	
	std::string b = "b";
	std::cout << "is_in(b, {\"a\", \"b\", \"c\"}): " << std::boolalpha << is_in(b, {"a", "b", "c"}) << std::endl;
	
	return 0;
}
