#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros_utils/std.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_std");
	ros::NodeHandle nh;
	
	// -- ENTER_TO_CONTINUE() -----------------------------------------------------
	
	ENTER_TO_CONTINUE("do something");
	
	// -- is_in() -----------------------------------------------------------------
	
	std::cout << "is_in(\"a\", {\"a\", \"b\", \"c\"}): " << std::boolalpha << is_in("a", {"a", "b", "c"}) << std::endl;
	std::cout << "is_in(\"d\", {\"a\", \"b\", \"c\"}): " << std::boolalpha << is_in("d", {"a", "b", "c"}) << std::endl;
	
	std::string b = "b";
	std::cout << "is_in(b, {\"a\", \"b\", \"c\"}): " << std::boolalpha << is_in(b, {"a", "b", "c"}) << std::endl;
	
	// -- cout overloads for container (array, vector) ----------------------------
	
	std::vector<std::string>   vec = { "Hello", "vector" };
	std::array<std::string, 2> arr = { "Hello", "array" }; // todo
	
	std::cout << vec << std::endl;
	std::cout << arr << std::endl;

	return 0;
}
