#pragma once

#include <string>
#include <iostream>
#include <ros/ros.h>

namespace ros
{
	
	bool
	init(const std::string& name);

	ros::NodeHandle*
	make_node(const std::string& name);
}

namespace ros::param
{
	
	template <typename T>
	auto read = [](const std::string& name, T fallback)
	{	
		static auto init = ros::init("ros_utils_read");
		
		if (T param; ros::param::get(name, param))
			return param;
		else
			return T(fallback);
	};
	
}

namespace ros::topic
{
	void
	read();
	// {
		// lookup in map if topic exists, return mutexed value
		// map string : { mutex, thread }
		
		// create async listener
	
	// };

}