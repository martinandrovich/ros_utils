#pragma once

#include <string>
#include <iostream>
#include <memory>
#include <regex>
#include <filesystem>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float64.h>

namespace ros
{

	bool
	init(const std::string& name);

	void
	wait_for_init();

	std::shared_ptr<ros::NodeHandle>
	make_node(const std::string& name);
}

namespace ros::param
{
	template <typename T>
	inline auto
	read(const std::string& name, T fallback)
	{
		static auto init = ros::init("rosutils_read");

		if (T param; ros::param::get(name, param))
			return param;
		else
			return T(fallback);
	};

}

namespace ros::package
{
	inline std::string
	find(std::string path)
	{
		// https://regex101.com/r/435Fpr/1
		std::smatch m;
		if (not std::regex_search(path, m, std::regex("package:\\/\\/(\\w*)\\/?(\\S*)?")))
			return "";

		auto path_pkg = ros::package::getPath(m[1]);
		if (path_pkg.empty())
			{ ROS_WARN_STREAM("Package '" << m[1].str() << "' was not found in ros::package::find()."); return ""; }

		path = path_pkg + "/" + m[2].str();
		if (not m[2].str().empty() and not std::filesystem::exists(path))
			ROS_WARN_STREAM("File '" << path << "' was not found in ros::package::find().");

		return path;
	}
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

	template <typename T>
	inline void
	flush(const std::string& topic, ros::Duration dur = ros::Duration(0.5))
	{
		ROS_INFO_STREAM("Flushing: '" << topic << "' for " << dur.toSec() << " second(s)...");

		static ros::NodeHandle nh;
		auto sub = nh.subscribe<T>(topic, 1, [&](auto& msg){ /* do nothing */ });

		auto t = ros::Time::now() + dur;
		while (ros::Time::now() < t)
			ros::spinOnce();
	}

	template <typename T>
	inline void
	publish(const std::string& topic, const T& msg, bool latch = false)
	{
		static ros::NodeHandle nh;
		static auto pub = nh.advertise<T>(topic, 1, latch);
		pub.publish(msg);
	};

}