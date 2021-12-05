#pragma once

#include <string>
#include <memory>
#include <filesystem>

#include <pcl/io/pcd_io.h>
#include <ros_utils/ros.h>

namespace pcl
{
	template<typename PointT>
	std::shared_ptr<pcl::PointCloud<PointT>>
	load_cloud(std::string path)
	{
		// assert extension
		if (std::filesystem::path(path).extension() != ".pcd")
			throw std::invalid_argument("File is not a .pcd file in pcl::load_file().");

		// deduce package if specified using 'package://'
		if (path.substr(0,10) == "package://")
			path = ros::package::find(path);

		// load point cloud
		auto cloud = std::make_shared<pcl::PointCloud<PointT>>();
		pcl::io::loadPCDFile<PointT>(path, *cloud);
		return cloud;
	}
}