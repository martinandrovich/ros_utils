#include "ros_utils/gazebo.h"

#include <thread>
#include <atomic>
#include <chrono>

#include <ros/ros.h>
#include <ros_utils/ros.h>

#include <pcl/io/io.h>
#include <pcl_conversions/pcl_conversions.h>

sensor_msgs::Image
gazebo::camera::get_img()
{
	ros::topic::flush<sensor_msgs::Image>(this->topic_img_raw);
	return *(ros::topic::waitForMessage<sensor_msgs::Image>(this->topic_img_raw));
}

sensor_msgs::CameraInfo
gazebo::camera::get_info()
{
	return *(ros::topic::waitForMessage<sensor_msgs::CameraInfo>(this->topic_camera_info));
}

void
gazebo::camera::set_pose(const geometry_msgs::Pose& pose)
{
	gazebo::move_model(this->name, pose);
}

void
gazebo::camera::set_pose(const std::array<double, 3>& pos, std::array<double, 3> rpy)
{
	gazebo::move_model(this->name, pos, rpy);
}

template<typename CloudT>
boost::shared_ptr<CloudT>
gazebo::kinect::get_cloud()
{
	static_assert(std::is_same<CloudT, sensor_msgs::PointCloud2>::value || std::is_same<CloudT, pcl::PointCloud<pcl::PointXYZ>>::value,
	              "Wrong type. Use sensor_msgs::PointCloud2 or pcl::PointCloud<pcl::PointXYZ>.");

	// get cloud
	ros::topic::flush<sensor_msgs::PointCloud2>(this->topic_depth_cloud);
	auto cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(this->topic_depth_cloud);

	if constexpr (std::is_same<CloudT, sensor_msgs::PointCloud2>::value)
	{
		return boost::make_shared<CloudT>(*cloud); // return non-const data
	}
	else
	if constexpr (std::is_same<CloudT, pcl::PointCloud<pcl::PointXYZ>>::value)
	{
		auto cloud_pcl = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		pcl::fromROSMsg(*cloud, *cloud_pcl);
		return cloud_pcl;
	}
}

// TEMPLATE SPECIALIZATIONS
template boost::shared_ptr<sensor_msgs::PointCloud2> gazebo::kinect::get_cloud<sensor_msgs::PointCloud2>();
template boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> gazebo::kinect::get_cloud<pcl::PointCloud<pcl::PointXYZ>>();
