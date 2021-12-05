#include "ros_utils/gazebo.h"

#include <thread>
#include <atomic>
#include <chrono>

#include <ros/ros.h>
#include <ros_utils/ros.h>

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
CloudT
gazebo::kinect::get_cloud()
{
	static_assert(std::is_same<CloudT, sensor_msgs::PointCloud>::value || std::is_same<CloudT, sensor_msgs::PointCloud2>::value,
	              "Wrong type. Use sensor_msgs::PointCloud or sensor_msgs::PointCloud2.");

	if constexpr (std::is_same<CloudT, sensor_msgs::PointCloud>::value)
	{
		ros::topic::flush<sensor_msgs::PointCloud>(this->topic_depth_cloud);
		return *(ros::topic::waitForMessage<sensor_msgs::PointCloud>(this->topic_depth_cloud));
	}
	else
	if constexpr (std::is_same<CloudT, sensor_msgs::PointCloud2>::value)
	{
		ros::topic::flush<sensor_msgs::PointCloud2>(this->topic_depth_cloud);
		return *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>(this->topic_depth_cloud));
	}
}

// TEMPLATE SPECIALIZATIONS
template sensor_msgs::PointCloud gazebo::kinect::get_cloud<sensor_msgs::PointCloud>();
template sensor_msgs::PointCloud2 gazebo::kinect::get_cloud<sensor_msgs::PointCloud2>();
