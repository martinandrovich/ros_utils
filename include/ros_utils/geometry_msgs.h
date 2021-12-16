#pragma once

#include <array>
#include <Eigen/Eigen>

#include <geometry_msgs/Pose.h>

namespace geometry_msgs
{

	// -- geometry_msgs::Pose------------------------------------------------------

	geometry_msgs::Pose
	make_pose(const std::array<double, 3>& pos, const std::array<double, 3>& rpy);

	geometry_msgs::Pose
	make_pose(const std::array<double, 6>& pose);

	geometry_msgs::Pose
	make_pose(const std::array<double, 3>& pos, const Eigen::Quaternion<double>& ori);
	
	geometry_msgs::Pose
	make_pose(const Eigen::Isometry3d& T);

	inline auto
	read_pose(const geometry_msgs::Pose& pose)
	{
		struct
		{
			std::array<double, 3> pos;
			std::array<double, 3> rpy;
			Eigen::Quaternion<double> ori;
		} pose_struct;

		pose_struct.ori  = Eigen::Quaternion<double>({ pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z });
		const auto euler = pose_struct.ori.toRotationMatrix().eulerAngles(0, 1, 2);

		pose_struct.pos = { pose.position.x, pose.position.y, pose.position.z };
		pose_struct.rpy = { euler[0], euler[1], euler[2] };

		return pose_struct;
	}
	
	inline double
	diff_xy(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b)
	{
		return std::sqrt(std::pow(a.position.x - b.position.x, 2) + std::pow(a.position.y - b.position.y, 2));
	}
}
