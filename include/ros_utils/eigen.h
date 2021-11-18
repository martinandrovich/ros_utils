#pragma once

#include <array>
#include <Eigen/Eigen>

#include <geometry_msgs/Pose.h>

namespace Eigen
{

	Eigen::Isometry3d
	make_tf(const std::array<double, 3>& pos, const Eigen::Vector3d& axis, double theta);

	// Eigen::Isometry3d // todo
	// make_tf(const std::array<double, 3>& pos, const std::array<double, 3>& axis, double theta);

	Eigen::Isometry3d
	make_tf(const std::array<double, 3>& pos, const std::array<double, 3>& rpy = {0, 0, 0});

	Eigen::Isometry3d
	make_tf(const geometry_msgs::Pose& pose);
}