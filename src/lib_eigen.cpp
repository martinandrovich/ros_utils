#include "ros_utils/eigen.h"

#include <ros_utils/geometry_msgs.h>

Eigen::Isometry3d
Eigen::make_tf(const std::array<double, 3>& pos, const Eigen::Vector3d& axis, double theta)
{
	Eigen::Isometry3d T = Eigen::Translation3d(pos[0], pos[1], pos[2]) *
	                      Eigen::AngleAxisd(theta, axis);
	return T;
}

Eigen::Isometry3d
Eigen::make_tf(const std::array<double, 3>& pos, const std::array<double, 3>& rpy)
{
	Eigen::Isometry3d T = Eigen::Translation3d(pos[0], pos[1], pos[2]) *
	                      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) *
	                      Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
	                      Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ());

	return T;
}

Eigen::Isometry3d
Eigen::make_tf(const geometry_msgs::Pose& pose)
{
	return make_tf(geometry_msgs::read_pose(pose).pos, geometry_msgs::read_pose(pose).rpy);
}

Eigen::MatrixXd
Eigen::pseudo_inverse(const Eigen::MatrixXd& M, double lambda)
{
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::JacobiSVD<Eigen::MatrixXd>::SingularValuesType sing_vals = svd.singularValues();
	Eigen::MatrixXd S = M;  // copying the dimensions of M, its content is not needed.
	S.setZero();

	for (size_t i = 0; i < sing_vals.size(); ++i)
		S(i, i) = (sing_vals(i)) / (sing_vals(i) * sing_vals(i) + lambda * lambda);

	return Eigen::MatrixXd(svd.matrixV() * S.transpose() * svd.matrixU().transpose());;
}