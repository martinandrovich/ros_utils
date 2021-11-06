#include <ros_utils/geometry_msgs.h>
#include <eigen_conversions/eigen_msg.h>

geometry_msgs::Pose
geometry_msgs::make_pose(const std::array<double, 3>& pos, const std::array<double, 3>& rpy)
{
	geometry_msgs::Pose pose;

	pose.position.x = pos[0];
	pose.position.y = pos[1];
	pose.position.z = pos[2];

	Eigen::Quaterniond quat;

	quat = Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX()) *
		   Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
		   Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) ;

	pose.orientation.w = quat.w();
	pose.orientation.x = quat.x();
	pose.orientation.y = quat.y();
	pose.orientation.z = quat.z();

	return pose;
}

geometry_msgs::Pose
geometry_msgs::make_pose(const std::array<double, 6>& pose)
{
	return geometry_msgs::make_pose
	(
		{ pose[0], pose[1], pose[2] },
		{ pose[3], pose[4], pose[5] }
	);
}

geometry_msgs::Pose
geometry_msgs::make_pose(const std::array<double, 3>& pos, const Eigen::Quaternion<double>& ori)
{
	geometry_msgs::Pose pose;

	pose.position.x = pos[0];
	pose.position.y = pos[1];
	pose.position.z = pos[2];

	pose.orientation.w = ori.w();
	pose.orientation.x = ori.x();
	pose.orientation.y = ori.y();
	pose.orientation.z = ori.z();

	return pose;
}

geometry_msgs::Pose
geometry_msgs::make_pose(const Eigen::Isometry3d& T)
{
	static geometry_msgs::Pose pose;
	tf::poseEigenToMsg(T, pose);

	return pose;
}