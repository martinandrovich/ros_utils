#include <ros_utils/gazebo.h>
#include <ros_utils/ros.h>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>

gazebo_msgs::ModelStates
gazebo::get_model_states()
{
	return *(ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states"));
}

gazebo_msgs::LinkStates
gazebo::get_link_states()
{
	return *(ros::topic::waitForMessage<gazebo_msgs::LinkStates>("/gazebo/link_states"));
}

gazebo_msgs::ModelState
gazebo::get_model_state(const std::string& name, const std::string& ref)
{
	static auto init = ros::init("get_model_state");

	gazebo_msgs::GetModelState srv;
	srv.request.model_name = name;
	srv.request.relative_entity_name = ref;
	
	if (not ros::service::call("/gazebo/get_model_state", srv) or not srv.response.success)
		{ ROS_ERROR_STREAM("Service call failed in gazebo::get_model_state(\"" << name << "\")."); throw; }
	
	gazebo_msgs::ModelState model_state;
	model_state.model_name = name;
	model_state.reference_frame = ref;
	model_state.pose = srv.response.pose;
	model_state.twist = srv.response.twist;

	return model_state;
}

gazebo_msgs::LinkState
gazebo::get_link_state(const std::string& name, const std::string& ref)
{
	static auto init = ros::init("/gazebo/get_link_state");

	gazebo_msgs::GetLinkState srv;
	srv.request.link_name = name;
	srv.request.reference_frame = ref;
	
	if (not ros::service::call("/gazebo/get_link_state", srv) or not srv.response.success)
		{ ROS_ERROR_STREAM("Service call failed in gazebo::get_link_state(\"" << name << "\")."); throw; }

	return srv.response.link_state;
}

geometry_msgs::Pose
gazebo::get_pose(const std::string& name, const std::string& ref)
{
	if (name.find("::") != std::string::npos) // contains '::'
		return get_link_state(name, ref).pose;
	else
		return get_model_state(name, ref).pose;
}

Eigen::Isometry3d
gazebo::get_tf(const std::string& from, const std::string& to)
{
	static Eigen::Isometry3d tf;
	auto pose = gazebo::get_pose(to, from);
	tf::poseMsgToEigen(pose, tf);

	return tf;
}