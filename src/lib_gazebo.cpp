#include "ros_utils/gazebo.h"

#include <regex>

#include <ros_utils/ros.h>
#include <ros_utils/geometry_msgs.h>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/DeleteModel.h>

// -- simulation --------------------------------------------------------------

// ...

// -- models and states -------------------------------------------------------

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
		throw std::runtime_error("Failed service call with for model '" + name + "' in get_model_state().");
	
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
		throw std::runtime_error("Failed service call with for link '" + name + "' in get_link_state().");

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

void
gazebo::spawn_model(const std::string& model, const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy)
{
	if (not std::regex_match(name, std::regex("^" + model + "[0-9]+$")))
		throw std::invalid_argument("spawn_model(): object <name> must follow the pattern '<model>n' where n is some number (e.g. bottle1)");
	
	// SpawnModel service can be used, but XML cannot be laoded from database
	// https://pastebin.com/UTWJSScZ

	// define command
	auto cmd = std::string("rosrun gazebo_ros spawn_model") +
		" -database " + model + " -sdf" +
		" -model " + name +
		" -x " + std::to_string(pos[0]) +
		" -y " + std::to_string(pos[1]) +
		" -z " + std::to_string(pos[2]) +
		" -R " + std::to_string(rpy[0]) +
		" -P " + std::to_string(rpy[1]) +
		" -Y " + std::to_string(rpy[2]);
	
	// execute command
	system(cmd.c_str());
}

void
gazebo::spawn_model(const std::string& model, const std::string& name, const geometry_msgs::Pose& pose)
{	
	spawn_model(model, name, geometry_msgs::read_pose(pose).pos, geometry_msgs::read_pose(pose).rpy);
}

void
gazebo::delete_model(const std::string& name)
{
	static auto init = ros::init("delete_model");

	gazebo_msgs::DeleteModel srv;
	srv.request.model_name = name;
	
	if (not ros::service::call("/gazebo/delete_model", srv) or not srv.response.success)
		ROS_ERROR_STREAM("Service call failed in gazebo::delete_model(\"" << name << "\").");
}