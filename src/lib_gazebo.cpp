#include "ros_utils/gazebo.h"

#include <regex>

#include <ros_utils/ros.h>
#include <ros_utils/geometry_msgs.h>
#include <ros_utils/std.h>

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <eigen_conversions/eigen_msg.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/DeleteModel.h>
#include <gazebo_msgs/SetModelState.h>

// -- simulation --------------------------------------------------------------

void
gazebo::set_simulation(bool state)
{
	std_srvs::Empty srv;
	std::string srv_name = (state) ? "/gazebo/unpause_physics" : "/gazebo/pause_physics";

	if (not ros::service::waitForService(srv_name, ros::Duration(5.0)))
		throw std::runtime_error("Timed out after 5 sec while waiting in gazebo::set_simulation().");

	if (not ros::service::call(srv_name, srv))
		throw std::runtime_error("Failed service call to set simulation in gazebo::set_simulation().");
}

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
	gazebo_msgs::GetModelState srv;
	srv.request.model_name = name;
	srv.request.relative_entity_name = ref;

	if (not ros::service::call("/gazebo/get_model_state", srv) or not srv.response.success)
		throw std::runtime_error("Failed service call with for model '" + name + "' in gazebo::get_model_state().");

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
	gazebo_msgs::GetLinkState srv;
	srv.request.link_name = name;
	srv.request.reference_frame = ref;

	if (not ros::service::call("/gazebo/get_link_state", srv) or not srv.response.success)
		throw std::runtime_error("Failed service call with for link '" + name + "' in gazebo::get_link_state().");

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
	if (not is_in(name, {"camera", "projector", "camera_stereo", "kinect"}) and not std::regex_match(name, std::regex("^" + model + "[0-9]+$")))
		throw std::invalid_argument("spawn_model(): object <name> must follow the pattern '<model>n' where n is an integer (e.g. bottle1)");

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
	gazebo_msgs::DeleteModel srv;
	srv.request.model_name = name;

	if (not ros::service::call("/gazebo/delete_model", srv) or not srv.response.success)
		ROS_ERROR_STREAM("Could not delete model '" << name << "' in gazebo::delete_model().");
}

void
gazebo::move_model(const std::string& name, const geometry_msgs::Pose& pose)
{
	gazebo_msgs::SetModelState srv;
	srv.request.model_state.model_name = name;
	srv.request.model_state.reference_frame = "world";
	srv.request.model_state.pose = pose;

	if (not ros::service::call("/gazebo/set_model_state", srv) or not srv.response.success)
		throw std::runtime_error("Failed service call with for model '" + name + "' in gazebo::move_model().");
}
void
gazebo::move_model(const std::string& name, const std::array<double, 3>& pos, std::array<double, 3> rpy)
{
	// keep current orientation
	if (std::any_of(rpy.begin(), rpy.end(), [](auto& x){ return x == INFINITY; }))
		rpy = geometry_msgs::read_pose(gazebo::get_pose(name)).rpy;
	
	gazebo::move_model(name, geometry_msgs::make_pose(pos, rpy));
}