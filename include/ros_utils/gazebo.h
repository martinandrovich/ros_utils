#pragma once

#include <sensor_msgs/JointState.h>
#include <gazebo_msgs/LinkStates.h>
#include <gazebo_msgs/ModelStates.h>

namespace gazebo
{
	gazebo_msgs::ModelStates
	get_model_states();
	
	gazebo_msgs::LinkStates
	get_link_states();

	sensor_msgs::JointState
	get_joint_states();
	
	geometry_msgs::Pose
	// geometry_msgs::PoseTwist
	get_model_state(const std::string& name, const std::string& relative_to = "");
	
	auto
	get_state(const std::string& name);
	
	void
	spawn_model(const std::string& model, const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy = { 0, 0, 0 });
	
	void
	spawn_model(const std::string& model, const std::string& name, const geometry_msgs::Pose& pose);
	
	void
	move_model(const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy = { INFINITY, INFINITY, INFINITY});
	
	void
	remove_model(const std::string& name);
}