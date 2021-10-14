#pragma once

#include <Eigen/Eigen>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>

namespace gazebo
{
	gazebo_msgs::ModelStates
	get_model_states();
	
	gazebo_msgs::LinkStates
	get_link_states();

	sensor_msgs::JointState
	get_joint_states();
	
	gazebo_msgs::ModelState
	get_model_state(const std::string& name, const std::string& ref = "world");
	
	gazebo_msgs::LinkState
	get_link_state(const std::string& name, const std::string& ref = "world");
	
	geometry_msgs::Pose
	get_pose(const std::string& name /* model or link */, const std::string& ref = "world");
	
	Eigen::Isometry3d
	get_tf(const std::string& from, const std::string& to);
	
	void
	spawn_model(const std::string& model, const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy = { 0, 0, 0 });
	
	void
	spawn_model(const std::string& model, const std::string& name, const geometry_msgs::Pose& pose);
	
	void
	move_model(const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy = { INFINITY, INFINITY, INFINITY});
	
	void
	remove_model(const std::string& name);
}