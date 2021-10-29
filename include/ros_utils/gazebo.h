#pragma once

#include <string>
#include <unordered_map>
#include <Eigen/Eigen>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>

namespace gazebo
{
	// -- simulation --------------------------------------------------------------
	
	void // todo
	set_simulation(bool state);
	
	void // todo
	set_projector(bool state);
	
	// -- models and states -------------------------------------------------------
	
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
	
	void // todo
	spawn_model(const std::string& model, const std::string& name, const geometry_msgs::Pose& pose);
	
	void // todo
	spawn_model(const std::string& model, const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy = { 0, 0, 0 });
	
	void // todo
	move_model(const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy = { INFINITY, INFINITY, INFINITY});
	
	void // todo
	remove_model(const std::string& name);
	
	// -- computer vision ---------------------------------------------------------
	
	template<typename ImageT = sensor_msgs::Image> // or cv::Mat 
	ImageT // todo
	get_camera_img();
	
	void
	get_camera_info(); // todo
	
	template<typename ImageT = sensor_msgs::Image> // or cv::Mat 
	std::unordered_map<std::string, ImageT> // todo
	get_stereo_camera_imgs();
	
	void // todo
	get_stereo_camera_info();

	void // todo
	get_point_cloud();
	
}