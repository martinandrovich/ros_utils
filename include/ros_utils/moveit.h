#pragma once

namespace moveit
{
	// moveit_msgs::CollisionObject
	// make_mesh_cobj(const std::string& name, const std::string& planning_frame, const std::array<double, 3>& pos, const std::array<double, 4>& ori = { 1, 0, 0, 0 });

	// void
	// set_obj_color(const std::string& name, const std::string& planning_frame, const std_msgs::ColorRGBA& color) {};

	// void
	// move_base(moveit::core::RobotState& state, const std::array<double, 3>& offset, const std::string& virtual_joint_name = "world_offset");
	
	// void
	// move_base(moveit::core::RobotState& state, const geometry_msgs::Pose& offset, const std::string& virtual_joint_name = "world_offset");
	
	// std::vector<moveit_msgs::CollisionObject>
	// get_collision_objects(const std::string& planning_frame, const std::vector<std::string>& excludes = { "ur5", "camera_stereo", "openni_kinect", "ground_plane", "projector" });
}
