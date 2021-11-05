#pragma once

#include <string>

#include <geometry_msgs/Pose.h>
#include <moveit/planning_scene/planning_scene.h>
#include <geometric_shapes/shape_operations.h>
#include <moveit_msgs/CollisionObject.h>

namespace moveit
{
	moveit_msgs::CollisionObject
	make_mesh_cobj(
		const std::string& name,
		const std::string& path, // path to .dae file
		const std::string& planning_frame,
		const geometry_msgs::Pose& pose
	);

	std::vector<moveit_msgs::CollisionObject>
	get_gazebo_cobjs(
		const std::string& planning_frame,
		const std::vector<std::string>& exclude = { "ground_plane" }
	);

	moveit_msgs::PlanningScene
	add_cobjs(
		planning_scene::PlanningScenePtr& planning_scene,
		const std::vector<moveit_msgs::CollisionObject>& cobjs,
		bool remove_attached_cobjs = true
	);

	void
	set_floating_jnt_pose(
		moveit::core::RobotState& state,
		const std::string& joint,
		const geometry_msgs::Pose& pose
	);

	// void
	// set_obj_color(const std::string& name, const std::string& planning_frame, const std_msgs::ColorRGBA& color) {};
}
