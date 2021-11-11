#include "ros_utils/moveit.h"

#include <ros/ros.h>
#include <ros_utils/gazebo.h>
#include <ros_utils/std.h>

#include <geometry_msgs/Pose.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shape_operations.h>

moveit_msgs::CollisionObject
moveit::make_mesh_cobj(const std::string& name, const std::string& path, const std::string& planning_frame, const geometry_msgs::Pose& pose)
{
	// make a collision object from a 3D mesh

	// construct mesh
	// https://answers.ros.org/question/246467/moveit-attach-object-error/
	shape_msgs::Mesh mesh;
	shapes::Mesh* mesh_ptr;
	shapes::ShapeMsg shape_msg;

	if (mesh_ptr = shapes::createMeshFromResource(path); mesh_ptr == nullptr)
		throw std::runtime_error("The mesh file '" + path + "' could not be located.");

	shapes::constructMsgFromShape(mesh_ptr, shape_msg);
	mesh = boost::get<shape_msgs::Mesh>(shape_msg);

	// construct collision object
	moveit_msgs::CollisionObject co;
	co.id = name;
	co.meshes.resize(1);
	co.mesh_poses.resize(1);
	co.meshes[0] = mesh;
	co.header.frame_id = planning_frame;
	co.mesh_poses[0].position = pose.position;
	co.mesh_poses[0].orientation = pose.orientation;
	co.operation = co.ADD;

	return co;
}

std::vector<moveit_msgs::CollisionObject>
moveit::get_gazebo_cobjs(const std::string& planning_frame, const std::vector<std::string>& exclude)
{
	// get model states from Gazebo
	auto model_states = gazebo::get_model_states();

	// create a vector of collision objects by iterating all non-excluded models
	std::vector<moveit_msgs::CollisionObject> vec_cobjs;
	for (auto i = 0; i < model_states.name.size(); i++)
	{
		if (auto model = model_states.name[i]; not is_in(model, exclude))
		{
			// get model name (remove any digits from string, e.g. bottle3 -> bottle)
			auto name = model;
			name.erase(std::remove_if(name.begin(), name.end(), &isdigit), name.end());

			// get pose of model
			const auto& pose = model_states.pose[i];

			// get path to .dae model
			ROS_WARN_ONCE("moveit::get_gazebo_cobjs() expects 3D model of <name> to be located at 'package://<pkg>/models/<name>/<name>.dae'.");
			const auto pkg = std::string("rovi_models");
			const auto path = "package://" + pkg + "/models/" + name + "/" + name + ".dae";

			// construct and add colision object
			const auto cobj = moveit::make_mesh_cobj(model, path, planning_frame, pose);
			vec_cobjs.push_back(cobj);
		}
	}

	return vec_cobjs;
}

moveit_msgs::PlanningScene
moveit::add_cobjs(planning_scene::PlanningScenePtr& planning_scene, const std::vector<moveit_msgs::CollisionObject>& cobjs, bool remove_attached_cobjs)
{
	static moveit_msgs::PlanningScene planning_scene_msg;
	planning_scene->getPlanningSceneMsg(planning_scene_msg);

	planning_scene_msg.world.collision_objects = cobjs;

	planning_scene_msg.world.collision_objects[0].id;
	planning_scene_msg.robot_state.attached_collision_objects[0].object.id;

	if (remove_attached_cobjs)
	{
		planning_scene_msg.robot_state.attached_collision_objects = {};
	}
	else
	{
		// keep attached objects
		auto& a = planning_scene_msg.world.collision_objects;
		auto& b = planning_scene_msg.robot_state.attached_collision_objects;
		
		// remove any elements from a that are also in b
		// matching by their id
		a.erase(std::remove_if(a.begin(), a.end(),
			[&](auto& elem_a){ return std::find_if(b.begin(), b.end(), [&](auto& elem_b){ return elem_a.id == elem_b.object.id; }) != b.end(); }
		), end(a));
	}

	planning_scene->setPlanningSceneMsg(planning_scene_msg);

	return planning_scene_msg;
}

void
moveit::set_floating_jnt_pose(moveit::core::RobotState& state, const std::string& joint, const geometry_msgs::Pose& pose)
{
	// RobotState has a floating virtual joint, which can be set
	// http://docs.ros.org/en/melodic/api/moveit_core/html/classmoveit_1_1core_1_1RobotState.html#ad08c92a61d43013714ec3894cd67a297

	// ROS_INFO_STREAM(state.getRobotModel()->getRootJointName());
	// ROS_INFO_STREAM(state.getRobotModel()->getRootJoint()->getTypeName()); // Must be 'Floating'

	const auto jnt_type = state.getRobotModel()->getRootJoint()->getTypeName();
	if (jnt_type != "Floating")
		throw std::runtime_error("Joint type of '" + joint + "' must be 'Floating'.");

	// how to know where "/trans_x" comes from
	// std::cout << state.getStateTreeString() << std::endl;

	state.setVariablePositions({
		{ joint + "/trans_x", pose.position.x },
		{ joint + "/trans_y", pose.position.y },
		{ joint + "/trans_z", pose.position.z },
		{ joint + "/rot_x", pose.orientation.x },
		{ joint + "/rot_y", pose.orientation.y },
		{ joint + "/rot_z", pose.orientation.z },
		{ joint + "/rot_w", pose.orientation.w }
	});
}