#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros_utils/moveit.h>
#include <ros_utils/geometry_msgs.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_moveit");
	ros::NodeHandle nh;
	
	// this example uses the rovi_models package for locating .dae files
	// first start a Gazebo workcell with some objects
	// roslaunch rovi_system workcell.launch objects:=true

	// make a single collision object from a mesh
	auto name = std::string("bottle");
	auto path = "package://rovi_models/models/" + name + "/" + name + ".dae";
	auto pose = geometry_msgs::make_pose({ 1.0, 0, 0 });
	auto cobj = moveit::make_mesh_cobj(name, path, "planning_frame", pose);

	std::cout << "cobj.id: " << cobj.id << "\n"
	          << "cobj.meshes[0].triangles.size(): " << cobj.meshes[0].triangles.size() << "\n"
	          << "cobj.header.frame_id: " << cobj.header.frame_id << "\n"
	          << "cobj.mesh_poses[0]: " << cobj.mesh_poses[0] << "\n"
	          << std::endl;

	// get a vector of collission objects from Gazebo (excluding some)
	auto exclude  = std::vector<std::string>{ "ur5", "camera_stereo", "openni_kinect", "ground_plane", "projector" };
	auto vec_cobj = moveit::get_gazebo_cobjs("planning_frame", exclude);
	
	std::cout << "vec_cobj.size(): " << vec_cobj.size() << "\n"
	          << "vec_cobj[0].id: " << vec_cobj[0].id << "\n"
	          << std::endl;

	return 0;
}
