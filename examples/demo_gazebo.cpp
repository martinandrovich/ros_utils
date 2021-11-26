#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros_utils/gazebo.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_gazebo");
	ros::NodeHandle nh;

	// first run
	// roslaunch gazebo_ros shapes_world.launch
	
	std::cout << "Please run 'roslaunch gazebo_ros shapes_world.launch' first; press [ENTER] to continue..." << std::endl;
	std::cin.get();

	// ------------------------------------------------------------------------------

	// get model states
	auto model_states = gazebo::get_model_states();
	std::cout << "model states:\n\n" << model_states << std::endl;

	// get link states
	auto link_states = gazebo::get_link_states();
	std::cout << "link states:\n\n" << link_states << std::endl;

	// ------------------------------------------------------------------------------

	// get model state
	auto model = model_states.name[2];
	auto model_state = gazebo::get_model_state(model);
	std::cout << "pose of '" << model << "':\n\n" << model_state.pose << std::endl;

	auto model_other = model_states.name[1];
	auto model_state_rel = gazebo::get_model_state(model, model_other);
	std::cout << "pose of '" << model << "' relative to '" << model_other << "': \n\n" << model_state_rel.pose << std::endl;

	// ------------------------------------------------------------------------------

	// get link state
	auto link = link_states.name[2];
	auto link_state = gazebo::get_link_state(link, "world");
	std::cout << "pose of '" << link << "':\n\n" << link_state.pose << std::endl;

	auto link_other = link_states.name[1];
	auto link_state_rel = gazebo::get_link_state(link, link_other);
	std::cout << "pose of '" << link << "' relative to '" << link_other << "': \n\n" << link_state_rel.pose << std::endl;

	// ------------------------------------------------------------------------------
	
	// poses (automatic name deduction)
	std::cout << "pose of '" << model << "':\n\n" << gazebo::get_pose(model) << std::endl;
	std::cout << "pose of '" << link << "':\n\n" << gazebo::get_pose(link) << std::endl;
	
	// transform
	std::cout << "gazebo::get_tf(\"world\", model):\n" << gazebo::get_tf("world", model).matrix() << std::endl;
	
	// ------------------------------------------------------------------------------
	
	// sensors

	// spawn sensors
	// ...
	
	// projector
	// ...
	
	// camera
	// ...
	
	// camera_stereo
	// ...
	
	// kinect
	// ...

	return 0;
}
