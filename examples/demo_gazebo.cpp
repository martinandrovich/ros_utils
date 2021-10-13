#include <string>

#include <ros/ros.h>
#include <ros_utils/gazebo.h>

#include <variant>
#include <vector>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_gazebo");
	ros::NodeHandle nh;
	
	// first run
	// roslaunch gazebo_ros shapes_world.launch
	
	// get model states
	auto model_states = gazebo::get_model_states();
	std::cout << "model states:\n\n" << model_states << std::endl;

	// get model pose
	auto name = model_states.name[1];
	auto model_pose = gazebo::get_model_state(name);
	std::cout << "pose of '" << name << "':\n\n" << model_pose << std::endl;

	return 0;
}
