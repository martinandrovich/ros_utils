#include <ros_utils/gazebo.h>
#include <ros_utils/ros.h>

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>

gazebo_msgs::ModelStates
gazebo::get_model_states()
{
	return *(ros::topic::waitForMessage<gazebo_msgs::ModelStates>("/gazebo/model_states"));
}

geometry_msgs::Pose
gazebo::get_model_state(const std::string& name, const std::string& relative_to)
{
	static auto init = ros::init("get_model_state");

	gazebo_msgs::GetModelState srv;
	srv.request.model_name = name;
	srv.request.relative_entity_name = relative_to;

	// auto success = client.call(srv);
	ros::service::call("/gazebo/get_model_state", srv);

	return srv.response.pose;
}