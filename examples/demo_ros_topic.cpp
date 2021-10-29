#include <string>

#include <ros/ros.h>
#include <ros_utils/ros.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_ros_topic");
	ros::NodeHandle nh;
	
	ros::Rate lr(1); // Hz
	std_msgs::Float64 msg;
	while (ros::ok())
	{
		msg.data = 3.14;
		// ros::topic::publish<std_msgs::Float64>("/demo_ros_topic", msg);
		ros::topic::publish("/demo_ros_topic", msg);
		lr.sleep();
	}

	return 0;
}
