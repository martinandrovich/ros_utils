#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros_utils/gazebo.h>
#include <ros_utils/std.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "demo_gazebo_sensors");
	ros::NodeHandle nh;

	// first run
	// roslaunch gazebo_ros shapes_world.launch

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("spawn sensors");

	// spawn sensors (with default topics etc.)
	// these are defined in rovi_models pkg

	gazebo::spawn_model("camera", "camera", { 3.0, 0.0, 1.5 }, { 0, M_PI_4, M_PI });
	gazebo::spawn_model("kinect", "kinect", { 3.0, 0.0, 2.5 }, { 0, M_PI_4, M_PI });

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("test kinect");

	{ // get cloud and cout info
		auto cloud = gazebo::kinect().get_cloud<sensor_msgs::PointCloud2>();
		std::cout << "cloud->data.size(): " << cloud->data.size() << std::endl;
		std::cout << "cloud->width: " << cloud->width << std::endl;
		std::cout << "cloud->height: " << cloud->height << std::endl << std::endl;

		auto cloud2 = gazebo::kinect().get_cloud<sensor_msgs::PointCloud2>();
		std::cout << "cloud2->data.size(): " << cloud2->data.size() << std::endl;
		std::cout << "cloud2->width: " << cloud2->width << std::endl;
		std::cout << "cloud2->height: " << cloud2->height << std::endl;
	}

	// ------------------------------------------------------------------------------

	ENTER_TO_CONTINUE("test camera");

	{ // get and show image
		auto img = gazebo::camera().get_img();
		auto mat = cv_bridge::toCvCopy(img)->image;
		cv::imshow("camera", mat);
		cv::waitKey();
		cv::destroyAllWindows();
	}

	ENTER_TO_CONTINUE("move camera");

	// move camera (keep current orientation)
	gazebo::camera().set_pose({5, 0, 2});

	{ // get and show image
		auto img = gazebo::camera().get_img();
		auto mat = cv_bridge::toCvCopy(img)->image;
		cv::imshow("camera", mat);
		cv::waitKey();
		cv::destroyAllWindows();
	}

	// ------------------------------------------------------------------------------

	return 0;
}
