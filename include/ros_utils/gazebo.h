#pragma once

#include <string>
#include <array>
#include <regex>
#include <unordered_map>
#include <Eigen/Eigen>

#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/LinkStates.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>

namespace gazebo
{
	// -- simulation --------------------------------------------------------------

	void
	set_simulation(bool state);

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

	void
	spawn_model(const std::string& model, const std::string& name, const std::array<double, 3>& pos, const std::array<double, 3>& rpy = {0, 0, 0});

	void
	spawn_model(const std::string& model, const std::string& name, const geometry_msgs::Pose& pose);

	void
	delete_model(const std::string& name);

	void
	move_model(const std::string& name, const geometry_msgs::Pose& pose);

	void
	move_model(const std::string& name, const std::array<double, 3>& pos, std::array<double, 3> rpy = {INFINITY, INFINITY, INFINITY});

	// -- sensors -----------------------------------------------------------------
	// should probaby have their own file with Sensor base class - maybe even own pkg for xacro'ed models etc. (gazebo_sensors)

	struct projector // todo
	{
		// gazebo::projector().set(true);
		// gazebo::projector("/my/projector").set(true);

		projector() = default;
		projector(const std::string& name);
		projector(const std::string& name, const std::string& topic_texture);

		void
		set(bool state);

		void
		set_texture(const std::string& path);

	private:
		std::string name = "projector";
		std::string topic_texture = name + "/texture";
	};

	// ----------------------------------------------------------------------------

	struct camera
	{
		// gazebo::camera().get_img();
		// gazebo::camera("namespace/camera_name").get_img<cv::Mat>();
		// auto [P, K, H, pose] = gazebo::camera().get_info(); ???

		camera() = default;
		camera(const std::string& name);
		camera(const std::string& name, const std::string& topic_img_raw, const std::string& topic_camera_info);

		sensor_msgs::Image
		get_img();

		sensor_msgs::CameraInfo
		get_info();

		void
		set_pose(const geometry_msgs::Pose& pose);
		
		void // keep the current orientation
		set_pose(const std::array<double, 3>& pos, std::array<double, 3> rpy = {INFINITY, INFINITY, INFINITY});
		
		// geometry_msgs::Pose // todo
		// get_pose();

	private:
		std::string name = "camera";
		std::string topic_img_raw = name + "/image_raw";
		std::string topic_camera_info = name + "/camera_info";
	};

	// ----------------------------------------------------------------------------

	struct camera_stereo // todo
	{
		// auto imgs = gazebo::camera_stereo().get_imgs();
		// auto img_l = imgs["left"] or imgs[0] ;
		// auto img_l = gazebo::camera_stereo().get_imgs()["left"];
		// gazebo::camera_stereo("namespace/camera_stereo_name").get_img<cv::Mat>();

		camera_stereo() = default;
		camera_stereo(const std::string& name);
		camera_stereo(const std::string& name, const std::string& topic_img_raw, const std::string& topic_camera_info);

		template<typename T>
		struct StereoContainer
		{
			auto operator [] (size_t i) const { return imgs[i]; }
			auto operator [](const std::string& s) const { return (s == "left") ? imgs[0] : imgs[1]; };
		private:
			std::array<T, 2> imgs;
			friend struct camera_stereo;
		};

		template<typename ImageT = sensor_msgs::Image>
		StereoContainer<ImageT>
		get_imgs();

		StereoContainer<sensor_msgs::CameraInfo>
		get_info();

	private:
		std::string name = "camera_stereo";
		std::string topic_img_raw = name + "/image_raw";
		std::string topic_camera_info = name + "/camera_info";
	};

	// ----------------------------------------------------------------------------

	struct kinect // todo
	{
		kinect() = default;
		kinect(const std::string& name);

		template<typename CloudT = sensor_msgs::PointCloud2> // sensor_msgs::PointCloud or sensor_msgs::PointCloud2
		CloudT
		get_cloud();

		// void
		// get_img();

		// sensor_msgs::CameraInfo
		// get_info();

	private:
		std::string name                    = "kinect";
		std::string topic_color_img_raw     = name + "/color/image_raw";
		std::string topic_color_camera_info = name + "/color/camera_info";
		std::string topic_depth_img_raw     = name + "/depth/image_raw";
		std::string topic_depth_camera_info = name + "/depth/camera_info";
		std::string topic_depth_cloud       = name + "/depth/points";
	};
}