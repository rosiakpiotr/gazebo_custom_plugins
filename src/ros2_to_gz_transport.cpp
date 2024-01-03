#include "gazebo_custom_plugins/ros2_to_gz_transport.hpp"
#include "gazebo/common/Console.hh"
#include "gazebo_custom_plugins/CommandPitchAngle.pb.h"
#include <iostream>
#include <std_msgs/msg/detail/float32__struct.hpp>
#include <string>

using namespace gazebo;

GZ_REGISTER_MODEL_PLUGIN(ROS2ToGzTransportBridge)

/////////////////////////////////////////////////
ROS2ToGzTransportBridge::ROS2ToGzTransportBridge()
{
	gzmsg << "ROS2ToGzTransportBridge initializing\n";
}

/////////////////////////////////////////////////
void ROS2ToGzTransportBridge::Load(physics::ModelPtr _model,
				   sdf::ElementPtr _sdf)
{
	this->model = _model;
	this->sdf = _sdf;

	ros_node = gazebo_ros::Node::Get(sdf);
	transport_node = transport::NodePtr(new transport::Node());
	transport_node->Init();


	auto dummy_pub = ros_node->create_publisher<std_msgs::msg::Float32MultiArray>("/plugin_is_alive", 10);
	_rclcpp_pubs.push_back(dummy_pub);

	auto pub =
		this->transport_node->Advertise<mav_msgs::msgs::CommandPitchAngle>("/gazebo/default/iris/gazebo/command/pitch_angle");
	auto sub = ros_node->create_subscription<std_msgs::msg::Float32MultiArray>("/vpp/pitch_angle",
	10, [pub](std_msgs::msg::Float32MultiArray::SharedPtr message) {
		mav_msgs::msgs::CommandPitchAngle transport_msg;

		std::cout << "Callback: ";

		for (const float &pitch : message->data) {
			transport_msg.add_pitch_angle(pitch);
			std::cout << pitch << "\t";
		}

		std::cout << std::endl;;
		pub->Publish(transport_msg);
	});
	_rclcpp_subs.push_back(sub);
	_transport_pubs.push_back(pub);

}