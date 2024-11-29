#include "gazebo_custom_plugins/ros2_to_gz_transport.hpp"
#include "gazebo_custom_plugins/CommandPitchAngle.pb.h"

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

	std::string pitch_angle_pub_topic_ = "";
	std::string vpp_state_sub_topic_ = "";

	getSdfParam<std::string>(_sdf, "pitchAnglePubTopic", pitch_angle_pub_topic_, pitch_angle_pub_topic_);
	getSdfParam<std::string>(_sdf, "vppStateSubTopic", vpp_state_sub_topic_, vpp_state_sub_topic_);

	ros_node = gazebo_ros::Node::Get(sdf);
	transport_node = transport::NodePtr(new transport::Node());
	transport_node->Init();

	auto pub =
		this->transport_node->Advertise<mav_msgs::msgs::CommandPitchAngle>(pitch_angle_pub_topic_);
	auto sub = ros_node->create_subscription<std_msgs::msg::Float32MultiArray>("/vpp/pitch_angle",
	10, [pub](std_msgs::msg::Float32MultiArray::SharedPtr message) {
		mav_msgs::msgs::CommandPitchAngle transport_msg;

		// std::cout << "Callback: ";

		for (const float &pitch : message->data) {
			transport_msg.add_pitch_angle(pitch);
			// std::cout << pitch << "\t";
		}

		// std::cout << std::endl;;
		pub->Publish(transport_msg);
	});
	_rclcpp_subs.push_back(sub);
	_transport_pubs.push_back(pub);

	_vpp_state_ros2_pub = ros_node->create_publisher<px4_msgs::msg::VppState>("/vpp/vpp_state", 9);
	auto vpp_state_sub = transport_node->Subscribe<mav_msgs::msgs::VppState>(vpp_state_sub_topic_, [this](
	VppStatePtr & msg) {
		px4_msgs::msg::VppState vpp_state_ros2_msg;

		vpp_state_ros2_msg.thrust = msg->thrust();
		vpp_state_ros2_msg.torque = msg->torque();
		vpp_state_ros2_msg.aoa = msg->aoa();
		vpp_state_ros2_msg.pitch = msg->pitch();
		vpp_state_ros2_msg.rpm = msg->rpm();
		vpp_state_ros2_msg.airspeed = msg->airspeed();
		vpp_state_ros2_msg.advance_ratio = msg->advance_ratio();
		vpp_state_ros2_msg.motor_index = msg->motor_index();

		_vpp_state_ros2_pub->publish(vpp_state_ros2_msg);

		// std::cout << "VppState: " << msg->DebugString() << std::endl;
	});

	_transport_subs.push_back(vpp_state_sub);

}