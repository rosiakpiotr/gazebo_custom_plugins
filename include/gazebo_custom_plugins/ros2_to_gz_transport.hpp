/* Desc: ROS2 and Gazebo Classic transport library bridge plugin.
 * Author: Piotr Rosiak
 */

#ifndef _GAZEBO_ROS2_TO_GZ_TRANSPORT_PLUGIN_HH_
#define _GAZEBO_ROS2_TO_GZ_TRANSPORT_PLUGIN_HH_

#include "gazebo/transport/TransportTypes.hh"

#include <rclcpp/publisher.hpp>
#include <rclcpp/publisher_base.hpp>
#include <rclcpp/subscription_base.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>


#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>

#include <builtin_interfaces/msg/time.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

#include <gazebo_custom_plugins/CommandPitchAngle.pb.h>

namespace gazebo
{
class GAZEBO_VISIBLE ROS2ToGzTransportBridge : public ModelPlugin
{
public:
	ROS2ToGzTransportBridge();
	virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

private:

	transport::NodePtr transport_node;
	gazebo_ros::Node::SharedPtr ros_node;

	sdf::ElementPtr sdf;
	physics::ModelPtr model;
	std::vector<event::ConnectionPtr> connections;

	std::vector<rclcpp::PublisherBase::SharedPtr> _rclcpp_pubs;
	std::vector<rclcpp::SubscriptionBase::SharedPtr> _rclcpp_subs;

	// std::vector<transport::SubscriberPtr> _transport_subs;
	std::vector<transport::PublisherPtr> _transport_pubs;

	common::Time lastUpdateTime;
};
} // namespace gazebo
#endif