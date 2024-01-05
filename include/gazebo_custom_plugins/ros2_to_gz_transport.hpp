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
#include <px4_msgs/msg/vpp_state.hpp>

#include <gazebo/common/Plugin.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>

#include <builtin_interfaces/msg/time.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>


#include "gazebo_custom_plugins/VppState.pb.h"


namespace gazebo
{

/**
 * \brief Obtains a parameter from sdf.
 * \param[in] sdf Pointer to the sdf object.
 * \param[in] name Name of the parameter.
 * \param[out] param Param Variable to write the parameter to.
 * \param[in] default_value Default value, if the parameter not available.
 * \param[in] verbose If true, gzerror if the parameter is not available.
 */
template<class T>
bool getSdfParam(sdf::ElementPtr sdf, const std::string &name, T &param, const T &default_value, const bool &verbose =
			 false)
{
	if (sdf->HasElement(name)) {
		param = sdf->GetElement(name)->Get<T>();
		return true;

	} else {
		param = default_value;
	}

	return false;
}

typedef const boost::shared_ptr<const mav_msgs::msgs::VppState> VppStatePtr;

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

	rclcpp::Publisher<px4_msgs::msg::VppState>::SharedPtr _vpp_state_ros2_pub {};

	std::vector<transport::SubscriberPtr> _transport_subs;
	std::vector<transport::PublisherPtr> _transport_pubs;

	common::Time lastUpdateTime;
};
} // namespace gazebo
#endif