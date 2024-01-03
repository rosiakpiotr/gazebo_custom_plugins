/* Desc: A basic gimbal controller
 * Author: Piotr Rosiak
 */

#ifndef _GAZEBO_MOTOR_MODEL_VPP_PLUGIN_HH_
#define _GAZEBO_MOTOR_MODEL_VPP_PLUGIN_HH_

#include <string>
#include <vector>
#include <memory>
#include <optional>
#include <iostream>
#include <chrono>

#include <gazebo/common/PID.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/util/system.hh>
#include <gazebo/sensors/sensors.hh>
#include <ignition/math.hh>

#include <builtin_interfaces/msg/time.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <gazebo_ros/utils.hpp>

using namespace std::chrono_literals;

#define TARGET_ANGLES_TOPIC "/gimbal/setpoint"
#define JOINT_STATE_TOPIC "/gimbal/state"

#define YAW_SETPOINT_FITLER_TIME_CONSTANT 0.4
#define PITCH_SETPOINT_FITLER_TIME_CONSTANT 0.4

// Borrowed from "common.h" at "PX4-Autopilot\Tools\simulation\gazebo-classic\sitl_gazebo-classic\include"
class FirstOrderFilter
{
public:
    FirstOrderFilter(double T) : T(T), state(0)
    {
    }

    double updateFilter(double u, double Ts)
    {
        double alpha = exp(-Ts / T);
        state = alpha * state + (1 - alpha) * u;
        return state;
    }

protected:
    double T;
    double state;
};

namespace gazebo
{
    class GAZEBO_VISIBLE GazeboMotorModelVpp : public ModelPlugin
    {
    public:
        GazeboMotorModelVpp();
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void Init();

    private:
        void OnUpdate();

        // ROS RELATED
        // ---------------------------------------------
        void initROS();

        gazebo_ros::Node::SharedPtr rosNode;
        // rclcpp::Publisher<gimbal_interfaces::msg::GimbalAngles>::SharedPtr jointStatePub;
        rclcpp::TimerBase::SharedPtr jointStatePubTimer;
        // rclcpp::Subscription<gimbal_interfaces::msg::GimbalAngles>::SharedPtr cmdAnglesSub;
        // ---------------------------------------------

        void setPIDForAxis(std::string axis_name, common::PID);
        void loadPIDsFromSDF();

        sdf::ElementPtr sdf;
        physics::ModelPtr model;
        std::vector<event::ConnectionPtr> connections;

        physics::JointPtr yawJoint;
        physics::JointPtr rollJoint;
        physics::JointPtr pitchJoint;

        FirstOrderFilter pitchSetpointFilter;
        FirstOrderFilter yawSetpointFilter;

        double rollSetpoint{0.0};
        double pitchSetpoint{-0.78539};
        double yawSetpoint{0.0};

        common::PID pitchPid;
        common::PID rollPid;
        common::PID yawPid;
        common::Time lastUpdateTime;
    };
}
#endif

/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <rotors_model/motor_model.hpp>
#include "CommandMotorSpeed.pb.h"
#include "gazebo/transport/transport.hh"
#include "gazebo/msgs/msgs.hh"
#include "MotorSpeed.pb.h"
#include "Float.pb.h"
#include "Wind.pb.h"

#include "common.h"

namespace turning_direction
{
    const static int CCW = 1;
    const static int CW = -1;
}

namespace gazebo
{
    // Default values
    static const std::string kDefaultNamespace = "";
    static const std::string kDefaultCommandSubTopic = "/gazebo/command/motor_speed";
    static const std::string kDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num";
    static const std::string kDefaultMotorVelocityPubTopic = "/motor_speed";
    std::string wind_sub_topic_ = "/world_wind";

    typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
    typedef const boost::shared_ptr<const physics_msgs::msgs::Wind> WindPtr;

    /*
    // Protobuf test
    typedef const boost::shared_ptr<const mav_msgs::msgs::MotorSpeed> MotorSpeedPtr;
    static const std::string kDefaultMotorTestSubTopic = "motors";
    */

    // Set the max_force_ to the max double value. The limitations get handled by the FirstOrderFilter.
    static constexpr double kDefaultMaxForce = std::numeric_limits<double>::max();
    static constexpr double kDefaultMotorConstant = 8.54858e-06;
    static constexpr double kDefaultMomentConstant = 0.016;
    static constexpr double kDefaultTimeConstantUp = 1.0 / 80.0;
    static constexpr double kDefaultTimeConstantDown = 1.0 / 40.0;
    static constexpr double kDefaulMaxRotVelocity = 838.0;
    static constexpr double kDefaultRotorDragCoefficient = 1.0e-4;
    static constexpr double kDefaultRollingMomentCoefficient = 1.0e-6;
    static constexpr double kDefaultRotorVelocitySlowdownSim = 10.0;

    class GazeboMotorModel : public MotorModel, public ModelPlugin
    {
    public:
        GazeboMotorModel()
            : ModelPlugin(),
              MotorModel()
        {
        }

        virtual ~GazeboMotorModel();

        virtual void InitializeParams();
        virtual void Publish();
        // void testProto(MotorSpeedPtr &msg);
    protected:
        virtual void UpdateForcesAndMoments();
        /// \brief A function to check the motor_Failure_Number_ and stimulate motor fail
        /// \details Doing joint_->SetVelocity(0,0) for the flagged motor to fail
        virtual void UpdateMotorFail();
        virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
        virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:
        std::string command_sub_topic_{kDefaultCommandSubTopic};
        std::string motor_failure_sub_topic_{kDefaultMotorFailureNumSubTopic};
        std::string joint_name_;
        std::string link_name_;
        std::string motor_speed_pub_topic_{kDefaultMotorVelocityPubTopic};
        std::string namespace_;

        int motor_number_{0};
        int turning_direction_{turning_direction::CW};

        int motor_Failure_Number_{0}; /*!< motor_Failure_Number is (motor_number_ + 1) as (0) is considered no_fail. Publish accordingly */
        int tmp_motor_num;            // A temporary variable used to print msg

        int screen_msg_flag = 1;

        double max_force_{kDefaultMaxForce};
        double max_rot_velocity_{kDefaulMaxRotVelocity};
        double moment_constant_{kDefaultMomentConstant};
        double motor_constant_{kDefaultMotorConstant};
        double ref_motor_rot_vel_{0.0};
        double rolling_moment_coefficient_{kDefaultRollingMomentCoefficient};
        double rotor_drag_coefficient_{kDefaultRotorDragCoefficient};
        double rotor_velocity_slowdown_sim_{kDefaultRotorVelocitySlowdownSim};
        double time_constant_down_{kDefaultTimeConstantDown};
        double time_constant_up_{kDefaultTimeConstantUp};

        bool reversible_{false};

        transport::NodePtr node_handle_;
        transport::PublisherPtr motor_velocity_pub_;
        transport::SubscriberPtr command_sub_;
        transport::SubscriberPtr motor_failure_sub_; /*!< Subscribing to motor_failure_sub_topic_; receiving motor number to fail, as an integer */
        transport::SubscriberPtr wind_sub_;

        ignition::math::Vector3d wind_vel_;

        physics::ModelPtr model_;
        physics::JointPtr joint_;
        common::PID pid_;
        bool use_pid_;
        physics::LinkPtr link_;
        /// \brief Pointer to the update event connection.
        event::ConnectionPtr updateConnection_;

        boost::thread callback_queue_thread_;
        void QueueThread();
        std_msgs::msgs::Float turning_velocity_msg_;
        void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
        void MotorFailureCallback(const boost::shared_ptr<const msgs::Int> &fail_msg); /*!< Callback for the motor_failure_sub_ subscriber */
        void WindVelocityCallback(const boost::shared_ptr<const physics_msgs::msgs::Wind> &msg);

        std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
        /*
          // Protobuf test
          std::string motor_test_sub_topic_;
          transport::SubscriberPtr motor_sub_;
        */
    };
}
