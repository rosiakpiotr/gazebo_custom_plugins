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

#include <cmath>
#include <boost/bind.hpp>
// #include <algorithm>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <rotors_model/motor_model.hpp>
#include "gazebo_custom_msg/CommandMotorSpeed.pb.h"
#include "gazebo_custom_msg/CommandPitchAngle.pb.h"
#include "gazebo_custom_msg/VppState.pb.h"
#include "gazebo/transport/transport.hh"

#include "propeller.h"

#include "common.h"

namespace turning_direction
{
    const static int CCW = 1;
    const static int CW = -1; // NOLINT
}

namespace gazebo
{

    namespace sdf_helpers
    {

        template <typename T>
        T get_if_has_or_err_msg(const sdf::ElementPtr &sdf, const std::string &elem_name, const T &default_value, const std::string &err_msg)
        {
            T ret_val = default_value;
            if (sdf->HasElement(elem_name))
            {
                ret_val = sdf->GetElement(elem_name)->Get<T>();
            }
            else
            {
                gzerr << err_msg << "\n";
            }
            return ret_val;
        };

    }

    // Default values
    static const std::string kDefaultNamespace;
    static const std::string kDefaultCommandSubTopic = "/gazebo/command/motor_speed";
    static const std::string kDefaultCommandVppSubTopic = "/gazebo/command/pitch_angle";
    static const std::string kDefaultMotorFailureNumSubTopic = "/gazebo/motor_failure_num";
    static const std::string kDefaultMotorVelocityPubTopic = "/motor_speed";
    static const std::string wind_sub_topic_ = "/wind";
    static const std::string kDefaultVppStatePubTopic = "/gazebo/state/vpp_state";

    typedef const boost::shared_ptr<const mav_msgs::msgs::CommandMotorSpeed> CommandMotorSpeedPtr;
    typedef const boost::shared_ptr<const sknr::msgs::CommandPitchAngle> CommandPitchAnglePtr;
    typedef const boost::shared_ptr<const msgs::Wind> WindPtr;

    // typedef const boost::shared_ptr<const mav_msgs::msgs::VppState> VppStatePtr;

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
        GazeboMotorModel() : propeller_(PropellerParams())
        {
        }

        ~GazeboMotorModel() override;

        void InitializeParams() override;
        void Publish() override {};

    protected:
        void UpdateMotorVelocity();
        void UpdateForcesAndMoments() override;
        /// \brief A function to check the motor_Failure_Number_ and stimulate motor fail
        /// \details Doing joint_->SetVelocity(0,0) for the flagged motor to fail
        virtual void UpdateMotorFail();
        void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
        virtual void OnUpdate(const common::UpdateInfo & /*_info*/);

    private:
        std::string command_sub_topic_{kDefaultCommandSubTopic};
        std::string command_vpp_sub_topic_{kDefaultCommandVppSubTopic};
        std::string motor_failure_sub_topic_{kDefaultMotorFailureNumSubTopic};
        std::string joint_name_;
        std::string link_name_;
        std::string motor_speed_pub_topic_{kDefaultMotorVelocityPubTopic};
        std::string vpp_state_pub_topic_{kDefaultVppStatePubTopic};
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
        double ref_prop_pitch_angle_{15.0};
        double rolling_moment_coefficient_{kDefaultRollingMomentCoefficient};
        double rotor_drag_coefficient_{kDefaultRotorDragCoefficient};
        double rotor_velocity_slowdown_sim_{kDefaultRotorVelocitySlowdownSim};
        double time_constant_down_{kDefaultTimeConstantDown};
        double time_constant_up_{kDefaultTimeConstantUp};

        bool reversible_{false};

        transport::NodePtr node_handle_;
        transport::PublisherPtr motor_velocity_pub_;
        transport::PublisherPtr vpp_state_pub_;
        transport::SubscriberPtr command_vpp_sub_;
        transport::SubscriberPtr command_sub_;
        transport::SubscriberPtr
            motor_failure_sub_; /*!< Subscribing to motor_failure_sub_topic_; receiving motor number to fail, as an integer */
        transport::SubscriberPtr wind_sub_;

        ignition::math::Vector3d wind_vel_;

        Propeller propeller_;

        physics::ModelPtr model_;
        physics::JointPtr joint_;
        common::PID pid_;
        bool use_pid_;
        physics::LinkPtr link_;
        /// \brief Pointer to the update event connection.
        event::ConnectionPtr updateConnection_;

        boost::thread callback_queue_thread_;
        void QueueThread();
        sknr::msgs::VppState vpp_state_msg;
        double last_sent_timestamp = 0;
        bool send_diagnostics_now = true;
        void VelocityCallback(CommandMotorSpeedPtr &rot_velocities);
        void PitchAngleCallback(CommandPitchAnglePtr &pitch_angles);
        void MotorFailureCallback(const boost::shared_ptr<const msgs::Int>
                                      &fail_msg); /*!< Callback for the motor_failure_sub_ subscriber */
        void WindVelocityCallback(WindPtr &msg);

        std::unique_ptr<FirstOrderFilter<double>> rotor_velocity_filter_;
        std::unique_ptr<FirstOrderFilter<double>> vpp_angle_filter_;
    };

}
