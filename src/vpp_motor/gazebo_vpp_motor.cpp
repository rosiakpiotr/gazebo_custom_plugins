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

#include "gazebo_custom_plugins/vpp_motor/gazebo_vpp_motor.h"

namespace gazebo
{

    GazeboMotorModel::~GazeboMotorModel()
    {
        updateConnection_->~Connection();
        use_pid_ = false;
    }

    void GazeboMotorModel::InitializeParams() {}

    void GazeboMotorModel::Load(physics::ModelPtr _model,
                                sdf::ElementPtr _sdf) // NOLINT
    {
        model_ = _model;

        namespace_.clear();

        namespace_ = sdf_helpers::get_if_has_or_err_msg<std::string>(
            _sdf, "robotNamespace", namespace_,
            "[gazebo_motor_vpp] Please specify a robotNamespace.");

        node_handle_ = transport::NodePtr(new transport::Node());
        node_handle_->Init(namespace_);

        joint_name_ = sdf_helpers::get_if_has_or_err_msg<std::string>(
            _sdf, "jointName", joint_name_,
            "[gazebo_motor_vpp] Please specify a jointName, where the rotor is "
            "attached.");

        // Get the pointer to the joint.
        joint_ = model_->GetJoint(joint_name_);

        if (joint_ == nullptr)
        {
            gzthrow("[gazebo_motor_model] Couldn't find specified joint \""
                    << joint_name_ << "\".");
        }

        // setup joint control pid to control joint
        if (_sdf->HasElement("joint_control_pid") || true)
        { // NOLINT
            sdf::ElementPtr pid =
                _sdf->GetElement("jointName"); // Changed for anything that is for sure
            // in sdf just so errors gone

            auto get_sdf_or_default = [](const sdf::ElementPtr &elem,
                                         const std::string &key,
                                         double default_val) -> double
            {
                return elem->HasElement(key) ? elem->Get<double>(key) : default_val;
            };

            pid_.Init(get_sdf_or_default(pid, "__p", 0.0035),
                      get_sdf_or_default(pid, "__i", 0.0001),
                      get_sdf_or_default(pid, "__d", 0.000001),
                      get_sdf_or_default(pid, "__iMax", 100),
                      get_sdf_or_default(pid, "__iMin", -100),
                      get_sdf_or_default(pid, "__cmdMax", 7.5158),
                      get_sdf_or_default(pid, "__cmdMin", -1));
            use_pid_ = true;
        }
        else
        {
            use_pid_ = false;
        }

        link_name_ = sdf_helpers::get_if_has_or_err_msg<std::string>(
            _sdf, "linkName", joint_name_,
            "[gazebo_motor_vpp] Please specify a linkName of the rotor.");

        link_ = model_->GetLink(link_name_);

        if (link_ == nullptr)
        {
            gzthrow("[gazebo_motor_model] Couldn't find specified link \"" << link_name_
                                                                           << "\".");
        }

        motor_number_ = sdf_helpers::get_if_has_or_err_msg<int>(
            _sdf, "motorNumber", motor_number_,
            "[gazebo_motor_vpp] Please specify a motorNumber.");

        std::string turning_direction = sdf_helpers::get_if_has_or_err_msg<
            std::string>(
            _sdf, "turningDirection", "cw",
            "[gazebo_motor_vpp] Please specify a turning direction ('cw' or 'ccw').");

        if (turning_direction == "cw")
        {
            turning_direction_ = turning_direction::CW;
        }
        else if (turning_direction == "ccw")
        {
            turning_direction_ = turning_direction::CCW;
        }
        else
        {
            gzerr << "[gazebo_motor_model] Please only use 'cw' or 'ccw' as "
                     "turningDirection.\n";
        }

        reversible_ = sdf_helpers::get_if_has_or_err_msg<bool>(
            _sdf, "reversible", false,
            "[gazebo_motor_vpp] Please specify if motor is reversible (true or "
            "false).");

        getSdfParam<std::string>(_sdf, "commandSubTopic", command_sub_topic_,
                                 command_sub_topic_);
        getSdfParam<std::string>(_sdf, "commandVppSubTopic", command_vpp_sub_topic_,
                                 command_vpp_sub_topic_);
        getSdfParam<std::string>(_sdf, "motorSpeedPubTopic", motor_speed_pub_topic_,
                                 motor_speed_pub_topic_);
        getSdfParam<std::string>(_sdf, "vppStatePubTopic", vpp_state_pub_topic_,
                                 vpp_state_pub_topic_);

        getSdfParam<double>(_sdf, "rotorDragCoefficient", rotor_drag_coefficient_,
                            rotor_drag_coefficient_);
        getSdfParam<double>(_sdf, "rollingMomentCoefficient",
                            rolling_moment_coefficient_, rolling_moment_coefficient_);
        getSdfParam<double>(_sdf, "maxRotVelocity", max_rot_velocity_,
                            max_rot_velocity_);
        getSdfParam<double>(_sdf, "motorConstant", motor_constant_, motor_constant_);
        getSdfParam<double>(_sdf, "momentConstant", moment_constant_,
                            moment_constant_);

        getSdfParam<double>(_sdf, "timeConstantUp", time_constant_up_,
                            time_constant_up_);
        getSdfParam<double>(_sdf, "timeConstantDown", time_constant_down_,
                            time_constant_down_);
        getSdfParam<double>(_sdf, "rotorVelocitySlowdownSim",
                            rotor_velocity_slowdown_sim_, 10);

        double propD = propeller_.getDiameter();
        getSdfParam<double>(_sdf, "propDiameterInch", propD, propD);
        propeller_.setDiameterInch(propD);

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        updateConnection_ = event::Events::ConnectWorldUpdateBegin(
            [this](auto &&PH1)
            {
                OnUpdate(std::forward<decltype(PH1)>(PH1));
            });

        command_sub_ = node_handle_->Subscribe<mav_msgs::msgs::CommandMotorSpeed>(
            "~/" + model_->GetName() + command_sub_topic_,
            &GazeboMotorModel::VelocityCallback, this);
        command_vpp_sub_ = node_handle_->Subscribe<sknr::msgs::CommandPitchAngle>(
            "~/" + model_->GetName() + command_vpp_sub_topic_,
            &GazeboMotorModel::PitchAngleCallback, this);

        // std::cout << "[gazebo_motor_model]: Subscribe to gz topic: "<<
        // motor_failure_sub_topic_ << std::endl;
        motor_failure_sub_ = node_handle_->Subscribe<msgs::Int>(
            motor_failure_sub_topic_, &GazeboMotorModel::MotorFailureCallback, this);

        vpp_state_pub_ = node_handle_->Advertise<sknr::msgs::VppState>(
            "~/" + model_->GetName() + vpp_state_pub_topic_);
        wind_sub_ = node_handle_->Subscribe<msgs::Wind>(
            "~/" + wind_sub_topic_, &GazeboMotorModel::WindVelocityCallback, this);

        // Create the first order filter.
        rotor_velocity_filter_ = std::make_unique<FirstOrderFilter<double>>(
            time_constant_up_, time_constant_down_, ref_motor_rot_vel_);
        vpp_angle_filter_ = std::make_unique<FirstOrderFilter<double>>(
            0.1, 0.1, ref_prop_pitch_angle_);
    }

    // This gets called by the world update start event.
    void GazeboMotorModel::OnUpdate(const common::UpdateInfo &_info)
    {
        auto timenow = _info.simTime.Double();
        sampling_time_ = timenow - prev_sim_time_;
        prev_sim_time_ = timenow;
        if ((_info.simTime.Double() - last_sent_timestamp) > 0.02)
        {
            send_diagnostics_now = true;
            last_sent_timestamp = _info.simTime.Double();
        }
        UpdateMotorVelocity();
        UpdateForcesAndMoments();
        UpdateMotorFail();
        Publish();
    }

    void GazeboMotorModel::VelocityCallback(CommandMotorSpeedPtr &rot_velocities)
    {
        if (rot_velocities->motor_speed_size() < motor_number_)
        {
            std::cout << "You tried to access index " << motor_number_
                      << " of the MotorSpeed message array which is of size "
                      << rot_velocities->motor_speed_size() << "." << '\n';
        }
        else
        {
            ref_motor_rot_vel_ = std::min(
                static_cast<double>(rot_velocities->motor_speed(motor_number_)),
                static_cast<double>(max_rot_velocity_));
        }
    }

    void GazeboMotorModel::PitchAngleCallback(CommandPitchAnglePtr &pitch_angles)
    {
        // std::cout << "Pitch angle callback fired." << '\n';

        if (pitch_angles->pitch_angle_size() < motor_number_)
        {
            std::cout << "You tried to access index " << motor_number_
                      << " of the PitchAngle message array which is of size "
                      << pitch_angles->pitch_angle_size() << "." << '\n';
        }
        else
        {
            double pitch_angle = pitch_angles->pitch_angle(motor_number_);
            std::cout << "Pitch angle for motor " << motor_number_ << ": " << pitch_angle << '\n';
            ref_prop_pitch_angle_ = pitch_angle;
        }
    }

    void GazeboMotorModel::MotorFailureCallback(
        const boost::shared_ptr<const msgs::Int> &fail_msg)
    {
        motor_Failure_Number_ = fail_msg->data();
    }

    void GazeboMotorModel::UpdateForcesAndMoments()
    {
        motor_rot_vel_ = joint_->GetVelocity(0); // rad/s

        if (motor_rot_vel_ / (2 * M_PI) > 1 / (2 * sampling_time_))
        {
            gzerr << "Aliasing on motor [" << motor_number_
                  << "] might occur. Consider making smaller simulation time steps or "
                     "raising the rotor_velocity_slowdown_sim_ param.\n";
        }

#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Vector3d body_velocity = link_->WorldLinearVel();
        ignition::math::Vector3d joint_axis = joint_->GlobalAxis(0);
#else
        ignition::math::Vector3d body_velocity =
            ignitionFromGazeboMath(link_->GetWorldLinearVel());
        ignition::math::Vector3d joint_axis =
            ignitionFromGazeboMath(joint_->GetGlobalAxis(0));
#endif

        double real_motor_velocity = motor_rot_vel_ * rotor_velocity_slowdown_sim_;
        double ref_prop_pitch_angle =
            vpp_angle_filter_->updateFilter(ref_prop_pitch_angle_, sampling_time_);
        ignition::math::Vector3d relative_wind_velocity = body_velocity - wind_vel_;
        ignition::math::Vector3d velocity_parallel_to_rotor_axis =
            (relative_wind_velocity.Dot(joint_axis)) * joint_axis;

        double rpm = real_motor_velocity * 60.0 / (2.0 * M_PI);
        double airspeed = velocity_parallel_to_rotor_axis.Length();
        double pitch = ref_prop_pitch_angle;
        double thrust = propeller_.getThrust(
            {.omega = real_motor_velocity, .airspeed = airspeed, .pitch = pitch});
        double torque = propeller_.getTorque(
            {.omega = real_motor_velocity, .airspeed = airspeed, .pitch = pitch});

        // Computing angle of attack
        // Angle of attack at 3/4 of propeller radius is representative of entire prop
        double radius34 = 3.0 / 4.0 * propeller_.getDiameter() / 2.0;
        // Taking absolute value of tan because real_motor_velocity can be negative
        // and tangent is odd.
        double tan =
            real_motor_velocity != 0
                ? std::abs(std::tan(airspeed / (real_motor_velocity * radius34)))
                : 0;
        double aoa = pitch - (std::atan(tan) * 180.0 / M_PI);

        // Apply a force to the link.
        link_->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));

        // Forces from Philppe Martin's and Erwan Salaün's
        // 2010 IEEE Conference on Robotics and Automation paper
        // The True Role of Accelerometer Feedback in Quadrotor Control
        // - \omega * \lambda_1 * V_A^{\perp}
        ignition::math::Vector3d velocity_perpendicular_to_rotor_axis =
            relative_wind_velocity -
            (relative_wind_velocity.Dot(joint_axis)) * joint_axis;
        ignition::math::Vector3d air_drag = -std::abs(real_motor_velocity) *
                                            rotor_drag_coefficient_ *
                                            velocity_perpendicular_to_rotor_axis;
        // Apply air_drag to link.
        link_->AddForce(air_drag);
        // Moments
        // Getting the parent link, such that the resulting torques can be applied to
        // it.
        physics::Link_V parent_links = link_->GetParentJointsLinks();
        // The tansformation from the parent_link to the link_.
#if GAZEBO_MAJOR_VERSION >= 9
        ignition::math::Pose3d pose_difference =
            link_->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
#else
        ignition::math::Pose3d pose_difference = ignitionFromGazeboMath(
            link_->GetWorldCoGPose() - parent_links.at(0)->GetWorldCoGPose());
#endif
        // torque = thrust * moment_constant_;
        torque *= -turning_direction_;
        ignition::math::Vector3d drag_torque(0, 0, torque);
        // ignition::math::Vector3d drag_torque(0, 0, torque);
        // Transforming the drag torque into the parent frame to handle arbitrary
        // rotor orientations.
        ignition::math::Vector3d drag_torque_parent_frame =
            pose_difference.Rot().RotateVector(drag_torque);
        parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

        // Torque at joint as motor counter torque
        // joint_->SetForce(0, torque);

        ignition::math::Vector3d rolling_moment;
        // - \omega * \mu_1 * V_A^{\perp}
        rolling_moment = -std::abs(real_motor_velocity) * turning_direction_ *
                         rolling_moment_coefficient_ *
                         velocity_perpendicular_to_rotor_axis;
        parent_links.at(0)->AddTorque(rolling_moment);

        // Every couple iterations publish the state
        if (send_diagnostics_now)
        {
            vpp_state_msg.set_thrust(thrust);
            vpp_state_msg.set_torque(torque);
            vpp_state_msg.set_aoa(aoa);
            vpp_state_msg.set_pitch(pitch);
            vpp_state_msg.set_rpm(rpm);
            vpp_state_msg.set_airspeed(airspeed);
            vpp_state_msg.set_advance_ratio(airspeed /
                                            (rpm * propeller_.getDiameter()));
            vpp_state_msg.set_motor_index(motor_number_);
            vpp_state_msg.set_power(std::abs(torque * real_motor_velocity));

            // using namespace std::chrono;

            // // // Use auto keyword to avoid typing long
            // // // type definitions to get the timepoint
            // // // at this instant use function now()
            // auto start = high_resolution_clock::now();
            vpp_state_pub_->Publish(vpp_state_msg);
            send_diagnostics_now = false;

            // auto stop = high_resolution_clock::now();
            // auto duration = duration_cast<microseconds>(stop - start);

            // // To get the value of duration use the count()
            // // member function on the duration object
            // auto dur = duration.count();
            // if (dur > 5) {
            //     std::cout << "UpdateForcesAndMoments() took: " << dur << "
            //     microseconds" << '\n';
            // }
        }
    }

    void GazeboMotorModel::UpdateMotorFail()
    {
        if (motor_number_ == motor_Failure_Number_ - 1)
        {
            // motor_constant_ = 0.0;
            joint_->SetVelocity(0, 0);

            if (screen_msg_flag)
            {
                std::cout << "Motor number [" << motor_Failure_Number_
                          << "] failed!  [Motor thrust = 0]" << '\n';
                tmp_motor_num = motor_Failure_Number_;

                screen_msg_flag = 0;
            }
        }
        else if (motor_Failure_Number_ == 0 && motor_number_ == tmp_motor_num - 1)
        {
            if (!screen_msg_flag)
            {
                // motor_constant_ = kDefaultMotorConstant;
                std::cout << "Motor number [" << tmp_motor_num
                          << "] running! [Motor thrust = (default)]" << '\n';
                screen_msg_flag = 1;
            }
        }
    }

    void GazeboMotorModel::WindVelocityCallback(WindPtr &msg)
    {
        std::cout << "Wind velocity callback" << '\n';
        wind_vel_ = ignition::math::Vector3d(msg->linear_velocity().x(),
                                             msg->linear_velocity().y(),
                                             msg->linear_velocity().z());
    }

    void GazeboMotorModel::UpdateMotorVelocity()
    {
        use_pid_ = false;
        // Apply the filter on the motor's velocity.
        double ref_motor_rot_vel = use_pid_ ? ref_motor_rot_vel_
                                            : rotor_velocity_filter_->updateFilter(
                                                  ref_motor_rot_vel_, sampling_time_);

#if USE_PID // FIXME: disable PID for now, it does not play nice with the PX4 CI
        // system. NOLINT

        if (use_pid_)
        {
            auto measVelocity = joint_->GetVelocity(0);
            double err =
                -((ref_motor_rot_vel / rotor_velocity_slowdown_sim_) - measVelocity);
            err *= turning_direction_;
            // Update max command value so it resembles behaviour of real BLDC motor.
            pid_.SetCmdMax(std::min(measVelocity * (-0.01) + 7.5158, 7.5158));
            double rotorForce = pid_.Update(err, sampling_time_);
            joint_->SetForce(0, rotorForce);
            if (motor_number_ == 0)
            {
                std::cout << rotorForce << "\t" << measVelocity << "\t" << err << "\n";
            }
        }
        else
        {
#if GAZEBO_MAJOR_VERSION >= 7
            // Not desirable to use SetVelocity for parts of a moving model
            // impact on rest of the dynamic system is non-physical.
            joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel /
                                       rotor_velocity_slowdown_sim_);
#elif GAZEBO_MAJOR_VERSION >= 6
            // Not ideal as the approach could result in unrealistic impulses, and
            // is only available in ODE
            joint_->SetParam("fmax", 0, 2.0);
            joint_->SetParam("vel", 0,
                             turning_direction_ * ref_motor_rot_vel /
                                 rotor_velocity_slowdown_sim_);
#endif
        }

#else
        joint_->SetVelocity(0, turning_direction_ * ref_motor_rot_vel /
                                   rotor_velocity_slowdown_sim_);
#endif /* if 0 */
    }

    GZ_REGISTER_MODEL_PLUGIN(GazeboMotorModel);
} // namespace gazebo
