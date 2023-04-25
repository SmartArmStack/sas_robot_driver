#pragma once
/*
# Copyright (c) 2016-2022 Murilo Marques Marinho
#
#    This file is part of sas_robot_driver.
#
#    sas_robot_driver is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_robot_driver is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/

#include <atomic>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <sas_robot_driver/sas_robot_driver.h>
#include <sas_core/sas_object.hpp>

using namespace rclcpp;

namespace sas
{

class RobotDriverInterface: private sas::Object
{
private:
    std::atomic_bool enabled_;
    std::string topic_prefix_;

    Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_joint_states_;
    VectorXd joint_positions_;
    VectorXd joint_velocities_;
    VectorXd joint_forces_;
    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_joint_limits_min_;
    VectorXd joint_limits_min_;
    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_joint_limits_max_;
    VectorXd joint_limits_max_;
    Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscriber_home_state_;
    VectorXi home_states_;

    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_target_joint_positions_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_target_joint_velocities_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_target_joint_forces_;
    Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_homing_signal_;
    Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_clear_positions_signal_;

    void _callback_joint_states(const sensor_msgs::msg::JointState& msg);
    void _callback_joint_limits_min(const std_msgs::msg::Float64MultiArray& msg);
    void _callback_joint_limits_max(const std_msgs::msg::Float64MultiArray& msg);
    void _callback_home_states(const std_msgs::msg::Int32MultiArray& msg);
public:
    RobotDriverInterface() = delete;
    RobotDriverInterface(const RobotDriverInterface&) = delete;

//#ifdef IS_SAS_PYTHON_BUILD
//    RobotDriverInterface(const std::string& topic_prefix);
//#endif
    RobotDriverInterface(Node &node, const std::string topic_prefix="GET_FROM_NODE");
//    RobotDriverInterface(ros::NodeHandle& publisher_nodehandle, ros::NodeHandle& subscriber_nodehandle, const std::string topic_prefix);

    void send_target_joint_positions(const VectorXd& target_joint_positions);
    void send_target_joint_velocities(const VectorXd& target_joint_velocities);
    void send_target_joint_forces(const VectorXd& target_joint_forces);
    void send_homing_signal(const VectorXi& homing_signal);
    void send_clear_positions_signal(const VectorXi& clear_positions_signal);

    VectorXd get_joint_positions() const;
    VectorXd get_joint_velocities() const;
    VectorXd get_joint_forces() const;
    std::tuple<VectorXd, VectorXd> get_joint_limits() const;
    VectorXi get_home_states() const;

    bool is_enabled(const RobotDriver::Functionality& supported_functionality=RobotDriver::Functionality::PositionControl) const;
    std::string get_topic_prefix() const;
};

}

