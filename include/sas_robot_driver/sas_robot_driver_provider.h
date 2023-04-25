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

class RobotDriverProvider: private sas::Object
{
private:
    std::shared_ptr<Node> node_;

    std::string node_prefix_;
    RobotDriver::Functionality currently_active_functionality_;

    Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_states_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_joint_limits_min_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_joint_limits_max_;
    Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_home_state_;

    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_target_joint_positions_;
    VectorXd target_joint_positions_;
    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_target_joint_velocities_;
    VectorXd target_joint_velocities_;
    Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_target_joint_forces_;
    VectorXd target_joint_forces_;
    Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscriber_homing_signal_;
    VectorXi homing_signal_;
    Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr subscriber_clear_positions_signal_;
    VectorXi clear_positions_signal_;

    void _callback_target_joint_positions(const std_msgs::msg::Float64MultiArray &msg);
    void _callback_target_joint_velocities(const std_msgs::msg::Float64MultiArray &msg);
    void _callback_target_joint_forces(const std_msgs::msg::Float64MultiArray &msg);
    void _callback_homing_signal(const std_msgs::msg::Int32MultiArray& msg);
    void _callback_clear_positions_signal(const std_msgs::msg::Int32MultiArray &msg);
public:
    RobotDriverProvider() = delete;
    RobotDriverProvider(const RobotDriverProvider&) = delete;

//#ifdef IS_SAS_PYTHON_BUILD
//    RobotDriverProvider(const std::string& node_prefix);
//#endif
    RobotDriverProvider(std::shared_ptr<Node>& node, const std::string& node_prefix="GET_FROM_NODE");
//    RobotDriverProvider(Node& node_publisher, Node& node_subscriber, const std::string& node_prefix="GET_FROM_NODE");

    VectorXd get_target_joint_positions() const;
    VectorXd get_target_joint_velocities() const;
    VectorXd get_target_joint_forces() const;
    VectorXi get_homing_signal() const;
    VectorXi get_clear_positions_signal();
    RobotDriver::Functionality get_currently_active_functionality() const;

    bool is_enabled(const RobotDriver::Functionality& supported_functionality=RobotDriver::Functionality::PositionControl) const;

    void send_joint_states(const VectorXd& joint_positions,
                           const VectorXd& joint_velocities,
                           const VectorXd& joint_forces);
    void send_joint_limits(const std::tuple<VectorXd, VectorXd>& joint_limits);
    void send_home_state(const VectorXi& home_state);

    //Deprecated
    void send_joint_positions(const VectorXd& joint_positions);
};

}
