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
#   Author: Murilo M. Marinho, email: murilo@g.ecc.u-tokyo.ac.jp
#
# ################################################################*/

#include <tuple>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

#include <sas_robot_driver/sas_robot_driver.h>

namespace sas
{

class RobotDriverProvider
{
private:
    std::string node_prefix_;
    RobotDriver::SupportedFunctionality currently_active_functionality_;

    ros::Publisher publisher_joint_states_;
    ros::Publisher publisher_joint_limits_min_;
    ros::Publisher publisher_joint_limits_max_;
    ros::Publisher publisher_home_state_;

    ros::Subscriber subscriber_target_joint_positions_;
    VectorXd target_joint_positions_;
    ros::Subscriber subscriber_target_joint_velocities_;
    VectorXd target_joint_velocities_;
    ros::Subscriber subscriber_target_joint_forces_;
    VectorXd target_joint_forces_;
    ros::Subscriber subscriber_homing_signal_;
    VectorXi homing_signal_;
    ros::Subscriber subscriber_clear_positions_signal_;
    VectorXi clear_positions_signal_;

    void _callback_target_joint_positions(const std_msgs::Float64MultiArrayConstPtr& msg);
    void _callback_target_joint_velocities(const std_msgs::Float64MultiArrayConstPtr& msg);
    void _callback_target_joint_forces(const std_msgs::Float64MultiArrayConstPtr& msg);
    void _callback_homing_signal(const std_msgs::Int32MultiArrayConstPtr& msg);
    void _callback_clear_positions_signal(const std_msgs::Int32MultiArrayConstPtr& msg);
public:
    RobotDriverProvider() = delete;
    RobotDriverProvider(const RobotDriverProvider&) = delete;

    RobotDriverProvider(ros::NodeHandle& nodehandle, const std::string& node_prefix=ros::this_node::getName());
    RobotDriverProvider(ros::NodeHandle& publisher_nodehandle, ros::NodeHandle& subscriber_nodehandle, const std::string& node_prefix=ros::this_node::getName());

    VectorXd get_target_joint_positions() const;
    VectorXd get_target_joint_velocities() const;
    VectorXd get_target_joint_forces() const;
    VectorXi get_homing_signal() const;
    VectorXi get_clear_positions_signal() const;
    RobotDriver::SupportedFunctionality get_currently_active_functionality() const;

    bool is_enabled(const RobotDriver::SupportedFunctionality& supported_functionality=RobotDriver::SupportedFunctionality::PositionControl) const;

    void send_joint_states(const VectorXd& joint_positions,
                           const VectorXd& joint_velocities,
                           const VectorXd& joint_forces);
    void send_joint_limits(const std::tuple<VectorXd, VectorXd>& joint_limits);
    void send_home_state(const VectorXi& home_state);

    //Deprecated
    void send_joint_positions(const VectorXd& joint_positions);
};

}
