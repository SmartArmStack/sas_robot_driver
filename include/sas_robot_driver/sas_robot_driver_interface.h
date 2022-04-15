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
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/

#include <atomic>
#include <tuple>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <sas_robot_driver/sas_robot_driver.h>

namespace sas
{

class RobotDriverInterface
{
private:
    std::atomic_bool enabled_;
    std::string topic_prefix_;

    ros::Subscriber subscriber_joint_states_;
    VectorXd joint_positions_;
    VectorXd joint_velocities_;
    VectorXd joint_forces_;
    ros::Subscriber subscriber_joint_limits_min_;
    VectorXd joint_limits_min_;
    ros::Subscriber subscriber_joint_limits_max_;
    VectorXd joint_limits_max_;
    ros::Subscriber subscriber_home_state_;
    VectorXi home_states_;

    ros::Publisher publisher_target_joint_positions_;
    ros::Publisher publisher_target_joint_velocities_;
    ros::Publisher publisher_target_joint_forces_;
    ros::Publisher publisher_homing_signal_;
    ros::Publisher publisher_clear_positions_signal_;

    void _callback_joint_states(const sensor_msgs::JointStateConstPtr& msg);
    void _callback_joint_limits_min(const std_msgs::Float64MultiArrayConstPtr& msg);
    void _callback_joint_limits_max(const std_msgs::Float64MultiArrayConstPtr& msg);
    void _callback_home_states(const std_msgs::Int32MultiArrayConstPtr &msg);
public:
    RobotDriverInterface() = delete;
    RobotDriverInterface(const RobotDriverInterface&) = delete;

    RobotDriverInterface(ros::NodeHandle& nodehandle, const std::string topic_prefix);
    RobotDriverInterface(ros::NodeHandle& publisher_nodehandle, ros::NodeHandle& subscriber_nodehandle, const std::string topic_prefix);

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

    bool is_enabled(const RobotDriver::Functionality& control_mode=RobotDriver::Functionality::PositionControl) const;
    std::string get_topic_prefix() const;
};

}

