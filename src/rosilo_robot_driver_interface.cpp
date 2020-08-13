/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
#
#    This file is part of rosilo_robot_driver.
#
#    rosilo_robot_driver is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    rosilo_robot_driver is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with rosilo_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/
#include <rosilo_robot_driver/rosilo_robot_driver_interface.h>

#include <rosilo_conversions/rosilo_conversions.h>

namespace rosilo
{

void RobotDriverInterface::_callback_joint_states(const sensor_msgs::JointStateConstPtr &msg)
{
    joint_positions_ = std_vector_double_to_vectorxd(msg->position);
}

void RobotDriverInterface::_callback_joint_limits_min(const std_msgs::Float64MultiArray &msg)
{
    joint_limits_min_ = std_vector_double_to_vectorxd(msg.data);
}

void RobotDriverInterface::_callback_joint_limits_max(const std_msgs::Float64MultiArray &msg)
{
    joint_limits_max_ = std_vector_double_to_vectorxd(msg.data);
}

RobotDriverInterface::RobotDriverInterface(ros::NodeHandle &nodehandle, const std::string node_prefix):
    enabled_(false),
    node_prefix_(node_prefix)
{
    publisher_target_joint_positions_ = nodehandle.advertise<std_msgs::Float64MultiArray>(node_prefix_ + "set/target_joint_positions", 1);

    subscriber_joint_states_ = nodehandle.subscribe(node_prefix_ + "get/joint_states", 1, &RobotDriverInterface::_callback_joint_states, this);
    subscriber_joint_limits_min_ = nodehandle.subscribe(node_prefix_ + "get/joint_positions_min", 1, &RobotDriverInterface::_callback_joint_limits_min, this);
    subscriber_joint_limits_max_ = nodehandle.subscribe(node_prefix_ + "get/joint_positions_max", 1, &RobotDriverInterface::_callback_joint_limits_max, this);
}

RobotDriverInterface::RobotDriverInterface(ros::NodeHandle &publisher_nodehandle, ros::NodeHandle &subscriber_nodehandle, const std::string node_prefix):
    enabled_(false),
    node_prefix_(node_prefix)
{
    publisher_target_joint_positions_ = publisher_nodehandle.advertise<std_msgs::Float64MultiArray>(node_prefix_ + "set/target_joint_positions", 1);

    subscriber_joint_states_ = subscriber_nodehandle.subscribe(node_prefix_ + "get/joint_states", 1, &RobotDriverInterface::_callback_joint_states, this);
    subscriber_joint_limits_min_ = subscriber_nodehandle.subscribe(node_prefix_ + "get/joint_positions_min", 1, &RobotDriverInterface::_callback_joint_limits_min, this);
    subscriber_joint_limits_max_ = subscriber_nodehandle.subscribe(node_prefix_ + "get/joint_positions_max", 1, &RobotDriverInterface::_callback_joint_limits_max, this);
}

void RobotDriverInterface::send_target_joint_positions(const VectorXd &target_joint_positions)
{
    std_msgs::Float64MultiArray ros_msg;
    ros_msg.data = vectorxd_to_std_vector_double(target_joint_positions);
    publisher_target_joint_positions_.publish(ros_msg);
}

VectorXd RobotDriverInterface::get_joint_positions() const
{
    if(is_enabled())
        return joint_positions_;
    else
        throw std::runtime_error(ros::this_node::getName() + "::RobotDriverInterface::get_joint_positions()::trying to get joint positions but uninitialized.");
}

std::tuple<VectorXd, VectorXd> RobotDriverInterface::get_joint_limits() const
{
    if(is_enabled())
    {
        return std::make_tuple(joint_limits_min_, joint_limits_max_);
    }
    else
        throw std::runtime_error(ros::this_node::getName() + "::RobotDriverInterface::get_joint_limits()::trying to get joint positions but uninitialized.");
}


bool RobotDriverInterface::is_enabled() const
{
    return( joint_positions_.size()>0 &&
            joint_limits_min_.size()>0 &&
            joint_limits_max_.size()>0);
}


}
