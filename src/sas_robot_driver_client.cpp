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
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <sas_conversions/sas_conversions.hpp>
//#include <sas_common/sas_common.h>
using std::placeholders::_1;

namespace sas
{

void RobotDriverClient::_callback_joint_states(const sensor_msgs::msg::JointState &msg)
{
    joint_positions_ = std_vector_double_to_vectorxd(msg.position);
    joint_velocities_ = std_vector_double_to_vectorxd(msg.velocity);
    joint_forces_ = std_vector_double_to_vectorxd(msg.effort);
}

void RobotDriverClient::_callback_joint_limits_min(const std_msgs::msg::Float64MultiArray &msg)
{
    joint_limits_min_ = std_vector_double_to_vectorxd(msg.data);
}

void RobotDriverClient::_callback_joint_limits_max(const std_msgs::msg::Float64MultiArray &msg)
{
    joint_limits_max_ = std_vector_double_to_vectorxd(msg.data);
}

void RobotDriverClient::_callback_home_states(const std_msgs::msg::Int32MultiArray &msg)
{
    home_states_ = std_vector_int_to_vectorxi(msg.data);
}

//#ifdef IS_SAS_PYTHON_BUILD
//RobotDriverInterface::RobotDriverInterface(const std::string &topic_prefix):
//    RobotDriverInterface(sas::common::get_static_node_handle(),topic_prefix)
//{
//
//}
//#endif

RobotDriverClient::RobotDriverClient(std::shared_ptr<Node> &node, const std::string topic_prefix):
    sas::Object("sas::RobotDriverInterface"),
    node_(node),
    topic_prefix_(topic_prefix == "GET_FROM_NODE"? node->get_name() : topic_prefix)
{
    //    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotDriverInterface with prefix " + topic_prefix);
    RCLCPP_INFO_STREAM(node_->get_logger(),"::Initializing RobotDriverInterface with prefix " + topic_prefix);

    //    publisher_target_joint_positions_ = publisher_nodehandle.advertise<std_msgs::Float64MultiArray>(topic_prefix_ + "/set/target_joint_positions", 1);
    publisher_target_joint_positions_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix + "/set/target_joint_positions",1);
    //    publisher_target_joint_velocities_ = publisher_nodehandle.advertise<std_msgs::Float64MultiArray>(topic_prefix_ + "/set/target_joint_velocities", 1);
    publisher_target_joint_velocities_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix + "/set/target_joint_velocities",1);
    //    publisher_target_joint_forces_ = publisher_nodehandle.advertise<std_msgs::Float64MultiArray>(topic_prefix_ + "/set/target_joint_forces", 1);
    publisher_target_joint_forces_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix + "/set/target_joint_forces",1);
    //    publisher_homing_signal_ = publisher_nodehandle.advertise<std_msgs::Int32MultiArray>(topic_prefix_ + "/set/homing_signal", 1);
    publisher_homing_signal_ = node->create_publisher<std_msgs::msg::Int32MultiArray>(topic_prefix + "/set/homing_signal",1);
    //    publisher_clear_positions_signal_ = publisher_nodehandle.advertise<std_msgs::Int32MultiArray>(topic_prefix_ + "/set/clear_positions_signal", 1);
    publisher_clear_positions_signal_ = node->create_publisher<std_msgs::msg::Int32MultiArray>(topic_prefix + "/set/clear_positions_signal",1);

    //    subscriber_joint_states_ = subscriber_nodehandle.subscribe(topic_prefix_ + "/get/joint_states", 1, &RobotDriverInterface::_callback_joint_states, this);
    subscriber_joint_states_ = node->create_subscription<sensor_msgs::msg::JointState>(
                topic_prefix + "/get/joint_states", 1, std::bind(&RobotDriverClient::_callback_joint_states, this, _1)
                );
    //    subscriber_joint_limits_min_ = subscriber_nodehandle.subscribe(topic_prefix_ + "/get/joint_positions_min", 1, &RobotDriverInterface::_callback_joint_limits_min, this);
    subscriber_joint_limits_min_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                topic_prefix + "/get/joint_positions_min", 1, std::bind(&RobotDriverClient::_callback_joint_limits_min, this, _1)
                );
    //    subscriber_joint_limits_max_ = subscriber_nodehandle.subscribe(topic_prefix_ + "/get/joint_positions_max", 1, &RobotDriverInterface::_callback_joint_limits_max, this);
    subscriber_joint_limits_max_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                topic_prefix + "/get/joint_positions_max", 1, std::bind(&RobotDriverClient::_callback_joint_limits_max, this, _1)
                );
    //    subscriber_home_state_ = subscriber_nodehandle.subscribe(topic_prefix_ + "/get/home_states", 1, &RobotDriverInterface::_callback_home_states, this);
    subscriber_home_state_ = node->create_subscription<std_msgs::msg::Int32MultiArray>(
                topic_prefix + "/get/home_states", 1, std::bind(&RobotDriverClient::_callback_home_states, this, _1)
                );
}

//RobotDriverInterface::RobotDriverInterface(ros::NodeHandle &publisher_nodehandle, ros::NodeHandle &subscriber_nodehandle, const std::string topic_prefix):
//    enabled_(false),
//    topic_prefix_(topic_prefix)
//{
//    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotDriverInterface with prefix " + topic_prefix);
//    publisher_target_joint_positions_ = publisher_nodehandle.advertise<std_msgs::Float64MultiArray>(topic_prefix_ + "/set/target_joint_positions", 1);
//    publisher_target_joint_velocities_ = publisher_nodehandle.advertise<std_msgs::Float64MultiArray>(topic_prefix_ + "/set/target_joint_velocities", 1);
//    publisher_target_joint_forces_ = publisher_nodehandle.advertise<std_msgs::Float64MultiArray>(topic_prefix_ + "/set/target_joint_forces", 1);
//    publisher_homing_signal_ = publisher_nodehandle.advertise<std_msgs::Int32MultiArray>(topic_prefix_ + "/set/homing_signal", 1);
//    publisher_clear_positions_signal_ = publisher_nodehandle.advertise<std_msgs::Int32MultiArray>(topic_prefix_ + "/set/clear_positions_signal", 1);

//    subscriber_joint_states_ = subscriber_nodehandle.subscribe(topic_prefix_ + "/get/joint_states", 1, &RobotDriverInterface::_callback_joint_states, this);
//    subscriber_joint_limits_min_ = subscriber_nodehandle.subscribe(topic_prefix_ + "/get/joint_positions_min", 1, &RobotDriverInterface::_callback_joint_limits_min, this);
//    subscriber_joint_limits_max_ = subscriber_nodehandle.subscribe(topic_prefix_ + "/get/joint_positions_max", 1, &RobotDriverInterface::_callback_joint_limits_max, this);
//    subscriber_home_state_ = subscriber_nodehandle.subscribe(topic_prefix_ + "/get/home_states", 1, &RobotDriverInterface::_callback_home_states, this);
//}

void RobotDriverClient::send_target_joint_positions(const VectorXd &target_joint_positions)
{
    std_msgs::msg::Float64MultiArray ros_msg;
    ros_msg.data = vectorxd_to_std_vector_double(target_joint_positions);
    publisher_target_joint_positions_->publish(ros_msg);
}

void RobotDriverClient::send_target_joint_velocities(const VectorXd &target_joint_velocities)
{
    std_msgs::msg::Float64MultiArray ros_msg;
    ros_msg.data = vectorxd_to_std_vector_double(target_joint_velocities);
    publisher_target_joint_velocities_->publish(ros_msg);
}

void RobotDriverClient::send_target_joint_forces(const VectorXd &target_joint_efforts)
{
    std_msgs::msg::Float64MultiArray ros_msg;
    ros_msg.data = vectorxd_to_std_vector_double(target_joint_efforts);
    publisher_target_joint_forces_->publish(ros_msg);
}

void RobotDriverClient::send_homing_signal(const VectorXi &homing_signal)
{
    std_msgs::msg::Int32MultiArray ros_msg;
    ros_msg.data = vectorxi_to_std_vector_int(homing_signal);
    publisher_homing_signal_->publish(ros_msg);
}

void RobotDriverClient::send_clear_positions_signal(const VectorXi &clear_positions_signal)
{
    std_msgs::msg::Int32MultiArray ros_msg;
    ros_msg.data = vectorxi_to_std_vector_int(clear_positions_signal);
    publisher_clear_positions_signal_->publish(ros_msg);
}

VectorXd RobotDriverClient::get_joint_positions() const
{
    if(is_enabled())
        return joint_positions_;
    else
        throw std::runtime_error(topic_prefix_ + "::RobotDriverInterface::get_joint_positions()::trying to get joint positions but uninitialized.");
}

VectorXd RobotDriverClient::get_joint_velocities() const
{
    if(is_enabled(RobotDriver::Functionality::VelocityControl))
        return joint_velocities_;
    else
        throw std::runtime_error(topic_prefix_ + "::RobotDriverInterface::get_joint_velocities()::trying to get joint velocities but uninitialized.");
}

VectorXd RobotDriverClient::get_joint_forces() const
{
    if(is_enabled(RobotDriver::Functionality::ForceControl))
        return joint_forces_;
    else
        throw std::runtime_error(topic_prefix_ + "::RobotDriverInterface::get_joint_efforts()::trying to get joint efforts but uninitialized.");
}

std::tuple<VectorXd, VectorXd> RobotDriverClient::get_joint_limits() const
{
    if(is_enabled())
    {
        return std::make_tuple(joint_limits_min_, joint_limits_max_);
    }
    else
        throw std::runtime_error(topic_prefix_ + "::RobotDriverInterface::get_joint_limits()::trying to get joint limits but uninitialized.");
}

VectorXi RobotDriverClient::get_home_states() const
{
    if(is_enabled(RobotDriver::Functionality::Homing))
    {
        return home_states_;
    }
    else
        throw std::runtime_error(topic_prefix_ + "::RobotDriverInterface::get_home_states()::trying to get home states but uninitialized.");
}


bool RobotDriverClient::is_enabled(const RobotDriver::Functionality &control_mode) const
{
    switch(control_mode)
    {
    case RobotDriver::Functionality::PositionControl:
        return joint_positions_.size() > 0 && joint_limits_min_.size() > 0 && joint_limits_max_.size() > 0;
    case RobotDriver::Functionality::VelocityControl:
        return joint_velocities_.size() > 0;
    case RobotDriver::Functionality::ForceControl:
        return joint_forces_.size() > 0;
    case RobotDriver::Functionality::Homing:
        return home_states_.size() > 0;
    case RobotDriver::Functionality::ClearPositions:
        throw std::runtime_error(topic_prefix_+"::is_enabled() RobotDriver::Functionality::ClearPositions has no meaning in RobotDriverInterface::is_enabled().");
    case RobotDriver::Functionality::None:
        throw std::runtime_error(topic_prefix_+"::is_enabled() RobotDriver::Functionality::None has no meaning in RobotDriverInterface::is_enabled().");
    }
    throw std::runtime_error(topic_prefix_+"::is_enabled() Unknown RobotDriver::Functionality.");
}

std::string RobotDriverClient::get_topic_prefix() const
{
    return topic_prefix_;
}


}
