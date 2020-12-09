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
#include <rosilo_robot_driver/rosilo_robot_driver_provider.h>

#include <rosilo_conversions/rosilo_conversions.h>

namespace rosilo
{

void RobotDriverProvider::_callback_target_joint_positions(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    target_joint_positions_ = std_vector_double_to_vectorxd(msg->data);

    if(!enabled_)
        enabled_ = true;
}

RobotDriverProvider::RobotDriverProvider(ros::NodeHandle &nodehandle, const std::string &node_prefix):
    RobotDriverProvider(nodehandle, nodehandle, node_prefix)
{
    //Delegated to RobotDriverProvider::RobotDriverProvider(ros::NodeHandle &publisher_nodehandle, ros::NodeHandle &subscriber_nodehandle, const std::string &node_prefix)
}

RobotDriverProvider::RobotDriverProvider(ros::NodeHandle &publisher_nodehandle, ros::NodeHandle &subscriber_nodehandle, const std::string &node_prefix):
    enabled_(false),
    node_prefix_(node_prefix)
{
    publisher_joint_states_ = publisher_nodehandle.advertise<sensor_msgs::JointState>(node_prefix + "/get/joint_states", 1);
    publisher_joint_limits_min_ = publisher_nodehandle.advertise<std_msgs::Float64MultiArray>(node_prefix + "/get/joint_positions_min", 1);
    publisher_joint_limits_max_ = publisher_nodehandle.advertise<std_msgs::Float64MultiArray>(node_prefix + "/get/joint_positions_max", 1);
    publisher_reference_frame_ = publisher_nodehandle.advertise<geometry_msgs::PoseStamped>(node_prefix + "/get/reference_frame", 1);

    subscriber_target_joint_positions_ = subscriber_nodehandle.subscribe(node_prefix + "set/target_joint_positions", 1, &RobotDriverProvider::_callback_target_joint_positions, this);
}

VectorXd RobotDriverProvider::get_target_joint_positions() const
{
    if(is_enabled())
        return target_joint_positions_;
    else
        throw std::runtime_error(node_prefix_ + "::RobotDriverProvider::get_target_joint_positions() trying to get an uninitialized vector");
}

void RobotDriverProvider::send_joint_positions(const VectorXd &joint_positions)
{
    sensor_msgs::JointState ros_msg;
    ros_msg.position = vectorxd_to_std_vector_double(joint_positions);
    publisher_joint_states_.publish(ros_msg);
}

void RobotDriverProvider::send_joint_limits(const std::tuple<VectorXd, VectorXd> &joint_limits)
{
    std_msgs::Float64MultiArray ros_msg_min;
    ros_msg_min.data = vectorxd_to_std_vector_double(std::get<0>(joint_limits));
    publisher_joint_limits_min_.publish(ros_msg_min);

    std_msgs::Float64MultiArray ros_msg_max;
    ros_msg_max.data = vectorxd_to_std_vector_double(std::get<1>(joint_limits));
    publisher_joint_limits_max_.publish(ros_msg_max);
}

void RobotDriverProvider::send_reference_frame(const DQ &reference_frame)
{
    publisher_reference_frame_.publish(rosilo::dq_to_geometry_msgs_pose_stamped(reference_frame));
}

bool RobotDriverProvider::is_enabled() const
{
    return enabled_;
}

}

