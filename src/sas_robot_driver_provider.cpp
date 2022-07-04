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
#include <sas_robot_driver/sas_robot_driver_provider.h>
#include <sas_conversions/sas_conversions.h>
#include <sas_common/sas_common.h>

namespace sas
{

void RobotDriverProvider::_callback_target_joint_positions(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    target_joint_positions_ = std_vector_double_to_vectorxd(msg->data);
    currently_active_functionality_ = RobotDriver::Functionality::PositionControl;
}

void RobotDriverProvider::_callback_target_joint_velocities(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    target_joint_velocities_ = std_vector_double_to_vectorxd(msg->data);
    currently_active_functionality_ = RobotDriver::Functionality::VelocityControl;
}

void RobotDriverProvider::_callback_target_joint_forces(const std_msgs::Float64MultiArrayConstPtr &msg)
{
    target_joint_forces_ = std_vector_double_to_vectorxd(msg->data);
    currently_active_functionality_ = RobotDriver::Functionality::ForceControl;
}

void RobotDriverProvider::_callback_homing_signal(const std_msgs::Int32MultiArrayConstPtr &msg)
{
    homing_signal_ = std_vector_int_to_vectorxi(msg->data);
    currently_active_functionality_ = RobotDriver::Functionality::Homing;
}

/**
 * @brief RobotDriverProvider::_callback_clear_positions_signal. This callback behaves differently from
 * the other callbacks in that a clear positions signal is a "one-off" pulse. We still allow the users
 * to send 0s or 1s because in the same message only some of the joints should be cleared.
 * @param msg a suitable std::msgs::Int32MultiArrayConstPtr that will be managed by ROS.
 */
void RobotDriverProvider::_callback_clear_positions_signal(const std_msgs::Int32MultiArrayConstPtr &msg)
{
    VectorXi clear_positions_signal_temp(msg->data.size());

    //We keep the clear position flags as 1 until they are processed by get_clear_positions_signal()
    for(int i=0;i<clear_positions_signal_temp.size();i++)
    {
        if(clear_positions_signal_(i)==1 || msg->data[i]==1)
            clear_positions_signal_temp(i) = 1;
        else
            clear_positions_signal_temp(i) = 0;
    }
    clear_positions_signal_ = clear_positions_signal_temp;

    //Other callback have something similar but here seems unused
    //currently_active_functionality_ = RobotDriver::Functionality::ClearPositions;
}

RobotDriverProvider::RobotDriverProvider(ros::NodeHandle &nodehandle, const std::string &node_prefix):
    RobotDriverProvider(nodehandle, nodehandle, node_prefix)
{
    //Delegated
}

#ifdef IS_SAS_PYTHON_BUILD
RobotDriverProvider::RobotDriverProvider(const std::string &node_prefix):
    RobotDriverProvider(sas::common::get_static_node_handle(),node_prefix)
{
    //Delegated
}
#endif

RobotDriverProvider::RobotDriverProvider(ros::NodeHandle &publisher_nodehandle, ros::NodeHandle &subscriber_nodehandle, const std::string &node_prefix):
    node_prefix_(node_prefix),
    currently_active_functionality_(RobotDriver::Functionality::None)
{
    ROS_INFO_STREAM(ros::this_node::getName() + "::Initializing RobotDriverProvider with prefix " + node_prefix);
    publisher_joint_states_ = publisher_nodehandle.advertise<sensor_msgs::JointState>(node_prefix + "/get/joint_states", 1);
    publisher_joint_limits_min_ = publisher_nodehandle.advertise<std_msgs::Float64MultiArray>(node_prefix + "/get/joint_positions_min", 1);
    publisher_joint_limits_max_ = publisher_nodehandle.advertise<std_msgs::Float64MultiArray>(node_prefix + "/get/joint_positions_max", 1);
    publisher_home_state_ = publisher_nodehandle.advertise<std_msgs::Int32MultiArray>(node_prefix + "/get/home_states", 1);

    subscriber_target_joint_positions_ = subscriber_nodehandle.subscribe(node_prefix + "/set/target_joint_positions", 1, &RobotDriverProvider::_callback_target_joint_positions, this);
    subscriber_target_joint_velocities_ = subscriber_nodehandle.subscribe(node_prefix + "/set/target_joint_velocities", 1, &RobotDriverProvider::_callback_target_joint_velocities, this);
    subscriber_target_joint_forces_ = subscriber_nodehandle.subscribe(node_prefix + "/set/target_joint_forces", 1, &RobotDriverProvider::_callback_target_joint_forces, this);
    subscriber_homing_signal_ = subscriber_nodehandle.subscribe(node_prefix + "/set/homing_signal", 1, &RobotDriverProvider::_callback_homing_signal, this);
}

VectorXd RobotDriverProvider::get_target_joint_positions() const
{
    if(is_enabled(RobotDriver::Functionality::PositionControl))
        return target_joint_positions_;
    else
        throw std::runtime_error(node_prefix_ + "::RobotDriverProvider::get_target_joint_positions() trying to get an uninitialized vector");
}

VectorXd RobotDriverProvider::get_target_joint_velocities() const
{
    if(is_enabled(RobotDriver::Functionality::VelocityControl))
        return target_joint_velocities_;
    else
        throw std::runtime_error(node_prefix_ + "::RobotDriverProvider::get_target_joint_velocities() trying to get an uninitialized vector");
}

VectorXd RobotDriverProvider::get_target_joint_forces() const
{
    if(is_enabled(RobotDriver::Functionality::ForceControl))
        return target_joint_forces_;
    else
        throw std::runtime_error(node_prefix_ + "::RobotDriverProvider::get_target_joint_forces() trying to get an uninitialized vector");
}

/**
 * @brief get_homing_signal
 * @return a VectorXi with 1s for the joints that should be homed and 0s for the joints that should not be homed.
 */
VectorXi RobotDriverProvider::get_homing_signal() const
{
    if(is_enabled(RobotDriver::Functionality::Homing))
        return homing_signal_;
    else
        throw std::runtime_error(node_prefix_ + "::RobotDriverProvider::get_homing_signal() trying to get an uninitialized vector");
}

/**
 * @brief RobotDriverProvider::get_clear_positions_signal. Getting the clear positions signal also clears it.
 * @return a VectorXi with 0s for configurations that should not be cleared and 1 for positions that should be cleared.
 */
VectorXi RobotDriverProvider::get_clear_positions_signal()
{
    if(is_enabled(RobotDriver::Functionality::ClearPositions))
    {
        const VectorXi return_value(clear_positions_signal_);
        clear_positions_signal_ = VectorXi::Zero(return_value.size());
        return return_value;
    }
    else
        throw std::runtime_error(node_prefix_ + "::RobotDriverProvider::get_clear_positions_signal() trying to get an uninitialized vector");
}

RobotDriver::Functionality RobotDriverProvider::get_currently_active_functionality() const
{
    return currently_active_functionality_;
}

void RobotDriverProvider::send_joint_positions(const VectorXd &joint_positions)
{
    send_joint_states(joint_positions, VectorXd(), VectorXd());
}

/**
 * @brief Sends the current joint states through ROS.
 * @param joint_positions vector of . If not needed, use joint_positions=VectorXd().
 * @param joint_velocities. If not needed, use joint_velocities=VectorXd().
 * @param joint_forces. If not needed, use joint_forces=VectorXd().
 */
void RobotDriverProvider::send_joint_states(const VectorXd &joint_positions, const VectorXd &joint_velocities, const VectorXd &joint_forces)
{
    sensor_msgs::JointState ros_msg;
    if(joint_positions.size()>0)
        ros_msg.position = vectorxd_to_std_vector_double(joint_positions);
    if(joint_velocities.size()>0)
        ros_msg.velocity = vectorxd_to_std_vector_double(joint_velocities);
    if(joint_forces.size()>0)
        ros_msg.effort = vectorxd_to_std_vector_double(joint_forces);
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

void RobotDriverProvider::send_home_state(const VectorXi &home_state)
{
    std_msgs::Int32MultiArray ros_msg_home_state;
    ros_msg_home_state.data = vectorxi_to_std_vector_int(home_state);
    publisher_home_state_.publish(ros_msg_home_state);
}

bool RobotDriverProvider::is_enabled(const RobotDriver::Functionality& supported_functionality) const
{
    switch(supported_functionality)
    {
    case RobotDriver::Functionality::PositionControl:
        return target_joint_positions_.size() > 0;
    case RobotDriver::Functionality::VelocityControl:
        return target_joint_velocities_.size() > 0;
    case RobotDriver::Functionality::ForceControl:
        return target_joint_forces_.size() > 0;
    case RobotDriver::Functionality::Homing:
        return homing_signal_.size() > 0;
    case RobotDriver::Functionality::ClearPositions:
        return clear_positions_signal_.size() > 0;
    default:
        throw std::runtime_error(node_prefix_ + "::RobotDriverProvider::is_enabled() unknown control mode");
    }
}

}

