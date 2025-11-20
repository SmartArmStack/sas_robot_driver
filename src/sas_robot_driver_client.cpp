/*
# Copyright (c) 2016-2025 Murilo Marques Marinho
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
# ################################################################
# Contributors:
#
#   1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
#      Added the Watchdog functionality.
#
*/

#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <sas_conversions/sas_conversions.hpp>
#include <sas_common/sas_common.hpp>
using std::placeholders::_1;

namespace sas
{

void RobotDriverClient::_callback_joint_states(const sensor_msgs::msg::JointState &msg)
{
    const std::string this_topic(topic_prefix_ + "/get/joint_states");
    if(node_->count_publishers(this_topic)>1)
        throw std::runtime_error(this_topic + " must be exclusively published and there is more than one publisher connected.");

    joint_positions_ = std_vector_double_to_vectorxd(msg.position);
    joint_velocities_ = std_vector_double_to_vectorxd(msg.velocity);
    joint_forces_ = std_vector_double_to_vectorxd(msg.effort);
}

void RobotDriverClient::_callback_joint_limits_min(const std_msgs::msg::Float64MultiArray &msg)
{
    const std::string this_topic(topic_prefix_ + "/get/joint_positions_min");
    if(node_->count_publishers(this_topic)>1)
        throw std::runtime_error(this_topic + " must be exclusively published and there is more than one publisher connected.");

    joint_limits_min_ = std_vector_double_to_vectorxd(msg.data);
}

void RobotDriverClient::_callback_joint_limits_max(const std_msgs::msg::Float64MultiArray &msg)
{
    const std::string this_topic(topic_prefix_ + "/get/joint_positions_max");
    if(node_->count_publishers(this_topic)>1)
        throw std::runtime_error(this_topic + " must be exclusively published and there is more than one publisher connected.");

    joint_limits_max_ = std_vector_double_to_vectorxd(msg.data);
}

void RobotDriverClient::_callback_home_states(const std_msgs::msg::Int32MultiArray &msg)
{
    home_states_ = std_vector_int_to_vectorxi(msg.data);
}

/**
 * @brief mode_in_blacklist returns true if the specified mode is listed on the list of flags
 * @param mode The flag mode
 * @param list of flags the list of the flags
 * @return returns true if the specified mode is listed on list of flags. False otherwise.
 */
bool mode_in_blacklist(const RobotDriverClient::MODE_BLACKLIST_FLAG& mode,
                       const std::vector<RobotDriverClient::MODE_BLACKLIST_FLAG>& list_of_flags)
{
    //eturn(count(l.being(), l.end(), mode) == 0);
    return std::count(list_of_flags.begin(), list_of_flags.end(), mode) > 0;
}

RobotDriverClient::RobotDriverClient(const std::shared_ptr<Node> &node,
                                     const std::string topic_prefix,
                                     const std::vector<MODE_BLACKLIST_FLAG>& blacklisted_modes):
    sas::Object("sas::RobotDriverClient"),
    node_(node),
    blacklisted_modes_(blacklisted_modes),
    topic_prefix_(topic_prefix == "GET_FROM_NODE"? node->get_name() : topic_prefix)
{
    RCLCPP_INFO_STREAM(node_->get_logger(),"::Initializing RobotDriverClient with prefix " + topic_prefix);

    if(!mode_in_blacklist(MODE_BLACKLIST_FLAG::JOINT_CONTROL,blacklisted_modes_))
    {
        publisher_target_joint_positions_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix + "/set/target_joint_positions",1);
        publisher_target_joint_velocities_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix + "/set/target_joint_velocities",1);
        publisher_target_joint_forces_ = node->create_publisher<std_msgs::msg::Float64MultiArray>(topic_prefix + "/set/target_joint_forces",1);
        publisher_homing_signal_ = node->create_publisher<std_msgs::msg::Int32MultiArray>(topic_prefix + "/set/homing_signal",1);
        publisher_clear_positions_signal_ = node->create_publisher<std_msgs::msg::Int32MultiArray>(topic_prefix + "/set/clear_positions_signal",1);
    }

    if(!mode_in_blacklist(MODE_BLACKLIST_FLAG::WATCHDOG_CONTROL,blacklisted_modes_))
    {
        publisher_watchdog_trigger_ = node->create_publisher<sas_msgs::msg::WatchdogTrigger>(topic_prefix + "/set/watchdog_trigger", 1);
    }

    if(!mode_in_blacklist(MODE_BLACKLIST_FLAG::JOINT_MONITORING,blacklisted_modes_))
    {
        subscriber_joint_states_ = node->create_subscription<sensor_msgs::msg::JointState>(
                    topic_prefix + "/get/joint_states", 1, std::bind(&RobotDriverClient::_callback_joint_states, this, _1)
                    );
        subscriber_joint_limits_min_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                    topic_prefix + "/get/joint_positions_min", 1, std::bind(&RobotDriverClient::_callback_joint_limits_min, this, _1)
                    );
        subscriber_joint_limits_max_ = node->create_subscription<std_msgs::msg::Float64MultiArray>(
                    topic_prefix + "/get/joint_positions_max", 1, std::bind(&RobotDriverClient::_callback_joint_limits_max, this, _1)
                    );
        subscriber_home_state_ = node->create_subscription<std_msgs::msg::Int32MultiArray>(
                    topic_prefix + "/get/home_states", 1, std::bind(&RobotDriverClient::_callback_home_states, this, _1)
                    );
    }
}

void RobotDriverClient::send_target_joint_positions(const VectorXd &target_joint_positions)
{

    if (publisher_target_joint_positions_)
    {
        std_msgs::msg::Float64MultiArray ros_msg;
        ros_msg.data = vectorxd_to_std_vector_double(target_joint_positions);
        publisher_target_joint_positions_->publish(ros_msg);
    }
    else
        throw std::runtime_error("RobotDriverClient::"+std::string(__FUNCTION__)+"::This method is blacklisted");
}

void RobotDriverClient::send_target_joint_velocities(const VectorXd &target_joint_velocities)
{
    if(publisher_target_joint_velocities_)
    {
        std_msgs::msg::Float64MultiArray ros_msg;
        ros_msg.data = vectorxd_to_std_vector_double(target_joint_velocities);
        publisher_target_joint_velocities_->publish(ros_msg);
    }
    else
        throw std::runtime_error("RobotDriverClient::"+std::string(__FUNCTION__)+"::This method is blacklisted");

}

void RobotDriverClient::send_target_joint_forces(const VectorXd &target_joint_efforts)
{

    if (publisher_target_joint_forces_)
    {
        std_msgs::msg::Float64MultiArray ros_msg;
        ros_msg.data = vectorxd_to_std_vector_double(target_joint_efforts);
        publisher_target_joint_forces_->publish(ros_msg);
    }
    else
        throw std::runtime_error("RobotDriverClient::"+std::string(__FUNCTION__)+"::This method is blacklisted");
}

void RobotDriverClient::send_homing_signal(const VectorXi &homing_signal)
{
    if (publisher_homing_signal_)
    {
        std_msgs::msg::Int32MultiArray ros_msg;
        ros_msg.data = vectorxi_to_std_vector_int(homing_signal);
        publisher_homing_signal_->publish(ros_msg);
    }
    else
        throw std::runtime_error("RobotDriverClient::"+std::string(__FUNCTION__)+"::This method is blacklisted");
}

void RobotDriverClient::send_clear_positions_signal(const VectorXi &clear_positions_signal)
{
    if (publisher_clear_positions_signal_)
    {
        std_msgs::msg::Int32MultiArray ros_msg;
        ros_msg.data = vectorxi_to_std_vector_int(clear_positions_signal);
        publisher_clear_positions_signal_->publish(ros_msg);
    }
    else
        throw std::runtime_error("RobotDriverClient::"+std::string(__FUNCTION__)+"::This method is blacklisted");
}

/**
 * @brief RobotDriverClient::send_watchdog_trigger sends the Watchdog trigger
 * @param watchdog_trigger_status The desired status for the Watchdog
 * @param period_in_seconds The desired period in seconds.
 * @param maximum_acceptable_delay. The maximum allowed difference between the timepoint sent by the client and the timepoint
 *                                  registered by the server. This delay could be related to unsynchronised clocks between different computers
 *                                  or network delays.
 */
void RobotDriverClient::send_watchdog_trigger(const bool& watchdog_trigger_status,
                                              const double &period_in_seconds,
                                              const double &maximum_acceptable_delay)
{

    if (publisher_watchdog_trigger_)
    {
        sas_msgs::msg::WatchdogTrigger ros_msg;
        ros_msg.header = std_msgs::msg::Header();
        ros_msg.header.stamp = rclcpp::Clock().now();
        ros_msg.status = watchdog_trigger_status;
        ros_msg.period_in_seconds = period_in_seconds;
        ros_msg.maximum_acceptable_delay_in_seconds = maximum_acceptable_delay;
        publisher_watchdog_trigger_->publish(ros_msg);
    }
    else
        throw std::runtime_error("RobotDriverClient::"+std::string(__FUNCTION__)+"::This method is blacklisted");
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
    case sas::RobotDriver::Functionality::Watchdog:
        throw std::runtime_error(topic_prefix_+"::is_enabled() RobotDriver::Functionality::Watchdog has no meaning in RobotDriverInterface::is_enabled().");
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
