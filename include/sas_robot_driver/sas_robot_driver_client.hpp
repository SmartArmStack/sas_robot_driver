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
# ################################################################
# Contributors:
#
#   1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
#      Added the Watchdog functionality.
#
*/

#include <atomic>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sas_msgs/msg/watchdog_trigger.hpp>
#include <sas_core/sas_robot_driver.hpp>
#include <sas_core/sas_object.hpp>
#include <sas_msgs/msg/bool.hpp>


using namespace rclcpp;

namespace sas
{

/**
 * @brief Client interface for robot driver ROS topics.
 *
 * RobotDriverClient provides methods to publish target joint commands and
 * control signals, and to subscribe to feedback topics such as joint states,
 * joint limits and home state. It supports optional mode blacklisting to
 * disable certain functionalities from the client side.
 */
class RobotDriverClient: private sas::Object
{
public:
    /**
     * @brief Flags used to blacklist client modes when constructing the client.
     */
    enum class MODE_BLACKLIST_FLAG
    {
        JOINT_CONTROL=0,
        JOINT_MONITORING,
        WATCHDOG_CONTROL
    };
private:
    std::shared_ptr<Node> node_;

    std::vector<MODE_BLACKLIST_FLAG> blacklisted_modes_;
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
    Publisher<std_msgs::msg::Int32MultiArray>  ::SharedPtr publisher_homing_signal_;
    Publisher<std_msgs::msg::Int32MultiArray>  ::SharedPtr publisher_clear_positions_signal_;
    Publisher<sas_msgs::msg::WatchdogTrigger>  ::SharedPtr publisher_watchdog_trigger_;
    Publisher<sas_msgs::msg::Bool>             ::SharedPtr publisher_shutdown_signal_;

    void _callback_joint_states(const sensor_msgs::msg::JointState& msg);
    void _callback_joint_limits_min(const std_msgs::msg::Float64MultiArray& msg);
    void _callback_joint_limits_max(const std_msgs::msg::Float64MultiArray& msg);
    void _callback_home_states(const std_msgs::msg::Int32MultiArray& msg);
public:
    RobotDriverClient() = delete;
    RobotDriverClient(const RobotDriverClient&) = delete;

    /**
     * @brief Construct a new RobotDriverClient
     *
     * @param node Shared pointer to the rclcpp::Node used for communication.
     * @param topic_prefix Topic prefix used to compose topic/service names. Defaults to "GET_FROM_NODE".
     * @param blacklisted_modes Optional list of modes to blacklist/disable on the client side.
     */
    RobotDriverClient(const std::shared_ptr<Node> &node,
                      const std::string topic_prefix="GET_FROM_NODE",
                      const std::vector<MODE_BLACKLIST_FLAG>& blacklisted_modes = std::vector<MODE_BLACKLIST_FLAG>{});

    /**
     * @brief Publish target joint positions.
     *
     * @param target_joint_positions Vector containing desired joint positions.
     */
    void send_target_joint_positions(const VectorXd& target_joint_positions);

    /**
     * @brief Publish target joint velocities.
     *
     * @param target_joint_velocities Vector containing desired joint velocities.
     */
    void send_target_joint_velocities(const VectorXd& target_joint_velocities);

    /**
     * @brief Publish target joint forces/torques.
     *
     * @param target_joint_forces Vector containing desired joint forces/torques.
     */
    void send_target_joint_forces(const VectorXd& target_joint_forces);

    /**
     * @brief Send a homing signal to the robot.
     *
     * @param homing_signal Vector of integers representing homing signals per joint.
     */
    void send_homing_signal(const VectorXi& homing_signal);

    /**
     * @brief Send a signal to clear positions on the controller side.
     *
     * @param clear_positions_signal Vector of integers representing clear position commands per joint.
     */
    void send_clear_positions_signal(const VectorXi& clear_positions_signal);

    /**
     * @brief Trigger the watchdog from the client.
     *
     * @param watchdog_trigger_status true keep watchdog alive, false to kill.
     * @param period_in_seconds Watchdog period in seconds (default: 1.0).
     * @param maximum_acceptable_delay Maximum acceptable delay in seconds for the watchdog (default: 0.5).
     */
    void send_watchdog_trigger(const bool& watchdog_trigger_status,
                               const double& period_in_seconds = 1.0,
                               const double& maximum_acceptable_delay = 0.5);

    /**
     * @brief Request a shutdown via the robot driver topics.
     */
    void send_shutdown_signal();

    /**
     * @brief Get the last received joint positions.
     *
     * @return VectorXd Joint positions vector.
     */
    VectorXd get_joint_positions() const;

    /**
     * @brief Get the last received joint velocities.
     *
     * @return VectorXd Joint velocities vector.
     */
    VectorXd get_joint_velocities() const;

    /**
     * @brief Get the last received joint forces.
     *
     * @return VectorXd Joint forces vector.
     */
    VectorXd get_joint_forces() const;

    /**
     * @brief Retrieve the joint limits as a tuple (min, max).
     *
     * @return std::tuple<VectorXd, VectorXd> Pair of vectors (min_limits, max_limits).
     */
    std::tuple<VectorXd, VectorXd> get_joint_limits() const;

    /**
     * @brief Get the last received home states vector.
     *
     * @return VectorXi Home states per joint.
     */
    VectorXi get_home_states() const;

    /**
     * @brief Check whether the client is enabled for a given functionality.
     *
     * @param supported_functionality The functionality to check (default: PositionControl).
     * @return true if enabled, false otherwise.
     */
    bool is_enabled(const RobotDriver::Functionality& supported_functionality=RobotDriver::Functionality::PositionControl) const;

    /**
     * @brief Get the configured topic prefix.
     *
     * @return std::string Topic prefix used by this client.
     */
    std::string get_topic_prefix() const;
};

}

