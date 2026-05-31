#pragma once
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

#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <sas_core/sas_robot_driver.hpp>
#include <sas_core/sas_object.hpp>
#include <sas_msgs/msg/watchdog_trigger.hpp>
#include <sas_msgs/msg/bool.hpp>

using namespace rclcpp;

namespace sas
{

/**
 * @brief Server interface for robot driver ROS topics.
 *
 * RobotDriverServer exposes publishers for joint states, joint limits and home
 * state, and subscribes to control-related topics (target joint commands,
 * homing/clear signals and watchdog triggers). It provides getters for the
 * latest received commands, watchdog information and shutdown signal.
 */
class RobotDriverServer: private sas::Object
{
private:
    std::shared_ptr<Node> node_;

    std::string node_prefix_;
    RobotDriver::Functionality currently_active_functionality_;

    Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_joint_states_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_joint_limits_min_;
    Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_joint_limits_max_;
    Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_home_state_;


    Subscription<sas_msgs::msg::Bool>::SharedPtr subscriber_shutdown_signal_;
    bool shutdown_signal_;
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
    Subscription<sas_msgs::msg::WatchdogTrigger>::SharedPtr subscriber_watchdog_trigger_;
    bool watchdog_trigger_status_;
    bool watchdog_enabled_;
    double watchdog_period_in_seconds_;
    double watchdog_maximum_acceptable_delay_in_seconds_;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> time_point_from_the_client_;
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> time_point_from_the_server_;

    void _callback_shutdown_signal_(const sas_msgs::msg::Bool& msg);
    void _callback_target_joint_positions(const std_msgs::msg::Float64MultiArray &msg);
    void _callback_target_joint_velocities(const std_msgs::msg::Float64MultiArray &msg);
    void _callback_target_joint_forces(const std_msgs::msg::Float64MultiArray &msg);
    void _callback_homing_signal(const std_msgs::msg::Int32MultiArray& msg);
    void _callback_clear_positions_signal(const std_msgs::msg::Int32MultiArray &msg);
    void _callback_watchdog_trigger_state(const sas_msgs::msg::WatchdogTrigger& msg);
public:
    RobotDriverServer() = delete;
    RobotDriverServer(const RobotDriverServer&) = delete;

//see the discussion in sas_common to understand why this is commented out
//#ifdef IS_SAS_PYTHON_BUILD
//    RobotDriverServer(const std::string& node_prefix);
//#endif

    /**
     * @brief Construct a new RobotDriverServer
     *
     * @param node Shared pointer to the rclcpp::Node used for communication.
     * @param node_prefix Prefix used to compose ROS topic names (defaults to "GET_FROM_NODE").
     */
    RobotDriverServer(const std::shared_ptr<Node> &node, const std::string& node_prefix="GET_FROM_NODE");

    /**
     * @brief Get the most recent target joint positions received from clients.
     *
     * @return VectorXd Target joint positions vector.
     */
    VectorXd get_target_joint_positions() const;

    /**
     * @brief Get the most recent target joint velocities received from clients.
     *
     * @return VectorXd Target joint velocities vector.
     */
    VectorXd get_target_joint_velocities() const;

    /**
     * @brief Get the most recent target joint forces received from clients.
     *
     * @return VectorXd Target joint forces vector.
     */
    VectorXd get_target_joint_forces() const;

    /**
     * @brief Get the last received homing signal vector.
     *
     * @return VectorXi Homing signal per joint.
     */
    VectorXi get_homing_signal() const;

    /**
     * @brief Get the last received clear positions signal vector.
     *
     * @return VectorXi Clear positions signal per joint.
     */
    VectorXi get_clear_positions_signal();

    /**
     * @brief Get the currently active functionality of the robot driver.
     *
     * @return RobotDriver::Functionality Currently active functionality.
     */
    RobotDriver::Functionality get_currently_active_functionality() const;

    /**
     * @brief Check whether the server supports and is enabled for a functionality.
     *
     * @param supported_functionality The functionality to check (default: PositionControl).
     * @return true if supported and enabled, false otherwise.
     */
    bool is_enabled(const RobotDriver::Functionality& supported_functionality=RobotDriver::Functionality::PositionControl) const;

    /**
     * @brief Publish joint states (positions, velocities and forces) to clients.
     *
     * @param joint_positions Vector of joint positions.
     * @param joint_velocities Vector of joint velocities.
     * @param joint_forces Vector of joint forces.
     */
    void send_joint_states(const VectorXd& joint_positions,
                           const VectorXd& joint_velocities,
                           const VectorXd& joint_forces);

    /**
     * @brief Publish joint limits (min and max) to clients.
     *
     * @param joint_limits Tuple containing (min_limits, max_limits).
     */
    void send_joint_limits(const std::tuple<VectorXd, VectorXd>& joint_limits);

    /**
     * @brief Publish the home state vector to clients.
     *
     * @param home_state Vector representing home states per joint.
     */
    void send_home_state(const VectorXi& home_state);

    /**
     * @brief Get the time point received from the client for watchdog synchronization.
     *
     * @return std::chrono::time_point Time point from the client.
     */
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> get_watchdog_time_point_from_the_client() const;

    /**
     * @brief Get the time point captured on the server for watchdog synchronization.
     *
     * @return std::chrono::time_point Time point from the server.
     */
    std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> get_watchdog_time_point_from_the_server() const;

    /**
     * @brief Get the current watchdog trigger status as last received from the client.
     *
     * @return true if watchdog trigger is active, false otherwise.
     */
    bool get_watchdog_trigger_status() const;

    /**
     * @brief Query whether the watchdog is enabled on the server.
     *
     * @return true if enabled, false otherwise.
     */
    bool is_watchdog_enabled() const;

    /**
     * @brief Get the configured watchdog period in seconds.
     *
     * @return double Watchdog period in seconds.
     */
    double get_watchdog_period() const;

    /**
     * @brief Get the configured maximum acceptable delay for the watchdog in seconds.
     *
     * @return double Maximum acceptable delay in seconds.
     */
    double get_watchdog_maximum_acceptable_delay() const;

    /**
     * @brief Get the current shutdown signal state as last received from clients.
     *
     * @return true if a shutdown signal was received, false otherwise.
     */
    bool get_shutdown_signal() const;
};

}
