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
#      - Added the Watchdog functionality.
#      - Renamed robot_driver_provider_ to robot_driver_server_
#      - Added a new std::optional parameter in RobotDriverROSConfiguration to define the watchdog period
#
*/

#pragma once

#include <atomic>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sas_core/sas_shutdown_signaler.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_core/sas_robot_driver.hpp>
#include <sas_robot_driver/sas_robot_driver_server.hpp>

using namespace rclcpp;

namespace sas
{

/**
 * @brief Configuration structure for RobotDriverROS.
 *
 * Holds configuration values required to initialize a RobotDriverROS
 * instance, including topic prefixes, control thread timing and optional
 * watchdog parameters.
 */
struct RobotDriverROSConfiguration
{
    /// Prefix that identifies the robot driver provider node/topic namespace.
    std::string robot_driver_provider_prefix;

    /// Control thread sampling time in seconds.
    double thread_sampling_time_sec;

    /// Watchdog period in seconds. If <= 0 the watchdog may be considered disabled.
    double  watchdog_period_in_seconds;

    /// Joint position minimum limits (q_min.size() == number of joints).
    std::vector<double> q_min;

    /// Joint position maximum limits (q_max.size() == number of joints).
    std::vector<double> q_max;
};

/**
 * @brief ROS integration class for a RobotDriver implementation.
 *
 * RobotDriverROS manages the ROS node, integrates a RobotDriver instance with
 * ROS topics/services and runs the control loop. It also manages watchdog
 * handling and shutdown signaling between ROS and the RobotDriver.
 */
class RobotDriverROS
{
private:
    std::shared_ptr<Node> node_;

    RobotDriverROSConfiguration configuration_;
    std::atomic_bool* kill_this_node_; //Deprecated
    std::shared_ptr<ShutdownSignaler> shutdown_signaler_;
    std::shared_ptr<RobotDriver> robot_driver_;
    Clock clock_;
    RobotDriverServer robot_driver_server_;
    bool watchdog_started_;
    double watchdog_period_in_seconds_;
    double watchdog_maximum_acceptable_delay_in_seconds_;
    bool _should_shutdown() const;

public:
    RobotDriverROS(const RobotDriverROS&)=delete;
    RobotDriverROS()=delete;

    /**
     * @brief Construct a new RobotDriverROS object.
     *
     * @param node Shared pointer to the rclcpp::Node used for ROS communication.
     * @param robot_driver Shared pointer to the RobotDriver implementation to be wrapped.
     * @param configuration Configuration parameters for ROS integration and control loop.
     * @param shutdown_signaler Shared pointer used to signal shutdown between components.
     */
    RobotDriverROS(std::shared_ptr<Node>& node,
                   const std::shared_ptr<RobotDriver>& robot_driver,
                   const RobotDriverROSConfiguration& configuration,
                   const std::shared_ptr<ShutdownSignaler>& shutdown_signaler_);

    [[deprecated("Use RobotDriver(const std::shared_ptr<ShutdownSignaler>& shutdown_signaler_) instead.")]]
    RobotDriverROS(std::shared_ptr<Node>& node,
                   const std::shared_ptr<RobotDriver>& robot_driver,
                   const RobotDriverROSConfiguration& configuration,
                   std::atomic_bool* kill_this_node);

    /**
     * @brief Destructor; stops the control loop and performs cleanup.
     */
    ~RobotDriverROS();

    /**
     * @brief Run the control loop.
     *
     * This method contains the main control loop and will typically run until
     * a shutdown condition is met. Returns an integer status code on exit.
     *
     * @return int Exit/status code.
     */
    int control_loop();
};

}
