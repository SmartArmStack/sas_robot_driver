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
#pragma once

#include <atomic>
#include <vector>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <sas_core/sas_clock.hpp>
#include <sas_core/sas_robot_driver.hpp>
#include <sas_robot_driver/sas_robot_driver_server.hpp>

using namespace rclcpp;

namespace sas
{

struct RobotDriverROSConfiguration
{
    std::string robot_driver_provider_prefix;
    double thread_sampling_time_sec;
    std::vector<double> q_min;
    std::vector<double> q_max;
};

class RobotDriverROS
{
private:
    std::shared_ptr<Node> node_;

    RobotDriverROSConfiguration configuration_;
    std::atomic_bool* kill_this_node_;
    std::shared_ptr<RobotDriver> robot_driver_;
    Clock clock_;
    RobotDriverServer robot_driver_provider_;

    bool _should_shutdown() const;

public:
    RobotDriverROS(const RobotDriverROS&)=delete;
    RobotDriverROS()=delete;

    RobotDriverROS(std::shared_ptr<Node>& node,
                   const std::shared_ptr<RobotDriver>& robot_driver,
                   const RobotDriverROSConfiguration& configuration,
                   std::atomic_bool* kill_this_node);
    ~RobotDriverROS();

    int control_loop();
};

}
