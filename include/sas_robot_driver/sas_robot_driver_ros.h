/*
# Copyright (c) 2016-2023 Murilo Marques Marinho
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
#
#
#
# Contributors:
#      1. Murilo M. Marinho (murilomarinho@ieee.org)
#         - Original implementation.
#         - [2023/05/23] Initializing the robot_driver_'s joint limits in the constructor.
#
#      2. Juan Jose Quiroz Omana (juanjqogm@gmail.com)
#         - [2023/05/20] Added documentation.
#         - [2023/05/24] Modified the control_loop method to set the joint target velocities and
#                        Update the joint velocities and joint forces by using send_joint_states().
#
# ################################################################
*/

#pragma once

#include <atomic>
#include <vector>

//ROS related
#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <sas_robot_driver/sas_robot_driver.h>
#include <sas_robot_driver/sas_robot_driver_provider.h>
#include <sas_clock/sas_clock.h>

namespace sas
{

struct RobotDriverROSConfiguration
{
    std::string robot_driver_provider_prefix;
    int thread_sampling_time_nsec;
    std::vector<double> q_min;
    std::vector<double> q_max;
};

class RobotDriverROS
{
private:
    RobotDriverROSConfiguration configuration_;
    std::atomic_bool* kill_this_node_;
    RobotDriver* robot_driver_;
    Clock clock_;
    RobotDriverProvider robot_driver_provider_;

    bool _should_shutdown() const;

public:
    RobotDriverROS(const RobotDriverROS&)=delete;
    RobotDriverROS()=delete;

    RobotDriverROS(ros::NodeHandle& nodehandle, RobotDriver *robot_driver,
              const RobotDriverROSConfiguration& configuration,
              std::atomic_bool* kill_this_node);
    ~RobotDriverROS();

    int control_loop();
};

}
