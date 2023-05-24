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
#
#      2. Juan Jose Quiroz Omana (juanjqogm@gmail.com)
#         - [2023/05/24] Added the methods get_joint_velocities(),
#                        get_joint_forces(), and set_target_joint_velocities().
#
# ################################################################*/


#include <sas_robot_driver/sas_robot_driver.h>

namespace sas
{

RobotDriver::RobotDriver(std::atomic_bool *break_loops):
    break_loops_(break_loops)
{

}

std::tuple<VectorXd, VectorXd> RobotDriver::get_joint_limits() const
{
    return joint_limits_;
}

void RobotDriver::set_joint_limits(const std::tuple<VectorXd, VectorXd> &joint_limits)
{
    joint_limits_ = joint_limits;
}


/**
 * @brief This method returns the joint velocities.
 * @return A vector containing the joint velocities.
 */
VectorXd RobotDriver::get_joint_velocities()
{
    return joint_velocities_;
}


/**
 * @brief This method returns the joint torques.
 * @return A vector containing the joint torques.
 */
VectorXd RobotDriver::get_joint_forces()
{
    return joint_forces_;
}


/**
 * @brief This method sets the desired target joint velocities.
 * @param The desired target joint velocities.
 */
void RobotDriver::set_target_joint_velocities(const VectorXd &set_target_joint_velocities_rad_per_second)
{
    desired_joint_velocities_ = set_target_joint_velocities_rad_per_second;
}

}
