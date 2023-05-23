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
#         Responsible for the original implementation.
#
#      2. Juan Jose Quiroz Omana (juanjqogm@gmail.com) 
#         Added documentation.
#      
# ################################################################
*/

#include <sas_robot_driver/sas_robot_driver.h>

namespace sas
{

/**
 * @brief Constructor of the RobotDriver class
 * 
 * @param break_loops Flag used to break loops.
 */
RobotDriver::RobotDriver(std::atomic_bool *break_loops):
    break_loops_(break_loops)
{

}


/**
 * @brief This method returns the joint limits of your robotic system
 * 
 * @return joint_limits A tuple containing the joint limits
 *                      {q_min, q_max}, where q_min and q_max are
 *                      the lower and upper bounds, respectively.
 * 
 *       Example:
 * 
 *       VectorXd q_min;
 *       VectorXd q_max;
 *       std::tie(q_min, q_max) = get_joint_limits();
 * 
 */
std::tuple<VectorXd, VectorXd> RobotDriver::get_joint_limits() const
{
    return joint_limits_;
}


/**
 * @brief This method sets the joint limits of your robotic system.
 * 
 * @param joint_limits A tuple containing the joint limits
 *                      {q_min, q_max}, where q_min and q_max are
 *                      the lower and upper bounds, respectively.
 * 
 *       Example:
 * 
 *       VectorXd q_min = (VectorXd(3)<< -2,-2,-2).finished();
 *       VectorXd q_max = (VectorXd(3)<<  2, 2, 2).finished();
 *       std::tuple<VectorXd, VectorXd> joint_limits = {q_min, q_max};
 *       set_joint_limits(joint_limits);                
 */
void RobotDriver::set_joint_limits(const std::tuple<VectorXd, VectorXd> &joint_limits)
{
    joint_limits_ = joint_limits;
}

}
