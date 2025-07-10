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
# 2025.07.10: Removing CoppeliaSim and moving it to sas_robot_driver_coppeliasim
*/
#include "sas_robot_driver_ros_composer.hpp"
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
#include <dqrobotics/utils/DQ_Constants.h>
#include <sas_core/sas_core.hpp>
#include <sas_core/sas_clock.hpp>

namespace sas
{
RobotDriverROSComposer::RobotDriverROSComposer(const RobotDriverROSComposerConfiguration &configuration,
                                               std::shared_ptr<Node> &node,
                                               std::atomic_bool *break_loops):
    RobotDriver(break_loops),
    node_(node),
    configuration_(configuration)
{

    for(const std::string& topic_prefix: configuration.robot_driver_client_names)
    {
        RCLCPP_INFO_STREAM(node_->get_logger(),"::Adding RobotDriverClient driver with prefix "+topic_prefix);
        robot_driver_clients_.push_back(std::unique_ptr<RobotDriverClient>(new RobotDriverClient(node,topic_prefix)));
    }

    if(configuration_.override_joint_limits_with_robot_parameter_file)
    {
        DQ_robotics::DQ_SerialManipulatorDH smdh = DQ_robotics::DQ_JsonReader::get_from_json<DQ_robotics::DQ_SerialManipulatorDH>(configuration_.robot_parameter_file_path);
        joint_limits_ = {smdh.get_lower_q_limit(),smdh.get_upper_q_limit()};
    }
}

VectorXd RobotDriverROSComposer::get_joint_positions()
{

    VectorXd joint_positions;
    for(const auto& interface : robot_driver_clients_)
    {
        joint_positions = concatenate(joint_positions, interface->get_joint_positions());
    }
    return joint_positions;
}

void RobotDriverROSComposer::set_target_joint_positions(const VectorXd &set_target_joint_positions_rad)
{
    int accumulator = 0;
    for(const auto& interface : robot_driver_clients_)
    {
        interface->send_target_joint_positions(set_target_joint_positions_rad.segment(accumulator,interface->get_joint_positions().size()));
        accumulator+=interface->get_joint_positions().size();
    }
}

void RobotDriverROSComposer::set_joint_limits(const std::tuple<VectorXd, VectorXd>&)
{
    throw std::runtime_error("RobotDriverROSComposer::set_joint_limits::Not accepted.");
}

void RobotDriverROSComposer::connect()
{
    //nothing to do
}

void RobotDriverROSComposer::disconnect()
{
    //nothing to do
}

void RobotDriverROSComposer::initialize()
{

    bool initialized = false;
    while(not initialized and not (*break_loops_))
    {
        rclcpp::spin_some(node_);
        initialized = true;
        for(const auto& interface : robot_driver_clients_)
        {
            if(not interface->is_enabled())
                initialized = false;
        }
    }
}

void RobotDriverROSComposer::deinitialize()
{
    //nothing to do
}

RobotDriverROSComposer::~RobotDriverROSComposer()=default;

std::tuple<VectorXd, VectorXd> RobotDriverROSComposer::get_joint_limits()
{
    if(!configuration_.override_joint_limits_with_robot_parameter_file)
    {
        VectorXd joint_positions_min;
        VectorXd joint_positions_max;
        for(const auto& interface : robot_driver_clients_)
        {
            auto [joint_positions_min_l, joint_positions_max_l] = interface->get_joint_limits();
            joint_positions_min = concatenate(joint_positions_min, joint_positions_min_l);
            joint_positions_max = concatenate(joint_positions_max, joint_positions_max_l);
        }
        return {joint_positions_min, joint_positions_max};
    }
    return joint_limits_;
}

}
