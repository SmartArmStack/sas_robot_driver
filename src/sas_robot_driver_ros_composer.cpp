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
#include "sas_robot_driver_ros_composer.hpp"
//#include <ros/callback_queue_interface.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>
//#include <sas_common/sas_common.h>
#include <sas_core/sas_core.hpp>

namespace sas
{
RobotDriverROSComposer::RobotDriverROSComposer(const RobotDriverROSComposerConfiguration &configuration,
                                               std::shared_ptr<Node> &node,
                                               std::atomic_bool *break_loops):
    RobotDriver(break_loops),
    node_(node),
    configuration_(configuration),
    vi_(break_loops)
{
    if(configuration.use_real_robot)
    {
        for(const std::string& topic_prefix: configuration.robot_driver_client_names)
        {
            //ROS_INFO_STREAM(ros::this_node::getName()+"::Adding subrobot driver with prefix "+topic_prefix);
            RCLCPP_INFO_STREAM(node_->get_logger(),"::Adding RobotDriverClient driver with prefix "+topic_prefix);
            robot_driver_clients_.push_back(std::unique_ptr<RobotDriverClient>(new RobotDriverClient(node,topic_prefix)));
        }
    }

    if(configuration_.override_joint_limits_with_robot_parameter_file)
    {
        DQ_SerialManipulatorDH smdh = DQ_JsonReader::get_from_json<DQ_SerialManipulatorDH>(configuration_.robot_parameter_file_path);
        joint_limits_ = {smdh.get_lower_q_limit(),smdh.get_upper_q_limit()};
    }
}

VectorXd RobotDriverROSComposer::get_joint_positions()
{
    if(configuration_.use_real_robot)
    {
        VectorXd joint_positions;
        for(const auto& interface : robot_driver_clients_)
        {
            joint_positions = concatenate(joint_positions, interface->get_joint_positions());
        }
        return joint_positions;
    }
    else
    {
        return vi_.get_joint_positions(configuration_.coppeliasim_robot_joint_names);
    }
}

void RobotDriverROSComposer::set_target_joint_positions(const VectorXd &set_target_joint_positions_rad)
{
    if(configuration_.use_real_robot)
    {
        int accumulator = 0;
        for(const auto& interface : robot_driver_clients_)
        {
            interface->send_target_joint_positions(set_target_joint_positions_rad.segment(accumulator,interface->get_joint_positions().size()));
            accumulator+=interface->get_joint_positions().size();
        }
    }

    if(configuration_.use_coppeliasim)
    {
        if(configuration_.coppeliasim_dynamically_enabled_)
        {
            vi_.set_joint_target_positions(configuration_.coppeliasim_robot_joint_names,set_target_joint_positions_rad);
        }
        else
        {
            vi_.set_joint_positions(configuration_.coppeliasim_robot_joint_names,set_target_joint_positions_rad);
        }
    }
}

void RobotDriverROSComposer::set_joint_limits(const std::tuple<VectorXd, VectorXd>&)
{
    throw std::runtime_error("RobotDriverROSComposer::set_joint_limits::Not accepted.");
}

void RobotDriverROSComposer::connect()
{
    if(configuration_.use_coppeliasim)
    {
        if(!vi_.connect(configuration_.coppeliasim_ip,
                        configuration_.coppeliasim_port,
                        100,
                        10))
        {
            //throw std::runtime_error(ros::this_node::getName()+"::Unable to connect to CoppeliaSim.");
            throw std::runtime_error("::Unable to connect to CoppeliaSim.");
        }
        //ROS_INFO_STREAM(ros::this_node::getName()+"::Connected to CoppeliaSim");
        RCLCPP_INFO_STREAM(node_->get_logger(),"::Connected to CoppeliaSim");
    }
}

void RobotDriverROSComposer::disconnect()
{
    if(configuration_.use_coppeliasim)
        vi_.disconnect();
}

void RobotDriverROSComposer::initialize()
{
    if(configuration_.use_real_robot)
    {
        bool initialized = false;
        while(not initialized and not (*break_loops_))
        {
            //ros::spinOnce();
            rclcpp::spin_some(node_);
            initialized = true;
            for(const auto& interface : robot_driver_clients_)
            {
                if(not interface->is_enabled())
                    initialized = false;
            }
        }
        if(configuration_.use_coppeliasim)
        {
            //Send initial values of the real robot to CoppeliaSim
            vi_.set_joint_positions(configuration_.coppeliasim_robot_joint_names,get_joint_positions());
            if(configuration_.coppeliasim_dynamically_enabled_)
                vi_.set_joint_target_positions(configuration_.coppeliasim_robot_joint_names,get_joint_positions());
        }
    }
    else
    {
        //Call it once to initialize the CoppeliaSim streaming.
        vi_.get_joint_positions(configuration_.coppeliasim_robot_joint_names);
    }
}

void RobotDriverROSComposer::deinitialize()
{
    //nothing to do
}

RobotDriverROSComposer::~RobotDriverROSComposer()=default;

//Defined last because QTCreator messes up the identation because of the auto [,] operator.
std::tuple<VectorXd, VectorXd> RobotDriverROSComposer::get_joint_limits() const
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
