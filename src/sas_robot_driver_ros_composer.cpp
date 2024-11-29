/*
# Copyright (c) 2016-2024 Murilo Marques Marinho
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
    vi_ = std::make_shared<DQ_CoppeliaSimInterfaceZMQ>();
    cstm_ = std::make_unique<CoppeliaSimThreadManager>(vi_,
                                                       configuration_.coppeliasim_robot_joint_names,
                                                       break_loops);


    if(configuration.use_real_robot)
    {
        for(const std::string& topic_prefix: configuration.robot_driver_client_names)
        {
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
        return cstm_->get_joint_positions();
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
        cstm_->set_joint_positions(set_target_joint_positions_rad);
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
        if(!vi_->connect(configuration_.coppeliasim_ip,
                         configuration_.coppeliasim_port,
                         1000))
        {
            throw std::runtime_error("::Unable to connect to CoppeliaSim.");
        }
        RCLCPP_INFO_STREAM(node_->get_logger(),"::Connected to CoppeliaSim");
    }
}

void RobotDriverROSComposer::disconnect()
{
    if(configuration_.use_coppeliasim)
        vi_->disconnect();
}

void RobotDriverROSComposer::initialize()
{
    if(configuration_.use_real_robot)
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
        if(configuration_.use_coppeliasim)
        {
            //Send initial values of the real robot to CoppeliaSim
            vi_->set_joint_positions(configuration_.coppeliasim_robot_joint_names,get_joint_positions());
            if(configuration_.coppeliasim_dynamically_enabled_)
                vi_->set_joint_target_positions(configuration_.coppeliasim_robot_joint_names,get_joint_positions());
            cstm_->start_loop();
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
    if(!configuration_.override_joint_limits_with_robot_parameter_file && configuration_.use_real_robot)
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
    else
    {
        if(configuration_.use_coppeliasim)
        {
            //TODO: Obtain the joint limits from the simulator. This does not seem to be trivial as of now.
            int dof = get_joint_positions().size();
            auto joint_positions_max = VectorXd::Ones(dof)*2*pi;
            auto joint_positions_min = -joint_positions_max;
            return {joint_positions_min, joint_positions_max};
        }
    }
    return joint_limits_;
}

RobotDriverROSComposer::CoppeliaSimThreadManager::CoppeliaSimThreadManager(
    const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> &vi,
    const std::vector<std::string> &joint_names,
    std::atomic_bool *break_loops):
    vi_(vi),
    break_loops_(break_loops),
    joint_names_(joint_names)
{

}

RobotDriverROSComposer::CoppeliaSimThreadManager::~CoppeliaSimThreadManager()
{
    if(thread_ and thread_->joinable())
        thread_->join();
}

VectorXd RobotDriverROSComposer::CoppeliaSimThreadManager::get_joint_positions()
{
    std::lock_guard<std::mutex> lock(q_from_ci_mutex_);
    return q_from_ci_;
}

void RobotDriverROSComposer::CoppeliaSimThreadManager::set_joint_positions(const VectorXd &q)
{
    std::unique_lock<std::mutex> lock(q_to_ci_mutex_, std::try_to_lock);
    if(lock.owns_lock())
    {
        q_to_ci_ = q;
    }
}

void RobotDriverROSComposer::CoppeliaSimThreadManager::start_loop()
{
    if(!thread_)
    {
        thread_ = std::make_unique<std::thread>(&CoppeliaSimThreadManager::loop, this);
    }
}

void RobotDriverROSComposer::CoppeliaSimThreadManager::loop()
{
    sas::Clock clock(0.01);
    clock.init();
    while(!(*break_loops_))
    {
        {
            std::unique_lock<std::mutex> lock(q_from_ci_mutex_, std::try_to_lock);
            if(lock.owns_lock())
            {
                q_from_ci_ = vi_->get_joint_positions(joint_names_);
            }
        }
        clock.update_and_sleep();
        {
            std::scoped_lock<std::mutex> lock(q_to_ci_mutex_);
            if(q_to_ci_.size() > 0)
            {
                vi_->set_joint_positions(joint_names_,q_to_ci_);
                vi_->set_joint_target_positions(joint_names_,q_to_ci_);
            }
        }
    }
}


}
