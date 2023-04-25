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
# ################################################################*/

#include <exception>
#include <atomic>

#include <eigen3/Eigen/Dense>

#include <sas_core/sas_robot_driver.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>

#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>

using namespace Eigen;

namespace sas
{

struct RobotDriverROSComposerConfiguration
{
    bool use_real_robot;
    std::vector<std::string> vrep_robot_joint_names;
    std::string vrep_ip;
    int vrep_port;
    bool vrep_dynamically_enabled_ = false;
    std::vector<std::string> robot_driver_interface_topic_prefixes;
    std::string robot_parameter_file_path;
};

class RobotDriverROSComposer: public RobotDriver
{
protected:
    std::shared_ptr<Node> node_;

    RobotDriverROSComposerConfiguration configuration_;
    DQ_VrepInterface vi_;
    std::vector<std::unique_ptr<sas::RobotDriverClient>> robot_driver_interface_vector_;
    //std::atomic_bool* break_loops_;
    //std::tuple<VectorXd, VectorXd> joint_limits_;
    //RobotDriver(std::atomic_bool* break_loops);
    RobotDriverROSComposer()=delete;
    RobotDriverROSComposer(const RobotDriverROSComposer&)=delete;
public:
    RobotDriverROSComposer(const RobotDriverROSComposerConfiguration& configuration,
                           std::shared_ptr<Node>& node,
                           std::atomic_bool *break_loops);

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& set_target_joint_positions_rad) override;
    void set_joint_limits(const std::tuple<VectorXd, VectorXd>&) override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

    ~RobotDriverROSComposer();

};
}



