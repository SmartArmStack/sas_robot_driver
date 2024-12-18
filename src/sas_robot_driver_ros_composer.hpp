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

#include <atomic>
#include <memory>
#include <mutex>

#include <eigen3/Eigen/Dense>

#include <sas_core/sas_robot_driver.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>

#include <dqrobotics/interfaces/coppeliasim/DQ_CoppeliaSimInterfaceZMQ.h>

using namespace Eigen;

namespace sas
{

struct RobotDriverROSComposerConfiguration
{
    bool use_real_robot;

    bool use_coppeliasim;
    std::vector<std::string> coppeliasim_robot_joint_names;
    std::string coppeliasim_ip;
    int coppeliasim_port;
    bool coppeliasim_dynamically_enabled_ = false;

    std::vector<std::string> robot_driver_client_names;

    bool override_joint_limits_with_robot_parameter_file;
    std::string robot_parameter_file_path;
};

class RobotDriverROSComposer: public RobotDriver
{
protected:
    std::shared_ptr<Node> node_;

    RobotDriverROSComposerConfiguration configuration_;
    std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> vi_;
    std::vector<std::unique_ptr<sas::RobotDriverClient>> robot_driver_clients_;

    class CoppeliaSimThreadManager
    {
        std::unique_ptr<std::thread> thread_;
        std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ> vi_;
        std::vector<std::string> joint_names_;

        std::mutex q_from_ci_mutex_;
        VectorXd q_from_ci_;

        std::mutex q_to_ci_mutex_;
        VectorXd q_to_ci_;

        std::atomic_bool *break_loops_;

        CoppeliaSimThreadManager()=delete;
        CoppeliaSimThreadManager(const CoppeliaSimThreadManager&)=delete;

    public:
        CoppeliaSimThreadManager(const std::shared_ptr<DQ_CoppeliaSimInterfaceZMQ>& vi,
                                 const std::vector<std::string>& joint_names,
                                 std::atomic_bool *break_loops);
        ~CoppeliaSimThreadManager();
        VectorXd get_joint_positions();
        void set_joint_positions(const VectorXd& q);
        void start_loop();
        void loop();
    };

    std::unique_ptr<CoppeliaSimThreadManager> cstm_;

    RobotDriverROSComposer()=delete;
    RobotDriverROSComposer(const RobotDriverROSComposer&)=delete;
public:
    RobotDriverROSComposer(const RobotDriverROSComposerConfiguration& configuration,
                           std::shared_ptr<Node>& node,
                           std::atomic_bool *break_loops);

    VectorXd get_joint_positions() override;
    void set_target_joint_positions(const VectorXd& set_target_joint_positions_rad) override;
    std::tuple<VectorXd, VectorXd> get_joint_limits() override;
    void set_joint_limits(const std::tuple<VectorXd, VectorXd>&) override;

    void connect() override;
    void disconnect() override;

    void initialize() override;
    void deinitialize() override;

    ~RobotDriverROSComposer();

};
}


