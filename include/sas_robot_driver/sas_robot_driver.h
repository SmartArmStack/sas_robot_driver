#pragma once
/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
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
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/

#include <exception>
#include <atomic>

#include <eigen3/Eigen/Dense>

using namespace Eigen;

namespace sas
{
class RobotDriver
{
protected:
    std::atomic_bool* break_loops_;

    RobotDriver(std::atomic_bool* break_loops);

    RobotDriver()=delete;
    RobotDriver(const RobotDriver&)=delete;
public:
    enum class Functionality{
        None=0,
        PositionControl,
        VelocityControl,
        ForceControl,
        Homing,
        ClearPositions
    };

    virtual VectorXd get_joint_positions() = 0;
    virtual void set_target_joint_positions(const VectorXd& set_target_joint_positions_rad) = 0;
    virtual std::tuple<VectorXd, VectorXd> get_joint_limits() = 0;
    virtual void set_joint_limits(const std::tuple<VectorXd, VectorXd>& joint_limits) = 0;

    virtual void connect()=0;
    virtual void disconnect()=0;

    virtual void initialize()=0;
    virtual void deinitialize()=0;
};
}



