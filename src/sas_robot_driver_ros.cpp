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
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>

namespace sas
{

RobotDriverROS::RobotDriverROS(std::shared_ptr<Node> &node,
                               const std::shared_ptr<RobotDriver> &robot_driver,
                               const RobotDriverROSConfiguration &configuration,
                               std::atomic_bool *kill_this_node):
    node_(node),
    configuration_(configuration),
    kill_this_node_(kill_this_node),
    robot_driver_(robot_driver),
    clock_(configuration.thread_sampling_time_sec),
    robot_driver_provider_(node,configuration_.robot_driver_provider_prefix)
{

}

int RobotDriverROS::control_loop()
{
    try{
        clock_.init();

        RCLCPP_INFO_STREAM(node_->get_logger(),"::Waiting to connect with robot...");
        robot_driver_->connect();
        RCLCPP_INFO_STREAM(node_->get_logger(),"::Connected to robot.");

        RCLCPP_INFO_STREAM(node_->get_logger(),"::Initializing robot...");
        robot_driver_->initialize();
        RCLCPP_INFO_STREAM(node_->get_logger(),"::Robot initialized.");

        while(not _should_shutdown())
        {
            clock_.update_and_sleep();

            rclcpp::spin_some(node_);
            if(robot_driver_provider_.is_enabled())
            {
                robot_driver_->set_target_joint_positions(robot_driver_provider_.get_target_joint_positions());
            }
            if(robot_driver_provider_.is_enabled(RobotDriver::Functionality::VelocityControl))
            {
                 try{robot_driver_->set_target_joint_velocities(robot_driver_provider_.get_target_joint_velocities());} catch(...){}
            }
            if(robot_driver_provider_.is_enabled(RobotDriver::Functionality::ForceControl))
            {
                try{robot_driver_->set_target_joint_torques(robot_driver_provider_.get_target_joint_forces());} catch(...){}
            }


            auto joint_positions{robot_driver_->get_joint_positions()};
            VectorXd joint_velocities;
            try{joint_velocities = robot_driver_->get_joint_velocities();} catch(...){}
            VectorXd joint_torques;
            try{joint_torques = robot_driver_->get_joint_torques();} catch(...){}

            robot_driver_provider_.send_joint_states(joint_positions, joint_velocities, joint_torques);
            robot_driver_provider_.send_joint_limits(robot_driver_->get_joint_limits());
            rclcpp::spin_some(node_);
        }
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"::Exception caught::" << e.what());
    }
    catch(...)
    {
        RCLCPP_ERROR_STREAM(node_->get_logger(),"::Unexpected error or exception caught");
    }

    return 0;
}


bool RobotDriverROS::_should_shutdown() const
{
    return (*kill_this_node_);
}


RobotDriverROS::~RobotDriverROS()
{
    robot_driver_->deinitialize();
    robot_driver_->disconnect();
}
}
