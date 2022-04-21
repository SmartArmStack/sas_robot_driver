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
#   Author: Murilo M. Marinho, email: murilo@g.ecc.u-tokyo.ac.jp
#
# ################################################################*/
#include <sas_robot_driver/sas_robot_driver_ros.h>
#include <dqrobotics/utils/DQ_Math.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>

namespace sas
{

RobotDriverROS::RobotDriverROS(ros::NodeHandle &nodehandle, const RobotDriverROSConfiguration &configuration, std::atomic_bool *kill_this_node):
    configuration_(configuration),
    kill_this_node_(kill_this_node),
    clock_(configuration.thread_sampling_time_nsec),
    robot_driver_provider_(nodehandle,configuration_.robot_driver_provider_prefix)
{

}

int RobotDriverROS::control_loop()
{
    try{
        clock_.init();
        ROS_INFO_STREAM(ros::this_node::getName() << "::Waiting to connect with robot...");
        robot_driver_->connect();
        robot_driver_->initialize();
        ROS_INFO_STREAM(ros::this_node::getName() << "::Connected to robot.");

        while(not _should_shutdown())
        {
            clock_.update_and_sleep();

            ros::spinOnce();
            if(robot_driver_provider_.is_enabled())
            {
                robot_driver_->set_target_joint_positions(robot_driver_provider_.get_target_joint_positions());
            }

            robot_driver_provider_.send_joint_positions(robot_driver_->get_joint_positions());
            robot_driver_provider_.send_joint_limits(robot_driver_->get_joint_limits());
            ros::spinOnce();
        }
    }
    catch(const std::exception& e)
    {
        ROS_WARN_STREAM(ros::this_node::getName() + "::Error or exception caught::" << e.what());
    }
    catch(...)
    {
        ROS_WARN_STREAM(ros::this_node::getName() + "::Unexpected error or exception caught");
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
