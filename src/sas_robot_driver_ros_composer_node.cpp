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
#include <sas_common/sas_common.h>
#include "sas_robot_driver_ros_composer.h"
#include <sas_robot_driver/sas_robot_driver_ros.h>
using namespace sas;

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        throw std::runtime_error(ros::this_node::getName() + "::Error setting the signal int handler.");
    }

    ros::init(argc, argv, "sas_robot_driver_ros_composer_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;

    try
    {
        ROS_INFO_STREAM(ros::this_node::getName()+"::Loading parameters from parameter server.");
        RobotDriverROSComposerConfiguration robot_driver_ros_composer_configuration;
        get_ros_param(nh,"/use_real_robot",robot_driver_ros_composer_configuration.use_real_robot);
        get_ros_param(nh,"/vrep_robot_joint_names",robot_driver_ros_composer_configuration.vrep_robot_joint_names);
        get_ros_param(nh,"/vrep_ip",robot_driver_ros_composer_configuration.vrep_ip);
        get_ros_param(nh,"/vrep_port",robot_driver_ros_composer_configuration.vrep_port);
        if(ros::param::has(ros::this_node::getName()+"/vrep_dynamically_enabled")) //Added 2022/08/01
            get_ros_param(nh,"/vrep_dynamically_enabled",robot_driver_ros_composer_configuration.vrep_dynamically_enabled_);
        get_ros_param(nh,"/robot_driver_interface_node_prefixes",robot_driver_ros_composer_configuration.robot_driver_interface_topic_prefixes);
        get_ros_param(nh,"/robot_parameter_file_path",robot_driver_ros_composer_configuration.robot_parameter_file_path);
        RobotDriverROSConfiguration robot_driver_ros_configuration;
        get_ros_param(nh,"/thread_sampling_time_nsec",robot_driver_ros_configuration.thread_sampling_time_nsec);
        robot_driver_ros_configuration.robot_driver_provider_prefix = ros::this_node::getName();
        ROS_INFO_STREAM(ros::this_node::getName()+"::Parameters OK.");


        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating RobotDriverROSComposer.");
        sas::RobotDriverROSComposer robot_driver_composer(robot_driver_ros_composer_configuration,nh,&kill_this_process);
        ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(nh,
                                             &robot_driver_composer,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);
        robot_driver_ros.control_loop();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
    }

    return 0;
}
