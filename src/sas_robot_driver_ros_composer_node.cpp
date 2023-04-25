/*
# Copyright (c) 2016-2022 Murilo Marques Marinodeo
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
#   Author: Murilo M. Marinodeo, email: murilomarinodeo@ieee.org
#
# ################################################################*/
#include <exception>
#include <rclcpp/rclcpp.hpp>
//#include <sas_common/sas_common.h>
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

template<class T>
void get_ros_param(std::shared_ptr<rclcpp::Node>& node, const std::string& name, T& t, const bool& is_local_name=true)
{
    const std::string prefix = is_local_name?node->get_name():"";
    if(!node->get_parameter(prefix+name,t))
    {
        throw std::runtime_error(prefix + "::Error loading " + name);
    }
}


int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
    {
        //throw std::runtime_error(ros::this_node::getName() + "::Error setting the signal int handler.");
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    //ros::init(argc, argv, "sas_robot_driver_ros_composer_node", ros::init_options::NoSigintHandler);

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    //ros::NodeHandle node;
    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_ros_composer_node");

    try
    {
        //ROS_INFO_STREAM(node->get_name()+"::Loading parameters from parameter server.");
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");
        RobotDriverROSComposerConfiguration robot_driver_ros_composer_configuration;
        get_ros_param(node,"/use_real_robot",robot_driver_ros_composer_configuration.use_real_robot);
        get_ros_param(node,"/vrep_robot_joint_names",robot_driver_ros_composer_configuration.vrep_robot_joint_names);
        get_ros_param(node,"/vrep_ip",robot_driver_ros_composer_configuration.vrep_ip);
        get_ros_param(node,"/vrep_port",robot_driver_ros_composer_configuration.vrep_port);
        get_ros_param(node,"/vrep_dynamically_enabled",robot_driver_ros_composer_configuration.vrep_dynamically_enabled_);
        get_ros_param(node,"/robot_driver_interface_node_prefixes",robot_driver_ros_composer_configuration.robot_driver_interface_topic_prefixes);
        get_ros_param(node,"/robot_parameter_file_path",robot_driver_ros_composer_configuration.robot_parameter_file_path);
        RobotDriverROSConfiguration robot_driver_ros_configuration;
        get_ros_param(node,"/thread_sampling_time_sec",robot_driver_ros_configuration.thread_sampling_time_sec);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_name();
        //ROS_INFO_STREAM(ros::this_node::getName()+"::Parameters OK.");
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");


        //ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating RobotDriverROSComposer.");
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverROSComposer.");
        auto robot_driver_composer = std::make_shared<sas::RobotDriverROSComposer>(robot_driver_ros_composer_configuration,node,&kill_this_process);
        //ROS_INFO_STREAM(ros::this_node::getName()+"::Instantiating RobotDriverROS.");
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver_composer,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);
        robot_driver_ros.control_loop();
    }
    catch (const std::exception& e)
    {
        //ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
    }

    return 0;
}
