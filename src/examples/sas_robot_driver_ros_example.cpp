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
#include <rclcpp/rclcpp.hpp>
#include <sas_core/examples/sas_robot_driver_example.hpp>
#include <sas_conversions/eigen3_std_conversions.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>

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
        throw std::runtime_error("::Error setting the signal int handler.");
    }

    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);

    auto node = std::make_shared<rclcpp::Node>("sas_robot_driver_ros_example");

    try
    {
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Loading parameters from parameter server.");

        sas::RobotDriverExampleConfiguration configuration;
        node->get_parameter_or("robot_name",configuration.name,std::string("ExampleRobotInROS_123"));
        std::vector<double> initial_jointpositions;
        std::vector<double> initial_joint_positions_or = {1,2,3,4,5,6,7};
        node->get_parameter_or("initial_joint_positions",initial_jointpositions,initial_joint_positions_or);
        configuration.initial_joint_positions = sas::std_vector_double_to_vectorxd(initial_jointpositions);

        sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
        node->get_parameter_or("thread_sampling_time_sec",robot_driver_ros_configuration.thread_sampling_time_sec,0.001);
        robot_driver_ros_configuration.robot_driver_provider_prefix = node->get_name();

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");


        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverExample.");
        auto robot_driver_example = std::make_shared<sas::RobotDriverExample>(configuration,
                                                                              &kill_this_process);

        RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Instantiating RobotDriverROS.");
        sas::RobotDriverROS robot_driver_ros(node,
                                             robot_driver_example,
                                             robot_driver_ros_configuration,
                                             &kill_this_process);
        robot_driver_ros.control_loop();

    }
    catch (const std::exception& e)
    {
        //ROS_ERROR_STREAM(ros::this_node::getName() + "::Exception::" + e.what());
        RCLCPP_ERROR_STREAM_ONCE(node->get_logger(), std::string("::Exception::") + e.what());
    }

    //The [rclcpp] error is a known issue in Humble, possibly already fixed in rolling.
    //https://github.com/ros2/rclcpp/pull/2019
    RCLCPP_WARN_STREAM_ONCE(node->get_logger(), "::Ignore the [rclcpp] error below, it is a bug in ROS 2 Humble.");

    return 0;
}
