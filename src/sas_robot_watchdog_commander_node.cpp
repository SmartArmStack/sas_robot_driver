/*
# Copyright (c) 2016-2025 Murilo Marques Marinho
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
#   Author: Juan Jose Quiroz Omana, email: juanjose.quirozomana@manchester.ac.uk
#
# ################################################################*/

#include <rclcpp/rclcpp.hpp>
#include <sas_core/sas_clock.hpp>
#include <sas_common/sas_common.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <dqrobotics/utils/DQ_Math.h>

using namespace DQ_robotics;

#include<signal.h>
static std::atomic_bool kill_this_process(false);
void sig_int_handler(int)
{
    kill_this_process = true;
}

int main(int argc, char** argv)
{
    if(signal(SIGINT, sig_int_handler) == SIG_ERR)
        throw std::runtime_error("::Error setting the signal int handler.");


    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    auto node = std::make_shared<rclcpp::Node>("sas_robot_watchdog_commander_node");

    double thread_sampling_time_sec;
    sas::get_ros_parameter(node, "thread_sampling_time_sec", thread_sampling_time_sec);

    double period;
    sas::get_ros_parameter(node, "watchdog_period", period);

    double maximum_acceptable_delay;
    sas::get_ros_parameter(node, "watchdog_maximum_acceptable_delay", maximum_acceptable_delay);

    std::string robot_name;
    sas::get_ros_parameter(node,"robot_name", robot_name);
    RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::thread_sampling_time_sec: " + std::to_string(thread_sampling_time_sec));
    RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::robot_name: " + robot_name);
    RCLCPP_INFO_STREAM_ONCE(node->get_logger(), "::Parameters OK.");

    // Initialize the RobotDriverClient
    // This client is not allowed to command the robot joints. To achieve this behaviour,
    // we blacklist the JOINT_CONTROL mode.
    using MODE_BLACKLIST_FLAG = sas::RobotDriverClient::MODE_BLACKLIST_FLAG;
    std::vector<MODE_BLACKLIST_FLAG> blacklist_mode = {MODE_BLACKLIST_FLAG::JOINT_CONTROL};
    sas::RobotDriverClient rdi(node, robot_name, blacklist_mode);


    sas::Clock clock{thread_sampling_time_sec};
    clock.init();
    // For some iterations. Note that this can be stopped with CTRL+C.
    while (!kill_this_process)
    {
        clock.update_and_sleep();
        RCLCPP_INFO_STREAM_ONCE(node->get_logger(),"Watchdog status: true");
        rdi.send_watchdog_trigger(true, period, maximum_acceptable_delay);
        rclcpp::spin_some(node);
    }


}
