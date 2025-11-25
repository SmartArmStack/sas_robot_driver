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
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################
# Contributors:
#
#   1. Juan Jose Quiroz Omana (juanjose.quirozomana@manchester.ac.uk)
#      - Added the Watchdog functionality.
#      - Renamed robot_driver_provider_ to robot_driver_server_
#
*/

#include <sas_common/sas_common.hpp>
#include <sas_robot_driver/sas_robot_driver_ros.hpp>
#include <dqrobotics/utils/DQ_Math.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>

/**
 * @brief are_approximately_equal returns true if two doubles are approximately equal.
 * @param a
 * @param b
 * @param epsilon The desired threshold
 * @return a boolean flag
 */
bool are_approximately_equal(const double &a, const double &b, const double &epsilon)
{
    return std::abs(a - b) < epsilon;
}

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
    robot_driver_server_(node,configuration_.robot_driver_provider_prefix),
    watchdog_started_{false}
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


            if (robot_driver_server_.get_shutdown_status())
                throw std::runtime_error("The shutdown command was received!");

            if(robot_driver_server_.is_enabled())
            {
                robot_driver_->set_target_joint_positions(robot_driver_server_.get_target_joint_positions());
            }
            if(robot_driver_server_.is_enabled(RobotDriver::Functionality::VelocityControl))
            {
                 try{robot_driver_->set_target_joint_velocities(robot_driver_server_.get_target_joint_velocities());} catch(...){}
            }
            if(robot_driver_server_.is_enabled(RobotDriver::Functionality::ForceControl))
            {
                try{robot_driver_->set_target_joint_torques(robot_driver_server_.get_target_joint_forces());} catch(...){}
            }
            if(robot_driver_server_.is_enabled(RobotDriver::Functionality::Watchdog))
            {
                if (!watchdog_started_)
                {   // This portion of code is executed only one time
                    // Initialize the watchdog.
                    watchdog_period_in_seconds_                   = robot_driver_server_.get_watchdog_period();
                    watchdog_maximum_acceptable_delay_in_seconds_ = robot_driver_server_.get_watchdog_maximum_acceptable_delay();

                    RCLCPP_INFO_STREAM(node_->get_logger(), "::Watchdog initialized with a " << watchdog_period_in_seconds_  << " second period");
                    // If the elapsed time between the triggers is higher than the watchdog period, an exception is thrown

                    RCLCPP_INFO_STREAM(node_->get_logger(), "::Watchdog initialized with a maximum acceptable delay of " <<watchdog_maximum_acceptable_delay_in_seconds_<< " seconds");
                    // If the time difference between the time point of signal that was sent (using the client computer's clock) and the time point
                    // when the watchdog signal was received (using the computer's clock on which the server is running) is higher than the watchdog_maximum_acceptable_delay,
                    // an exception is thrown by the robot driver.

                    const std::chrono::nanoseconds period = std::chrono::duration_cast<std::chrono::nanoseconds>(
                        std::chrono::duration<double>(watchdog_period_in_seconds_));
                    watchdog_started_ = true;
                    robot_driver_->watchdog_set_maximum_acceptable_delay(watchdog_maximum_acceptable_delay_in_seconds_);

                    //-----------------------------------------------------------------------------------------/
                    robot_driver_->watchdog_start(period);
                    //--- For developers: Do not put more code after this point---//
                }else{
                    // Check if the period and the maximum acceptable delay changed.
                    if (!are_approximately_equal(watchdog_period_in_seconds_,robot_driver_server_.get_watchdog_period(), DQ_robotics::DQ_threshold))
                        throw std::runtime_error("Invalid operation. The watchdog period changed from "+std::to_string(watchdog_period_in_seconds_)+
                                                 " to " +std::to_string(robot_driver_server_.get_watchdog_period()));


                    if (!are_approximately_equal(watchdog_maximum_acceptable_delay_in_seconds_, robot_driver_server_.get_watchdog_maximum_acceptable_delay(), DQ_robotics::DQ_threshold))
                        throw std::runtime_error("Invalid operation. The watchdog maximum acceptable delay changed from "+std::to_string(watchdog_maximum_acceptable_delay_in_seconds_)+
                                                 " to " +std::to_string(robot_driver_server_.get_watchdog_maximum_acceptable_delay()));
                }
                
                try{robot_driver_->watchdog_trigger(robot_driver_server_.get_watchdog_time_point_from_the_client(),
                                                robot_driver_server_.get_watchdog_time_point_from_the_server(),
                                                robot_driver_server_.get_watchdog_trigger_status());} catch(...){}
               

                // Any exception from the watchdog thread control loop will be rethrown by check_for_watchdog_exceptions(), and
                // consequently the main control loop must stop.
                robot_driver_->check_for_watchdog_exceptions();

            }


            auto joint_positions{robot_driver_->get_joint_positions()};
            VectorXd joint_velocities;
            try{joint_velocities = robot_driver_->get_joint_velocities();} catch(...){}
            VectorXd joint_torques;
            try{joint_torques = robot_driver_->get_joint_torques();} catch(...){}

            robot_driver_server_.send_joint_states(joint_positions, joint_velocities, joint_torques);
            robot_driver_server_.send_joint_limits(robot_driver_->get_joint_limits());
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
