#pragma once
/*
# Copyright (c) 2016-2020 Murilo Marques Marinho
#
#    This file is part of rosilo_robot_driver.
#
#    rosilo_robot_driver is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    rosilo_robot_driver is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with rosilo_robot_driver.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@nml.t.u-tokyo.ac.jp
#
# ################################################################*/

#include <atomic>
#include <tuple>

#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <rosilo_robot_driver/rosilo_robot_driver.h>

namespace rosilo
{

class RobotDriverProvider
{
private:
    std::atomic_bool enabled_;

    ros::Publisher publisher_joint_states_;
    ros::Publisher publisher_joint_limits_min_;
    ros::Publisher publisher_joint_limits_max_;

    ros::Subscriber subscriber_target_joint_positions_;
    VectorXd target_joint_positions_;

    void _callback_target_joint_positions(const std_msgs::Float64MultiArrayConstPtr& msg);
public:
    RobotDriverProvider() = delete;
    RobotDriverProvider(const RobotDriverProvider&) = delete;

    RobotDriverProvider(ros::NodeHandle& nodehandle);
    RobotDriverProvider(ros::NodeHandle& publisher_nodehandle, ros::NodeHandle& subscriber_nodehandle);

    VectorXd get_target_joint_positions() const;
    void send_joint_positions(const VectorXd& joint_positions);
    void send_joint_limits(const std::tuple<VectorXd, VectorXd>& joint_limits);

    bool is_enabled() const;
};

}
