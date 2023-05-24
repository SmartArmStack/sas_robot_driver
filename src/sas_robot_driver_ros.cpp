/*
# Copyright (c) 2016-2023 Murilo Marques Marinho
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
#
#
#
# Contributors:
#      1. Murilo M. Marinho (murilomarinho@ieee.org)
#         - Original implementation.
#         - [2023/05/23] Initializing the robot_driver_'s joint limits in the constructor.
#
#      2. Juan Jose Quiroz Omana (juanjqogm@gmail.com)
#         - [2023/05/20] Added documentation.
#         - [2023/05/24] Modified the control_loop method to set the joint target velocities and
#                        update the joint velocities and joint forces by using send_joint_states().
#      
# ################################################################
*/


#include <sas_robot_driver/sas_robot_driver_ros.h>
#include <sas_conversions/sas_conversions.h>
#include <dqrobotics/utils/DQ_Math.h>
#include <dqrobotics/interfaces/json11/DQ_JsonReader.h>

namespace sas
{

/**
 * @brief Constructor of the RobotDriverROS class.
 * 
 * @param nodehandle The roscpp's interface for creating subscribers, publishers, etc.
 * @param robot_driver The robot driver class of your robot, which must be a concrete class of the RobotDriver class.
 * @param configuration The configuration of the ROS robot driver.
 * @param kill_this_node Flag used to kill the current node. 
 * 
 *      Example of usage:
 *
 *      ros::NodeHandle nh;
 *      static std::atomic_bool kill_this_process(false);
 *      
 *      // Create a RobotDriverROSConfiguration struct
 *      sas::RobotDriverROSConfiguration robot_driver_ros_configuration;
 *      std::vector<double> q_min = {-2.3093,-1.5133,-2.4937, -2.7478,-2.4800, 0.8521, -2.6895};
 *      std::vector<double> q_max = { 2.3093, 1.5133, 2.4937, -0.4461, 2.4800, 4.2094,  2.6895};
 *      robot_driver_ros_configuration.thread_sampling_time_nsec = 4000000;
 *      robot_driver_ros_configuration.q_min = q_min;
 *      robot_driver_ros_configuration.q_max = q_max;
 *      robot_driver_ros_configuration.robot_driver_provider_prefix = ros::this_node::getName();
 * 
 *      // Create an object of RobotDriverMyRobot, where RobotDriverMyRobot is the class that manages
 *      // the communication with your robot. RobotDriverMyRobot must be a concrete class of RobotDriver()
 *      // A driver example for the Densowave VS050 robot manipulator (RobotDriverDenso) is provided here:
 *      // https://github.com/SmartArmStack/sas_robot_driver_denso/blob/master/include/sas_robot_driver_denso/sas_robot_driver_denso.h
 *      
 *      
 *      sas::RobotDriverMyRobot robot_driver_my_robot(custom_struct_for_my_robot_configuration, 
 *                                                       &kill_this_process);
 *
 *      
 *      sas::RobotDriverROS robot_driver_ros(nh,
 *                                            &robot_driver_my_robot,
 *                                            robot_driver_ros_configuration,
 *                                            &kill_this_process);
 * 
 *     A complete example is provided in 
 *     https://github.com/SmartArmStack/sas_robot_driver_denso/blob/master/src/sas_robot_driver_denso_node.cpp
 * 
 */
RobotDriverROS::RobotDriverROS(ros::NodeHandle &nodehandle,
                               RobotDriver* robot_driver,
                               const RobotDriverROSConfiguration &configuration,
                               std::atomic_bool *kill_this_node):
    configuration_(configuration),
    kill_this_node_(kill_this_node),
    robot_driver_(robot_driver),
    clock_(configuration.thread_sampling_time_nsec),
    robot_driver_provider_(nodehandle,configuration_.robot_driver_provider_prefix)
{
    //[2023/05/23] Initializing the robot_driver_'s joint limits in the constructor.
    //TODO: Check if the limits have not already been initialized in a better way?
    auto current_limits = robot_driver_->get_joint_limits();
    if(std::get<0>(current_limits).size() == 0
       || std::get<1>(current_limits).size() == 0)
    {
        auto q_min_vectorxd = std_vector_double_to_vectorxd(configuration.q_min);
        auto q_max_vectorxd = std_vector_double_to_vectorxd(configuration.q_max);
        robot_driver_->set_joint_limits({q_min_vectorxd,q_max_vectorxd});
    }
    else
    {
        ROS_WARN_STREAM(ros::this_node::getName() << "::Ignoring RobotDriverROS joint limits, they were already set by driver.");
    }
}


/**
 * @brief This method starts the communication loop with the robot. 
 *        At this stage, your robot is ready to be commanded. 
 *        
 * 
 * @return A return 0 means that the program did what it was intended to do.
 */
int RobotDriverROS::control_loop()
{
    try{
        clock_.init();
        ROS_INFO_STREAM(ros::this_node::getName() << "::Waiting to connect with robot...");
        robot_driver_->connect();
        ROS_INFO_STREAM(ros::this_node::getName() << "::Connected to robot.");
        ROS_INFO_STREAM(ros::this_node::getName() << "::Initializing robot...");
        robot_driver_->initialize();
        ROS_INFO_STREAM(ros::this_node::getName() << "::Robot initialized.");

        while(not _should_shutdown())
        {
            clock_.update_and_sleep();
            ros::spinOnce();

            if(robot_driver_provider_.is_enabled())
            {
                robot_driver_->set_target_joint_positions (robot_driver_provider_.get_target_joint_positions());

            }

            if(robot_driver_provider_.is_enabled(RobotDriver::Functionality::VelocityControl))
            {
                robot_driver_->set_target_joint_velocities(robot_driver_provider_.get_target_joint_velocities());
            }

            robot_driver_provider_.send_joint_limits(robot_driver_->get_joint_limits());


            VectorXd joint_velocities;
            VectorXd joint_forces;

            try{joint_velocities = robot_driver_->get_joint_velocities();} catch(...){}
            try{joint_forces     = robot_driver_->get_joint_forces    ();} catch(...){}

            robot_driver_provider_.send_joint_states(robot_driver_->get_joint_positions(),
                                                     joint_velocities,
                                                     joint_forces);



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


/**
 * @brief Destructor of  RobotDriverROS class. This method stops the communication with the robot
 * 
 */
RobotDriverROS::~RobotDriverROS()
{
    robot_driver_->deinitialize();
    robot_driver_->disconnect();
}
}
