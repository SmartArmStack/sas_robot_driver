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
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include <rclcpp/rclcpp.hpp>

#include <sas_core/sas_robot_driver.hpp>
#include <sas_robot_driver/sas_robot_driver_client.hpp>
#include <sas_robot_driver/sas_robot_driver_server.hpp>

namespace py = pybind11;
using RDC = sas::RobotDriverClient;
using RDS = sas::RobotDriverServer;

PYBIND11_MODULE(_sas_robot_driver, m) {

    py::enum_<sas::RobotDriver::Functionality>(m, "Functionality")
            .value("None",    sas::RobotDriver::Functionality::None)
            .value("PositionControl",sas::RobotDriver::Functionality::PositionControl)
            .value("VelocityControl",sas::RobotDriver::Functionality::VelocityControl)
            .value("ForceControl",sas::RobotDriver::Functionality::ForceControl)
            .value("Homing",sas::RobotDriver::Functionality::Homing)
            .value("ClearPositions",sas::RobotDriver::Functionality::ClearPositions)
            .export_values();

    py::class_<RDC>(m, "RobotDriverClient")
            .def(py::init<const std::shared_ptr<rclcpp::Node>&,const std::string&>())
            .def("send_target_joint_positions",&RDC::send_target_joint_positions)
            .def("send_target_joint_velocities",&RDC::send_target_joint_velocities)
            .def("send_target_joint_forces",&RDC::send_target_joint_forces)
            .def("send_homing_signal",&RDC::send_homing_signal)
            .def("send_clear_positions_signal",&RDC::send_clear_positions_signal)
            .def("get_joint_positions",&RDC::get_joint_positions)
            .def("get_joint_velocities",&RDC::get_joint_velocities)
            .def("get_joint_forces",&RDC::get_joint_forces)
            .def("get_joint_limits",&RDC::get_joint_limits)
            .def("get_home_states",&RDC::get_home_states)
            .def("is_enabled",&RDC::is_enabled,"Returns true if the RobotDriverInterface is enabled.",py::arg("supported_functionality")=sas::RobotDriver::Functionality::PositionControl)
            .def("get_topic_prefix",&RDC::get_topic_prefix);

    py::class_<RDS>(m, "RobotDriverServer")
            .def(py::init<const std::shared_ptr<rclcpp::Node>&,const std::string&>())
            .def("get_target_joint_positions",&RDS::get_target_joint_positions)
            .def("get_target_joint_velocities",&RDS::get_target_joint_velocities)
            .def("get_target_joint_forces",&RDS::get_target_joint_forces)
            .def("get_homing_signal",&RDS::get_homing_signal)
            .def("get_clear_positions_signal",&RDS::get_clear_positions_signal)
            .def("get_currently_active_functionality",&RDS::get_currently_active_functionality)
            .def("is_enabled",&RDS::is_enabled,"Returns true if the RobotDriverProvider is enabled.",py::arg("supported_functionality")=sas::RobotDriver::Functionality::PositionControl)
            .def("send_joint_states",&RDS::send_joint_states)
            .def("send_joint_limits",&RDS::send_joint_limits)
            .def("send_home_state",&RDS::send_home_state);

}
