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

#include <sas_robot_driver/sas_robot_driver.h>
#include <sas_robot_driver/sas_robot_driver_interface.h>
#include <sas_robot_driver/sas_robot_driver_provider.h>

namespace py = pybind11;
using RDI = sas::RobotDriverClient;
using RDP = sas::RobotDriverServer;

PYBIND11_MODULE(_sas_robot_driver, m) {

    py::enum_<sas::RobotDriver::Functionality>(m, "Functionality")
            .value("None",    sas::RobotDriver::Functionality::None)
            .value("PositionControl",sas::RobotDriver::Functionality::PositionControl)
            .value("VelocityControl",sas::RobotDriver::Functionality::VelocityControl)
            .value("ForceControl",sas::RobotDriver::Functionality::ForceControl)
            .value("Homing",sas::RobotDriver::Functionality::Homing)
            .value("ClearPositions",sas::RobotDriver::Functionality::ClearPositions)
            .export_values();

    py::class_<RDI>(m, "RobotDriverInterface")
            .def(py::init<std::shared_ptr<rclcpp::Node>&, const std::string&>())
            .def("send_target_joint_positions",&RDI::send_target_joint_positions)
            .def("send_target_joint_velocities",&RDI::send_target_joint_velocities)
            .def("send_target_joint_forces",&RDI::send_target_joint_forces)
            .def("send_homing_signal",&RDI::send_homing_signal)
            .def("send_clear_positions_signal",&RDI::send_clear_positions_signal)
            .def("get_joint_positions",&RDI::get_joint_positions)
            .def("get_joint_velocities",&RDI::get_joint_velocities)
            .def("get_joint_forces",&RDI::get_joint_forces)
            .def("get_joint_limits",&RDI::get_joint_limits)
            .def("get_home_states",&RDI::get_home_states)
            .def("is_enabled",&RDI::is_enabled,"Returns true if the RobotDriverInterface is enabled.",py::arg("supported_functionality")=sas::RobotDriver::Functionality::PositionControl)
            .def("get_topic_prefix",&RDI::get_topic_prefix);

    py::class_<RDP>(m, "RobotDriverProvider")
            .def(py::init<std::shared_ptr<rclcpp::Node>&, const std::string&>())
            .def("get_target_joint_positions",&RDP::get_target_joint_positions)
            .def("get_target_joint_velocities",&RDP::get_target_joint_velocities)
            .def("get_target_joint_forces",&RDP::get_target_joint_forces)
            .def("get_homing_signal",&RDP::get_homing_signal)
            .def("get_clear_positions_signal",&RDP::get_clear_positions_signal)
            .def("get_currently_active_functionality",&RDP::get_currently_active_functionality)
            .def("is_enabled",&RDP::is_enabled,"Returns true if the RobotDriverProvider is enabled.",py::arg("supported_functionality")=sas::RobotDriver::Functionality::PositionControl)
            .def("send_joint_states",&RDP::send_joint_states)
            .def("send_joint_limits",&RDP::send_joint_limits)
            .def("send_home_state",&RDP::send_home_state);

}
