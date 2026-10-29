# Copyright (c) 2012-2026 Murilo Marques Marinho
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
"""Round-trip tests for the RobotDriverClient and RobotDriverServer bindings.

Both peers run in-process on rclcpp nodes attached to the rclpy-initialized
context, mirroring how simulator drivers use them (see simros2_manager).
"""

import time

import numpy as np
import pytest
import rclpy
from rclpy.utilities import ok, try_shutdown

from sas_common import rclcpp_init, rclcpp_Node, rclcpp_shutdown, rclcpp_spin_some
from sas_robot_driver import (
    Functionality,
    RobotDriverClient,
    RobotDriverServer,
)

TIMEOUT = 15.0


@pytest.fixture(scope="module")
def rclpy_context():
    """Initialize the ROS context once for the whole module."""
    if not ok():
        rclpy.init()
    try:
        rclcpp_init()
        cpp_initialized = True
    except RuntimeError:
        cpp_initialized = False
    yield
    if cpp_initialized:
        rclcpp_shutdown()
    try_shutdown()


def _pump(*nodes):
    for node in nodes:
        rclcpp_spin_some(node)


def _wait_until(predicate, *nodes, timeout=TIMEOUT):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        _pump(*nodes)
        try:
            if predicate():
                return True
        except RuntimeError:
            pass
        time.sleep(0.005)
    try:
        return predicate()
    except RuntimeError:
        return False


def _close(a, b):
    try:
        return np.allclose(np.asarray(a, dtype=float), np.asarray(b, dtype=float),
                           atol=1e-9)
    except RuntimeError:
        return False


Q_MIN = [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0]
Q_MAX = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]


def _prime(server, q, qd=None, tau=None):
    """Publish the state/limit streams the client getters gate on."""
    server.send_joint_states(q,
                             qd if qd is not None else [0.0] * len(q),
                             tau if tau is not None else [0.0] * len(q))
    server.send_joint_limits((Q_MIN, Q_MAX))
    server.send_home_state([0] * len(q))


def test_target_joint_positions_round_trip(rclpy_context):
    client_node = rclcpp_Node("rd_client_node")
    server_node = rclcpp_Node("rd_server_node")
    prefix = "/test_rd_positions"
    client = RobotDriverClient(client_node, prefix)
    server = RobotDriverServer(server_node, prefix)

    target = [0.1, -0.2, 0.3, -0.4, 0.5, -0.6]
    client.send_target_joint_positions(target)
    assert _wait_until(lambda: _close(server.get_target_joint_positions(), target),
                       client_node, server_node)
    assert server.get_currently_active_functionality() == Functionality.PositionControl


def test_target_joint_velocities_round_trip(rclpy_context):
    client_node = rclcpp_Node("rd_client_node_vel")
    server_node = rclcpp_Node("rd_server_node_vel")
    prefix = "/test_rd_velocities"
    client = RobotDriverClient(client_node, prefix)
    server = RobotDriverServer(server_node, prefix)

    target = [0.0, 0.5, -0.5, 1.0, -1.0, 0.25]
    client.send_target_joint_velocities(target)
    assert _wait_until(lambda: _close(server.get_target_joint_velocities(), target),
                       client_node, server_node)
    assert server.get_currently_active_functionality() == Functionality.VelocityControl


def test_target_joint_forces_round_trip(rclpy_context):
    client_node = rclcpp_Node("rd_client_node_force")
    server_node = rclcpp_Node("rd_server_node_force")
    prefix = "/test_rd_forces"
    client = RobotDriverClient(client_node, prefix)
    server = RobotDriverServer(server_node, prefix)

    target = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    client.send_target_joint_forces(target)
    assert _wait_until(lambda: _close(server.get_target_joint_forces(), target),
                       client_node, server_node)


def test_homing_and_clear_signals_round_trip(rclpy_context):
    client_node = rclcpp_Node("rd_client_node_home")
    server_node = rclcpp_Node("rd_server_node_home")
    prefix = "/test_rd_home"
    client = RobotDriverClient(client_node, prefix)
    server = RobotDriverServer(server_node, prefix)

    signal = [1, 0, 1, 0, 1, 0]

    q = [0.0] * 6

    def homing_arrived():
        client.send_homing_signal(signal)
        _prime(server, q)
        return _close(server.get_homing_signal(), signal)

    assert _wait_until(homing_arrived, client_node, server_node)

    clear = [0, 1, 0, 1, 0, 1]

    def clear_arrived():
        client.send_clear_positions_signal(clear)
        _prime(server, q)
        return _close(server.get_clear_positions_signal(), clear)

    assert _wait_until(clear_arrived, client_node, server_node)


def test_joint_states_round_trip(rclpy_context):
    client_node = rclcpp_Node("rd_client_node_states")
    server_node = rclcpp_Node("rd_server_node_states")
    prefix = "/test_rd_states"
    client = RobotDriverClient(client_node, prefix)
    server = RobotDriverServer(server_node, prefix)

    q = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]
    qd = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    tau = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    def states_arrived():
        server.send_joint_states(q, qd, tau)
        server.send_joint_limits((Q_MIN, Q_MAX))
        return (_close(client.get_joint_positions(), q)
                and _close(client.get_joint_velocities(), qd)
                and _close(client.get_joint_forces(), tau))

    assert _wait_until(states_arrived, client_node, server_node)


def test_joint_limits_and_home_states_round_trip(rclpy_context):
    client_node = rclcpp_Node("rd_client_node_limits")
    server_node = rclcpp_Node("rd_server_node_limits")
    prefix = "/test_rd_limits"
    client = RobotDriverClient(client_node, prefix)
    server = RobotDriverServer(server_node, prefix)

    q_min = [-1.0, -2.0, -3.0, -4.0, -5.0, -6.0]
    q_max = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    server.send_joint_limits((q_min, q_max))
    homed = [1, 1, 0, 0, 1, 1]
    server.send_home_state(homed)

    def arrived():
        server.send_joint_states([0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                                 [0.0] * 6, [0.0] * 6)
        server.send_joint_limits((q_min, q_max))
        server.send_home_state(homed)
        limits = client.get_joint_limits()
        return (_close(limits[0], Q_MIN) and _close(limits[1], Q_MAX)
                and _close(client.get_home_states(), homed))

    assert _wait_until(arrived, client_node, server_node)


def test_enabled_queries(rclpy_context):
    client_node = rclcpp_Node("rd_client_node_enabled")
    server_node = rclcpp_Node("rd_server_node_enabled")
    prefix = "/test_rd_enabled"
    client = RobotDriverClient(client_node, prefix)
    server = RobotDriverServer(server_node, prefix)

    assert client.get_topic_prefix() == prefix

    q = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]

    def both_enabled():
        client.send_target_joint_positions(q)
        _prime(server, q)
        return (client.is_enabled(Functionality.PositionControl)
                and server.is_enabled(Functionality.PositionControl))

    assert _wait_until(both_enabled, client_node, server_node)
